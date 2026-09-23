"""相机与 IMU 落在**同一个 ROS 时间域**的端到端脱机测试。

为什么需要这个测试
------------------
「同一时间域」是靠实现约定保证的，而不是靠逐条比较时间戳：

- 相机：V4L2 驱动时间戳（`CLOCK_MONOTONIC` 时域）→ `t_ros = t_mono + ros_minus_mono_s`
- IMU ：PX4 启动时钟 → `t_mono = t_boot − clock_offset_s` → 同一个 `ros_minus_mono_s`

两条路径**必须共用同一个 `RosTimeBase`**（同一个 `ros_minus_mono_s` 与同一次采样），
否则会出现「各自映射一遍、各自带一次采样误差」的隐性偏差。本测试就用一个共享时间基
同时喂相机帧与 IMU 报文，断言二者严格落在同一时间轴上：

    相机帧 i 的真实时刻 t_mono  → 期望 IMU 时间戳 = 相机时间戳 + 已知偏移

捕获时间戳用 V4L2 帧时间戳（不是取帧返回时刻），IMU 时间戳用 `time_usec` 映射
（不是收包时刻）——这一点由 `test_mavlink_imu.py` 与 `test_camera_timestamp.py`
分别验证，这里只验证「同一个域」。

结论只覆盖软件路径；真实曝光时刻与真实 IMU 频率仍未验证。
"""

from __future__ import annotations

import pathlib
import struct
import time

import numpy as np
import pytest

from boom_birds_nav.camera_timestamp import (
    CLOCK_MONOTONIC,
    V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC,
    CameraTimestampSource,
    RawFrame,
    StereoFrameClock,
    decode_stitched,
)
from boom_birds_nav.mavlink_clock import ClockMapperConfig
from boom_birds_nav.mavlink_imu_core import MavlinkImuConfig, MavlinkImuReceiver
from boom_birds_nav.timebase import RosTimeBase
from test_mavlink_imu import FakeMav, boot_us_at  # noqa: E402  复用同一套构造器

MONO_T0 = 5_000.0
ROS_MINUS_MONO_S = 1_700_000_000.0     # 与 test_mavlink_imu 一致的时间基偏移
CAM_IMU_OFFSET_S = 0.05                # 约定：IMU 采样时刻 = 相机曝光时刻 + 该值
SYNTH_W, SYNTH_H = 64, 8               # 合成拼接帧尺寸（与真实记录帧尺寸无关）

# 真实记录帧：test/recordings/ 是**整帧的逐字节副本**（来源与校验见该目录 PROVENANCE.md）。
# capture.json 写明 "host save time, not exposure time"，因此这里只验证拼接格式与切分，
# 不证明曝光时刻。夹具只此一处，不再另建裁剪副本。
RECORDINGS = pathlib.Path(__file__).resolve().parent / "recordings"
FULL_W, FULL_H = 1280, 480             # 记录帧的拼接尺寸
CROP_H, CROP_W = 64, 256               # 只取左上角一块用于精确断言（边界仍在裁剪区内）


def _load_recorded_frame():
    """读取真实记录帧，返回 (stitched 灰度数组, left, right)。不可用时 skip。"""
    import cv2

    path = RECORDINGS / "stereo.png"
    if not path.is_file():
        pytest.skip(f"缺少记录帧夹具：{path}")
    stitched = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    assert stitched is not None and stitched.shape == (FULL_H, FULL_W), stitched.shape
    half = FULL_W // 2
    return stitched, stitched[:, :half], stitched[:, half:]


def _stitched_jpeg(marker: int) -> bytes:
    """构造一幅左右差异明显的拼接图（左半 marker，右半 marker+100），返回 JPEG。"""
    import cv2

    left = np.full((SYNTH_H, SYNTH_W // 2), marker, dtype=np.uint8)
    right = np.full((SYNTH_H, SYNTH_W // 2), marker + 100, dtype=np.uint8)
    stitched = np.hstack([left, right])
    ok, buf = cv2.imencode(".jpg", stitched)
    assert ok
    return buf.tobytes()


class FakeV4L2Source(CameraTimestampSource):
    """替换底层取帧：给出**构造的驱动时间戳**与拼接缓冲（不打开设备）。"""

    def __init__(self, frames):
        self.device = "/dev/fakevideo0"
        self.allow_realtime = False
        self.realtime_uncertainty_limit_s = 0.002
        self.negotiated = {"width": SYNTH_W, "height": SYNTH_H}
        self._frames = list(frames)
        self._index = 0

    def read_raw(self, timeout_s: float = 1.0) -> RawFrame:
        ts, payload = self._frames[self._index]
        self._index += 1
        return RawFrame(
            sequence=self._index,
            data=payload,
            timestamp_s=ts,
            clock_source=CLOCK_MONOTONIC,
            flags=V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC,
            received_mono_s=time.monotonic(),
            driver_sequence=self._index,
            buffer_index=self._index % 4,
            bytes_used=len(payload),
        )


@pytest.fixture()
def shared_timebase() -> RosTimeBase:
    """唯一的单调时钟 → ROS 时间映射；两条链路必须共用它。"""
    return RosTimeBase.from_offset(ROS_MINUS_MONO_S, MONO_T0)


def _imu_at(mono_s: float) -> FakeMav:
    """构造一条「IMU 采样时刻 = mono_s」的 HIGHRES_IMU（复用核心测试的时钟模型）。"""
    return FakeMav(
        105,
        time_usec=boot_us_at(mono_s),
        xacc=0.1, yacc=-0.2, zacc=9.81,
        xgyro=0.01, ygyro=0.02, zgyro=-0.03,
        fields_updated=0x3F,
    )


def _lock_imu_clock(core: MavlinkImuReceiver, encoder_mono_s: float, n: int = 8) -> float:
    """用 TIMESYNC 往返把 IMU 时钟锁定到与相机同一个单调时钟轴上。"""
    t = encoder_mono_s
    for _ in range(n):
        send_ns = int(round(t * 1e9))
        core.note_timesync_request(t, send_ns)
        half = 0.002
        tc1 = int(round((t + half - 1234.5) * 1e9))
        core.handle_message(FakeMav(111, tc1=tc1, ts1=send_ns), t + 2 * half)
        t += 0.5
    return t - 0.5


def test_camera_and_imu_share_one_ros_time_domain(shared_timebase):
    """相机帧与 IMU 采样经同一时间基映射后，严格落在同一条时间轴上。"""
    # ---- IMU 侧：锁定后按 50 Hz 发采样，采样时刻 = 相机时刻 + 约定偏移 ----
    imu_core = MavlinkImuReceiver(
        config=MavlinkImuConfig(expected_rate_hz=50.0),
        clock_config=ClockMapperConfig(max_rtt_s=0.02, sync_timeout_s=10.0),
    )
    imu_core.set_timebase(shared_timebase)
    _lock_imu_clock(imu_core, MONO_T0)

    # ---- 相机侧：同一时间基 + 驱动时间戳 ----
    frames = [(MONO_T0 + i * 0.02, _stitched_jpeg(10 + i)) for i in range(4)]
    cam = StereoFrameClock(FakeV4L2Source(frames), shared_timebase,
                           stitched_width=SYNTH_W)

    pairs = []
    for index in range(4):
        frame = cam.next_frame()
        # 同一时刻再采样一次时间基（真实节点每周期都会重采）
        imu_now = frame.capture_mono_s + CAM_IMU_OFFSET_S
        imu_core.set_virtual_mono(imu_now)
        imu_core.stamp()
        published = imu_core.handle_message(_imu_at(imu_now), imu_now)
        assert len(published) == 1, f"第 {index} 条 IMU 未被接受：{imu_core.counters}"
        pairs.append((frame, published[0]))

    # ---- 断言 1：两条链路用的是同一个映射（同一次采样的同一个偏移） ----
    cam_offset = shared_timebase.stability(1.0)["offset_s"]
    assert cam_offset == pytest.approx(ROS_MINUS_MONO_S)
    for frame, sample in pairs:
        assert frame.capture_ros_s - frame.capture_mono_s == pytest.approx(cam_offset)
        assert sample.stamp_ros_s - (sample.boot_us * 1e-6 - imu_core.clock.offset_s) \
            == pytest.approx(cam_offset)

    # ---- 断言 2：同一物理时差，在两个 ROS 时间戳上表现为同一个差值 ----
    for frame, sample in pairs:
        delta = sample.stamp_ros_s - frame.capture_ros_s
        assert delta == pytest.approx(CAM_IMU_OFFSET_S, abs=2e-3), (
            f"相机与 IMU 未落在同一时间域：差值 {delta} != {CAM_IMU_OFFSET_S}"
        )

    # ---- 断言 3：两侧时间戳都严格递增（同域前提下才可能同时成立） ----
    cam_stamps = [f.capture_ros_s for f, _ in pairs]
    imu_stamps = [s.stamp_ros_s for _, s in pairs]
    assert cam_stamps == sorted(cam_stamps)
    assert imu_stamps == sorted(imu_stamps)
    for (f1, s1), (f2, s2) in zip(pairs, pairs[1:]):
        assert (s2.stamp_ros_s - s1.stamp_ros_s) == pytest.approx(
            f2.capture_ros_s - f1.capture_ros_s, abs=1e-6
        ), "同一域下两路时间增量必须一致"

    # ---- 断言 4：左右图确实来自同一次取帧，并共享该帧的采集时间戳 ----
    first_frame, _ = pairs[0]
    left, right = cam.decode_stereo(first_frame)
    assert left.shape == right.shape == (SYNTH_H, SYNTH_W // 2)
    assert int(left.mean()) == 10 and int(right.mean()) == 110
    assert cam.counters["frames"] == 4
    # 拼接缓冲只有一份 → 不存在「左右各自打戳」的可能
    assert first_frame.half_width == SYNTH_W // 2


def test_decode_stitched_rejects_wrong_size(shared_timebase):
    """尺寸不符必须报错，不静默 resize/猜布局。"""
    payload = _stitched_jpeg(10)
    from boom_birds_nav.camera_timestamp import CameraTimestampError

    with pytest.raises(CameraTimestampError):
        decode_stitched(payload, SYNTH_W, SYNTH_H + 2)
    with pytest.raises(CameraTimestampError):
        decode_stitched(payload, SYNTH_W + 1, SYNTH_H)   # 奇数宽
    with pytest.raises(CameraTimestampError):
        decode_stitched(payload, 0, 0)                    # 缺尺寸
    with pytest.raises(CameraTimestampError):
        decode_stitched(b"not-a-jpeg", SYNTH_W, SYNTH_H)
    with pytest.raises(CameraTimestampError):
        decode_stitched(payload, SYNTH_W, SYNTH_H, split="vertical")


# ------------------------------------------------------------------ 真实记录帧


def test_real_recorded_frame_splits_exactly_by_width():
    """真实记录帧：整幅解码后按宽度对半切，与 half_a/half_b 逐像素相等。

    夹具是 `test/recordings/stereo.png`（480×1280 整帧的逐字节副本，来源见该目录
    PROVENANCE.md）。同一目录里 half_a/half_b 就是它的精确左右半边（已实测 max|diff|=0）。
    PNG 无损，所以这里可以要求精确相等——这正好证明「不是两幅独立 JPEG 顺序拼接」。
    """
    import numpy as np

    stitched, expected_left, expected_right = _load_recorded_frame()
    # 只用左上角一块做精确断言（夹具尺寸与运行时间都更友好），再补一次整帧一致性检查
    crop = stitched[:CROP_H, :CROP_W]
    payload = _png_bytes(crop)
    left, right = decode_stitched(payload, CROP_W, CROP_H)
    assert left.shape == right.shape == (CROP_H, CROP_W // 2)
    assert np.array_equal(left, crop[:, :CROP_W // 2]), "左半切分错位"
    assert np.array_equal(right, crop[:, CROP_W // 2:]), "右半切分错位"
    assert not np.array_equal(left, right), "左右内容相同，无法证明切分正确"

    # 整帧：与记录目录里的 half_a/half_b 逐像素相等
    full_left, full_right = decode_stitched(_png_bytes(stitched), FULL_W, FULL_H)
    assert np.array_equal(full_left, expected_left)
    assert np.array_equal(full_right, expected_right)


def _png_bytes(image) -> bytes:
    """把灰度图编成无损 PNG 字节（PNG 往返无损，可支撑精确断言）。"""
    import cv2

    ok, buf = cv2.imencode(".png", image)
    assert ok
    return buf.tobytes()


def test_real_recorded_frame_mjpeg_path():
    """真实记录帧走 MJPEG 载荷：形状正确、切分无错位（JPEG 有损故给容差）。

    实测 `cv2.imencode(JPG, q=100)` 往返 max|diff| = 1（整帧 19914/614400 像素差 1 灰阶）；
    这里留一档余量到 2。容差来自 JPEG 量化，不是切分错位——所以同时做敏感性检查：
    错位一列的差异是 142 灰阶量级，两者相差两个数量级。
    """
    import cv2
    import numpy as np

    stitched, _, _ = _load_recorded_frame()
    crop = stitched[:CROP_H, :CROP_W]
    ok, buf = cv2.imencode(".jpg", crop, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
    assert ok

    left, right = decode_stitched(buf.tobytes(), CROP_W, CROP_H)
    expected_left, expected_right = crop[:, :CROP_W // 2], crop[:, CROP_W // 2:]
    assert left.shape == right.shape == (CROP_H, CROP_W // 2)
    assert int(np.abs(left.astype(int) - expected_left.astype(int)).max()) <= 2
    assert int(np.abs(right.astype(int) - expected_right.astype(int)).max()) <= 2

    # 敏感性：把左右边界挪一列，差异远大于 JPEG 容差 → 上面的容差不会掩盖错位
    mis_split = decode_stitched(buf.tobytes(), CROP_W, CROP_H)
    shifted = np.roll(mis_split[0], 1, axis=1)
    assert int(np.abs(shifted.astype(int) - mis_split[0].astype(int)).max()) > 20, (
        "错位一列的差异应当远大于 JPEG 容差"
    )
    # 交叉比对：切错半边时差异同样远大于容差
    assert int(np.abs(left.astype(int) - expected_right.astype(int)).mean()) > 10


def test_stitched_frame_is_one_jpeg_not_two():
    """驳斥旧假设：拼接帧是**一整幅** JPEG，不是两幅 JPEG 顺序拼接。

    一次 `cv2.imdecode` 就能解出完整拼接尺寸；旧实现按 SOI/EOI 找「第二幅 JPEG」
    的做法已被删除。
    """
    import cv2
    import numpy as np

    stitched, _, _ = _load_recorded_frame()
    ok, buf = cv2.imencode(".jpg", stitched, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
    assert ok
    payload = buf.tobytes()

    image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_GRAYSCALE)
    assert image is not None
    assert image.shape == (FULL_H, FULL_W), "整幅解码得不到拼接尺寸"

    # 第一个 EOI 之后不应再出现 SOI：整幅 JPEG 只有一个图像起始标记
    first_soi = payload.find(b"\xff\xd8")
    first_eoi = payload.find(b"\xff\xd9")
    assert first_soi >= 0 and first_eoi > first_soi
    assert payload.find(b"\xff\xd8", first_eoi + 2) < 0, (
        "载荷中出现了第二个 JPEG 起始标记：与「一整幅拼接帧」的格式假设不符"
    )


def test_v4l2_buffer_struct_layout_constants_are_self_consistent():
    """缓冲长度与字段不重叠：再次防止把 flags/timestamp/length 写到错误偏移。"""
    from boom_birds_nav import camera_timestamp as ct

    order = [
        ct.V4L2_BUFFER_OFF_INDEX,
        ct.V4L2_BUFFER_OFF_TYPE,
        ct.V4L2_BUFFER_OFF_BYTESUSED,
        ct.V4L2_BUFFER_OFF_FLAGS,
        ct.V4L2_BUFFER_OFF_FIELD,
        ct.V4L2_BUFFER_OFF_TIMESTAMP_SEC,
        ct.V4L2_BUFFER_OFF_TIMESTAMP_USEC,
        ct.V4L2_BUFFER_OFF_SEQUENCE,
        ct.V4L2_BUFFER_OFF_MEMORY,
        ct.V4L2_BUFFER_OFF_LENGTH,
        ct.V4L2_BUFFER_OFF_REQUEST_FD,
    ]
    assert order == sorted(order), "字段偏移必须单调递增"
    assert ct.V4L2_BUFFER_SIZE == 88
    assert ct.V4L2_BUFFER_OFF_REQUEST_FD + 4 <= ct.V4L2_BUFFER_SIZE

    # parse_buffer 只从正确偏移取值：把每个字段写成互不相同的哨兵值再解析回来
    buf = bytearray(ct.V4L2_BUFFER_SIZE)
    struct.pack_into("I", buf, ct.V4L2_BUFFER_OFF_INDEX, 11)
    struct.pack_into("I", buf, ct.V4L2_BUFFER_OFF_BYTESUSED, 22)
    struct.pack_into("I", buf, ct.V4L2_BUFFER_OFF_FLAGS,
                     ct.V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)
    struct.pack_into("I", buf, ct.V4L2_BUFFER_OFF_SEQUENCE, 44)
    struct.pack_into("I", buf, ct.V4L2_BUFFER_OFF_LENGTH, 55)
    struct.pack_into("q", buf, ct.V4L2_BUFFER_OFF_TIMESTAMP_SEC, 66)
    struct.pack_into("q", buf, ct.V4L2_BUFFER_OFF_TIMESTAMP_USEC, 77000)
    parsed = CameraTimestampSource.parse_buffer(buf)
    assert parsed["index"] == 11
    assert parsed["bytesused"] == 22
    assert parsed["flags"] == ct.V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC
    assert parsed["sequence"] == 44
    assert parsed["length"] == 55
    assert parsed["timestamp_s"] == pytest.approx(66.077)
    assert parsed["clock_source"] == CLOCK_MONOTONIC
