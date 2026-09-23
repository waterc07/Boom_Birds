"""唯一采集源（真实 V4L2 采集 / 录制帧回放）的脱机测试：**不打开任何设备**。

为什么需要这些测试
------------------
需求是「相机只能在一个地方打开」，且回放与真实采集必须走**同一条**发布/解码路径。
因此这里验证两件事：

1. `ReplayFrameSource` 与 `V4L2FrameSource` 满足同一个 `FrameSource` 协议，
   一次 `next_frame()` 只产出**一个** `StereoFrame`（一整幅拼接载荷 + 一个时间戳）；
2. 真实采集路径的字段解析（时间戳在 `v4l2_buffer` 偏移 24/32、`bytesused` 在 8、
   时域由 `flags` 决定）在**不开设备**的前提下被完整走到。

V4L2 用假 ioctl：`os.open` 返回真实 memfd（使 `select`/`mmap` 语义仍然真实），
`fcntl.ioctl` 按 `VIDIOC_*` 分派，字段严格按 `camera_timestamp` 的常量偏移写回。
手法与 `test_v4l2_driver_sim.py` 相同；这里额外覆盖：时域未知、时间戳倒退、
协商尺寸/格式不符、载荷必须取自 `bytesused`，以及可解码的真实载荷。

边界（不要从本文件的通过推断硬件行为）
----------------------------------------
- 录制帧**没有曝光时间戳**（见 `test/recordings/PROVENANCE.md`）：回放时间戳是
  `start_mono_s + i*period_s` 的合成占位值，不代表任何真实曝光时刻；
- 真实曝光时刻、驱动 `flags` 的真实取值、真机取帧稳定性都只能在硬件上验收；
- 本文件全程不触碰 `/dev/video*`：`os.open` 被替换为记录器/假驱动。
"""

from __future__ import annotations

import dataclasses
import inspect
import os
import pathlib
import struct

import numpy as np
import pytest

from boom_birds_nav import camera_timestamp as ct
from boom_birds_nav import stereo_capture as sc
from boom_birds_nav.camera_timestamp import CameraTimestampError, StereoFrame, decode_stitched
from boom_birds_nav.stereo_capture import (
    FrameSource,
    ReplayExhausted,
    ReplayFrameSource,
    V4L2FrameSource,
    discover_recorded_frames,
)
from boom_birds_nav.timebase import RosTimeBase

RECORDINGS = pathlib.Path(__file__).resolve().parent / "recordings"
STEREO_PNG = RECORDINGS / "stereo.png"
HALF_A_PNG = RECORDINGS / "half_a.png"
HALF_B_PNG = RECORDINGS / "half_b.png"
FULL_W, FULL_H = 1280, 480                  # 录制帧的拼接尺寸（不是相机协商结果）
START_MONO_S = 5_000.0
ROS_MINUS_MONO_S = 1_700_000_000.0          # 与既有时间域测试一致的量级
# 用 1/64 而不是 1/60：1/64 是二进制精确的，这样「相邻帧恰好相差 period_s」可以在
# ROS 域（约 1.7e9，双精度分辨率约 4.8e-7 s）里做**逐位相等**的断言，而不是用容差掩盖。
PERIOD_S = 1.0 / 64.0


def _timebase() -> RosTimeBase:
    """确定性时间基（不读真实时钟）：离线回放与单测都用它。"""
    return RosTimeBase.from_offset(ROS_MINUS_MONO_S, START_MONO_S)


def _gray(path: pathlib.Path):
    import cv2

    return cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)


def _write_png(path: pathlib.Path, image) -> None:
    import cv2

    ok, buf = cv2.imencode(".png", image)
    assert ok
    path.write_bytes(buf.tobytes())


def _stitched_jpeg(width=64, height=8, left=10, right=200) -> bytes:
    """构造一幅左右差异明显的整幅拼接 JPEG（与 test_time_domain_integration 同法）。"""
    import cv2

    left_half = np.full((height, width // 2), left, dtype=np.uint8)
    right_half = np.full((height, width // 2), right, dtype=np.uint8)
    ok, buf = cv2.imencode(".jpg", np.hstack([left_half, right_half]))
    assert ok
    return buf.tobytes()


# ================================================================== 回放源：真实录制帧


def test_real_frame_replay_splits_exactly_and_carries_the_file_bytes():
    """录制帧回放：`frame.stitched` 是**文件自身字节**，解码后与 half_a/half_b 逐像素相等。

    为什么要断言「字节等于文件」：回放不许悄悄改变格式（把 PNG 重编码成 JPEG 会引入
    有损误差，「逐像素相等」这条断言就再也立不住）。录制帧是 PNG（无损），真实采集是
    MJPEG（有损）——两者共用的是**解码/切分路径**（`decode_stitched` 对整幅载荷做一次
    `cv2.imdecode` 再按宽度对半切），不是编码格式。
    """
    file_bytes = STEREO_PNG.read_bytes()
    source = ReplayFrameSource(
        STEREO_PNG, start_mono_s=START_MONO_S, period_s=PERIOD_S, timebase=_timebase()
    )
    frame = source.next_frame()

    assert frame.stitched == file_bytes, "回放必须原样携带文件字节，不得重编码"
    assert (frame.stitched_width, frame.height, frame.half_width) == (
        FULL_W, FULL_H, FULL_W // 2,
    )

    left, right = decode_stitched(frame.stitched, FULL_W, FULL_H)
    assert np.array_equal(left, _gray(HALF_A_PNG)), "左半切分错位或经过了有损编码"
    assert np.array_equal(right, _gray(HALF_B_PNG)), "右半切分错位或经过了有损编码"
    assert not np.array_equal(left, right), "左右内容相同就证明不了切分正确"
    source.close()


def test_one_next_frame_yields_one_frame_sharing_one_timestamp():
    """一次 `next_frame()` = 一次底层取帧 = **一个** `StereoFrame`，左右共享它唯一的时间戳。

    为什么这就是「共享」的保证（而不是靠约定）：载荷是**一整幅**图，`decode_stitched`
    只从**同一份** `stitched` 字节里按宽度切出两半；而 `StereoFrame` 上只有
    `capture_mono_s` / `capture_ros_s` 这一组采集时间字段，根本不存在第二个字段可以
    存放「另一目」的时间。于是「一个时间戳被两半共享」是结构性的，不是纪律性的。
    """
    source = ReplayFrameSource(
        STEREO_PNG, start_mono_s=START_MONO_S, period_s=PERIOD_S, timebase=_timebase()
    )
    frame = source.next_frame()

    assert isinstance(frame, StereoFrame)
    assert frame.sequence == 1
    described = source.describe()
    assert described["frames_delivered"] == 1, "一次调用只允许交付一帧"
    assert described["reads"] == 1, "一次 next_frame 只允许一次底层取帧"

    fields = {field.name for field in dataclasses.fields(StereoFrame)}
    assert {"capture_mono_s", "capture_ros_s", "capture_uncertainty_s"} <= fields
    assert not any("left" in name or "right" in name for name in fields), (
        "StereoFrame 不允许出现按目分开的时间/载荷字段（那会允许左右各自打戳）"
    )
    # 两半解出来之后也没有各自的时间戳可比较：它们只可能来自上面这一个字段。
    left, right = decode_stitched(frame.stitched, frame.stitched_width, frame.height)
    assert left.shape == right.shape == (FULL_H, FULL_W // 2)
    source.close()


def test_replay_timestamp_domain_is_exact_and_monotonic():
    """回放时间戳：`capture_ros_s - capture_mono_s` 恰为偏移，相邻帧恰好相差 period_s。

    `start_mono_s` 与偏移都取整数、`period_s` 取 1/64（二进制精确），因此这里的
    「相等」是逐位相等：ROS 域约 1.7e9 时双精度分辨率约 4.8e-7 s，用容差比较会掩盖
    「时间戳被反复映射/被重新采样」这类真实缺陷。

    注意：这组时间戳是**合成占位值**，不代表任何真实曝光时刻（录制帧没有曝光时间戳）。
    """
    source = ReplayFrameSource(
        STEREO_PNG, start_mono_s=START_MONO_S, period_s=PERIOD_S, timebase=_timebase()
    )
    frames = [source.next_frame() for _ in range(3)]

    for frame in frames:
        assert frame.capture_ros_s - frame.capture_mono_s == ROS_MINUS_MONO_S
        assert frame.clock_source == ct.CLOCK_MONOTONIC
    for index in range(1, len(frames)):
        assert frames[index].capture_ros_s - frames[index - 1].capture_ros_s == PERIOD_S
        assert frames[index].capture_mono_s - frames[index - 1].capture_mono_s == PERIOD_S
    stamps = [frame.capture_ros_s for frame in frames]
    assert stamps == sorted(stamps) and len(set(stamps)) == len(stamps)
    source.close()


def test_replay_driver_sequence_is_the_frame_index_absolute_across_loops():
    """`driver_sequence` = 绝对帧序号（0 起）；循环回放继续累加，时间戳不会倒退。"""
    source = ReplayFrameSource(
        [STEREO_PNG, STEREO_PNG],
        start_mono_s=START_MONO_S,
        period_s=PERIOD_S,
        timebase=_timebase(),
    )
    frames = [source.next_frame() for _ in range(4)]
    assert [frame.driver_sequence for frame in frames] == [0, 1, 2, 3]
    assert [frame.sequence for frame in frames] == [1, 2, 3, 4]
    stamps = [frame.capture_mono_s for frame in frames]
    assert stamps == sorted(stamps) and len(set(stamps)) == len(stamps)
    source.close()


def test_replay_pair_of_halves_becomes_one_stitched_payload():
    """左右半图成对时，回放仍给出**一整幅**拼接载荷（不是两幅图字节首尾相接）。

    真实采集链的格式是「一整幅 MJPEG，解码后按宽度对半切」。半图条目因此必须水平拼接
    成一幅再编码（PNG 无损，不引入有损误差）——否则 `decode_stitched` 解不出拼接尺寸，
    或者解出来是错的布局。这里用真实夹具的半图，断言拼出来的结果与其左右半逐像素相等。
    """
    source = ReplayFrameSource(
        [HALF_A_PNG, HALF_B_PNG],
        start_mono_s=START_MONO_S,
        period_s=PERIOD_S,
        timebase=_timebase(),
    )
    assert source.describe()["frame_count"] == 1, "一对半图只算一帧"
    frame = source.next_frame()
    assert (frame.stitched_width, frame.height) == (FULL_W, FULL_H)
    left, right = decode_stitched(frame.stitched, FULL_W, FULL_H)
    assert np.array_equal(left, _gray(HALF_A_PNG))
    assert np.array_equal(right, _gray(HALF_B_PNG))
    source.close()


def test_replay_directory_discovery_is_sorted_and_reports_skipped(tmp_path):
    """目录发现：整幅帧优先、成对半图可回放、落单半图**跳过并给出原因**。

    为什么落单半图必须跳过：半图不是整幅帧，把它当整幅会静默把帧宽减半
    （切出来的一半再对半切），这种错误在图像上几乎看不出来。
    为什么 `<base>.png` 存在时它的半图要跳过：半图是整幅的派生物，重复回放等于同一
    内容发两遍。
    """
    half = np.full((FULL_H, FULL_W // 2), 7, dtype=np.uint8)
    _write_png(tmp_path / "pair_a.png", half)
    _write_png(tmp_path / "pair_b.png", half + 100)
    _write_png(tmp_path / "whole.png", np.hstack([half, half + 100]))
    _write_png(tmp_path / "whole_a.png", half)
    _write_png(tmp_path / "whole_b.png", half + 100)
    _write_png(tmp_path / "lonely_a.png", half)

    discovered = discover_recorded_frames(tmp_path)
    assert [(entry.files, entry.is_pair) for entry in discovered.entries] == [
        ((str(tmp_path / "pair_a.png"), str(tmp_path / "pair_b.png")), True),
        ((str(tmp_path / "whole.png"),), False),
    ], "目录扫描必须按文件名排序，且整幅帧优先于其半图"
    skipped = dict(discovered.skipped)
    assert "落单" in skipped[str(tmp_path / "lonely_a.png")]
    assert "整幅帧" in skipped[str(tmp_path / "whole_a.png")]

    # 落单半图不会被当成帧：回放这个目录只能得到 2 帧（成对半图 + 整幅帧）。
    source = ReplayFrameSource(
        tmp_path, start_mono_s=START_MONO_S, period_s=PERIOD_S, timebase=_timebase()
    )
    assert source.describe()["frame_count"] == 2
    assert source.describe()["skipped"], "跳过原因必须出现在 describe() 里（不静默丢帧）"
    left, right = decode_stitched(source.next_frame().stitched, FULL_W, FULL_H)
    assert int(left.mean()) == 7 and int(right.mean()) == 107
    source.close()


def test_replay_rejects_lone_half_odd_width_and_mixed_sizes(tmp_path):
    """显式给出的落单半图、奇数拼接宽、同源混合尺寸都必须报错（不猜、不静默）。"""
    lone_dir = tmp_path / "lone"
    lone_dir.mkdir()
    _write_png(lone_dir / "lonely_a.png", np.full((16, 32), 3, dtype=np.uint8))
    with pytest.raises(CameraTimestampError) as exc:
        ReplayFrameSource(
            lone_dir / "lonely_a.png",
            start_mono_s=START_MONO_S,
            period_s=PERIOD_S,
            timebase=_timebase(),
        )
    assert "半图" in str(exc.value)

    odd_dir = tmp_path / "odd"
    odd_dir.mkdir()
    _write_png(odd_dir / "odd.png", np.full((16, 33), 3, dtype=np.uint8))
    with pytest.raises(CameraTimestampError) as exc:
        ReplayFrameSource(
            odd_dir / "odd.png",
            start_mono_s=START_MONO_S,
            period_s=PERIOD_S,
            timebase=_timebase(),
        )
    assert "不是偶数" in str(exc.value)

    mixed_dir = tmp_path / "mixed"
    mixed_dir.mkdir()
    _write_png(mixed_dir / "a.png", np.full((8, 64), 3, dtype=np.uint8))
    _write_png(mixed_dir / "b.png", np.full((16, 64), 3, dtype=np.uint8))
    source = ReplayFrameSource(
        mixed_dir, start_mono_s=START_MONO_S, period_s=PERIOD_S, timebase=_timebase()
    )
    assert source.next_frame().height == 8
    with pytest.raises(CameraTimestampError) as exc:
        source.next_frame()
    assert "只允许一个拼接尺寸" in str(exc.value), "尺寸不符必须指名原因，不能静默发布"
    source.close()


def test_replay_exhaustion_and_loop_semantics():
    """`loop=False` 到末尾抛 `ReplayExhausted`（可预期的收尾），`loop=True` 则继续。"""
    one = ReplayFrameSource(
        STEREO_PNG,
        start_mono_s=START_MONO_S,
        period_s=PERIOD_S,
        timebase=_timebase(),
        loop=False,
    )
    one.next_frame()
    with pytest.raises(ReplayExhausted):
        one.next_frame()
    assert issubclass(ReplayExhausted, CameraTimestampError), "调用方可以只捕获基类"
    one.close()
    with pytest.raises(CameraTimestampError):
        one.next_frame()

    looping = ReplayFrameSource(
        STEREO_PNG, start_mono_s=START_MONO_S, period_s=PERIOD_S, timebase=_timebase()
    )
    stamps = [looping.next_frame().capture_mono_s for _ in range(3)]
    assert stamps == sorted(stamps) and len(set(stamps)) == 3
    looping.close()


def test_replay_never_opens_a_device(monkeypatch):
    """回放路径**零设备访问**：把 `os.open` 换成记录器，全程跑一遍并断言没有调用。

    `CameraTimestampSource.open()` 正是通过 `os.open(...)` 打开设备的，所以把它打桩
    既能记录、又能立刻拦住任何意外的设备访问（本任务不允许碰 `/dev/video*`）。
    """
    opened: list[str] = []

    def _record(path, *args, **kwargs):
        opened.append(str(path))
        raise AssertionError(f"回放路径不允许打开任何东西：{path}")

    monkeypatch.setattr(os, "open", _record)
    with ReplayFrameSource(
        STEREO_PNG, start_mono_s=START_MONO_S, period_s=PERIOD_S, timebase=_timebase()
    ) as source:
        frame = source.next_frame()
        decode_stitched(frame.stitched, FULL_W, FULL_H)
        assert source.describe()["device_opened"] is False
    assert opened == [], "回放只读文件，不得打开任何路径（更不允许打开设备）"


def test_replay_describe_marks_replay_and_no_exposure_timestamp():
    """`describe()` 必须显式标记 replay=True，并写明**没有曝光时间戳**与格式差异。"""
    source = ReplayFrameSource(
        STEREO_PNG, start_mono_s=START_MONO_S, period_s=PERIOD_S, timebase=_timebase()
    )
    source.next_frame()
    described = source.describe()

    assert described["replay"] is True and described["kind"] == "replay"
    assert described["clock_source"] == ct.CLOCK_MONOTONIC
    assert "曝光" in described["exposure_note"], "必须写明录制帧没有曝光时间戳"
    assert "PNG" in described["format_note"] and "MJPEG" in described["format_note"]
    assert described["counters"]["frames"] == 1
    assert described["device_opened"] is False
    assert described["stitched_width"] == FULL_W and described["half_width"] == FULL_W // 2
    # 合成时间戳有确定的定义，可复现：第 i 帧 = start_mono_s + i*period_s
    assert described["start_mono_s"] == START_MONO_S and described["period_s"] == PERIOD_S
    source.close()


# ================================================================== 真实采集路径（假 ioctl）


class MmapStub:
    """`mmap.mmap` 替身：只需支持 len / 切片 / close（与 test_v4l2_driver_sim 同法）。"""

    def __init__(self, data: bytes) -> None:
        self._data = data
        self.closed = False

    def __len__(self) -> int:
        return len(self._data)

    def __getitem__(self, item):
        return self._data[item]

    def close(self) -> None:
        self.closed = True


class FakeV4L2Driver:
    """最小假 V4L2 驱动：回应 S_FMT / REQBUFS / QUERYBUF / QBUF / DQBUF / STREAMON。

    字段严格按 `v4l2_buffer` 的真实布局写回：时间戳 offset 24/32、`bytesused` 8、
    `flags` 12、`sequence` 56、`length` 72。若 `camera_timestamp` 的常量写错，
    写回的字段就会被解析到别处，断言立刻失败（偏移量另有编译期 offsetof 核对，互补）。
    """

    def __init__(self, width, height, payload: bytes, frames, buffer_count: int = 3,
                 mmap_length: int | None = None, report_size=None,
                 report_pixel_format=None, pixel_format=None):
        self.width, self.height = int(width), int(height)
        self.report_size = tuple(report_size) if report_size else (self.width, self.height)
        self.pixel_format = (ct.V4L2_PIX_FMT_MJPEG if pixel_format is None
                             else int(pixel_format))
        self.report_pixel_format = (self.pixel_format if report_pixel_format is None
                                    else int(report_pixel_format))
        self.payload = payload
        self.frames = list(frames)          # [{t, flags, sequence}, ...]
        self.buffer_count = int(buffer_count)
        # 映射长度故意大于本帧有效字节数：用来证明载荷取自 bytesused 而不是 length。
        self.mmap_length = int(mmap_length if mmap_length is not None else len(payload) + 64)
        self.calls: list[int] = []
        self.opened_paths: list[str] = []
        self.qbuf_indices: list[int] = []
        self.streaming = False
        self.dqbuf_count = 0

    def mmap_bytes(self, length: int) -> bytes:
        """映射区内容 = 本帧载荷 + 可辨识的填充字节（填充绝不能被当成帧数据）。"""
        pad = bytes([0xAB]) * max(length - len(self.payload), 0)
        return (self.payload + pad)[:length]

    def ioctl(self, fd, request, arg, mutate=True):
        self.calls.append(request)
        if request == ct.VIDIOC_S_FMT:
            self._s_fmt(arg)
        elif request == ct.VIDIOC_REQBUFS:
            self._reqbufs(arg)
        elif request == ct.VIDIOC_QUERYBUF:
            self.qbuf_indices.append(
                struct.unpack_from("I", bytes(arg), ct.V4L2_BUFFER_OFF_INDEX)[0]
            )
            struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_LENGTH, self.mmap_length)
            struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_BYTESUSED, self.mmap_length)
        elif request == ct.VIDIOC_QBUF:
            self.qbuf_indices.append(
                struct.unpack_from("I", bytes(arg), ct.V4L2_BUFFER_OFF_INDEX)[0]
            )
        elif request == ct.VIDIOC_DQBUF:
            self._dqbuf(arg)
        elif request == ct.VIDIOC_STREAMON:
            self.streaming = True
        elif request == ct.VIDIOC_STREAMOFF:
            self.streaming = False
        return 0

    def _s_fmt(self, arg) -> None:
        struct.pack_into("IIIIII", arg, ct.V4L2_FORMAT_OFF_WIDTH,
                         self.report_size[0], self.report_size[1], self.report_pixel_format,
                         ct.V4L2_FIELD_NONE, 0, 0)

    def _reqbufs(self, arg) -> None:
        struct.pack_into("IIII", arg, 0, self.buffer_count,
                         ct.V4L2_BUF_TYPE_VIDEO_CAPTURE, ct.V4L2_MEMORY_MMAP, 0)

    def _dqbuf(self, arg) -> None:
        spec = self.frames[min(self.dqbuf_count, len(self.frames) - 1)]
        index = self.dqbuf_count % self.buffer_count
        self.dqbuf_count += 1
        struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_INDEX, index)
        struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_TYPE, ct.V4L2_BUF_TYPE_VIDEO_CAPTURE)
        struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_BYTESUSED, len(self.payload))
        struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_FLAGS, spec["flags"])
        struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_SEQUENCE, spec["sequence"])
        struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_MEMORY, ct.V4L2_MEMORY_MMAP)
        struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_LENGTH, self.mmap_length)
        struct.pack_into("q", arg, ct.V4L2_BUFFER_OFF_TIMESTAMP_SEC, int(spec["t"]))
        struct.pack_into("q", arg, ct.V4L2_BUFFER_OFF_TIMESTAMP_USEC,
                         int(round((spec["t"] - int(spec["t"])) * 1e6)))


def _frames(count: int, t0: float = 1000.5, dt: float = 0.02,
            flags: int = ct.V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC, sequence0: int = 12345):
    return [{"t": t0 + index * dt, "flags": flags, "sequence": sequence0 + index}
            for index in range(count)]


def _fake_v4l2_source(monkeypatch, driver: FakeV4L2Driver, request_size,
                      **source_kwargs):
    """装好假管线并构造 `V4L2FrameSource`；返回 (源, memfd, 真 os.close)。

    真实 memfd 保证 `select`/`mmap` 语义仍然真实；`os.open` 被替换成记录器，
    因此**不可能**打开真实设备（测试里还会核对被打开过的路径）。
    `os.close` 在假管线里是空操作，所以真 fd 由测试用返回的真函数收尾。
    """
    real_close = os.close                       # 先取真函数：下面会把 os.close 打桩
    fd = os.memfd_create("bb_fake_v4l2")
    stubs: list[MmapStub] = []

    def _open(path, *args, **kwargs):
        driver.opened_paths.append(str(path))
        return fd

    def _mmap(fileno, length, *args, **kwargs):
        stub = MmapStub(driver.mmap_bytes(length))
        stubs.append(stub)
        return stub

    monkeypatch.setattr(ct.os, "open", _open)
    monkeypatch.setattr(ct.os, "close", lambda handle: None)
    monkeypatch.setattr(ct.fcntl, "ioctl", driver.ioctl)
    monkeypatch.setattr(ct.mmap, "mmap", _mmap)
    source_kwargs.setdefault("timebase", _timebase())
    source = V4L2FrameSource(
        "/dev/fakevideo0", request_size[0], request_size[1], **source_kwargs
    )
    return source, fd, real_close


def test_v4l2_frame_timestamp_and_bytesused_come_from_the_driver(monkeypatch):
    """真实采集路径：时间戳取自 `v4l2_buffer` 偏移 24/32，载荷长度取自 `bytesused`。

    - 时间戳：两帧差必须**恰好**等于驱动给的 20 ms。若实现改用「收帧时刻」，两帧间隔
      只有微秒级，这条断言必然失败（不依赖本机 uptime 量级，因此在任何机器上都有效）。
    - 载荷：映射长度比本帧有效字节多 64 字节，`frame.stitched` 必须恰好等于载荷本身，
      多一个字节就说明读的是 `length` 而不是 `bytesused`（填充会被当成图像数据）。
    """
    payload = _stitched_jpeg(64, 8)
    driver = FakeV4L2Driver(64, 8, payload, _frames(2))
    source, fd, real_close = _fake_v4l2_source(
        monkeypatch, driver, (64, 8), fps=50, buffer_count=3
    )
    try:
        first = source.next_frame()
        second = source.next_frame()

        assert first.capture_mono_s == pytest.approx(1000.5, abs=1e-9)
        assert second.capture_mono_s == pytest.approx(1000.52, abs=1e-9)
        assert second.capture_mono_s - first.capture_mono_s == pytest.approx(0.02, abs=1e-9)
        assert first.capture_ros_s - first.capture_mono_s == ROS_MINUS_MONO_S
        assert first.clock_source == ct.CLOCK_MONOTONIC
        assert (first.driver_sequence, second.driver_sequence) == (12345, 12346)
        assert (first.stitched_width, first.height) == (64, 8)

        assert driver.mmap_length > len(payload), "映射长度必须大于有效字节，否则本断言无意义"
        assert first.stitched == payload, "载荷必须按 bytesused 截取，且内容与驱动一致"
        assert len(first.stitched) == len(payload)

        left, right = decode_stitched(first.stitched, 64, 8)
        assert abs(int(left.mean()) - 10) <= 2 and abs(int(right.mean()) - 200) <= 2

        # 缓冲必须还回驱动：每个出队的缓冲都要 QBUF 一次
        assert driver.qbuf_indices.count(0) >= 2
        assert source.describe()["counters"]["frames"] == 2
        assert driver.opened_paths == ["/dev/fakevideo0"], "只允许打开请求的那一个设备节点"
        assert not any(path.startswith("/dev/video") for path in driver.opened_paths)
    finally:
        source.close()
        real_close(fd)


def test_v4l2_unknown_timestamp_domain_is_rejected_without_fallback(monkeypatch):
    """时域不可核实（`flags` 未声明 MONOTONIC）→ 报错，**不回退**到收帧时刻、不发布帧。"""
    payload = _stitched_jpeg(64, 8)
    driver = FakeV4L2Driver(64, 8, payload,
                            _frames(1, flags=ct.V4L2_BUF_FLAG_TIMESTAMP_UNKNOWN))
    source, fd, real_close = _fake_v4l2_source(monkeypatch, driver, (64, 8))
    try:
        with pytest.raises(CameraTimestampError) as exc:
            source.next_frame()
        assert "不可核实" in str(exc.value)
        described = source.describe()
        assert described["counters"]["frames"] == 0, "不可核实的帧绝不能计入已发布帧"
        assert described["fetch_errors"] == 1
        assert described["clock_source"] is None, "从未观察到可核实的驱动时域"
        assert "不回退" in described["clock_source_note"]
    finally:
        source.close()
        real_close(fd)


def test_v4l2_backwards_driver_timestamp_is_rejected(monkeypatch):
    """驱动时间戳倒退（第二帧更早）→ 第二帧报错，且不计入已发布帧数。"""
    payload = _stitched_jpeg(64, 8)
    frames = _frames(2, t0=1000.52, dt=0.02)
    frames[1]["t"] = 1000.50                     # 倒退 20 ms
    driver = FakeV4L2Driver(64, 8, payload, frames)
    source, fd, real_close = _fake_v4l2_source(monkeypatch, driver, (64, 8))
    try:
        assert source.next_frame().capture_mono_s == pytest.approx(1000.52, abs=1e-9)
        with pytest.raises(CameraTimestampError) as exc:
            source.next_frame()
        assert "单调" in str(exc.value)
        described = source.describe()
        assert described["counters"]["backwards"] == 1
        assert described["counters"]["frames"] == 1
    finally:
        source.close()
        real_close(fd)


def test_v4l2_negotiated_size_mismatch_is_rejected(monkeypatch):
    """驱动协商出的尺寸与请求（拼接尺寸）不符 → 报错，绝不按错误尺寸切分左右目。"""
    payload = _stitched_jpeg(64, 8)
    driver = FakeV4L2Driver(64, 8, payload, _frames(1), report_size=(128, 8))
    source, fd, real_close = _fake_v4l2_source(monkeypatch, driver, (64, 8))
    try:
        with pytest.raises(CameraTimestampError) as exc:
            source.next_frame()                  # 惰性 open 也要在这一步失败
        assert "128x8" in str(exc.value) and "64x8" in str(exc.value)
        assert source.describe()["opened"] is False, "协商失败不得留下半开的设备"
    finally:
        source.close()
        real_close(fd)


def test_v4l2_negotiated_guard_checks_size_and_pixel_format_directly(monkeypatch):
    """本类的复核必须自己判断尺寸与像素格式（不依赖底层类的检查）。

    底层 `_s_fmt()` 已经查过尺寸，但协商不符最终会以「解码失败」的形式在**解码阶段**
    迟到报错；像素格式不符（请求 MJPG 得到 YUYV）更是让整幅载荷根本不是图像。
    这里直接篡改协商结果，证明本类的复核确实在做判断，而不是形同虚设。
    """
    payload = _stitched_jpeg(64, 8)
    driver = FakeV4L2Driver(64, 8, payload, _frames(1))
    source, fd, real_close = _fake_v4l2_source(monkeypatch, driver, (64, 8))
    try:
        source.open()
        source._source.negotiated = {
            "width": 640, "height": 480,
            "pixelformat": "MJPG", "pixelformat_raw": ct.V4L2_PIX_FMT_MJPEG,
        }
        with pytest.raises(CameraTimestampError) as exc:
            source._verify_negotiated()
        assert "协商尺寸" in str(exc.value)

        source._source.negotiated = {
            "width": 64, "height": 8,
            "pixelformat": "YUYV", "pixelformat_raw": ct.V4L2_PIX_FMT_YUYV,
        }
        with pytest.raises(CameraTimestampError) as exc:
            source._verify_negotiated()
        assert "像素格式" in str(exc.value)
    finally:
        source.close()
        real_close(fd)


def test_v4l2_describe_reports_negotiated_format_and_counters(monkeypatch):
    """`describe()` 给出协商格式、观察到的时钟源与计数器，并说明没有曝光标定。"""
    payload = _stitched_jpeg(64, 8)
    driver = FakeV4L2Driver(64, 8, payload, _frames(1), report_size=(64, 8))
    source, fd, real_close = _fake_v4l2_source(monkeypatch, driver, (64, 8), fps=50)
    try:
        before = source.describe()
        assert before["opened"] is False and before["negotiated"] == {}
        assert before["clock_source"] is None

        source.next_frame()
        described = source.describe()
        assert described["kind"] == "v4l2" and described["replay"] is False
        assert described["negotiated"]["width"] == 64
        assert described["negotiated"]["height"] == 8
        assert described["negotiated"]["pixelformat"] == "MJPG"
        assert described["requested"]["pixelformat"] == "MJPG"
        assert described["clock_source"] == ct.CLOCK_MONOTONIC
        assert described["streaming"] is True
        assert described["counters"]["frames"] == 1
        assert described["frames_delivered"] == 1
        assert "曝光" in described["exposure_note"]
        source.close()
        assert source.describe()["closed"] is True and source.describe()["opened"] is False
    finally:
        real_close(fd)


def test_sources_require_explicit_clock_inputs():
    """时间基/周期不允许有「默认值」：缺了就报错，而不是假定 ROS 时域等于单调时域。

    回放源的 `timebase` 是必填关键字参数：**漏掉**它直接 `TypeError`（结构上不可能忘），
    显式传 `None` 则被参数校验拦下；采集源的 `timebase` 为兼容位置参数写法留了默认值，
    因此必须由参数校验报错。两条路径都不允许「默认偏移 0」这种错误假设成立。
    """
    fake_device = "/dev/never_opened_by_this_test"
    with pytest.raises(ValueError) as exc:
        V4L2FrameSource(fake_device, 64, 8)          # 参数校验在建 fd 之前就失败
    assert "timebase" in str(exc.value)
    with pytest.raises(ValueError) as exc:
        V4L2FrameSource(fake_device, 64, 8, timebase=None)
    assert "timebase" in str(exc.value)

    with pytest.raises(TypeError):
        ReplayFrameSource(STEREO_PNG, start_mono_s=START_MONO_S, period_s=PERIOD_S)
    with pytest.raises(ValueError) as exc:
        ReplayFrameSource(STEREO_PNG, start_mono_s=START_MONO_S, period_s=PERIOD_S,
                          timebase=None)
    assert "timebase" in str(exc.value)

    with pytest.raises(ValueError) as exc:
        ReplayFrameSource(STEREO_PNG, start_mono_s=START_MONO_S, period_s=0.0,
                          timebase=_timebase())
    assert "period_s" in str(exc.value)


# ================================================================== 两条路径同源


def test_both_sources_expose_the_same_decode_entry_point(monkeypatch):
    """两种源都提供 `decode(frame)` 与 `clock`（同一个 `StereoFrameClock`）。

    消费方（`stereo_source` 节点）按 `source.decode(frame)` → `source.clock.decode_stereo(frame)`
    的顺序取左右目。这里把两条入口都跑到，并断言与模块级 `decode_stitched` 结果**逐像素相同**
    ——两条路径的解码/切分确实只有一份实现。
    """
    file_bytes = STEREO_PNG.read_bytes()
    replay = ReplayFrameSource(
        STEREO_PNG, start_mono_s=START_MONO_S, period_s=PERIOD_S, timebase=_timebase()
    )
    driver = FakeV4L2Driver(FULL_W, FULL_H, file_bytes, _frames(1), buffer_count=2)
    capture, fd, real_close = _fake_v4l2_source(monkeypatch, driver, (FULL_W, FULL_H))
    try:
        reference = decode_stitched(file_bytes, FULL_W, FULL_H)
        for source in (replay, capture):
            frame = source.next_frame()
            for left, right in (source.decode(frame), source.clock.decode_stereo(frame)):
                assert np.array_equal(left, reference[0])
                assert np.array_equal(right, reference[1])
            assert source.decode(frame)[0].shape == (FULL_H, FULL_W // 2)
            # 协议要求的三个成员仍然齐全（decode/clock 只是同一条解码路径的入口）
            assert isinstance(source, FrameSource)
    finally:
        capture.close()
        replay.close()
        real_close(fd)


def test_both_sources_satisfy_the_same_protocol(monkeypatch):
    """两种实现都满足 `FrameSource`，节点里不需要按类型分支。"""
    replay = ReplayFrameSource(
        STEREO_PNG, start_mono_s=START_MONO_S, period_s=PERIOD_S, timebase=_timebase()
    )
    driver = FakeV4L2Driver(64, 8, _stitched_jpeg(64, 8), _frames(1))
    capture, fd, real_close = _fake_v4l2_source(monkeypatch, driver, (64, 8))
    try:
        assert isinstance(replay, FrameSource)
        assert isinstance(capture, FrameSource)
        for source in (replay, capture):
            frame = source.next_frame()
            assert isinstance(frame, StereoFrame)
            assert isinstance(source.describe(), dict)
            source.close()
    finally:
        real_close(fd)


def test_real_capture_and_replay_share_the_same_decode_path(monkeypatch):
    """「一个采集源」的验收：同一份真实录制帧分别经真实采集路径与回放路径发布，
    得到**逐字节相同**的载荷与**完全相同**的切分结果。

    这正是需求要的语义等价：两条路径的差别只在「帧从哪来」，之后的解码/切分
    （`decode_stitched` → `cv2.imdecode` → 按宽度对半切）只有一份实现。
    反过来：若有人给回放另写一套解码（例如重编码成 JPEG、或改成两幅图拼接），
    这里的逐字节相等就会失败。
    """
    file_bytes = STEREO_PNG.read_bytes()
    driver = FakeV4L2Driver(FULL_W, FULL_H, file_bytes, _frames(1, t0=1000.5),
                            buffer_count=2)
    capture, fd, real_close = _fake_v4l2_source(monkeypatch, driver, (FULL_W, FULL_H))
    replay = ReplayFrameSource(
        STEREO_PNG, start_mono_s=START_MONO_S, period_s=PERIOD_S, timebase=_timebase()
    )
    try:
        captured = capture.next_frame()
        replayed = replay.next_frame()

        assert captured.stitched == replayed.stitched == file_bytes
        for frame in (captured, replayed):
            left, right = decode_stitched(frame.stitched, FULL_W, FULL_H)
            assert np.array_equal(left, _gray(HALF_A_PNG))
            assert np.array_equal(right, _gray(HALF_B_PNG))
            assert frame.clock_source == ct.CLOCK_MONOTONIC
        # 时间戳当然不同（一个来自驱动、一个是合成占位值），但语义完全一致：
        # 一个帧对象、一个时间戳、一整幅载荷。
        assert captured.capture_ros_s != replayed.capture_ros_s
        assert set(dataclasses.fields(captured)) == set(dataclasses.fields(replayed))
    finally:
        capture.close()
        replay.close()
        real_close(fd)


def test_module_has_exactly_one_capture_path():
    """结构性核对：本模块只有一处真实采集源构造点，且不存在第二条相机采集路径。

    为什么：需求是「相机只能在一个地方打开」。`depth_preview.py` 的
    `cv2.VideoCapture` 与历史文件回放各有一套取帧、时间戳语义不同，无法互相替代。
    这条断言钉住「真实采集 = 唯一的构造点 = `V4L2FrameSource`」，回放则只读文件。
    """
    text = inspect.getsource(sc)
    assert "VideoCapture" not in text, "不允许出现第二条相机采集路径"
    assert text.count("CameraTimestampSource(") == 1, "只允许一处真实采集源构造点"
    assert {"FrameSource", "V4L2FrameSource", "ReplayFrameSource"} <= set(sc.__all__)
