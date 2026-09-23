"""相机采集时间戳接口的脱机测试：ABI 布局、模拟 ioctl 取帧、录制帧回放、格式拒绝、时域规则。

本文件**不打开任何设备**：不开相机、不 mmap、不 STREAMON、不连飞控。V4L2 路径用真 pipe fd
加上打桩的 `fcntl.ioctl` 驱动；录制帧路径只读磁盘上的 PNG 字节。

结论的边界（必读）
------------------
`recordings/` 里的三张 PNG 是**录下来的真实帧**（Windows 资料目录快照），来源：

    data/stereo_depth/windows_snapshot_20260922/captures/20260911_200503_356811/
        stereo.png（480×1280 整幅拼接帧）、half_a.png / half_b.png（各 480×640）

该目录的 `capture.json` 自己写明 `"timestamp": "host save time, not exposure time"`
（另一批目录写作 `timestamp_note`：目录时间是主机保存时间，**不是传感器曝光时间**）。
因此这些帧**不携带曝光时间戳**，回放测试里出现的单调时间戳是测试自己塞进去的**占位值**：

  - 回放测试证明的只是「一整幅拼接帧 → 整幅解码 → 按宽度对半切」这套**格式/切分**逻辑
    与现有采集链（`stereo_depth/depth_preview.py`）一致，且左右两半来自同一次取帧、
    共享同一个 `capture_ros_s`；
  - 它**不证明曝光时刻**：曝光时刻到驱动时间戳之间的偏差仍属待硬件标定项
    （见模块 docstring 与 README 的待验收清单），本文件任何结论都不能当作真机时间戳验收。

为什么 ABI 必须编译 C 头文件
----------------------------
`struct v4l2_buffer` 的字段偏移曾经写错过，症状是「时间戳读成别的字节、bytesused 读成
length，但代码照常返回一个看似有效的时间戳」。这类错误在纯 Python 侧无法自证，只能拿
`<linux/videodev2.h>` 的编译期真值（`sizeof` / `offsetof`）逐个字段核对，差一个字节即失败。
"""

from __future__ import annotations

import dataclasses
import fcntl
import importlib.util
import inspect
import os
import pathlib
import re
import shutil
import struct
import subprocess
import time

import cv2
import numpy as np
import pytest

from boom_birds_nav import camera_timestamp as ct
from boom_birds_nav.camera_timestamp import (
    CLOCK_MONOTONIC,
    CLOCK_REALTIME,
    CLOCK_UNKNOWN,
    V4L2_BUF_FLAG_TIMESTAMP_COPY,
    V4L2_BUF_FLAG_TIMESTAMP_MASK,
    V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC,
    V4L2_BUF_FLAG_TIMESTAMP_UNKNOWN,
    V4L2_BUF_TYPE_VIDEO_CAPTURE,
    V4L2_BUFFER_SIZE,
    V4L2_MEMORY_MMAP,
    VIDIOC_DQBUF,
    VIDIOC_QBUF,
    CameraTimestampError,
    CameraTimestampSource,
    RawFrame,
    StereoFrame,
    StereoFrameClock,
    decode_stitched,
    probe_v4l2,
    realtime_to_monotonic_offset_s,
    timestamp_source_from_flags,
)
from boom_birds_nav.timebase import RosTimeBase, TimeBaseSample

# 夹具目录：本文件同级（pathlib 解析，测试从任何工作目录启动都能找到）。
RECORDINGS = pathlib.Path(__file__).resolve().parent / "recordings"
# 录制帧的原始目录名（写进断言的失败信息里，便于回溯到资料目录）。
RECORDED_CAPTURE = "20260911_200503_356811"
STITCH_WIDTH, STITCH_HEIGHT = 1280, 480
HALF_WIDTH = STITCH_WIDTH // 2
ROS_OFFSET_S = 1_700_000_000.0   # 固定「ROS − MONO」偏移，离线回放用的时间基
# 录制回放用的**占位**单调时间戳：录制帧没有曝光时间戳，这个值没有任何曝光含义。
SYNTHETIC_MONO_S = 1000.5
# MJPEG 是有损的：q=100 时本机实测整幅往返最大偏差 1 灰阶（19914/614400 像素受影响），
# 任何质量下都不为零。留 1 灰阶余量给不同 OpenCV/libjpeg 构建；作为对照，切错一列的
# 最大偏差是 142 灰阶、左右互换是 195 灰阶，所以这个界限仍能决定性地抓住切分错误。
JPEG_ROUNDTRIP_MARGIN = 2


# ------------------------------------------------------------------ 夹具与小工具


def _fixture_bytes(name: str) -> bytes:
    """读录制帧夹具原始字节；缺失即失败（不静默跳过：夹具是本文件的主要证据）。"""
    path = RECORDINGS / name
    if not path.is_file():
        pytest.fail(
            f"缺少录制帧夹具 {path}；应从 "
            f"data/stereo_depth/windows_snapshot_20260922/captures/{RECORDED_CAPTURE}/ 复制"
        )
    return path.read_bytes()


def _fixture_gray(name: str) -> np.ndarray:
    image = cv2.imdecode(np.frombuffer(_fixture_bytes(name), np.uint8), cv2.IMREAD_GRAYSCALE)
    assert image is not None, f"{name} 无法解码为灰度图"
    return image


def _max_abs_diff(a: np.ndarray, b: np.ndarray) -> int:
    """两幅灰度图的最大绝对差（转 int16，避免 uint8 相减回绕）。"""
    return int(np.abs(a.astype(np.int16) - b.astype(np.int16)).max())


def _flat_stitched_jpeg(width: int, height: int, left: int, right: int) -> bytes:
    """构造左右已知（左=left、右=right）的平场拼接 JPEG，模拟相机输出的整幅 MJPEG。

    刻意用平场：8×8 块内只有 DC 分量，量化步长为 1 时 JPEG 往返**逐像素无损**
    （本机实测 q=100/95/90/75 偏差均为 0），所以这些用例的期望值可以写死并精确断言。
    """
    stitched = np.hstack(
        [
            np.full((height, width // 2), left, dtype=np.uint8),
            np.full((height, width - width // 2), right, dtype=np.uint8),
        ]
    )
    ok, buf = cv2.imencode(".jpg", stitched, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
    assert ok, "测试自身的 JPEG 编码失败"
    return buf.tobytes()


def _mjpeg_payload_from_recorded_stereo() -> bytes:
    """把录制帧 stereo.png 编码成 MJPEG 载荷（快照只留了 PNG，没有原始 MJPG 字节）。

    这只是「形状与真实采集一致的整幅拼接 JPEG」，不是相机原始输出；
    它的用途是验证切分逻辑，不涉及任何时间信息。
    """
    ok, buf = cv2.imencode(
        ".jpg", _fixture_gray("stereo.png"), [int(cv2.IMWRITE_JPEG_QUALITY), 100]
    )
    assert ok, "编码录制帧失败"
    return buf.tobytes()


class RecordingSource(CameraTimestampSource):
    """回放源：把给定载荷当成一次 `read_raw()` 的结果（不开设备、不做 ioctl）。

    只替换底层取帧；`to_monotonic()` 等时域逻辑仍是模块真实现。`timestamp_s` 是
    **合成占位值**（录制帧没有曝光时间戳），`reads` 用来断言「一帧只取一次」。
    """

    def __init__(
        self,
        payloads,
        timestamps=None,
        size=(STITCH_WIDTH, STITCH_HEIGHT),
        clock_source: str = CLOCK_MONOTONIC,
    ) -> None:
        self.device = "/dev/recorded"
        self.allow_realtime = False
        self.realtime_uncertainty_limit_s = 0.002
        self.negotiated = {"width": size[0], "height": size[1]}
        self._payloads = list(payloads)
        self._timestamps = (
            list(timestamps)
            if timestamps is not None
            else [SYNTHETIC_MONO_S + 0.0167 * i for i in range(len(self._payloads))]
        )
        self._clock_source = clock_source
        self.reads = 0

    def read_raw(self, timeout_s: float = 1.0) -> RawFrame:
        if self.reads >= len(self._payloads):
            raise CameraTimestampError("回放帧已用尽")
        payload = self._payloads[self.reads]
        timestamp_s = self._timestamps[self.reads]
        self.reads += 1
        return RawFrame(
            sequence=self.reads,
            data=payload,
            timestamp_s=timestamp_s,
            clock_source=self._clock_source,
            flags=V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC,
            received_mono_s=time.monotonic(),
            driver_sequence=1000 + self.reads,   # 驱动自己的帧计数，与模块计数分开
            buffer_index=0,
            bytes_used=len(payload),
        )


class DecreasingOffsetTimebase:
    """时间基桩：单调时钟正常前进，但每次采样给出的 ROS 偏移在下降。

    用于验证 `StereoFrameClock` 的第二道单调性检查（映射到 ROS 域之后），
    真实场景对应「ROS 时钟被 NTP 步进」。
    """

    def __init__(self, offsets) -> None:
        self._offsets = list(offsets)
        self._i = 0

    def sample(self) -> TimeBaseSample:
        offset = self._offsets[min(self._i, len(self._offsets) - 1)]
        self._i += 1
        return TimeBaseSample(
            mono_s=0.0, ros_s=offset, offset_s=offset, uncertainty_s=0.0, bracket_s=0.0
        )


# ================================================================== 1. ABI 布局

# 打印 sizeof 与全部字段偏移；字段名与模块常量一一对应。
_ABI_C_SOURCE = r"""
#include <linux/videodev2.h>
#include <stddef.h>
#include <stdio.h>

int main(void) {
    printf("sizeof=%zu\n", sizeof(struct v4l2_buffer));
    printf("index=%zu\n", offsetof(struct v4l2_buffer, index));
    printf("type=%zu\n", offsetof(struct v4l2_buffer, type));
    printf("bytesused=%zu\n", offsetof(struct v4l2_buffer, bytesused));
    printf("flags=%zu\n", offsetof(struct v4l2_buffer, flags));
    printf("field=%zu\n", offsetof(struct v4l2_buffer, field));
    printf("timestamp=%zu\n", offsetof(struct v4l2_buffer, timestamp));
    printf("timestamp.tv_sec=%zu\n", offsetof(struct v4l2_buffer, timestamp.tv_sec));
    printf("timestamp.tv_usec=%zu\n", offsetof(struct v4l2_buffer, timestamp.tv_usec));
    printf("sequence=%zu\n", offsetof(struct v4l2_buffer, sequence));
    printf("memory=%zu\n", offsetof(struct v4l2_buffer, memory));
    printf("length=%zu\n", offsetof(struct v4l2_buffer, length));
    printf("request_fd=%zu\n", offsetof(struct v4l2_buffer, request_fd));
    return 0;
}
"""

# C 头文件里的字段 → 模块常量。多一个少一个都会被覆盖性断言抓住。
_ABI_FIELD_TO_CONSTANT = {
    "sizeof": ct.V4L2_BUFFER_SIZE,
    "index": ct.V4L2_BUFFER_OFF_INDEX,
    "type": ct.V4L2_BUFFER_OFF_TYPE,
    "bytesused": ct.V4L2_BUFFER_OFF_BYTESUSED,
    "flags": ct.V4L2_BUFFER_OFF_FLAGS,
    "field": ct.V4L2_BUFFER_OFF_FIELD,
    "timestamp": ct.V4L2_BUFFER_OFF_TIMESTAMP_SEC,
    "timestamp.tv_sec": ct.V4L2_BUFFER_OFF_TIMESTAMP_SEC,
    "timestamp.tv_usec": ct.V4L2_BUFFER_OFF_TIMESTAMP_USEC,
    "sequence": ct.V4L2_BUFFER_OFF_SEQUENCE,
    "memory": ct.V4L2_BUFFER_OFF_MEMORY,
    "length": ct.V4L2_BUFFER_OFF_LENGTH,
    "request_fd": ct.V4L2_BUFFER_OFF_REQUEST_FD,
}


def test_v4l2_buffer_layout_matches_c_header(tmp_path):
    """模块里的缓冲区偏移常量必须等于 `<linux/videodev2.h>` 的编译期真值。

    为什么不做「等价写法」的宽容断言：偏移错一个字节就能得到「看似有效」的时间戳
    （这正是上一次的缺陷形态），所以这里逐字段要求**完全相等**，并把两边的数值都写进
    失败信息。偏移再次漂移时本用例必然失败。
    """
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("未找到 gcc：无法用 <linux/videodev2.h> 的 offsetof() 核对 ABI 布局")
    header = pathlib.Path("/usr/include/linux/videodev2.h")
    if not header.is_file():
        pytest.skip("缺少内核头文件 /usr/include/linux/videodev2.h（需 linux-libc-dev）")

    source = tmp_path / "v4l2_buffer_abi.c"
    binary = tmp_path / "v4l2_buffer_abi"
    source.write_text(_ABI_C_SOURCE, encoding="utf-8")
    compiled = subprocess.run(
        [gcc, "-std=gnu11", "-O0", "-o", str(binary), str(source)],
        capture_output=True,
        text=True,
    )
    # 头文件存在却编译不过：属于真实的 ABI/结构体变化，必须失败而不是跳过。
    assert compiled.returncode == 0, f"ABI 探针编译失败：\n{compiled.stderr}"
    run = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run.returncode == 0, f"ABI 探针运行失败：{run.stderr}"

    truth = {}
    for line in run.stdout.splitlines():
        key, sep, value = line.partition("=")
        if sep:
            truth[key.strip()] = int(value)
    assert set(truth) == set(_ABI_FIELD_TO_CONSTANT), (
        f"C 探针输出的字段集合 {sorted(truth)} 与待核对集合 "
        f"{sorted(_ABI_FIELD_TO_CONSTANT)} 不一致（新增字段必须一并核对）"
    )

    mismatched = {
        key: (expected, truth[key])
        for key, expected in _ABI_FIELD_TO_CONSTANT.items()
        if truth[key] != expected
    }
    assert not mismatched, "v4l2_buffer 布局与 C 头文件不一致（模块常量已漂移）：" + "；".join(
        f"{key}: 模块常量={expected}，C 头文件={actual}"
        for key, (expected, actual) in mismatched.items()
    )

    # timeval 起始处就是 tv_sec，模块没有单独的「时间戳结构体偏移」常量，这里显式钉住。
    assert truth["timestamp"] == truth["timestamp.tv_sec"] == ct.V4L2_BUFFER_OFF_TIMESTAMP_SEC
    assert truth["timestamp.tv_usec"] - truth["timestamp.tv_sec"] == ct.TIMEVAL_SEC_SIZE


def test_check_abi_accepts_this_64bit_platform():
    """本机是 64 位 Linux，`check_abi()` 不得报错（88 字节布局的前提）。"""
    assert struct.calcsize("P") == 8, "本用例的布局假设只对 64 位 Linux 成立"
    CameraTimestampSource.check_abi()


def test_check_abi_rejects_non_64bit_layout(monkeypatch):
    """非 64 位必须显式报错：按 88 字节错读会得到「看似有效」的错时间戳。

    `struct` 是模块级共享对象，这里打桩的窗口只覆盖紧随其后的一次调用。
    """
    monkeypatch.setattr(ct.struct, "calcsize", lambda _fmt: 4)
    with pytest.raises(CameraTimestampError):
        CameraTimestampSource.check_abi()

# ================================================================== 2. 缓冲构造与解析


def test_buf_pack_puts_fields_at_documented_offsets():
    """`_buf_pack(index)` 是 QUERYBUF/QBUF 入参的唯一构造入口。

    入参只允许带 index/type/memory 三个字段，其余必须保持**零**：这两个 ioctl 的入参
    若带脏值，驱动可能直接拒绝请求（表现为「明明有理却取不到帧」）。
    """
    source = CameraTimestampSource("/dev/never", STITCH_WIDTH, STITCH_HEIGHT)
    packed = source._buf_pack(3)
    assert len(packed) == V4L2_BUFFER_SIZE
    expected = bytearray(V4L2_BUFFER_SIZE)
    struct.pack_into("I", expected, ct.V4L2_BUFFER_OFF_INDEX, 3)
    struct.pack_into("I", expected, ct.V4L2_BUFFER_OFF_TYPE, V4L2_BUF_TYPE_VIDEO_CAPTURE)
    struct.pack_into("I", expected, ct.V4L2_BUFFER_OFF_MEMORY, V4L2_MEMORY_MMAP)
    assert bytes(packed) == bytes(expected)


def test_parse_buffer_reads_every_field_at_its_documented_offset():
    """`parse_buffer()` 是唯一的解析入口：每个字段写在常量偏移上，必须逐字段对上。

    每个字段给一个互不相同的哨兵值，这样「读错偏移」不可能碰巧对上。
    """
    raw = bytearray(b"\xab" * V4L2_BUFFER_SIZE)          # 未写字段的毒值
    struct.pack_into("I", raw, ct.V4L2_BUFFER_OFF_INDEX, 3)
    struct.pack_into("I", raw, ct.V4L2_BUFFER_OFF_TYPE, V4L2_BUF_TYPE_VIDEO_CAPTURE)
    struct.pack_into("I", raw, ct.V4L2_BUFFER_OFF_BYTESUSED, 1234)
    struct.pack_into("I", raw, ct.V4L2_BUFFER_OFF_FLAGS, V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)
    struct.pack_into("I", raw, ct.V4L2_BUFFER_OFF_FIELD, 1)
    struct.pack_into("q", raw, ct.V4L2_BUFFER_OFF_TIMESTAMP_SEC, 4321)
    struct.pack_into("q", raw, ct.V4L2_BUFFER_OFF_TIMESTAMP_USEC, 250000)
    struct.pack_into("I", raw, ct.V4L2_BUFFER_OFF_SEQUENCE, 77)
    struct.pack_into("I", raw, ct.V4L2_BUFFER_OFF_MEMORY, V4L2_MEMORY_MMAP)
    struct.pack_into("I", raw, ct.V4L2_BUFFER_OFF_LENGTH, 4096)
    struct.pack_into("i", raw, ct.V4L2_BUFFER_OFF_REQUEST_FD, -1)

    parsed = CameraTimestampSource.parse_buffer(raw)
    assert parsed["index"] == 3
    assert parsed["bytesused"] == 1234
    assert parsed["flags"] == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC
    assert parsed["sequence"] == 77
    assert parsed["length"] == 4096
    assert parsed["timestamp_s"] == pytest.approx(4321.25, abs=1e-9)
    assert parsed["clock_source"] == CLOCK_MONOTONIC


def test_parse_buffer_does_not_take_timestamp_from_a_wrong_offset():
    """反面用例：时间戳只认自己的偏移，写在别处（如旧的错误偏移）不得被当成时间戳。"""
    raw = bytearray(b"\xab" * V4L2_BUFFER_SIZE)
    struct.pack_into("q", raw, ct.V4L2_BUFFER_OFF_FIELD, 4321)       # 偏移 16：不是时间戳
    struct.pack_into("q", raw, ct.V4L2_BUFFER_OFF_SEQUENCE, 4321)    # 偏移 56：也不是
    parsed = CameraTimestampSource.parse_buffer(raw)
    assert parsed["timestamp_s"] != pytest.approx(4321.0)
    assert abs(parsed["timestamp_s"]) > 1e6, "毒值应解出明显的垃圾时间戳，而不是 4321"


def test_parse_buffer_rejects_short_buffer():
    """长度不足 88 字节即报错：不能按越界偏移读，也不返回半截默认值。"""
    with pytest.raises(CameraTimestampError) as exc:
        CameraTimestampSource.parse_buffer(bytes(V4L2_BUFFER_SIZE - 1))
    assert str(V4L2_BUFFER_SIZE) in str(exc.value)


# ================================================================== 3. 模拟 ioctl 取帧


@pytest.fixture
def fake_v4l2(monkeypatch):
    """假设备：真 pipe fd（供 `select()` 判定可读）+ 打桩的 `fcntl.ioctl`。

    为什么从系统调用入手：模块的取帧路径没有「注入取帧实现」的接缝
    （`read_raw()` 直接调 `select` + `fcntl.ioctl`，只有一个纯函数接缝
    `parse_buffer()`），所以只能在最外层替换 ioctl。返回的 `calls` 记录实际发生的
    DQBUF/QBUF，用来断言「取一帧、还一帧」。

    DQBUF 的入参缓冲先被填满 0xAB 毒值再写真实字段：任何字段读错偏移都会拿到
    0xABAB… 这样的垃圾值，断言随即失败——这正是偏移曾经写错时的症状。
    """

    created = []

    def _make(payloads, *, index=1, bytesused=3000, length=4096, sequence=77,
              flags=V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC, sec=4321, usec=250000):
        read_fd, write_fd = os.pipe()               # pipe 只用来让 select 报告「可读」
        created.append((read_fd, write_fd))
        os.write(write_fd, b"\x00")
        source = CameraTimestampSource("/dev/fakevideo0", STITCH_WIDTH, STITCH_HEIGHT)
        source._fd = read_fd
        source._buffers = list(payloads)
        calls = {"dqbuf": 0, "qbuf_indices": []}

        def fake_ioctl(fd, request, arg=0, mutate_flag=True):
            if request == VIDIOC_DQBUF:
                calls["dqbuf"] += 1
                arg[:] = b"\xab" * V4L2_BUFFER_SIZE
                struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_INDEX, index)
                struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_TYPE, V4L2_BUF_TYPE_VIDEO_CAPTURE)
                struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_BYTESUSED, bytesused)
                struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_FLAGS, flags)
                struct.pack_into("q", arg, ct.V4L2_BUFFER_OFF_TIMESTAMP_SEC, sec)
                struct.pack_into("q", arg, ct.V4L2_BUFFER_OFF_TIMESTAMP_USEC, usec)
                struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_SEQUENCE, sequence)
                struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_MEMORY, V4L2_MEMORY_MMAP)
                struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_LENGTH, length)
                struct.pack_into("i", arg, ct.V4L2_BUFFER_OFF_REQUEST_FD, -1)
            elif request == VIDIOC_QBUF:
                calls["qbuf_indices"].append(
                    struct.unpack_from("I", arg, ct.V4L2_BUFFER_OFF_INDEX)[0]
                )
            return 0

        monkeypatch.setattr(fcntl, "ioctl", fake_ioctl)
        return source, calls

    yield _make
    for read_fd, write_fd in created:
        os.close(read_fd)
        os.close(write_fd)


def test_read_raw_reads_dqbuf_fields_from_their_real_layout(fake_v4l2):
    """模拟一次 DQBUF：四个返回值必须分别来自 index / bytesused / flags / sequence。

    断言刻意互相区分：驱动侧 index=1（而入参 `_buf_pack(0)` 写的是 0）、bytesused=3000
    而 length=4096、时间戳 4321.25 而不是毒值。任何一项读错偏移，下面的断言都会失败。
    """
    payload_a = b"\x11" * 4096
    payload_b = b"\x22" * 4096
    source, calls = fake_v4l2([payload_a, payload_b])

    raw = source.read_raw(timeout_s=0.5)

    # 时间戳来自 timestamp.tv_sec@24 / tv_usec@32（不是别的字节，也不是取帧时刻）
    assert raw.timestamp_s == pytest.approx(4321.25, abs=1e-9)
    assert raw.clock_source == CLOCK_MONOTONIC
    assert raw.flags == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC
    # 有效字节数来自 bytesused，不是映射长度 length
    assert raw.bytes_used == 3000
    assert len(raw.data) == 3000
    assert raw.data == payload_b[:3000], "载荷必须取自 index 指到的那个缓冲"
    # 缓冲下标与驱动帧计数分别来自 index / sequence
    assert raw.buffer_index == 1
    assert raw.driver_sequence == 77
    assert raw.sequence == 1, "模块自己的帧计数独立于驱动计数"
    assert raw.received_mono_s > 0.0
    # 取一帧必须还一帧，且还的是同一个下标（否则流会停）
    assert calls["dqbuf"] == 1
    assert calls["qbuf_indices"] == [1]
    # 时域换算仍是真实现：MONOTONIC 直接用
    mono_s, uncertainty_s = source.to_monotonic(raw)
    assert mono_s == pytest.approx(4321.25, abs=1e-9)
    assert uncertainty_s == 0.0


def test_read_raw_falls_back_to_length_when_bytesused_is_zero(fake_v4l2):
    """bytesused=0 时退回映射长度 length（模块的既定行为，写在注释里，这里钉住）。"""
    source, _ = fake_v4l2([b"\x33" * 4096], index=0, bytesused=0, length=2048)
    raw = source.read_raw(timeout_s=0.5)
    assert raw.bytes_used == 2048
    assert len(raw.data) == 2048


def test_read_raw_rejects_buffer_index_out_of_mapped_range(fake_v4l2):
    """驱动给出的 index 超出自认为映射的范围时必须报错，不能越界取缓冲。"""
    source, _ = fake_v4l2([b"\x44" * 4096], index=5)
    with pytest.raises(CameraTimestampError) as exc:
        source.read_raw(timeout_s=0.5)
    assert "缓冲索引" in str(exc.value)


# ================================================================== 4. 时域判定（沿用旧用例）


def test_timestamp_source_from_v4l2_flags():
    assert timestamp_source_from_flags(V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) == CLOCK_MONOTONIC
    assert timestamp_source_from_flags(0x00001000) == CLOCK_UNKNOWN
    assert timestamp_source_from_flags(V4L2_BUF_FLAG_TIMESTAMP_COPY) == CLOCK_UNKNOWN
    assert timestamp_source_from_flags(V4L2_BUF_FLAG_TIMESTAMP_UNKNOWN) == CLOCK_UNKNOWN
    assert timestamp_source_from_flags(0) == CLOCK_UNKNOWN
    # 同时置位其它标志位不影响时域判定
    assert timestamp_source_from_flags(
        V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC | 0x00000004
    ) == CLOCK_MONOTONIC
    assert V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC & V4L2_BUF_FLAG_TIMESTAMP_MASK


def test_monotonic_timestamp_used_as_is():
    """MONOTONIC 与 `time.monotonic()` 同域：直接使用，不引入任何偏移。"""
    source = CameraTimestampSource("/dev/never", 64, 24)
    frame = RawFrame(1, b"x", 12345.678, CLOCK_MONOTONIC, 0, 0.0)
    mono_s, uncertainty_s = source.to_monotonic(frame)
    assert mono_s == pytest.approx(12345.678)
    assert uncertainty_s == 0.0


def test_monotonic_timestamp_must_be_positive():
    """非正的 MONOTONIC 只可能是没打上戳：不能当成有效时间发出去。"""
    source = CameraTimestampSource("/dev/never", 64, 24)
    with pytest.raises(CameraTimestampError):
        source.to_monotonic(RawFrame(1, b"x", 0.0, CLOCK_MONOTONIC, 0, 0.0))


def test_unknown_domain_is_rejected():
    source = CameraTimestampSource("/dev/never", 64, 24)
    with pytest.raises(CameraTimestampError) as exc:
        source.to_monotonic(RawFrame(1, b"x", 1.0, CLOCK_UNKNOWN, 0x1000, 0.0))
    assert "不可核实" in str(exc.value)


def test_realtime_requires_explicit_opt_in():
    """REALTIME 会随 NTP 步进而变：默认拒绝，显式允许后才换算。"""
    strict = CameraTimestampSource("/dev/never", 64, 24)
    frame = RawFrame(1, b"x", time.time(), CLOCK_REALTIME, 0, 0.0)
    with pytest.raises(CameraTimestampError):
        strict.to_monotonic(frame)

    lenient = CameraTimestampSource(
        "/dev/never", 64, 24, allow_realtime=True, realtime_uncertainty_limit_s=0.05
    )
    mono_s, uncertainty_s = lenient.to_monotonic(frame)
    assert abs(mono_s - time.monotonic()) < 0.05
    assert uncertainty_s >= 0.0


def test_realtime_offset_estimate_sanity():
    """REALTIME→MONOTONIC 偏移只做量级自检：单调时钟原点早于 epoch，偏移应为负。"""
    offset, uncertainty = realtime_to_monotonic_offset_s()
    assert offset < 0
    assert 0.0 <= uncertainty < 0.05


# ================================================================== 5. 同帧左右共享时间戳


def test_driver_domain_backwards_frames_are_rejected():
    """驱动时间戳未单调递增即报错，且不得计入已发布帧数。"""
    payload = _flat_stitched_jpeg(64, 8, left=10, right=200)
    source = RecordingSource([payload, payload], timestamps=[1000.5, 1000.4], size=(64, 8))
    clock = StereoFrameClock(source, RosTimeBase.from_offset(ROS_OFFSET_S, SYNTHETIC_MONO_S))
    clock.next_frame()
    with pytest.raises(CameraTimestampError):
        clock.next_frame()
    assert clock.counters["backwards"] == 1
    assert clock.counters["frames"] == 1


def test_ros_domain_backwards_frames_are_rejected():
    """映射到 ROS 域后不递增也必须报错（对应 ROS 时钟被 NTP 步进）。"""
    payload = _flat_stitched_jpeg(64, 8, left=10, right=200)
    source = RecordingSource([payload, payload], timestamps=[1000.5, 1000.6], size=(64, 8))
    clock = StereoFrameClock(source, DecreasingOffsetTimebase([ROS_OFFSET_S, ROS_OFFSET_S - 1.0]))
    clock.next_frame()
    with pytest.raises(CameraTimestampError) as exc:
        clock.next_frame()
    assert "ROS 时间域" in str(exc.value)
    assert clock.counters["backwards"] == 1
    assert clock.counters["frames"] == 1


def test_untraceable_domain_blocks_real_link():
    """驱动时域不可核实时必须报错，不得回退到收帧时刻，也不得发布任何帧。"""
    payload = _flat_stitched_jpeg(64, 8, left=10, right=200)
    source = RecordingSource(
        [payload],
        timestamps=[time.time()],
        size=(64, 8),
        clock_source=CLOCK_REALTIME,     # 未显式允许 → 必须拒绝
    )
    clock = StereoFrameClock(source, RosTimeBase.from_offset(ROS_OFFSET_S, SYNTHETIC_MONO_S))
    with pytest.raises(CameraTimestampError):
        clock.next_frame()
    assert clock.counters["frames"] == 0


def test_ros_domain_timestamps_increase_across_frames():
    """连续帧的 ROS 时间戳必须严格递增，且保持驱动时间戳的间隔。"""
    payload = _flat_stitched_jpeg(64, 8, left=10, right=200)
    timestamps = [1000.5, 1000.5167, 1000.5333]
    source = RecordingSource([payload] * 3, timestamps=timestamps, size=(64, 8))
    clock = StereoFrameClock(source, RosTimeBase.from_offset(ROS_OFFSET_S, SYNTHETIC_MONO_S))
    stamps = [clock.next_frame().capture_ros_s for _ in range(3)]
    assert stamps == sorted(stamps)
    # 间隔必须与驱动时间戳的间隔一致。期望值直接取输入差值（第三个间隔是 16.6 ms，
    # 不能假定等间隔）；映射到 1.7e9 的 ROS 域后 double 在该量级的分辨率约 2.4e-7 s，
    # 所以余量取 1e-6，而不是要求逐位相等。
    for index in range(1, len(timestamps)):
        expected = timestamps[index] - timestamps[index - 1]
        assert stamps[index] - stamps[index - 1] == pytest.approx(expected, abs=1e-6)
    assert clock.counters["frames"] == 3


def test_next_frame_defers_decoding_to_decode_stereo():
    """取帧与解码解耦：坏载荷在 `next_frame()` 不报错，只在 `decode_stereo()` 报错。

    为什么这样定：`next_frame()` 不做无谓解码（大帧解码开销可观），
    因此「帧内容坏了」属于解码阶段的问题；但坏帧绝不能悄悄产出一幅图。
    """
    source = RecordingSource([b"\x00\x01\x02\x03" * 8], size=(64, 8))
    clock = StereoFrameClock(source, RosTimeBase.from_offset(ROS_OFFSET_S, SYNTHETIC_MONO_S))
    frame = clock.next_frame()
    assert frame.stitched == b"\x00\x01\x02\x03" * 8, "取帧不做解码，字节原样保留"
    with pytest.raises(CameraTimestampError):
        clock.decode_stereo(frame)


def test_probe_missing_device_reports_error(tmp_path):
    """探测不存在的设备：报告 exists=False 且带 error，不抛异常、不尝试打开。"""
    report = probe_v4l2(str(tmp_path / "nope"))
    assert report["exists"] is False
    assert "error" in report

# ================================================================== 6. 录制帧回放


def test_recorded_halves_are_exact_halves_of_recorded_stereo():
    """夹具自证：half_a/half_b 确实是 stereo.png 的左/右半，逐像素完全一致。

    这是后面所有切分断言的**真值来源**——没有这一条，「切得对」就没有可比的基准。
    PNG 无损，所以这里要求完全相等（本机实测成立）。左右还必须彼此不同，
    否则「切错列」或「左右互换」都看不出来。
    """
    stitched = _fixture_gray("stereo.png")
    half_a, half_b = _fixture_gray("half_a.png"), _fixture_gray("half_b.png")
    assert stitched.shape == (STITCH_HEIGHT, STITCH_WIDTH)
    assert half_a.shape == half_b.shape == (STITCH_HEIGHT, HALF_WIDTH)
    assert np.array_equal(stitched[:, :HALF_WIDTH], half_a)
    assert np.array_equal(stitched[:, HALF_WIDTH:], half_b)
    assert not np.array_equal(half_a, half_b)


def test_decode_stitched_splits_recorded_frame_exactly():
    """整幅解码 + 按宽度对半切：与录制帧的左右半**逐像素完全一致**。

    喂进去的是 stereo.png 的原始字节（`cv2.imdecode` 同样吃 PNG），所以这条断言不掺
    任何编解码误差，钉住的就是「切分列 = 宽度的一半」这一件事。
    注意：录制帧没有曝光时间戳，本用例只谈图像切分，不涉及任何时间信息。
    """
    left, right = decode_stitched(_fixture_bytes("stereo.png"), STITCH_WIDTH, STITCH_HEIGHT)
    assert left.dtype == np.uint8 and right.dtype == np.uint8
    assert np.array_equal(left, _fixture_gray("half_a.png"))
    assert np.array_equal(right, _fixture_gray("half_b.png"))


def test_decode_stitched_matches_recorded_halves_through_mjpeg():
    """MJPEG 路径：整幅 JPEG 解出的左右半必须与录制帧的左右半一致到编解码误差内。

    为什么这里不能要求「逐像素完全相等」：MJPEG 是有损的。本机实测 q=100 整幅往返的
    最大偏差是 1 灰阶（19914/614400 像素受影响），任何质量下都不为零——经 JPEG 之后
    「逐像素等于 PNG」在数学上不可能成立。作为对照，切错一列是 142 灰阶、左右互换是
    195 灰阶，所以 `<= 2` 的界限仍能决定性地抓住切分错误（真正的无损断言用 PNG 载荷，
    见上一条用例）。
    """
    payload = _mjpeg_payload_from_recorded_stereo()
    left, right = decode_stitched(payload, STITCH_WIDTH, STITCH_HEIGHT)
    half_a, half_b = _fixture_gray("half_a.png"), _fixture_gray("half_b.png")
    assert left.shape == right.shape == (STITCH_HEIGHT, HALF_WIDTH)
    assert _max_abs_diff(left, half_a) <= JPEG_ROUNDTRIP_MARGIN
    assert _max_abs_diff(right, half_b) <= JPEG_ROUNDTRIP_MARGIN

    # 同一载荷独立解码后按宽度切分，必须与 decode_stitched 的结果**完全一致**：
    # 说明那点差值只来自 JPEG 本身，不来自本模块的切分。
    decoded = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_GRAYSCALE)
    assert np.array_equal(left, decoded[:, :HALF_WIDTH])
    assert np.array_equal(right, decoded[:, HALF_WIDTH:])
    # 灵敏度自证：上面那个容差不会放过真实的切分错位
    assert _max_abs_diff(decoded[:, 1:HALF_WIDTH + 1], half_a) > 50


def test_recorded_frame_replay_shares_one_capture_timestamp():
    """一次 `read_raw()` → 一个 `capture_ros_s`，左右两半由它共享。

    注意：录制帧**没有曝光时间戳**，这里的时间戳是测试塞进去的合成占位值。
    本用例只证明「同一帧只映射一次时间、左右不各打各的戳」，不代表真实曝光时刻。
    """
    payload = _mjpeg_payload_from_recorded_stereo()
    source = RecordingSource([payload])
    clock = StereoFrameClock(
        source,
        RosTimeBase.from_offset(ROS_OFFSET_S, SYNTHETIC_MONO_S),
        stitched_width=STITCH_WIDTH,
    )
    frame = clock.next_frame()

    assert source.reads == 1, "一个双目帧只允许一次底层取帧"
    assert frame.sequence == 1 and clock.counters["frames"] == 1
    assert frame.stitched == payload, "拼接帧原样携带，切分留给 decode_stereo()"
    assert frame.driver_sequence == 1001
    assert (frame.stitched_width, frame.height, frame.half_width) == (
        STITCH_WIDTH,
        STITCH_HEIGHT,
        HALF_WIDTH,
    )
    assert frame.clock_source == CLOCK_MONOTONIC
    assert frame.capture_mono_s == pytest.approx(SYNTHETIC_MONO_S)
    assert frame.capture_ros_s == pytest.approx(SYNTHETIC_MONO_S + ROS_OFFSET_S)

    # 一份时间戳被两半共享：帧上不存在第二个时间字段可以放「另一目」的时间
    left, right = clock.decode_stereo(frame)
    reference_left, reference_right = decode_stitched(payload, STITCH_WIDTH, STITCH_HEIGHT)
    assert np.array_equal(left, reference_left) and np.array_equal(right, reference_right)
    assert _max_abs_diff(left, _fixture_gray("half_a.png")) <= JPEG_ROUNDTRIP_MARGIN
    assert _max_abs_diff(right, _fixture_gray("half_b.png")) <= JPEG_ROUNDTRIP_MARGIN


def test_stereo_frame_header_carries_the_single_capture_timestamp():
    """`frame.header()` 的时间必须就是 `capture_ros_s`（左右共用同一个 header 时间）。

    需要 ROS 消息包（`std_msgs`/`builtin_interfaces`）；缺失时跳过本用例，
    不影响上面那些不依赖 ROS 消息的断言。
    """
    pytest.importorskip("std_msgs.msg", reason="header() 依赖 ROS 消息包")
    payload = _mjpeg_payload_from_recorded_stereo()
    source = RecordingSource([payload])
    clock = StereoFrameClock(source, RosTimeBase.from_offset(ROS_OFFSET_S, SYNTHETIC_MONO_S))
    frame = clock.next_frame()
    stamp = frame.header().stamp
    assert stamp.sec == int(frame.capture_ros_s)
    assert stamp.nanosec * 1e-9 == pytest.approx(
        frame.capture_ros_s - int(frame.capture_ros_s), abs=1e-9
    )


def test_recorded_frame_uses_negotiated_size_when_width_not_given():
    """`stitched_width=0` 时沿用 S_FMT 协商出的宽度（回放源给出 1280×480）。"""
    payload = _mjpeg_payload_from_recorded_stereo()
    source = RecordingSource([payload])
    clock = StereoFrameClock(source, RosTimeBase.from_offset(ROS_OFFSET_S, SYNTHETIC_MONO_S))
    frame = clock.next_frame()
    assert (frame.stitched_width, frame.height) == (STITCH_WIDTH, STITCH_HEIGHT)
    left, right = clock.decode_stereo(frame)
    assert left.shape == right.shape == (STITCH_HEIGHT, HALF_WIDTH)
    assert _max_abs_diff(left, _fixture_gray("half_a.png")) <= JPEG_ROUNDTRIP_MARGIN


# ================================================================== 7. 格式不匹配必须报错


@pytest.mark.parametrize(
    "kwargs, expected_message",
    [
        # 奇数宽度：无法对半切，必须早于解码就拒绝
        (dict(stitched_width=STITCH_WIDTH + 1, height=STITCH_HEIGHT), "不是偶数"),
        # 尺寸与 JPEG 实际尺寸不符：不得 resize、不得猜布局
        (dict(stitched_width=STITCH_WIDTH - 2, height=STITCH_HEIGHT), "不一致"),
        (dict(stitched_width=STITCH_WIDTH, height=STITCH_HEIGHT + 2), "不一致"),
        # 没有尺寸信息：不得假定一个默认值
        (dict(stitched_width=0, height=STITCH_HEIGHT), "缺少拼接帧尺寸"),
        (dict(stitched_width=STITCH_WIDTH, height=0), "缺少拼接帧尺寸"),
        # 不支持的切分方式：目前只有按宽度对半切
        (
            dict(stitched_width=STITCH_WIDTH, height=STITCH_HEIGHT, split="vertical"),
            "不支持的切分方式",
        ),
    ],
)
def test_decode_stitched_rejects_format_mismatch(kwargs, expected_message):
    """四种格式不匹配都必须抛 `CameraTimestampError`，并且报出**对应**的原因。

    断言消息片段而不是只看异常类型：否则「因为别的原因碰巧报错」也会蒙混过关。
    所有这些分支都必须报错——静默 resize 或猜布局会让左右目串位，
    而串位的深度/位姿错误极难从结果上看出来。
    """
    payload = _mjpeg_payload_from_recorded_stereo()
    with pytest.raises(CameraTimestampError) as exc:
        decode_stitched(payload, **kwargs)
    assert expected_message in str(exc.value)


def test_decode_stitched_rejects_undecodable_payload():
    """解不出图（不是 MJPEG）即报错，不得返回空数组或按字节切分。"""
    with pytest.raises(CameraTimestampError) as exc:
        decode_stitched(b"\x00\x01\x02\x03" * 64, STITCH_WIDTH, STITCH_HEIGHT)
    assert "解码失败" in str(exc.value)


def test_decode_stereo_rejects_frame_with_wrong_size():
    """`frame` 里带的尺寸与 JPEG 实际尺寸不符时同样报错（不缩放、不猜）。"""
    payload = _flat_stitched_jpeg(64, 8, left=10, right=200)
    source = RecordingSource([payload], size=(64, 8))
    clock = StereoFrameClock(source, RosTimeBase.from_offset(ROS_OFFSET_S, SYNTHETIC_MONO_S))
    frame = clock.next_frame()
    wrong = dataclasses.replace(frame, height=16)      # 谎报高度
    with pytest.raises(CameraTimestampError) as exc:
        clock.decode_stereo(wrong)
    assert "不一致" in str(exc.value)


# ================================================================== 8. 切分实现只有一份


def test_decode_stereo_delegates_to_module_level_decode_stitched(monkeypatch):
    """`StereoFrameClock.decode_stereo` 必须把整幅字节交给模块级 `decode_stitched`。

    为什么：模块的格式约定是「一份切分实现」。如果时钟类自己再切一次（或先 resize），
    就会出现两份可能不一致的切分逻辑，串位类缺陷会重新出现。这里用打桩记录实参，
    把委托关系和传参一起钉住。
    """
    left_sentinel, right_sentinel = object(), object()
    seen = {}

    def spy(data, stitched_width, height, split="horizontal"):
        seen.update(data=data, stitched_width=stitched_width, height=height, split=split)
        return left_sentinel, right_sentinel

    monkeypatch.setattr(ct, "decode_stitched", spy)
    payload = _flat_stitched_jpeg(64, 8, left=10, right=200)
    source = RecordingSource([payload], size=(64, 8))
    clock = StereoFrameClock(source, RosTimeBase.from_offset(ROS_OFFSET_S, SYNTHETIC_MONO_S))
    frame = clock.next_frame()

    result = clock.decode_stereo(frame)
    assert result[0] is left_sentinel and result[1] is right_sentinel
    assert seen == {
        "data": frame.stitched,
        "stitched_width": 64,
        "height": 8,
        "split": "horizontal",
    }

    # 结构性核对：时钟类自身不得解码或切片（否则等于第二份实现）
    clock_source = inspect.getsource(StereoFrameClock)
    assert "imdecode" not in clock_source
    assert "[:, " not in clock_source


def test_stereo_frame_has_no_per_eye_payload():
    """`StereoFrame` 只持有整幅拼接字节：不再有左右字节块，也没有每目时间戳字段。

    为什么：旧实现把左右当成两段独立 JPEG 字节（`frame.left`/`frame.right`），
    与「一整幅 MJPEG」的真实采集格式不符。若有人再按那种形状读写，必须立刻失败。
    """
    fields = set(StereoFrame.__dataclass_fields__)
    assert {"stitched", "stitched_width", "height", "capture_ros_s"} <= fields
    assert not {"left", "right", "left_ts", "right_ts", "left_ros_s"} & fields
    assert not hasattr(StereoFrameClock, "_split"), "旧 API：切分曾是时钟类的私有方法"

    frame = StereoFrame(
        sequence=1,
        stitched=b"",
        capture_mono_s=SYNTHETIC_MONO_S,
        capture_ros_s=SYNTHETIC_MONO_S + ROS_OFFSET_S,
        capture_uncertainty_s=0.0,
        stitched_width=STITCH_WIDTH,
        height=STITCH_HEIGHT,
        clock_source=CLOCK_MONOTONIC,
    )
    assert frame.half_width == HALF_WIDTH, "半宽是派生量，不是另一个可写字段"


@pytest.mark.parametrize("width, height", [(64, 8), (STITCH_WIDTH, STITCH_HEIGHT)])
def test_decode_stitched_splits_by_width_with_known_asymmetric_pattern(width, height):
    """已知非对称图案（左半 17、右半 233）：期望值由测试先写死，不来自实现。

    平场图案让 JPEG 往返无损，所以这里可以要求**完全相等**，同时证明两件事：
      - 切分列正好在宽度的一半（左半里不许混进右半的像素，反之亦然）；
      - 顺序是「左半在前」（左右互换即失败）。
    """
    payload = _flat_stitched_jpeg(width, height, left=17, right=233)
    left, right = decode_stitched(payload, width, height)
    assert left.shape == right.shape == (height, width // 2)
    assert np.array_equal(left, np.full((height, width // 2), 17, np.uint8))
    assert np.array_equal(right, np.full((height, width // 2), 233, np.uint8))
    assert not np.array_equal(left, right)


def test_documented_format_matches_existing_pipeline():
    """模块声称的「一整幅 MJPEG、整幅解码、按宽度对半切」必须与现有采集链一致。

    依据是 `stereo_depth/depth_preview.py::StereoProcessor.rectify()`：它同样把**整幅**帧
    交给 `cv2.imdecode(packet.reshape(-1), ...)`，再按宽度切成两半（`image[:, :C]` /
    `image[:, C:]`）。这里只读该文件源码做形态核对（不导入、不建处理器、不碰相机），
    用来防止「模块按 A 格式切分、采集链按 B 格式产出」这类静默错位。

    注意一处**已知差异**：采集链按 BGR 解码，本模块按灰度解码。录制帧本身是灰度内容，
    两者在「切分列」上一致；这条差异不影响切分位置，但接入真实节点时需按发布格式确认。
    """
    spec = importlib.util.find_spec("depth_preview")     # conftest 已把 stereo_depth 加进 sys.path
    if spec is None or not spec.origin:
        pytest.skip("找不到 stereo_depth/depth_preview.py：无法核对采集链的格式约定")
    text = pathlib.Path(spec.origin).read_text(encoding="utf-8")

    assert re.search(r"cv2\.imdecode\(packet\.reshape\(-1\)", text), (
        "采集链不再把整幅拼接帧交给 cv2.imdecode：模块的格式假设需要重新核对"
    )
    left_cols = [int(c) for c in re.findall(r"image\[:, :(\d+)\]", text)]
    right_cols = [int(c) for c in re.findall(r"image\[:, (\d+):\]", text)]
    assert left_cols and left_cols == right_cols, (
        f"采集链里的按宽度切分不唯一或不一致：左 {left_cols} / 右 {right_cols}"
    )
    assert len(set(left_cols)) == 1, f"采集链里出现了多个切分列：{left_cols}"
    resize = re.search(r"cv2\.resize\(image, \((\d+), (\d+)\)", text)
    assert resize, "采集链里找不到拼接帧的缩放目标尺寸"
    assert int(resize.group(1)) == 2 * left_cols[0], (
        f"采集链的切分列 {left_cols[0]} 不是拼接宽度 {resize.group(1)} 的一半"
    )
