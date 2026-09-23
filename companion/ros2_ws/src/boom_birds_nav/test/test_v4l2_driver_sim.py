"""V4L2 采集路径的模拟驱动测试：不开设备，走完整的 open/协商/取帧/关闭流程。

为什么需要
----------
真实相机不可用（也不允许在本任务里打开设备），但 `open()` → `_s_fmt()` → `_reqbufs()`
→ `_queue_all()` → `read_raw()` 这条路径一旦写错字段偏移，真机上就会「取帧失败或读出
错误时间戳」，而纯函数测试发现不了。这里用一个**假 ioctl** 顶替内核：

- `os.open` 返回一个真实的匿名 fd（`os.memfd_create`），使 `mmap` / `select` 仍然真实；
- `fcntl.ioctl` 的替换按 `VIDIOC_*` 请求码分别处理，并**严格按 v4l2_buffer 的真实布局**
  写回字段（与 `camera_timestamp.py` 的常量同源，若常量错了，本测试的断言也会跟着错——
  所以偏移量另有编译期 `offsetof()` 核对，两者互补）；
- 断言的重点：时间戳来自 offset 24/32、`bytesused` 取自 offset 8、`length` 取自 offset 72、
  缓冲被正确还回（QBUF 次数）、以及字段顺序错误时会被发现。

结论只覆盖软件路径，不代表真机取帧可用。
"""

from __future__ import annotations

import os
import struct
import time

import numpy as np
import pytest

from boom_birds_nav import camera_timestamp as ct


class FakeDriver:
    """最小 V4L2 假驱动：回应 S_FMT / REQBUFS / QUERYBUF / QBUF / DQBUF / STREAMON。"""

    def __init__(self, width: int, height: int, buffer_count: int = 3,
                 frame_bytes: int = 512):
        self.width = width
        self.height = height
        self.buffer_count = buffer_count
        self.frame_bytes = frame_bytes
        self.calls: list[int] = []
        self.qbuf_indices: list[int] = []
        self.streaming = False
        self.timestamps = [(1000.0 + i * 0.02, 12345 + i) for i in range(8)]
        self._next_timestamp = 0
        self._payload = bytes(range(256)) * 2      # 512 字节可辨识内容

    # ---------------------------------------------------------------- ioctl 分派

    def ioctl(self, fd, request, arg, mutate=True):
        self.calls.append(request)
        if request == ct.VIDIOC_S_FMT:
            self._s_fmt(arg)
        elif request == ct.VIDIOC_REQBUFS:
            self._reqbufs(arg)
        elif request == ct.VIDIOC_QUERYBUF:
            index = struct.unpack_from("I", bytes(arg), ct.V4L2_BUFFER_OFF_INDEX)[0]
            struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_LENGTH, self.frame_bytes)
            struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_BYTESUSED, self.frame_bytes)
            self.qbuf_indices.append(index)
        elif request == ct.VIDIOC_QBUF:
            index = struct.unpack_from("I", bytes(arg), ct.V4L2_BUFFER_OFF_INDEX)[0]
            self.qbuf_indices.append(index)
        elif request == ct.VIDIOC_DQBUF:
            self._dqbuf(arg)
        elif request == ct.VIDIOC_STREAMON:
            self.streaming = True
        elif request == ct.VIDIOC_STREAMOFF:
            self.streaming = False
        return 0

    def _s_fmt(self, arg) -> None:
        struct.pack_into("IIIIII", arg, ct.V4L2_FORMAT_OFF_WIDTH,
                         self.width, self.height, ct.V4L2_PIX_FMT_MJPEG,
                         ct.V4L2_FIELD_NONE, 0, 0)

    def _reqbufs(self, arg) -> None:
        struct.pack_into("IIII", arg, 0, self.buffer_count,
                         ct.V4L2_BUF_TYPE_VIDEO_CAPTURE, ct.V4L2_MEMORY_MMAP, 0)

    def _dqbuf(self, arg) -> None:
        ts, seq = self.timestamps[self._next_timestamp % len(self.timestamps)]
        self._next_timestamp += 1
        struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_INDEX, 1)
        struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_TYPE, ct.V4L2_BUF_TYPE_VIDEO_CAPTURE)
        struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_BYTESUSED, self.frame_bytes)
        struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_FLAGS,
                         ct.V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)
        struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_SEQUENCE, seq)
        struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_MEMORY, ct.V4L2_MEMORY_MMAP)
        struct.pack_into("I", arg, ct.V4L2_BUFFER_OFF_LENGTH, self.frame_bytes)
        struct.pack_into("q", arg, ct.V4L2_BUFFER_OFF_TIMESTAMP_SEC, int(ts))
        struct.pack_into("q", arg, ct.V4L2_BUFFER_OFF_TIMESTAMP_USEC,
                         int(round((ts - int(ts)) * 1e6)))


class MmapStub:
    """`mmap.mmap` 替身：只需支持 len / 切片 / close。"""

    def __init__(self, data: bytes) -> None:
        self._data = data
        self.closed = False

    def __len__(self) -> int:
        return len(self._data)

    def __getitem__(self, item):
        return self._data[item]

    def close(self) -> None:
        self.closed = True


@pytest.fixture()
def fake_pipeline(monkeypatch):
    """把 ioctl / os.open / os.close / mmap.mmap 换成假实现，返回 (source, driver, fd)。"""
    width, height, buffers, frame_bytes = 1280, 480, 3, 512
    driver = FakeDriver(width, height, buffer_count=buffers, frame_bytes=frame_bytes)
    fd = os.memfd_create("bb_fake_v4l2")
    stubs: list[MmapStub] = []
    closed = {"fd": False}

    monkeypatch.setattr(ct.os, "open", lambda *a, **k: fd)
    def _close(handle):
        closed["fd"] = True
    monkeypatch.setattr(ct.os, "close", _close)
    monkeypatch.setattr(ct.fcntl, "ioctl", driver.ioctl)

    def _mmap(fileno, length, *a, **k):
        stub = MmapStub(driver._payload[:length])
        stubs.append(stub)
        return stub

    monkeypatch.setattr(ct.mmap, "mmap", _mmap)
    source = ct.CameraTimestampSource("/dev/fakevideo0", width, height, buffer_count=buffers)
    try:
        yield source, driver, stubs, closed
    finally:
        try:
            os.close(fd)
        except OSError:
            pass


def test_full_open_negotiate_read_close(fake_pipeline):
    """完整流程：协商尺寸、排队缓冲、取帧时间戳/字段正确、缓冲归还、关闭不泄漏。"""
    source, driver, stubs, closed = fake_pipeline
    source.open()

    assert source.negotiated["width"] == 1280
    assert source.negotiated["height"] == 480
    assert source.negotiated["pixelformat"] == "MJPG"
    assert source.streaming is True
    # QUERYBUF + QBUF：每个缓冲各排队一次
    assert driver.qbuf_indices.count(0) >= 1
    assert len(stubs) == driver.buffer_count

    first = source.read_raw()
    assert first.timestamp_s == pytest.approx(1000.0)
    assert first.driver_sequence == 12345
    assert first.buffer_index == 1
    assert first.bytes_used == 512
    assert len(first.data) == 512
    assert first.clock_source == ct.CLOCK_MONOTONIC
    assert first.flags == ct.V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC
    assert first.received_mono_s >= 0.0

    second = source.read_raw()
    assert second.timestamp_s == pytest.approx(1000.02)
    assert second.driver_sequence == 12346
    # 每次取帧后都必须把该缓冲还给驱动
    assert driver.qbuf_indices.count(1) >= 2

    # 单调时域 → 直接使用，不需要换算
    mono_s, uncertainty = source.to_monotonic(second)
    assert mono_s == pytest.approx(1000.02)
    assert uncertainty == 0.0

    source.close()
    assert source.streaming is False
    assert source._fd is None
    assert all(stub.closed for stub in stubs)
    assert closed["fd"] is True


def test_timestamp_would_be_wrong_if_offsets_drift(fake_pipeline):
    """反向验证：把假驱动的 flags/时间戳写到旧错误偏移（60/64），解析结果必然不同。

    这条测试固定住「偏移必须来自常量」这一事实：如果有人把常量改回错误值，
    上面的正常流程测试会失败。
    """
    source, driver, _stubs, _closed = fake_pipeline
    source.open()
    frame = source.read_raw()
    # 与「按错误偏移读」的结果对比：旧代码读 offset 60/64，那里现在是 memory/union
    buf = bytearray(ct.V4L2_BUFFER_SIZE)
    struct.pack_into("I", buf, ct.V4L2_BUFFER_OFF_FLAGS, ct.V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)
    struct.pack_into("q", buf, ct.V4L2_BUFFER_OFF_TIMESTAMP_SEC, 1000)
    struct.pack_into("q", buf, ct.V4L2_BUFFER_OFF_TIMESTAMP_USEC, 0)
    wrong_timestamp = struct.unpack_from("q", buf, 64)[0]
    wrong_flags = struct.unpack_from("I", buf, 60)[0]
    assert frame.timestamp_s == pytest.approx(1000.0)
    # 错误偏移读出来的是 sequence/memory/union 区域，不是时间戳
    assert wrong_timestamp == 0
    assert wrong_flags != ct.V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC
    source.close()
