"""真实相机采集的可复用时间戳接口（V4L2 帧时间戳优先）。

问题
----
现有 `depth_preview.py` 用 `cv2.VideoCapture(...).read()` 取帧，只能得到
「OpenCV 返回时刻」（`time.monotonic()`），那不是曝光时刻。同一拼接帧拆出的左右图
若各自打戳或各自打开相机，时间戳会不一致；把 ROS 发布时间当曝光时间更是错的。

本模块的规定
------------
1. 采集时间戳**只能**来自驱动：V4L2 `VIDIOC_DQBUF` 返回的 `v4l2_buffer.timestamp`。
   该时间戳的时钟源由 `flags` 的 `V4L2_BUF_FLAG_TIMESTAMP_*` 位说明。
2. 只接受可核实时域：`MONOTONIC`（内核 `CLOCK_MONOTONIC`，与 `time.monotonic()` 同域）
   直接使用；`REALTIME` 仅在显式允许时用「REALTIME→MONOTONIC 近似偏移」换算，
   默认拒绝，因为这个偏移会随 NTP 步进而变，必须由使用者确认。
   `UNKNOWN` 一律拒绝。
3. 取不到可信时间戳时**报错**（`CameraTimestampError`），不静默回退到收帧时刻，
   也不发布看似有效的时间戳。真实链路在驱动不支持时必须保持禁用。
4. 一个拼接帧只取一个时间戳，左右图共享；`StereoFrameClock` 负责这件事。
5. 曝光时刻到驱动时间戳之间仍可能有偏移（压缩、行读出、驱动打戳位置），
   这属于**待硬件标定项**，本模块只保证「可追溯到内核时域的同一时间戳」，
   不声称已经等于曝光中点。

依赖：标准库 + `fcntl`/`mmap`/`select`（Linux）。不需要 python3-v4l2 之类的封装。
"""

from __future__ import annotations

import ctypes
import fcntl
import mmap
import os
import select
import struct
import time
from dataclasses import dataclass

from .timebase import RosTimeBase, msg_from_ros_seconds

# ------------------------------------------------------------------ ioctl 常量
# 取自 <linux/videodev2.h>；用 _IO/_IOW/_IOWR 宏语义手工编码，避免额外依赖。
VIDIOC_QUERYCAP = 0x80685600
VIDIOC_ENUM_FMT = 0xC0405602
VIDIOC_S_FMT = 0xC0D05605
VIDIOC_REQBUFS = 0xC0145608
VIDIOC_QUERYBUF = 0xC0585609
VIDIOC_QBUF = 0xC058560F
VIDIOC_DQBUF = 0xC0585611
VIDIOC_STREAMON = 0x40045612
VIDIOC_STREAMOFF = 0x40045613

V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_MEMORY_MMAP = 1
V4L2_FIELD_NONE = 1
V4L2_PIX_FMT_MJPEG = 0x47504A4D          # 'MJPG'
V4L2_PIX_FMT_YUYV = 0x56595559           # 'YUYV'
V4L2_PIX_FMT_GREY = 0x59455247           # 'GREY'
V4L2_PIX_FMT_SRGGB8 = 0x42474752         # 'RGGB'

V4L2_BUF_FLAG_TIMESTAMP_MASK = 0x0000E000
V4L2_BUF_FLAG_TIMESTAMP_UNKNOWN = 0x00000000
V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC = 0x00002000
V4L2_BUF_FLAG_TIMESTAMP_COPY = 0x00004000

# ------------------------------------------------------------------ 结构体布局
# `struct v4l2_buffer`（/usr/include/linux/videodev2.h:1065）在 64 位 Linux
# （x86_64 / aarch64，本机实测）上的布局。**不要凭记忆写偏移**：这里的数值由
# 编译期 offsetof() 核对（见 test_camera_timestamp.py::test_v4l2_buffer_layout_matches_c_header）。
#
#     offset  0  __u32 index
#     offset  4  __u32 type
#     offset  8  __u32 bytesused
#     offset 12  __u32 flags           ← 时间戳时域标志在这里
#     offset 16  __u32 field
#     offset 24  struct timeval timestamp（tv_sec @24, tv_usec @32，各 8 字节）
#     offset 40  struct v4l2_timecode timecode（16 字节）
#     offset 56  __u32 sequence
#     offset 60  __u32 memory
#     offset 64  union m（指针/偏移，8 字节）
#     offset 72  __u32 length
#     offset 76  __u32 reserved2
#     offset 80  __s32 request_fd / __u32 reserved
#     总大小 88
V4L2_BUFFER_SIZE = 88
V4L2_BUFFER_OFF_INDEX = 0
V4L2_BUFFER_OFF_TYPE = 4
V4L2_BUFFER_OFF_BYTESUSED = 8
V4L2_BUFFER_OFF_FLAGS = 12
V4L2_BUFFER_OFF_FIELD = 16
V4L2_BUFFER_OFF_TIMESTAMP_SEC = 24
V4L2_BUFFER_OFF_TIMESTAMP_USEC = 32
V4L2_BUFFER_OFF_SEQUENCE = 56
V4L2_BUFFER_OFF_MEMORY = 60
V4L2_BUFFER_OFF_LENGTH = 72
V4L2_BUFFER_OFF_REQUEST_FD = 80

# `struct v4l2_format`（同一头文件）: type@0, 之后是 union；pix 部分
# width@0 height@4 pixelformat@8 field@12 bytesperline@16 sizeimage@20 ...
V4L2_FORMAT_SIZE = 208
V4L2_FORMAT_OFF_TYPE = 0
V4L2_FORMAT_OFF_WIDTH = 8
V4L2_FORMAT_OFF_HEIGHT = 12
V4L2_FORMAT_OFF_PIXELFORMAT = 16
V4L2_FORMAT_OFF_FIELD = 20

# `struct v4l2_requestbuffers`: count@0 type@4 memory@8 reserved@12（共 20 字节）
V4L2_REQUESTBUFFERS_SIZE = 20

TIMEVAL_SEC_SIZE = 8          # 64 位 Linux 的 time_t
TIMEVAL_USEC_SIZE = 8         # 64 位 Linux 的 suseconds_t

# 驱动时间戳与内核时域的对应关系（可核实来源）
CLOCK_MONOTONIC = "monotonic"
CLOCK_REALTIME = "realtime"
CLOCK_UNKNOWN = "unknown"

PIXEL_FORMAT_NAMES = {
    V4L2_PIX_FMT_MJPEG: "MJPG",
    V4L2_PIX_FMT_YUYV: "YUYV",
    V4L2_PIX_FMT_GREY: "GREY",
    V4L2_PIX_FMT_SRGGB8: "RGGB",
}


class CameraTimestampError(RuntimeError):
    """采集时间戳不可核实：真实链路必须保持禁用或修复采集路径。"""


def timestamp_source_from_flags(flags: int) -> str:
    """V4L2 `v4l2_buffer.flags` → 时间戳时钟源（可核实来源的判据）。"""
    kind = flags & V4L2_BUF_FLAG_TIMESTAMP_MASK
    if kind == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC:
        return CLOCK_MONOTONIC
    if kind == V4L2_BUF_FLAG_TIMESTAMP_COPY:
        return CLOCK_UNKNOWN
    if kind == V4L2_BUF_FLAG_TIMESTAMP_UNKNOWN:
        return CLOCK_UNKNOWN
    return CLOCK_UNKNOWN


def realtime_to_monotonic_offset_s() -> tuple[float, float]:
    """(CLOCK_MONOTONIC - CLOCK_REALTIME) 近似偏移与不确定度（秒）。

    REALTIME 时间戳只在显式允许时使用；该偏移会随 NTP 步进/调整而变，
    返回的不确定度用两次采样的漂移估计。
    """
    t0 = time.clock_gettime(time.CLOCK_REALTIME)
    m0 = time.clock_gettime(time.CLOCK_MONOTONIC)
    t1 = time.clock_gettime(time.CLOCK_REALTIME)
    m1 = time.clock_gettime(time.CLOCK_MONOTONIC)
    offset = 0.5 * ((m0 - t0) + (m1 - t1))
    uncertainty = abs((m1 - t1) - (m0 - t0)) + abs(t1 - t0) + abs(m1 - m0)
    return offset, uncertainty


@dataclass
class RawFrame:
    """一次底层取帧的结果，时间戳仍是**驱动时域**。"""

    sequence: int
    data: bytes
    timestamp_s: float          # 驱动时间戳（时域见 clock_source）
    clock_source: str           # monotonic | realtime | unknown
    flags: int = 0
    received_mono_s: float = 0.0   # 仅供诊断：明确不是采集时间戳
    driver_sequence: int = 0       # v4l2_buffer.sequence（驱动自己的帧计数）
    buffer_index: int = 0          # v4l2_buffer.index（本次出队的缓冲）
    bytes_used: int = 0            # v4l2_buffer.bytesused（本帧有效字节数）


@dataclass
class TimestampedFrame:
    """时间戳已归算到 Companion 单调时域的一帧。"""

    sequence: int
    data: bytes
    capture_mono_s: float           # 采集时间戳（CLOCK_MONOTONIC 秒）
    capture_ros_s: float            # 映射到 ROS 时间域
    capture_uncertainty_s: float    # 归算引入的不确定度（不含曝光偏差）
    clock_source: str
    received_mono_s: float


@dataclass
class StereoFrame:
    """同一次底层取帧得到的拼接帧，左右目**共享同一个采集时间戳**。

    拼接帧是**一整幅 MJPEG**（沿用现有采集链格式）；切分交给 `decode_stereo()`，
    时间戳与图像内容一一对应，不存在「左右各自打戳」的可能。
    """

    sequence: int
    stitched: bytes
    capture_mono_s: float
    capture_ros_s: float
    capture_uncertainty_s: float
    stitched_width: int
    height: int
    clock_source: str
    driver_sequence: int = 0

    @property
    def half_width(self) -> int:
        return self.stitched_width // 2

    def header(self):
        from std_msgs.msg import Header

        h = Header()
        h.stamp = msg_from_ros_seconds(self.capture_ros_s)
        return h


class CameraTimestampSource:
    """V4L2 采集：取帧并给出**驱动时域**时间戳。"""

    def __init__(
        self,
        device: str,
        width: int,
        height: int,
        fps: int = 60,
        pixel_format: int = V4L2_PIX_FMT_MJPEG,
        buffer_count: int = 4,
        allow_realtime: bool = False,
        realtime_uncertainty_limit_s: float = 0.002,
    ) -> None:
        self.device = device
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.pixel_format = int(pixel_format)
        self.buffer_count = max(int(buffer_count), 2)
        self.allow_realtime = bool(allow_realtime)
        self.realtime_uncertainty_limit_s = float(realtime_uncertainty_limit_s)
        self._fd: int | None = None
        self._buffers: list[mmap.mmap] = []
        self._sequence = 0
        self._last_mono_s: float | None = None
        self.timestamp_source: str | None = None
        self.negotiated = {}
        self.streaming = False

    # ---------------------------------------------------------------- 生命周期

    def open(self) -> None:
        self._fd = os.open(self.device, os.O_RDWR | os.O_NONBLOCK)
        try:
            self._s_fmt()
            self._reqbufs()
            self._queue_all()
            fcntl.ioctl(self._fd, VIDIOC_STREAMON, struct.pack("I", V4L2_BUF_TYPE_VIDEO_CAPTURE))
            self.streaming = True
        except Exception:
            self.close()
            raise

    def close(self) -> None:
        if self._fd is None:
            return
        try:
            if self.streaming:
                fcntl.ioctl(self._fd, VIDIOC_STREAMOFF, struct.pack("I", V4L2_BUF_TYPE_VIDEO_CAPTURE))
        except OSError:
            pass
        self.streaming = False
        for buf in self._buffers:
            try:
                buf.close()
            except Exception:  # noqa: BLE001
                pass
        self._buffers = []
        try:
            os.close(self._fd)
        except OSError:
            pass
        self._fd = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # ---------------------------------------------------------------- V4L2 细节

    @staticmethod
    def check_abi() -> None:
        """确认当前平台的结构体布局与常量一致（架构不同则显式报错，不静默错读）。"""
        if struct.calcsize("P") != 8:
            raise CameraTimestampError(
                "只支持 64 位 Linux 的 v4l2_buffer 布局（本实现按 88 字节结构体读写）"
            )

    def _s_fmt(self) -> None:
        buf = bytearray(V4L2_FORMAT_SIZE)
        buf[V4L2_FORMAT_OFF_TYPE:V4L2_FORMAT_OFF_TYPE + 4] = struct.pack(
            "I", V4L2_BUF_TYPE_VIDEO_CAPTURE
        )
        # union 的 pix 部分：width/height/pixelformat/field/bytesperline/sizeimage
        struct.pack_into("IIIIII", buf, V4L2_FORMAT_OFF_WIDTH,
                         self.width, self.height, self.pixel_format, V4L2_FIELD_NONE, 0, 0)
        fcntl.ioctl(self._fd, VIDIOC_S_FMT, buf)
        width, height, pixfmt = struct.unpack_from("III", buf, V4L2_FORMAT_OFF_WIDTH)
        self.negotiated = {
            "width": width,
            "height": height,
            "pixelformat": PIXEL_FORMAT_NAMES.get(pixfmt, hex(pixfmt)),
            "pixelformat_raw": pixfmt,
        }
        if (width, height) != (self.width, self.height):
            raise CameraTimestampError(
                f"驱动给出的尺寸 {width}x{height} 与请求 {self.width}x{self.height} 不一致"
            )

    def _reqbufs(self) -> None:
        buf = bytearray(V4L2_REQUESTBUFFERS_SIZE)
        struct.pack_into("IIII", buf, 0, self.buffer_count, V4L2_BUF_TYPE_VIDEO_CAPTURE,
                         V4L2_MEMORY_MMAP, 0)
        fcntl.ioctl(self._fd, VIDIOC_REQBUFS, buf)
        count = struct.unpack_from("I", buf, 0)[0]
        if count < 2:
            raise CameraTimestampError(f"驱动只提供了 {count} 个缓冲区，无法采集")
        self.buffer_count = count

    def _queue_all(self) -> None:
        for index in range(self.buffer_count):
            length = self._querybuf_length(index)
            mm = mmap.mmap(self._fd, length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
            self._buffers.append(mm)
            fcntl.ioctl(self._fd, VIDIOC_QBUF, self._buf_pack(index))

    def _buf_pack(self, index: int) -> bytearray:
        """构造 QUERYBUF/QBUF 入参：只需 index/type/memory 三个字段。"""
        self.check_abi()
        buf = bytearray(V4L2_BUFFER_SIZE)
        struct.pack_into("I", buf, V4L2_BUFFER_OFF_INDEX, index)
        struct.pack_into("I", buf, V4L2_BUFFER_OFF_TYPE, V4L2_BUF_TYPE_VIDEO_CAPTURE)
        struct.pack_into("I", buf, V4L2_BUFFER_OFF_MEMORY, V4L2_MEMORY_MMAP)
        return buf

    def _querybuf_length(self, index: int) -> int:
        """QUERYBUF 返回的 `length` 是映射长度（不是本帧有效字节数）。"""
        buf = self._buf_pack(index)
        fcntl.ioctl(self._fd, VIDIOC_QUERYBUF, buf)
        return struct.unpack_from("I", buf, V4L2_BUFFER_OFF_LENGTH)[0]

    @staticmethod
    def parse_buffer(buf) -> dict:
        """从 DQBUF 后的 v4l2_buffer 字节中解出各字段（唯一解析入口，便于单测）。"""
        raw = bytes(buf)
        if len(raw) < V4L2_BUFFER_SIZE:
            raise CameraTimestampError(
                f"v4l2_buffer 长度 {len(raw)} 小于预期 {V4L2_BUFFER_SIZE}"
            )
        sec = struct.unpack_from("q", raw, V4L2_BUFFER_OFF_TIMESTAMP_SEC)[0]
        usec = struct.unpack_from("q", raw, V4L2_BUFFER_OFF_TIMESTAMP_USEC)[0]
        flags = struct.unpack_from("I", raw, V4L2_BUFFER_OFF_FLAGS)[0]
        return {
            "index": struct.unpack_from("I", raw, V4L2_BUFFER_OFF_INDEX)[0],
            "bytesused": struct.unpack_from("I", raw, V4L2_BUFFER_OFF_BYTESUSED)[0],
            "flags": flags,
            "sequence": struct.unpack_from("I", raw, V4L2_BUFFER_OFF_SEQUENCE)[0],
            "length": struct.unpack_from("I", raw, V4L2_BUFFER_OFF_LENGTH)[0],
            "timestamp_s": sec + usec * 1e-6,
            "clock_source": timestamp_source_from_flags(flags),
        }

    # ---------------------------------------------------------------- 取帧

    def read_raw(self, timeout_s: float = 1.0) -> RawFrame:
        """从驱动取一帧，返回**未映射**的驱动时间戳。"""
        if self._fd is None:
            raise CameraTimestampError("设备未打开")
        ready, _, _ = select.select([self._fd], [], [], timeout_s)
        if not ready:
            raise CameraTimestampError(f"{self.device} 在 {timeout_s}s 内没有可读缓冲（取帧超时）")
        received = time.monotonic()
        buf = self._buf_pack(0)
        fcntl.ioctl(self._fd, VIDIOC_DQBUF, buf)
        parsed = self.parse_buffer(buf)
        index = parsed["index"]
        if index >= len(self._buffers):
            # 出队了的缓冲必须还回去，否则缓冲池会越用越少
            try:
                fcntl.ioctl(self._fd, VIDIOC_QBUF, self._buf_pack(index))
            except OSError:
                pass
            raise CameraTimestampError(
                f"驱动返回的缓冲索引 {index} 超出已映射范围 0..{len(self._buffers) - 1}"
            )
        try:
            # MMAP 采集用 bytesused（本帧有效字节数）；length 只是映射长度。
            used = parsed["bytesused"] or parsed["length"]
            payload = bytes(self._buffers[index][:used])
        finally:
            # 无论拷贝是否失败都要把缓冲还给驱动，否则会静默丢缓冲
            fcntl.ioctl(self._fd, VIDIOC_QBUF, self._buf_pack(index))
        self._sequence += 1
        return RawFrame(
            sequence=self._sequence,
            data=payload,
            timestamp_s=parsed["timestamp_s"],
            clock_source=parsed["clock_source"],
            flags=parsed["flags"],
            received_mono_s=received,
            driver_sequence=parsed["sequence"],
            buffer_index=index,
            bytes_used=used,
        )

    def to_monotonic(self, frame: RawFrame) -> tuple[float, float]:
        """驱动时间戳 → Companion 单调时钟秒，返回 (t_mono_s, 不确定度)。

        时域不可核实即抛 `CameraTimestampError`；不做静默回退。
        """
        if frame.clock_source == CLOCK_MONOTONIC:
            if frame.timestamp_s <= 0.0:
                raise CameraTimestampError(
                    f"{self.device} 驱动时间戳为 {frame.timestamp_s}，MONOTONIC 时域下无效"
                )
            return frame.timestamp_s, 0.0
        if frame.clock_source == CLOCK_REALTIME and self.allow_realtime:
            offset, uncertainty = realtime_to_monotonic_offset_s()
            if uncertainty > self.realtime_uncertainty_limit_s:
                raise CameraTimestampError(
                    f"REALTIME→MONOTONIC 偏移不确定度 {uncertainty * 1e3:.3f} ms 超过上限 "
                    f"{self.realtime_uncertainty_limit_s * 1e3:.3f} ms；"
                    "该时域不可核实，请改用 V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC"
                )
            return frame.timestamp_s + offset, uncertainty
        raise CameraTimestampError(
            f"{self.device} 的 V4L2 缓冲时间戳时域不可核实（flags=0x{frame.flags:08x}, "
            f"source={frame.clock_source}）。真实链路必须保持禁用，"
            "或改用支持 V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC 的采集路径/驱动。"
        )


def probe_v4l2(path: str) -> dict:
    """只读探测设备能力（不排队缓冲、不改变流状态）。"""
    report: dict = {"device": path, "exists": os.path.exists(path)}
    if not report["exists"]:
        report["error"] = "设备节点不存在"
        return report
    fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
    try:
        cap = bytearray(104)
        fcntl.ioctl(fd, VIDIOC_QUERYCAP, cap)
        driver, card, bus_info, version, capabilities = struct.unpack("16s32s32sII", bytes(cap[:88]))
        report.update(
            driver=driver.split(b"\x00")[0].decode(errors="replace"),
            card=card.split(b"\x00")[0].decode(errors="replace"),
            bus_info=bus_info.split(b"\x00")[0].decode(errors="replace"),
            version=version,
            capabilities=hex(capabilities),
            has_video_capture=bool(capabilities & 0x00000001),
            has_streaming=bool(capabilities & 0x04000000),
        )
    finally:
        os.close(fd)

    report["timestamp_source_hint"] = (
        "时间戳时域必须由实际 DQBUF 后的 v4l2_buffer.flags 判定；"
        "加 --size WxH 并去掉 --probe 即可抓帧观察"
    )
    return report


class StereoFrameClock:
    """把「一次底层取帧」变成「ROS 时间域的一个双目帧」。

    - 一个拼接帧只取一个驱动时间戳，左右图**共享**它；这里只做一次映射；
    - 时间倒退（驱动时域或映射后）立即报错，不发布看似有效的时间戳；
    - 拼接格式沿用现有采集链（见 `depth_preview.py`）：**一整幅拼接 JPEG/MJPEG**，
      解码后按宽度对半切左右目，而不是「两幅独立 JPEG 顺序拼接」。

    只做「取出 + 时间戳 + 切分」，不做校正/缩放；几何处理仍由 `stereo_depth` 的
    `StereoProcessor` 负责，避免出现第二份几何实现。
    """

    def __init__(
        self,
        source: CameraTimestampSource,
        timebase: RosTimeBase,
        stitched_width: int = 0,
        split: str = "horizontal",
    ) -> None:
        self.source = source
        self.timebase = timebase
        self.stitched_width = int(stitched_width)
        self.split = split
        self._last_mono_s: float | None = None
        self._last_ros_s: float | None = None
        self.sequence = 0
        self.counters = {
            "frames": 0,
            "timestamp_errors": 0,
            "backwards": 0,
            "realtime_mapped": 0,
            "decode_errors": 0,
        }

    def next_frame(self) -> StereoFrame:
        """取一帧并返回带 ROS 时间戳的原始拼接帧（不做解码，避免无谓开销）。"""
        raw = self.source.read_raw()
        capture_mono_s, extra_uncertainty = self.source.to_monotonic(raw)
        if raw.clock_source == CLOCK_REALTIME:
            self.counters["realtime_mapped"] += 1

        if self._last_mono_s is not None and capture_mono_s <= self._last_mono_s:
            self.counters["backwards"] += 1
            raise CameraTimestampError(
                f"采集时间戳未单调递增：{capture_mono_s:.6f} <= {self._last_mono_s:.6f}"
            )

        # 归算到 ROS 时间域：映射采样时刻与采集时刻的差决定额外不确定度。
        sample = self.timebase.sample()
        extra = max(sample.uncertainty_s, abs(capture_mono_s - sample.mono_s) * 1e-4)
        capture_ros_s = capture_mono_s + sample.offset_s
        if self._last_ros_s is not None and capture_ros_s <= self._last_ros_s:
            self.counters["backwards"] += 1
            raise CameraTimestampError("ROS 时间域映射后时间戳未递增（检查 ROS 时钟是否被步进）")

        self._last_mono_s = capture_mono_s
        self._last_ros_s = capture_ros_s
        self.sequence += 1
        self.counters["frames"] += 1
        return StereoFrame(
            sequence=self.sequence,
            stitched=raw.data,
            capture_mono_s=capture_mono_s,
            capture_ros_s=capture_ros_s,
            capture_uncertainty_s=extra_uncertainty + extra,
            stitched_width=self.stitched_width or self.source.negotiated.get("width", 0),
            height=self.source.negotiated.get("height", 0),
            clock_source=raw.clock_source,
            driver_sequence=getattr(raw, "driver_sequence", 0),
        )

    def decode_stereo(self, frame: StereoFrame):
        """按现有采集链的方式解码拼接帧并切成 (left, right) 灰度数组。

        与 `depth_preview.StereoProcessor` 相同的判据：整幅解码、尺寸必须等于
        `stitched_width × height`、按宽度对半切。尺寸不符即报错，不猜。
        解码失败会计入 `counters["decode_errors"]`（异常照旧抛出，不静默吞掉）。
        """
        try:
            left, right = decode_stitched(
                frame.stitched,
                stitched_width=frame.stitched_width,
                height=frame.height,
                split=self.split,
            )
        except CameraTimestampError:
            self.counters["decode_errors"] += 1
            raise
        return left, right


def decode_stitched(data: bytes, stitched_width: int, height: int, split: str = "horizontal"):
    """解码**一整幅**拼接帧并按宽度切成左右目；与 depth_preview 的格式一致。

    返回 (left, right) 两个单通道 `numpy` 数组（H×W/2）。任何不一致都抛错，
    不静默 resize、不猜布局。
    """
    import cv2
    import numpy as np

    if split != "horizontal":
        raise CameraTimestampError(f"不支持的切分方式：{split}")
    if stitched_width <= 0 or height <= 0:
        raise CameraTimestampError(
            f"缺少拼接帧尺寸（{stitched_width}x{height}）：请用 --size WxH 或先 S_FMT 协商"
        )
    if stitched_width % 2 != 0:
        raise CameraTimestampError(f"拼接宽度 {stitched_width} 不是偶数，无法对半切")
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise CameraTimestampError("拼接帧 JPEG 解码失败（采集格式可能不是 MJPEG）")
    if image.shape[:2] != (height, stitched_width):
        raise CameraTimestampError(
            f"拼接帧尺寸 {image.shape[1]}x{image.shape[0]} 与预期 "
            f"{stitched_width}x{height} 不一致，请检查相机输出模式"
        )
    half = stitched_width // 2
    return image[:, :half], image[:, half:]


def main(argv=None) -> int:
    """`python3 -m boom_birds_nav.camera_timestamp`：部署前核验时间戳能力。"""
    import argparse
    import json
    import sys

    ap = argparse.ArgumentParser(description="V4L2 采集时间戳能力探测（只读）")
    ap.add_argument("--probe", action="store_true", help="打印设备能力与时间戳时域")
    ap.add_argument("--device", default="", help="例如 /dev/video0")
    ap.add_argument("--size", default="", help="WxH，例如 2560x720")
    ap.add_argument("--fps", type=int, default=60)
    ap.add_argument("--frames", type=int, default=5, help="抓取帧数用于观察时间戳")
    ap.add_argument("--allow-realtime", action="store_true",
                    help="允许 REALTIME 时域（默认拒绝；需自行确认 NTP 状态）")
    ap.add_argument("--out", default="", help="把 JSON 报告写入文件")
    args = ap.parse_args(argv)

    if not args.device:
        print("需要 --device（例如 /dev/video0）", file=sys.stderr, flush=True)
        return 2

    report: dict = {"device": args.device, "probe": args.probe}
    if args.probe and not args.size:
        report.update(probe_v4l2(args.device))
        # 只读探测无法判定时间戳时域：明确标注为「未完成」而不是「通过」
        report.setdefault("verdict", "PROBE_INCOMPLETE")
    elif args.size:
        width, _, height = args.size.partition("x")
        source = CameraTimestampSource(
            args.device, int(width), int(height), fps=args.fps, allow_realtime=args.allow_realtime
        )
        try:
            with source:
                report["negotiated"] = source.negotiated
                samples = []
                for _ in range(max(args.frames, 1)):
                    raw = source.read_raw()
                    entry = {
                        "sequence": raw.sequence,
                        "driver_sequence": raw.driver_sequence,
                        "buffer_index": raw.buffer_index,
                        "clock_source": raw.clock_source,
                        "flags": hex(raw.flags),
                        "timestamp_s": raw.timestamp_s,
                        "payload_bytes": len(raw.data),
                        "bytesused": raw.bytes_used,
                        "receive_minus_timestamp_s": raw.received_mono_s - raw.timestamp_s,
                    }
                    try:
                        mono_s, unc = source.to_monotonic(raw)
                        entry["monotonic_s"] = mono_s
                        entry["uncertainty_s"] = unc
                    except CameraTimestampError as exc:
                        entry["error"] = str(exc)
                        report["verdict"] = "TIMESTAMP_UNUSABLE"
                    samples.append(entry)
                report["samples"] = samples
                if "verdict" not in report:
                    report["verdict"] = "TIMESTAMP_TRACEABLE"
        except OSError as exc:
            report["error"] = f"打开/配置失败：{exc}"
            report["verdict"] = "PROBE_FAILED"
    else:
        report["error"] = "需要 --size WxH 才能实际取帧"
        report["verdict"] = "PROBE_INCOMPLETE"

    text = json.dumps(report, ensure_ascii=False, indent=2)
    print(text, flush=True)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as fh:
            fh.write(text)
    return 0 if report.get("verdict") == "TIMESTAMP_TRACEABLE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
