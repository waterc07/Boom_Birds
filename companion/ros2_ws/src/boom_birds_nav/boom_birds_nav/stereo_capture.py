"""唯一采集源：真实 V4L2 采集与录制帧回放给出**语义相同**的双目帧。

为什么需要这一层
----------------
真实链路与回放链路必须走**同一条**「取帧 → 打戳 → 发布/解码」路径，否则「回放通过」
说明不了任何关于真实采集的事情，两条路径也会各自漂移：

- **相机只在一个地方打开。** 本模块把「取一帧」收敛成 `FrameSource`：
  `next_frame() -> StereoFrame`。两种实现只在「帧从哪来」上不同；节点里不应该出现
  「回放走一个分支、真相机走另一个分支」的代码，否则又会出现第二条采集路径。
- **时间戳语义只有一份。** 两种实现都经 `StereoFrameClock` 归算到 ROS 时间域：
  一次底层取帧只取一个驱动时间戳，整幅拼接帧（左右目）共享它。这条保证对回放同样
  成立，而且是**结构上的**（`StereoFrame` 只有一个 `capture_ros_s`），
  不是靠两处代码各自遵守约定。
- **回放不是「假装成真相机」。** 录制帧**没有曝光时间戳**（见
  `test/recordings/PROVENANCE.md`）：回放时间戳是 `start_mono_s + i * period_s`
  的合成占位值，`describe()` 里显式写明；录制帧是 PNG（无损）而真实采集是 MJPEG（有损），
  共用的是**解码/切分路径**（`camera_timestamp.decode_stitched` 对整幅载荷做一次
  `cv2.imdecode` 再按宽度对半切），不是编码格式。

未验证项（不要从本模块的测试结果推断硬件行为）
------------------------------------------------
- 真实曝光时刻、驱动 `flags` 报告的时域、真实取帧稳定性都只能在硬件上验收；
  本模块只保证「同一时间戳 + 同一条发布/解码路径」这一软件契约。
- 驱动时间戳到曝光中点之间仍可能有未知偏移（压缩、行读出、驱动打戳位置），
  属待硬件标定项，见 `camera_timestamp` 模块说明第 5 条。

边界
----
- 依赖：标准库 + `numpy` + `cv2`（与既有采集链一致）；不导入 ROS，可离线单测。
- 回放路径**永不打开设备**：唯一的设备打开点是 `V4L2FrameSource.open()`。
- 本模块只「取帧」，不做解码/校正/缩放：解码与切分仍是
  `camera_timestamp.decode_stereo`，几何仍是 `stereo_depth.StereoProcessor`，
  避免出现第二份实现。
"""

from __future__ import annotations

import pathlib
from dataclasses import dataclass
from typing import Protocol, Sequence, runtime_checkable

from .camera_timestamp import (
    CLOCK_MONOTONIC,
    PIXEL_FORMAT_NAMES,
    V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC,
    V4L2_PIX_FMT_MJPEG,
    CameraTimestampError,
    CameraTimestampSource,
    RawFrame,
    StereoFrame,
    StereoFrameClock,
)
from .timebase import RosTimeBase

__all__ = [
    "FrameSource",
    "V4L2FrameSource",
    "ReplayFrameSource",
    "ReplayExhausted",
    "RecordedFrame",
    "RecordingSet",
    "discover_recorded_frames",
    "REPLAY_EXPOSURE_NOTE",
    "REPLAY_FORMAT_NOTE",
    "V4L2_EXPOSURE_NOTE",
]

# 录制帧的两个事实，必须随 describe() 一起输出，避免把回放结果误读成硬件验收：
# 1) 没有曝光时间戳——文件/目录时间只是主机保存时刻；
# 2) PNG 无损、真实采集 MJPEG 有损——共用的是解码/切分路径，不是编码格式。
REPLAY_EXPOSURE_NOTE = (
    "录制帧不带曝光时间戳：文件时间只是主机保存时刻；"
    "回放时间戳是 start_mono_s + i*period_s 的合成占位值，不代表任何真实曝光时刻"
    "（见 test/recordings/PROVENANCE.md）"
)
REPLAY_FORMAT_NOTE = (
    "录制帧是 PNG（无损），真实采集是 MJPEG（有损）。两者共用同一条解码路径："
    "camera_timestamp.decode_stitched 对**整幅**载荷做一次 cv2.imdecode 再按宽度对半切。"
    "单帧条目直接使用文件自身字节（不重编码）；仅左右半图成对时，因必须给出「一整幅」"
    "载荷而水平拼接后重编码为 PNG（无损，不引入有损误差）"
)
V4L2_EXPOSURE_NOTE = (
    "驱动时间戳只保证可追溯到内核 CLOCK_MONOTONIC 时域，不等于曝光中点："
    "压缩/行读出/驱动打戳位置造成的偏移属于待硬件标定项"
)

# 允许的录制帧后缀（大小写不敏感）与半图后缀。
_IMAGE_SUFFIXES = (".png", ".jpg", ".jpeg")
_PAIR_SUFFIXES = ("_a", "_b")
# describe() 用它判断时间基采样是否过旧；不影响取帧本身。
_TIMEBASE_STALE_S = 1.0


@runtime_checkable
class FrameSource(Protocol):
    """帧源协议：真实采集与回放必须可互换（节点里不允许出现 isinstance 分支）。

    约定：一次 `next_frame()` = 一次「底层取帧」= 一个 `StereoFrame`。
    载荷是**一整幅**拼接图，左右目共享该帧唯一的采集时间戳。
    """

    def next_frame(self) -> StereoFrame:
        """取下一帧（除配置设备外不做别的工作；解码仍由 `decode_stereo` 负责）。"""
        ...

    def close(self) -> None:
        """幂等释放；回放源没有设备，只需标记关闭。"""
        ...

    def describe(self) -> dict:
        """诊断快照：格式/协商结果、时钟源、计数器与诚实的边界说明。"""
        ...

    def decode(self, frame: StereoFrame):
        """解码**一整幅**拼接帧并切成 (left, right) 灰度数组。

        为什么放进协议：真机与回放必须共用同一条解码/切分路径（实现是
        `StereoFrameClock.decode_stereo`），消费方不需要知道帧从哪来，
        也不会各自复制一份切分逻辑。
        """
        ...


# ------------------------------------------------------------------ 回放侧


@dataclass(frozen=True)
class RecordedFrame:
    """一条回放条目：源文件与是否由左右半图拼成（诊断用）。"""

    index: int
    files: tuple[str, ...]
    is_pair: bool


@dataclass(frozen=True)
class RecordingSet:
    """目录/文件列表的发现结果。

    `skipped` 是「没有」被当成回放帧的文件及原因（完整路径, 原因），
    必须能被 `describe()` 原样报出去——跳过帧不允许是静默行为。
    """

    entries: tuple[RecordedFrame, ...]
    skipped: tuple[tuple[str, str], ...]
    source: str


def _as_path_list(
    path_or_paths: str | pathlib.Path | Sequence[str | pathlib.Path],
) -> list[pathlib.Path]:
    """目录 / 单文件 / 文件列表 → pathlib 列表（三种输入形式都允许）。"""
    if isinstance(path_or_paths, (str, pathlib.Path)):
        return [pathlib.Path(path_or_paths)]
    return [pathlib.Path(p) for p in path_or_paths]


def _is_half(path: pathlib.Path) -> bool:
    return path.stem.endswith(_PAIR_SUFFIXES)


def _pair_paths(path: pathlib.Path) -> tuple[pathlib.Path, pathlib.Path] | None:
    """`<base>_a` + `<base>_b` 同时存在时返回二者；`_a`/`_b` 输入按 base 归并。"""
    stem = path.stem
    base = stem[:-2] if stem.endswith(_PAIR_SUFFIXES) else stem
    for suffix in _IMAGE_SUFFIXES:
        left = path.parent / f"{base}_a{suffix}"
        right = path.parent / f"{base}_b{suffix}"
        if left.is_file() and right.is_file():
            return left, right
    return None


def _halves_of(path: pathlib.Path) -> list[pathlib.Path]:
    """`<base>` 对应的半图文件（存在则返回），用于说明它们为何被跳过。"""
    stem = path.stem[:-2] if _is_half(path) else path.stem
    found = []
    for suffix in _IMAGE_SUFFIXES:
        for half in (f"{stem}_a{suffix}", f"{stem}_b{suffix}"):
            candidate = path.parent / half
            if candidate.is_file():
                found.append(candidate)
    return found


def _image_files(directory: pathlib.Path) -> list[pathlib.Path]:
    return sorted(
        (p for p in directory.iterdir() if p.is_file() and p.suffix.lower() in _IMAGE_SUFFIXES),
        key=lambda p: p.name,
    )


def discover_recorded_frames(
    path_or_paths: str | pathlib.Path | Sequence[str | pathlib.Path],
) -> RecordingSet:
    """把「目录 / 单个文件 / 显式文件列表」展开成回放条目（只做 stat，不解码）。

    规则（与既有 `stereo_source._collect_files` 的「拼接图或左右图对」一致，但更严格）：

    - `<base>.png`：整幅拼接帧 → 直接用文件自身字节（不重编码）；
    - `<base>_a.png` + `<base>_b.png`：一对半图 → 回放时水平拼成一幅再编码为 PNG；
    - 目录里落单的半图**跳过**并记入 `skipped`：半图不是整幅帧，把它当整幅会静默
      把帧宽减半（切出来的一半再对半切），这种错误在图上几乎看不出来；
    - 整幅帧 `<base>.png` 存在时，它的半图 `<base>_a/_b` 视为派生物，跳过（优先用整幅）；
    - 显式给出的文件必须能作为整幅帧使用：既不是整幅、又找不到配对的半图 → 直接报错，
      而不是悄悄少一帧。

    目录扫描按文件名排序（同一次运行结果可复现）；显式列表保持调用方给的顺序。
    """
    paths = _as_path_list(path_or_paths)
    if not paths:
        raise ValueError("未给出回放输入：需要目录、单个文件或文件列表")
    if len(paths) == 1 and paths[0].is_dir():
        return _discover_directory(paths[0])
    return _discover_explicit(paths)


def _discover_directory(directory: pathlib.Path) -> RecordingSet:
    candidates = _image_files(directory)
    found: list[tuple[str, RecordedFrame]] = []      # (排序键 = 首个文件名, 条目)
    skipped: list[tuple[str, str]] = []
    handled: set[str] = set()

    # 第一遍：整幅帧优先；它对应的半图（<base>_a/_b）视为派生物，不再单独成帧。
    for candidate in candidates:
        if _is_half(candidate) or candidate.name in handled:
            continue
        handled.add(candidate.name)
        found.append((candidate.name, RecordedFrame(len(found), (str(candidate),), False)))
        for half in _halves_of(candidate):
            if half.name in handled:
                continue
            handled.add(half.name)
            skipped.append((str(half), f"整幅帧 {candidate.name} 已存在，半图不再单独成帧"))

    # 第二遍：剩下的半图，只有能配成一对时才作为一条回放帧。
    for candidate in candidates:
        if not _is_half(candidate) or candidate.name in handled:
            continue
        pair = _pair_paths(candidate)
        if pair is None:
            handled.add(candidate.name)
            skipped.append((str(candidate), "落单的半图：找不到 _a/_b 配对"))
            continue
        handled.update(p.name for p in pair)
        found.append((
            pair[0].name,
            RecordedFrame(len(found), (str(pair[0]), str(pair[1])), True),
        ))

    found.sort(key=lambda item: item[0])
    entries = tuple(
        RecordedFrame(index, entry.files, entry.is_pair)
        for index, (_key, entry) in enumerate(found)
    )
    return RecordingSet(entries, tuple(skipped), f"dir:{directory}")


def _discover_explicit(paths: list[pathlib.Path]) -> RecordingSet:
    found: list[RecordedFrame] = []
    handled: set[str] = set()
    for path in paths:
        if not path.is_file():
            raise CameraTimestampError(f"回放输入不存在或不是普通文件：{path}")
        if path.name in handled:
            continue
        if _is_half(path):
            pair = _pair_paths(path)
            if pair is None:
                raise CameraTimestampError(
                    f"{path} 是半图（_a/_b）且找不到配对：回放条目必须是整幅拼接帧，"
                    "左右半图需要 <base>_a 与 <base>_b 同时存在"
                )
            handled.update(p.name for p in pair)
            found.append(RecordedFrame(len(found), (str(pair[0]), str(pair[1])), True))
        else:
            handled.add(path.name)
            found.append(RecordedFrame(len(found), (str(path),), False))
    return RecordingSet(tuple(found), (), "files:" + ",".join(str(p) for p in paths))


def _imread_gray(path: pathlib.Path):
    """半图 → 灰度数组。用 OpenCV 的官方入口，解不出即报错（不猜、不缩放）。"""
    import cv2

    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise CameraTimestampError(f"录制帧无法解码：{path}")
    return image


class _RecordedFrameReader(CameraTimestampSource):
    """回放专用的「假底层源」：只实现 `read_raw()`，**永不打开设备**。

    为什么让它充当 `CameraTimestampSource`：这样回放与真实采集共用同一个
    `StereoFrameClock`——时间戳归算、单调性检查、计数器都只有一份实现，
    「语义相同」由结构保证，而不是靠两处各自遵守约定。

    继承只用于复用 `to_monotonic()` 的时域判定；不调用任何 V4L2 方法，
    `open()` 被显式禁掉，防止有人从回放路径意外打开相机。
    """

    def __init__(self, owner: "ReplayFrameSource") -> None:
        # 不调用 super().__init__：这里没有任何 V4L2 状态，也不申请 fd/缓冲。
        self._owner = owner
        self.device = f"replay:{owner.recording.source}"
        # 回放时间戳是合成的：既不是 REALTIME 也不受 NTP 影响，故不接受 REALTIME 换算。
        self.allow_realtime = False
        self.realtime_uncertainty_limit_s = 0.0
        self.negotiated = {
            "width": owner.stitched_width,
            "height": owner.height,
            "pixelformat": "RECORDED",
            "pixelformat_raw": 0,
        }
        self.streaming = False
        self.reads = 0                     # 诊断：一次 next_frame() 只允许 +1

    def open(self) -> None:                # pragma: no cover - 结构性防线
        raise CameraTimestampError("回放源不打开任何设备：录制帧只从文件读取")

    def read_raw(self, timeout_s: float = 1.0) -> RawFrame:
        """给出一帧「驱动视角」的结果：合成时间戳 + 文件载荷（不阻塞、不涉及设备）。"""
        owner = self._owner
        index = owner.next_index()          # 绝对序号：循环回放时继续累加，时间戳仍递增
        payload = owner.payload_at(index)
        timestamp_s = owner.start_mono_s + index * owner.period_s
        self.reads += 1
        return RawFrame(
            sequence=self.reads,
            data=payload,
            timestamp_s=timestamp_s,
            clock_source=CLOCK_MONOTONIC,   # 合成时间戳声明在单调时域（见 describe）
            flags=V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC,
            # 回放没有「收帧时刻」：填合成时间戳以保证确定性；该字段不进入 StereoFrame。
            received_mono_s=timestamp_s,
            driver_sequence=index,
            buffer_index=index % 4,
            bytes_used=len(payload),
        )


class ReplayExhausted(CameraTimestampError):
    """回放已到末尾（`loop=False`）：调用方据此正常收尾，而不是当成取帧故障。"""


class ReplayFrameSource:
    """录制帧回放源：与真实采集**语义相同**的帧来源（绝不打开设备）。

    语义相同的三条（与 `V4L2FrameSource` 逐条对应）：

    1. 一次 `next_frame()` = 一次底层取帧 = 一个 `StereoFrame`；
    2. 载荷是**一整幅**拼接图，左右目由 `decode_stitched` 按宽度对半切，
       因此两半只可能共享同一个 `capture_ros_s`（`StereoFrame` 只有一个时间字段）；
    3. 时间戳经同一个 `StereoFrameClock` 归算到 ROS 时间域，检查与计数器只有一份实现。

    与真实采集的**不同点**（必须连同 `describe()` 一起读）：

    - 时间戳是合成占位值 `start_mono_s + i * period_s`：录制帧没有曝光时间戳；
    - 载荷是 PNG（无损），真实采集是 MJPEG（有损），共用的是解码/切分路径。

    参数
    ----
    path_or_paths: 目录、单个文件或文件列表（发现规则见 `discover_recorded_frames`）。
    start_mono_s / period_s: 合成时间戳的起点与间隔（`period_s = 1/fps`）。
    timebase: 必填的 `RosTimeBase`；不给默认值是为了不让「ROS 时域 == 单调时域」
        这种错误假设悄悄成立（离线用 `RosTimeBase.from_offset`）。
    """

    def __init__(
        self,
        path_or_paths: str | pathlib.Path | Sequence[str | pathlib.Path],
        *,
        start_mono_s: float,
        period_s: float,
        timebase: RosTimeBase,
        loop: bool = True,
        split: str = "horizontal",
    ) -> None:
        if timebase is None:
            raise ValueError(
                "必须显式给出 timebase（RosTimeBase）：ROS 时域与单调时域的偏移不能默认成 0"
            )
        if period_s <= 0.0:
            raise ValueError(f"period_s 必须为正（1/fps），当前 {period_s}")

        self.recording = discover_recorded_frames(path_or_paths)
        if not self.recording.entries:
            raise CameraTimestampError(f"没有可用录制帧：{self.recording.source}")
        self.timebase = timebase
        self.start_mono_s = float(start_mono_s)
        self.period_s = float(period_s)
        self.loop = bool(loop)
        self.split = split
        self._index = 0                    # 已交付的下一个绝对序号（跨循环继续累加）
        self._cache: tuple[int, bytes] | None = None
        self._last_driver_sequence: int | None = None
        self._closed = False

        # 先读第一帧：尺寸是整条回放链的契约（与真相机一样只有一个协商尺寸），
        # 读不出来或尺寸不合法都要在开始发布前失败，而不是发到一半才发现。
        first = self.recording.entries[0]
        payload, width, height = self._load_payload(first)
        self.stitched_width = width
        self.height = height
        self._check_shape(width, height, first)
        self._cache = (0, payload)

        self._reader = _RecordedFrameReader(self)
        self._clock = StereoFrameClock(
            self._reader, timebase, stitched_width=self.stitched_width, split=split
        )

    # ---------------------------------------------------------------- 取帧

    def next_frame(self) -> StereoFrame:
        if self._closed:
            raise CameraTimestampError("回放源已关闭：请重新构造，不要在关闭后继续取帧")
        frame = self._clock.next_frame()
        self._last_driver_sequence = frame.driver_sequence
        return frame

    def close(self) -> None:
        """回放源没有需要释放的设备：只标记关闭，使后续取帧明确失败。"""
        self._closed = True

    @property
    def clock(self) -> StereoFrameClock:
        """唯一的时间戳归算/解码点（`decode_stereo(frame)` 在这里）。"""
        return self._clock

    def decode(self, frame: StereoFrame):
        """解码并切成左右目：委托给 `StereoFrameClock.decode_stereo`（唯一实现）。"""
        return self._clock.decode_stereo(frame)

    def __enter__(self) -> "ReplayFrameSource":
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # ---------------------------------------------------------------- 内部

    def next_index(self) -> int:
        """下一个绝对帧序号（供 `_RecordedFrameReader` 使用）。"""
        if self._index >= len(self.recording.entries) and not self.loop:
            raise ReplayExhausted(
                f"回放已到末尾：{len(self.recording.entries)} 帧，loop=False"
                f"（源 {self.recording.source}）"
            )
        index = self._index
        self._index += 1
        return index

    def payload_at(self, index: int) -> bytes:
        """按绝对序号取载荷（同一序号只读一次；循环时复用条目但序号继续累加）。"""
        if self._cache is not None and self._cache[0] == index:
            return self._cache[1]
        entry = self.recording.entries[index % len(self.recording.entries)]
        payload, width, height = self._load_payload(entry)
        self._check_shape(width, height, entry)
        self._cache = (index, payload)
        return payload

    def _load_payload(self, entry: RecordedFrame) -> tuple[bytes, int, int]:
        """读出一个条目 → (整幅载荷字节, 宽, 高)。

        - 单帧条目：**原样**读文件字节作为载荷（不重编码、不换格式），尺寸由 OpenCV
          解码得到——与 `decode_stitched` 同一个解码器，避免出现第二套尺寸判据。
        - 半图成对：两半各自解码后水平拼成一幅，再编码为 PNG（无损）作为**一整幅**载荷；
          不是把两幅图的字节首尾相接（那样解不出拼接尺寸，也无法对半切）。
        """
        if not entry.is_pair:
            path = pathlib.Path(entry.files[0])
            data = path.read_bytes()
            image = self._decode(data, path)
            return data, int(image.shape[1]), int(image.shape[0])

        import cv2
        import numpy as np

        left = _imread_gray(pathlib.Path(entry.files[0]))
        right = _imread_gray(pathlib.Path(entry.files[1]))
        if left.shape != right.shape:
            raise CameraTimestampError(
                f"左右半图尺寸不一致：{entry.files[0]} {left.shape[1]}x{left.shape[0]} vs "
                f"{entry.files[1]} {right.shape[1]}x{right.shape[0]}"
            )
        stitched = np.hstack([left, right])
        ok, buf = cv2.imencode(".png", stitched)
        if not ok:                          # pragma: no cover - OpenCV 编码失败极罕见
            raise CameraTimestampError(f"半图拼接后编码失败：{entry.files}")
        return buf.tobytes(), int(stitched.shape[1]), int(stitched.shape[0])

    @staticmethod
    def _decode(data: bytes, where):
        """用与 `decode_stitched` 相同的入口解码，只为拿到尺寸并校验文件可解。"""
        import cv2
        import numpy as np

        image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise CameraTimestampError(f"录制帧无法解码：{where}")
        return image

    def _check_shape(self, width: int, height: int, entry: RecordedFrame) -> None:
        """尺寸契约：一个回放源只允许一个拼接尺寸，且宽必须为偶数（可对半切）。"""
        if self.stitched_width % 2 != 0:
            raise CameraTimestampError(
                f"拼接宽度 {self.stitched_width} 不是偶数：无法对半切左右目（{entry.files}）"
            )
        if (width, height) != (self.stitched_width, self.height):
            raise CameraTimestampError(
                f"回放条目 {entry.files} 的尺寸 {width}x{height} 与首帧 "
                f"{self.stitched_width}x{self.height} 不一致："
                "一个回放源只允许一个拼接尺寸（与真实相机只有一个协商尺寸一致）"
            )

    # ---------------------------------------------------------------- 诊断

    def describe(self) -> dict:
        counters = dict(self._clock.counters)
        return {
            "kind": "replay",
            "replay": True,                       # 明确标记：这不是真实采集
            "source": self.recording.source,
            "frame_count": len(self.recording.entries),   # 回放源里的帧数（有限）
            "next_index": self._index,
            "frames_delivered": counters["frames"],
            "reads": self._reader.reads,          # 应与 frames_delivered 相等（1 帧 = 1 次取帧）
            "driver_sequence": self._last_driver_sequence,
            "loop": self.loop,
            "split": self.split,
            "start_mono_s": self.start_mono_s,
            "period_s": self.period_s,
            "stitched_width": self.stitched_width,
            "height": self.height,
            "half_width": self.stitched_width // 2,
            "clock_source": CLOCK_MONOTONIC,
            "clock_source_note": "合成时间戳声明在单调时域：既不是驱动给出的，也不是曝光时刻",
            "device_opened": False,               # 回放路径从不打开设备
            "closed": self._closed,
            "counters": counters,
            "timebase": self.timebase.stability(_TIMEBASE_STALE_S),
            "skipped": [
                {"file": name, "reason": reason} for name, reason in self.recording.skipped
            ],
            "exposure_note": REPLAY_EXPOSURE_NOTE,
            "format_note": REPLAY_FORMAT_NOTE,
        }


# ------------------------------------------------------------------ 真实采集侧


class V4L2FrameSource:
    """真实采集源：**唯一**允许打开相机的地方（V4L2 MMAP + 驱动时间戳）。

    行为约定（与回放源逐条一致）：

    - 一次 `next_frame()` 只做一次底层取帧，返回一个拼接帧（不做解码）；
    - 时间戳全部交给 `StereoFrameClock` 归算，本类不自己算时间、不自己判定时域；
    - 任何「驱动没给出可核实结果」的情况一律 `CameraTimestampError`：
      不静默回退到收帧时刻、不发布看似有效的帧（真实链路必须保持禁用）。

    参数里的 `width`/`height` 是**拼接尺寸**（例如 1280x480）：驱动协商结果与它不一致
    就直接失败，因为按错误尺寸切分会把左右目串位，而串位造成的深度/位姿错误极难察觉。
    """

    def __init__(
        self,
        device: str,
        width: int,
        height: int,
        fps: int = 60,
        pixel_format: int = V4L2_PIX_FMT_MJPEG,
        buffer_count: int = 4,
        timebase: RosTimeBase | None = None,
        allow_realtime: bool = False,
        realtime_uncertainty_limit_s: float = 0.002,
        split: str = "horizontal",
    ) -> None:
        if timebase is None:
            raise ValueError(
                "必须显式给出 timebase（RosTimeBase）：ROS 时域与单调时域的偏移不能默认成 0，"
                "否则发布的时间戳会与真实 ROS 时钟相差一个未知常量"
            )
        self.device = str(device)
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.pixel_format = int(pixel_format)
        self.buffer_count = max(int(buffer_count), 2)
        self.allow_realtime = bool(allow_realtime)
        self.realtime_uncertainty_limit_s = float(realtime_uncertainty_limit_s)
        self.split = split
        self.timebase = timebase

        self._source = CameraTimestampSource(
            self.device,
            self.width,
            self.height,
            fps=self.fps,
            pixel_format=self.pixel_format,
            buffer_count=self.buffer_count,
            allow_realtime=self.allow_realtime,
            realtime_uncertainty_limit_s=self.realtime_uncertainty_limit_s,
        )
        # 拼接宽度显式交给时钟：帧上带的尺寸就是「请求/协商一致的拼接尺寸」。
        self._clock = StereoFrameClock(
            self._source, timebase, stitched_width=self.width, split=split
        )
        self._opened = False
        self._closed = False
        self._fetch_errors = 0
        self._last_clock_source: str | None = None

    # ---------------------------------------------------------------- 生命周期

    def open(self) -> None:
        """打开并配置设备（幂等）。`next_frame()` 会按需调用它。

        显式调用一次的好处：配置错误（设备不存在、尺寸或格式协商不符）在启动时就暴露，
        而不是等到第一帧发布才失败。
        """
        if self._closed:
            raise CameraTimestampError(
                "帧源已关闭：不允许重新打开设备（避免节点关闭后误重连相机）"
            )
        if self._opened:
            return
        # 唯一打开设备的地方：真实相机只在这里出现一次。
        self._source.open()
        try:
            self._verify_negotiated()
        except Exception:
            # 协商不符也要把 fd/缓冲还回去，不留一台半开的相机。
            self._source.close()
            raise
        self._opened = True

    def _verify_negotiated(self) -> None:
        """复核驱动**实际**协商结果；不符即报错，不做静默适配。

        `CameraTimestampSource._s_fmt()` 已经查过一次尺寸，这里再查一次是因为：协商不符
        会以「尺寸不一致」甚至「解码失败」的形式在**解码阶段**迟到报错，错误信息离原因很远；
        像素格式不符（例如请求 MJPG 却给了 YUYV）时整幅载荷根本不是图像，同样在这里显式失败。
        """
        negotiated = dict(self._source.negotiated)
        got_size = (negotiated.get("width"), negotiated.get("height"))
        if got_size != (self.width, self.height):
            raise CameraTimestampError(
                f"{self.device} 协商尺寸 {got_size[0]}x{got_size[1]} 与请求 "
                f"{self.width}x{self.height} 不一致：拒绝按错误尺寸切分左右目"
            )
        want_format = PIXEL_FORMAT_NAMES.get(self.pixel_format, hex(self.pixel_format))
        if negotiated.get("pixelformat_raw") != self.pixel_format:
            raise CameraTimestampError(
                f"{self.device} 协商像素格式 {negotiated.get('pixelformat')} 与请求 "
                f"{want_format} 不一致：整幅载荷将不是可解码的拼接帧"
            )

    def close(self) -> None:
        """幂等关闭：停流、释放缓冲与 fd（唯一打开点的对应释放点）。"""
        if self._closed:
            return
        self._source.close()
        self._opened = False
        self._closed = True

    @property
    def clock(self) -> StereoFrameClock:
        """唯一的时间戳归算/解码点（`decode_stereo(frame)` 在这里）。"""
        return self._clock

    def decode(self, frame: StereoFrame):
        """解码并切成左右目：委托给 `StereoFrameClock.decode_stereo`（唯一实现）。

        与回放源同一份实现：真机与回放的差别只在「帧从哪来」。
        """
        return self._clock.decode_stereo(frame)

    def __enter__(self) -> "V4L2FrameSource":
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # ---------------------------------------------------------------- 取帧

    def next_frame(self) -> StereoFrame:
        """取一帧真实采集结果。

        失败一律抛 `CameraTimestampError`：时域不可核实、时间戳倒退、取帧超时、
        缓冲索引越界等都由底层显式报错；这里只计数，不做任何掩盖。
        """
        if self._closed:
            raise CameraTimestampError("帧源已关闭：不允许继续取帧")
        if not self._opened:
            self.open()
        try:
            frame = self._clock.next_frame()
        except CameraTimestampError:
            self._fetch_errors += 1
            raise
        self._last_clock_source = frame.clock_source
        return frame

    # ---------------------------------------------------------------- 诊断

    def describe(self) -> dict:
        negotiated = dict(self._source.negotiated)
        want_format = PIXEL_FORMAT_NAMES.get(self.pixel_format, hex(self.pixel_format))
        return {
            "kind": "v4l2",
            "replay": False,
            "device": self.device,
            # 真实采集没有「有限的帧数」（帧数只增不减）；键存在是为了两种 describe()
            # 形状一致，消费方不必按类型分支。
            "frame_count": None,
            "requested": {
                "width": self.width,
                "height": self.height,
                "fps": self.fps,
                "pixelformat": want_format,
                "buffer_count": self.buffer_count,
            },
            "negotiated": negotiated,          # 驱动实际给出的尺寸/格式（open 后才有）
            "clock_source": self._last_clock_source,   # 最近一帧实际观察到的驱动时域
            "clock_source_note": (
                "来自 DQBUF 的 v4l2_buffer.flags；不可核实时取帧直接报错，不回退到收帧时刻"
            ),
            "allow_realtime": self.allow_realtime,
            "split": self.split,
            "opened": self._opened,
            "closed": self._closed,
            "streaming": self._source.streaming,
            "frames_delivered": self._clock.counters["frames"],
            "fetch_errors": self._fetch_errors,        # 取帧失败次数（含时域不可核实）
            "counters": dict(self._clock.counters),
            "timebase": self.timebase.stability(_TIMEBASE_STALE_S),
            "exposure_note": V4L2_EXPOSURE_NOTE,
        }
