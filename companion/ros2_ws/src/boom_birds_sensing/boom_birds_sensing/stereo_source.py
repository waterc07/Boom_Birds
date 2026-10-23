"""V4L2、记录帧回放与文件输入的双目采集和发布基类。

左右图和 CameraInfo 共用采集时间戳。合成输入由 boom_birds_sim 的子类提供，
发布、标定缩放和采集线程仍由本类实现。
"""

from __future__ import annotations
from boom_birds_control.runtime_config import DEFAULTS

import array
import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Header

from stereo_depth.orientation import SENSOR, imageOrientation, cameraMatrix, orientPair
from boom_birds_sensing.timebase import RosTimeBase
from boom_birds_sensing.camera_geometry import resizeRawPair, scaledRawSize

QOS_IMAGE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST)

# 支持的输入模式；`camera_timestamp` 那套只在真机/回放模式里用到，按需导入。
MODES = ("v4l2", "replay", "file")


def _loadCalibration(path: str) -> dict:
    """读取标定 npz；缺文件必须显式失败，不允许静默占位。"""
    calib = Path(path)
    if not calib.is_file():
        raise RuntimeError(
            f"标定文件不存在：{calib}。真机/回放模式必须提供标定，否则内参与基线不可信"
        )
    with np.load(calib) as archive:
        return {key: archive[key].copy() for key in archive.files}


def rawCameraInfo(calibration: dict, side: str, frame_id: str, width: int, height: int,
                    *, scale: float | str = "auto"):
    """由标定推导**原始**（未校正）某一目的 CameraInfo。

    标定里的 K1/K2、D1/D2 是标定原图尺度下的参数。采集分辨率与标定 image_size 不一致时
    **不能静默**按同一套内参发布：

    - `scale="auto"`：按 `width / 标定宽度` 缩放 fx/fy/cx/cy，畸变系数不变。
      **物理基线不缩放**：基线是米制刚体量，纯降采样不改变它；只有内参（像素量）随分辨率变。
      基线错误缩放会让 `P[2][3] = -fx·B` 以 scale 的平方衰减（0.5 倍尺寸下变成 1/4），
      下游三角化/深度尺度会整体错掉。
    - `scale=1.0`：要求尺寸完全一致，否则报错（真机建议用这个，把标定不匹配暴露出来）。

    实测现状（2026-09-23）：项目现有标定 `image_size=1280×960`，而记录帧与 stereo_depth
    采集链按每目 640×480 工作，比值 0.5——即当前默认会走缩放路径。这属于**跨分辨率复用
    内参**，必须在真机按实际分辨率重新标定确认后才能用于定位。

    双目外参：`T` 是右目相对左目的平移（标定约定 A→B 正视差水平双目，`T[0] < 0`）。
    CameraInfo 的 `P` 用 `[-fx·B, 0, cx]` 表达基线，`B` 是**米制物理基线**（不随缩放变化），
    `fx` 用当前发布分辨率下的值；`R` 为单位阵（原始图未校正）。

    返回值里的 `baseline_m` 始终是**物理基线**（米），不是像素量。
    """
    imageSize = tuple(int(v) for v in calibration["image_size"])
    if side not in ("left", "right"):
        raise ValueError(f"side 只能是 left/right，收到 {side}")

    nominal = float(width) / float(imageSize[0])
    if isinstance(scale, str):
        if scale != "auto":
            raise ValueError(f"scale 只能是 'auto' 或数值，收到 {scale!r}")
        scaleFactor = nominal
    else:
        scaleFactor = float(scale)
        if abs(scaleFactor - 1.0) > 1e-9 and abs(nominal - scaleFactor) > 1e-6:
            raise RuntimeError(
                f"显式 scale={scaleFactor} 与采集/标定宽度比 {nominal:.6f} 不一致"
            )
        if abs(scaleFactor - 1.0) <= 1e-9 and (width, height) != imageSize:
            raise RuntimeError(
                f"采集尺寸 {width}x{height} 与标定 image_size {imageSize} 不一致，"
                "且未允许缩放（scale=1.0）：原始内参不能跨分辨率复用"
            )
    scaled = abs(scaleFactor - 1.0) > 1e-9

    k = np.asarray(calibration["K1" if side == "left" else "K2"], dtype=float).reshape(3, 3)
    d = np.asarray(calibration["D1" if side == "left" else "D2"], dtype=float).ravel()
    t = np.asarray(calibration["T"], dtype=float).ravel()
    baselineM = float(np.linalg.norm(t))
    if not np.isclose(height / imageSize[1], scaleFactor, atol=1e-9, rtol=0):
        raise ValueError("原图尺寸必须与标定等比例")
    k = cameraMatrix(calibration, 1 if side == "left" else 2, (width, height))

    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width = int(width)
    info.height = int(height)
    info.distortion_model = "plumb_bob"
    info.d = [float(v) for v in d]
    info.k = [float(v) for v in k.ravel()]
    # 原始图未校正：R 为单位阵，P 与 K 同尺度并带基线项
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    fx, fy = float(k[0, 0]), float(k[1, 1])
    cx, cy = float(k[0, 2]), float(k[1, 2])
    tx = 0.0 if side == "left" else -fx * baselineM
    info.p = [fx, 0.0, cx, tx, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info, baselineM, scaleFactor


class StereoSourceNode(Node):
    """四种输入模式的唯一采集/发布节点。"""

    MODES = MODES

    def __init__(self, defaultMode="v4l2") -> None:
        super().__init__("boom_birds_stereo_source")

        # ---- 模式与输入 ----
        self.declare_parameter("mode", defaultMode)                 # v4l2 | replay | file
        self.declare_parameter("path", "")                      # file/replay：拼接图或目录
        self.declare_parameter("rate_hz", 5.0)                  # 发布上限（也是合成/回放节奏）
        self.declare_parameter("loop", True)                    # 回放/文件模式是否循环
        self.declare_parameter("opencv_threads", 0)
        self.declare_parameter("image_queue_depth", 10)
        self.declare_parameter("publish_queue_depth", 2)        # 采集与发布解耦的队列深度

        # ---- 真机采集（v4l2）----
        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("capture_width", 1280)           # 拼接帧宽度（含左右两目）
        self.declare_parameter("capture_height", 480)
        self.declare_parameter("capture_fps", 60)
        self.declare_parameter("capture_buffers", 4)
        self.declare_parameter("allow_realtime_timestamp", False)
        self.declare_parameter("realtime_uncertainty_limit_s", 0.002)
        self.declare_parameter("capture_timeout_s", 1.0)

        # ---- 回放 ----
        self.declare_parameter("replay_fps", 30.0)
        self.declare_parameter("replay_start_mono_s", -1.0)     # <0：用 ROS 时钟当前值

        # ---- 标定与话题 ----
        self.declare_parameter("calibration_file", "")
        # 原始 CameraInfo 的尺寸策略："auto" 按 采集宽度/标定宽度 缩放内参；
        # 1.0 要求尺寸完全一致（真机建议，逼出标定不匹配）。两者都不是"随便糊过去"。
        self.declare_parameter("raw_info_scale", "auto")
        self.declare_parameter("raw_output_scale", 1.0)
        self.declare_parameter("raw_decode_divisor", 1)
        # 设备/旧记录为 sensor_ab；已转正的图像回放必须显式声明，避免二次转换。
        self.declare_parameter("input_orientation", SENSOR)
        self.declare_parameter("left_topic", DEFAULTS.stereo_left_topic)
        self.declare_parameter("right_topic", DEFAULTS.stereo_right_topic)
        self.declare_parameter("stitched_topic", DEFAULTS.stereo_stitched_topic)
        # 左右 CameraInfo 各自与图像配对（ROS 惯例：一个相机一个 camera_info 话题）。
        # 留空则由对应的图像话题派生：<image_topic>/camera_info。
        self.declare_parameter("left_camera_info_topic", "")
        self.declare_parameter("right_camera_info_topic", "")
        self.declare_parameter("camera_info_topic_suffix", "camera_info")
        # 旧版把左右轮流发在同一个话题上。保留为**可选**（默认关闭），
        # 避免已有订阅者被静默破坏；新代码不要用它。
        self.declare_parameter("legacy_combined_camera_info_topic", "")
        self.declare_parameter("frame_id_left", DEFAULTS.raw_left_frame)
        self.declare_parameter("frame_id_right", DEFAULTS.raw_right_frame)
        self.declare_parameter("frame_id_stitched", DEFAULTS.raw_left_frame)
        self.declare_parameter("publish_stitched", True)
        self.declare_parameter("publish_mjpeg", False)
        self.declare_parameter("publish_raw_without_subscribers", True)
        self.declare_parameter("mjpeg_topic", DEFAULTS.stereo_stitched_topic + "/compressed")

        self._declareSourceParameters()

        # ---- 诊断 ----
        self.declare_parameter("stats_topic", DEFAULTS.stereo_status_topic)
        self.declare_parameter("stats_rate_hz", 1.0)

        self.mode = str(self.get_parameter("mode").value)
        if self.mode not in self.MODES:
            raise RuntimeError(f"未知 mode：{self.mode}（可用：{', '.join(self.MODES)}）")

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.frameLeft = str(self.get_parameter("frame_id_left").value)
        self.frameRight = str(self.get_parameter("frame_id_right").value)
        self.frameStitched = str(self.get_parameter("frame_id_stitched").value)
        self.bridge = CvBridge()
        self.decoded_sink = None
        self.raw_output_scale = float(self.get_parameter("raw_output_scale").value)
        scaledRawSize(1280, 960, self.raw_output_scale)
        self.input_orientation = str(self.get_parameter("input_orientation").value)
        orientPair(np.zeros((1, 1), np.uint8), np.zeros((1, 1), np.uint8),
                    self.input_orientation, SENSOR)
        self.raw_decode_divisor = int(self.get_parameter("raw_decode_divisor").value)
        if self.raw_decode_divisor not in (1, 2, 4):
            raise ValueError("raw_decode_divisor must be 1, 2, or 4")
        if self.mode not in ("v4l2", "replay") and self.raw_decode_divisor != 1:
            raise ValueError("raw_decode_divisor requires a JPEG capture or replay source")
        if self.raw_output_scale * self.raw_decode_divisor > 1:
            raise ValueError("raw_decode_divisor must not undersample the requested raw output")

        # 采集与发布解耦：采集线程只保留最近 N 帧，处理慢时丢旧帧而不是无限积压。
        depth = max(int(self.get_parameter("publish_queue_depth").value), 1)
        self._queue: list = []
        self._queueLock = threading.Lock()
        self._stop = threading.Event()
        self._readerThread: threading.Thread | None = None

        self.counters = {
            "frames_captured": 0,
            "frames_published": 0,
            "dropped_backlog": 0,
            "timestamp_rejected": 0,
            "capture_errors": 0,
            "last_error": None,
            "last_capture_mono_s": None,
            "last_capture_ros_s": None,
            "last_capture_uncertainty_s": None,
            "timestamp_note": (
                "v4l2 模式：时间戳来自 V4L2 驱动并映射到 ROS 时域，不是收包/发布时间；"
                "replay 模式：时间戳由回放器合成（记录帧没有曝光时间戳）"
            ),
        }
        self._queueDepth = depth

        # ---- 标定（v4l2/replay 必需）----
        self.calibration: dict | None = None
        self.baseline_m: float | None = None
        self._scaleWarned = False
        if self.mode in ("v4l2", "replay") or str(self.get_parameter("calibration_file").value):
            calibFile = str(self.get_parameter("calibration_file").value)
            self.calibration = _loadCalibration(calibFile)
            self.get_logger().info(f"已加载标定：{calibFile}")

        cvThreads = int(self.get_parameter("opencv_threads").value)
        if cvThreads < 0:
            raise ValueError("opencv_threads must be nonnegative")
        if cvThreads:
            cv2.setNumThreads(cvThreads)
        imageDepth = int(self.get_parameter("image_queue_depth").value)
        if imageDepth < 1:
            raise ValueError("image_queue_depth 必须为正整数")
        imageQos = QoSProfile(depth=imageDepth, reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST)

        # ---- 发布器 ----
        self.pub_left = self.create_publisher(Image, self.get_parameter("left_topic").value, imageQos)
        self.pub_right = self.create_publisher(Image, self.get_parameter("right_topic").value, imageQos)
        self.pub_mjpeg = self.create_publisher(
            CompressedImage, self.get_parameter("mjpeg_topic").value, imageQos)
        self.pub_stitched = self.create_publisher(
            Image, self.get_parameter("stitched_topic").value, imageQos
        )
        suffix = str(self.get_parameter("camera_info_topic_suffix").value).strip("/")

        def _infoTopic(explicitParam: str, imageParam: str) -> str:
            explicit = str(self.get_parameter(explicitParam).value).strip()
            if explicit:
                return explicit
            image = str(self.get_parameter(imageParam).value).rstrip("/")
            return f"{image}/{suffix}" if suffix else image

        self.infoTopicLeft = _infoTopic("left_camera_info_topic", "left_topic")
        self.infoTopicRight = _infoTopic("right_camera_info_topic", "right_topic")
        self.pubInfoLeft = self.create_publisher(CameraInfo, self.infoTopicLeft, imageQos)
        self.pubInfoRight = self.create_publisher(CameraInfo, self.infoTopicRight, imageQos)

        legacy = str(self.get_parameter("legacy_combined_camera_info_topic").value).strip()
        self.pubInfoLegacy = (
            self.create_publisher(CameraInfo, legacy, imageQos) if legacy else None
        )
        from std_msgs.msg import String

        self.pubStats = self.create_publisher(String, self.get_parameter("stats_topic").value, 10)

        # ---- 输入源 ----
        self.frame_source = None
        self._synthetic = None
        self._filePairs: list = []
        self._setupSource()
        self._setupSourceSubscriptions()

        self.timer = self.create_timer(1.0 / max(self.rate_hz, 0.1), self.tick)
        self.create_timer(1.0 / max(float(self.get_parameter("stats_rate_hz").value), 1e-3),
                          self.publishStats)
        self.get_logger().info(
            f"stereo_source 启动：mode={self.mode} rate={self.rate_hz}Hz "
            f"left={self.pub_left.topic_name} right={self.pub_right.topic_name} "
            f"left_info={self.infoTopicLeft} right_info={self.infoTopicRight}"
            + (f" legacy_info={self.pubInfoLegacy.topic_name}"
               if self.pubInfoLegacy is not None else "")
        )
        if self.mode in ("v4l2", "replay"):
            self.get_logger().warn(
                "v4l2/replay 模式的图像时间戳只保证「可追溯到采集/回放时刻」；"
                "V4L2 时间戳与真实曝光时刻的偏差仍需实机标定，不得当作已验证的曝光时间"
            )

    # ------------------------------------------------------------------ 输入源

    def _declareSourceParameters(self) -> None:
        pass

    def _setupSourceSubscriptions(self) -> None:
        pass

    def _setupSource(self) -> None:
        if self.mode == "v4l2":
            self._setupV4l2()
        elif self.mode == "replay":
            self._setupReplay()
        elif self.mode == "file":
            self._setupFile()
        else:
            raise RuntimeError("合成输入请使用 boom_birds_sim/synthetic_stereo_source")

    def _setupV4l2(self) -> None:
        from boom_birds_sensing.stereo_capture import V4L2FrameSource

        device = str(self.get_parameter("device").value)
        width = int(self.get_parameter("capture_width").value)
        height = int(self.get_parameter("capture_height").value)
        self.frame_source = V4L2FrameSource(
            device=device,
            width=width,
            height=height,
            fps=int(self.get_parameter("capture_fps").value),
            buffer_count=int(self.get_parameter("capture_buffers").value),
            timebase=RosTimeBase(self.get_clock()),
            allow_realtime=bool(self.get_parameter("allow_realtime_timestamp").value),
            realtime_uncertainty_limit_s=float(
                self.get_parameter("realtime_uncertainty_limit_s").value
            ),
            timeout_s=float(self.get_parameter("capture_timeout_s").value),
        )
        self.frame_source.open()
        self.get_logger().info(
            f"V4L2 已打开：{device} {width}x{height} "
            f"协商结果={self.frame_source.describe().get('negotiated')}"
        )

    def _setupReplay(self) -> None:
        from boom_birds_sensing.stereo_capture import ReplayFrameSource

        path = str(self.get_parameter("path").value)
        if not path:
            raise RuntimeError("replay 模式需要 path（拼接图、目录，或 _a/_b 图对所在目录）")
        startMono = float(self.get_parameter("replay_start_mono_s").value)
        if startMono < 0.0:
            startMono = time.monotonic()
        self.frame_source = ReplayFrameSource(
            path,
            timebase=RosTimeBase(self.get_clock()),
            start_mono_s=startMono,
            period_s=1.0 / max(float(self.get_parameter("replay_fps").value), 1e-3),
            loop=self.loop,
        )
        self.get_logger().info(
            f"回放源已加载：{path}（{self.frame_source.describe().get('frame_count')} 帧，"
            "时间戳由回放器合成，记录帧不含曝光时间戳）"
        )

    def _setupFile(self) -> None:
        import glob

        path = str(self.get_parameter("path").value)
        if not path:
            raise RuntimeError("file 模式需要 path（拼接图或目录）")
        if os.path.isdir(path):
            cands = sorted(glob.glob(os.path.join(path, "*.png"))
                           + glob.glob(os.path.join(path, "*.jpg")))
            for c in cands:
                base = os.path.splitext(c)[0]
                left = base + "_a.png" if os.path.exists(base + "_a.png") else c
                right = base + "_b.png" if os.path.exists(base + "_b.png") else c
                self._filePairs.append((left, right, c))
        else:
            self._filePairs.append((path, path, path))
        if not self._filePairs:
            raise RuntimeError("file 模式未找到可用输入（需拼接图 *.png/*.jpg 或目录）")
        self.get_logger().info(f"file 模式：{len(self._filePairs)} 帧")


    # ------------------------------------------------------------------ 采集线程

    def startReader(self) -> None:
        """v4l2/replay 的取帧放到独立线程：`readRaw()` 会阻塞，不能占用执行器。"""
        if self.frame_source is None:
            return
        self._readerThread = threading.Thread(target=self._readerLoop, name="stereo_reader",
                                               daemon=True)
        self._readerThread.start()

    def _readerLoop(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self.frame_source.nextFrame()
            except Exception as exc:  # noqa: BLE001 - 采集失败必须可见
                self.counters["capture_errors"] += 1
                self.counters["last_error"] = f"{type(exc).__name__}: {exc}"
                self.get_logger().warn(f"取帧失败（已拒绝发布该帧）：{exc}", throttle_duration_sec=2.0)
                time.sleep(0.05)
                continue
            self.counters["frames_captured"] += 1
            self.counters["last_capture_mono_s"] = frame.capture_mono_s
            self.counters["last_capture_ros_s"] = frame.capture_ros_s
            self.counters["last_capture_uncertainty_s"] = frame.capture_uncertainty_s
            with self._queueLock:
                self._queue.append(frame)
                while len(self._queue) > self._queueDepth:
                    self._queue.pop(0)
                    self.counters["dropped_backlog"] += 1

    def _takeFrame(self):
        with self._queueLock:
            if not self._queue:
                return None
            # 在线优先新鲜度，取最新帧并清积压；回放按 FIFO 保持剩余帧的采样顺序。
            if self.mode == "v4l2":
                frame = self._queue.pop()
                self.counters["dropped_backlog"] += len(self._queue)
                self._queue.clear()
                return frame
            return self._queue.pop(0)

    # ------------------------------------------------------------------ 发布

    def tick(self) -> None:
        if self.mode in ("v4l2", "replay"):
            self._tickCaptured()
        else:
            self._tickOffline()

    def _tickCaptured(self) -> None:
        """真机/回放：只发布采集线程交付的帧，时间戳完全沿用采集时间戳。"""
        frame = self._takeFrame()
        if frame is None:
            return
        stamp = _stampFromSeconds(frame.capture_ros_s)
        if bool(self.get_parameter("publish_mjpeg").value):
            packet = frame.stitched
            sourceOrientation = getattr(self, "input_orientation", SENSOR)
            targetOrientation = imageOrientation(getattr(self, "calibration", None) or {})
            if sourceOrientation != targetOrientation:
                # 压缩话题与左右 Image 使用同一方向；原设备 JPEG 只保留在采集记录。
                # 完整解码保持分辨率，避免下游减量解码再次降低采样尺寸。
                a, b = self.frame_source.decode(frame)
                a, b = orientPair(a, b, sourceOrientation, targetOrientation)
                packet = np.hstack([a, b])
            if isinstance(packet, np.ndarray):
                ok, packet = cv2.imencode(".jpg", packet)
                if not ok:
                    raise RuntimeError("MJPEG replay encoding failed")
                packet = packet.tobytes()
            msg = CompressedImage()
            msg.header = self._header(stamp, self.frameStitched)
            msg.format = "jpeg; boombirds_orientation=" + targetOrientation
            data = array.array("B")
            data.frombytes(packet)
            msg.data = data
            self.pub_mjpeg.publish(msg)

        rawPublishers = [self.pub_left, self.pub_right]
        if bool(self.get_parameter("publish_stitched").value):
            rawPublishers.append(self.pub_stitched)
        forceRaw = bool(self.get_parameter("publish_raw_without_subscribers").value)
        wantRaw = forceRaw or any(
            publisher.get_subscription_count() > 0 for publisher in rawPublishers)
        decodedSink = getattr(self, "decoded_sink", None)
        if not wantRaw and decodedSink is None:
            width, height = scaledRawSize(frame.stitched_width // 2, frame.height, self.raw_output_scale)
            self._publishCameraInfo(stamp, width, height)
        else:
            try:
                if self.raw_decode_divisor == 1:
                    left, right = self.frame_source.decode(frame)
                else:
                    left, right = self.frame_source.decode(frame, decode_divisor=self.raw_decode_divisor)
                targetOrientation = imageOrientation(getattr(self, "calibration", None) or {})
                left, right = orientPair(left, right, getattr(self, "input_orientation", SENSOR), targetOrientation)
                decodedLeft, decodedRight = left, right
                left, right = resizeRawPair(left, right, self.raw_output_scale * self.raw_decode_divisor)
            except Exception as exc:  # noqa: BLE001
                self.counters["capture_errors"] += 1
                self.counters["last_error"] = f"decode: {exc}"
                self.get_logger().warn(f"拼接帧解码失败（拒绝发布）：{exc}", throttle_duration_sec=2.0)
                return

            if decodedSink is not None:
                decodedSink(decodedLeft, decodedRight, stamp)
            header = Header()
            header.stamp = _stampFromSeconds(frame.capture_ros_s)
            header.frame_id = self.frameLeft
            if forceRaw or self.pub_left.get_subscription_count() > 0:
                self.pub_left.publish(self.bridge.cv2_to_imgmsg(np.ascontiguousarray(left), encoding="mono8", header=header))
            rightHeader = Header()
            rightHeader.stamp = header.stamp          # 同一帧：左右共享同一时间戳
            rightHeader.frame_id = self.frameRight
            if forceRaw or self.pub_right.get_subscription_count() > 0:
                self.pub_right.publish(self.bridge.cv2_to_imgmsg(np.ascontiguousarray(right), encoding="mono8", header=rightHeader))
            if bool(self.get_parameter("publish_stitched").value):
                st = Header()
                st.stamp = header.stamp
                st.frame_id = self.frameStitched
                self.pub_stitched.publish(
                    self.bridge.cv2_to_imgmsg(np.hstack([left, right]), encoding="mono8", header=st)
                )
            self._publishCameraInfo(header.stamp, left.shape[1], left.shape[0])
        self.counters["frames_published"] += 1

    def _tickOffline(self) -> None:
        """文件和子类输入沿用逐帧输入时间戳。"""
        try:
            left, right, stitched = self._loadOffline()
            left, right = orientPair(left, right, getattr(self, "input_orientation", SENSOR),
                                      imageOrientation(getattr(self, "calibration", None) or {}))
            left, right = resizeRawPair(left, right, self.raw_output_scale)
            if self.raw_output_scale != 1.0 or imageOrientation(getattr(self, "calibration", None) or {}) != SENSOR:
                stitched = np.hstack([left, right])
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"取帧失败：{exc}", throttle_duration_sec=2.0)
            return
        stamp = _stampFromSeconds(self._offlineCaptureRosS)
        self.pub_left.publish(self.bridge.cv2_to_imgmsg(left, encoding="mono8",
                                                        header=self._header(stamp, self.frameLeft)))
        self.pub_right.publish(self.bridge.cv2_to_imgmsg(right, encoding="mono8",
                                                         header=self._header(stamp, self.frameRight)))
        if bool(self.get_parameter("publish_stitched").value):
            self.pub_stitched.publish(self.bridge.cv2_to_imgmsg(
                stitched, encoding="mono8", header=self._header(stamp, self.frameStitched)
            ))
        self._publishCameraInfo(stamp, left.shape[1], left.shape[0])
        self.counters["frames_published"] += 1
        self.counters["frames_captured"] += 1
        self._offlineSeq = getattr(self, "_offlineSeq", 0) + 1



    def _loadOffline(self):
        self._offlineCaptureRosS = self.get_clock().now().nanoseconds * 1e-9
        seq = getattr(self, "_offlineSeq", 0)
        if self.loop:
            leftPath, rightPath, stitchedPath = self._filePairs[seq % len(self._filePairs)]
        else:
            if seq >= len(self._filePairs):
                raise RuntimeError("文件回放已到末尾（loop=false）")
            leftPath, rightPath, stitchedPath = self._filePairs[seq]
        stitched = cv2.imread(stitchedPath, cv2.IMREAD_GRAYSCALE)
        if stitched is None:
            raise RuntimeError(f"无法读取图像：{stitchedPath}")
        if leftPath == rightPath:
            half = stitched.shape[1] // 2
            return stitched[:, :half], stitched[:, half:], stitched
        left = cv2.imread(leftPath, cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(rightPath, cv2.IMREAD_GRAYSCALE)
        if left is None or right is None:
            raise RuntimeError(f"无法读取图像对：{leftPath} / {rightPath}")
        return left, right, np.hstack([left, right])

    def _publishCameraInfo(self, stamp, width: int, height: int) -> None:
        """左右各发一条 CameraInfo，各自发到与图像配对的话题；缺标定时不发并给出诊断。"""
        if self.calibration is None:
            return
        for side, frameId, pub in (
            ("left", self.frameLeft, self.pubInfoLeft),
            ("right", self.frameRight, self.pubInfoRight),
        ):
            try:
                info, baseline, scale = rawCameraInfo(
                    self.calibration, side, frameId, width, height,
                    scale=str(self.get_parameter("raw_info_scale").value),
                )
            except (RuntimeError, ValueError) as exc:
                self.counters["last_error"] = f"camera_info: {exc}"
                self.get_logger().warn(str(exc), throttle_duration_sec=10.0)
                return
            info.header.stamp = stamp
            pub.publish(info)
            if self.pubInfoLegacy is not None:
                self.pubInfoLegacy.publish(info)
            self.baseline_m = baseline
            self.counters["raw_info_scale"] = scale
            if abs(scale - 1.0) > 1e-9 and not self._scaleWarned:
                self._scaleWarned = True
                self.get_logger().warn(
                    f"采集尺寸与标定 image_size 不一致，已按 {scale:.6f} 缩放原始内参："
                    "这是跨分辨率复用内参，只有在「小尺寸是大尺寸的纯降采样且镜头/裁剪一致」"
                    "时才近似成立。真机应在此分辨率下重新标定后再用于定位。"
                )

    @staticmethod
    def _header(stamp, frame_id: str):
        h = Header()
        h.stamp = stamp
        h.frame_id = frame_id
        return h

    # ------------------------------------------------------------------ 诊断

    def publishStats(self) -> None:
        import json

        from std_msgs.msg import String

        stats = dict(self.counters)
        stats["mode"] = self.mode
        stats["image_orientation"] = imageOrientation(self.calibration or {})
        stats["input_orientation"] = self.input_orientation
        stats["compressed_image_orientation"] = imageOrientation(self.calibration or {})
        stats["queue_len"] = len(self._queue)
        stats["baseline_m"] = self.baseline_m
        stats["left_camera_info_topic"] = self.infoTopicLeft
        stats["right_camera_info_topic"] = self.infoTopicRight
        stats["legacy_combined_camera_info"] = (
            None if self.pubInfoLegacy is None else self.pubInfoLegacy.topic_name
        )
        stats.setdefault("raw_info_scale", None)
        stats["pose_depth_owner"] = "depth_core.rectified_camera_info（P1，320×240）"
        if self.frame_source is not None and hasattr(self.frame_source, "describe"):
            try:
                stats["source"] = self.frame_source.describe()
            except Exception as exc:  # noqa: BLE001
                stats["source"] = {"error": str(exc)}
        msg = String()
        msg.data = json.dumps(stats, ensure_ascii=False, default=str, sort_keys=True)
        self.pubStats.publish(msg)

    def shutdown(self) -> None:
        self._stop.set()
        if self._readerThread is not None and self._readerThread.is_alive():
            self._readerThread.join(timeout=2.0)
        if self.frame_source is not None:
            try:
                self.frame_source.close()
            except Exception:  # noqa: BLE001
                pass


def _stampFromSeconds(seconds: float):
    from builtin_interfaces.msg import Time

    sec = int(seconds // 1)
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    stamp = Time()
    stamp.sec = sec
    stamp.nanosec = nanosec
    return stamp


def main(argv=None) -> None:
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    oldHandlers = {sig: signal.signal(sig, signal.default_int_handler)
                    for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = StereoSourceNode()
        node.startReader()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:  # noqa: BLE001
        print(f"[boom_birds_sensing] stereo_source 启动失败：{exc}")
        raise
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in oldHandlers.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    main()
