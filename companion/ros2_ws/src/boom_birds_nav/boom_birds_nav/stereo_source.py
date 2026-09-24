"""唯一双目采集源：真机 V4L2 / 回放 / 合成 / 文件，四种模式共用同一套发布语义。

为什么把四种模式放在同一个节点
------------------------------
契约要求「一次采集只发生在一处，绝不让两个算法各自打开相机」。如果真机采集、回放、
合成各自是独立节点，就必然出现两套发布代码，谁在跑、谁占着 `/dev/videoN` 也难以判断。
因此这里用一个节点、一个 `mode` 参数表达输入来源，发布语义完全一致：

- 一个拼接帧 → 一次采集 → 一个采集时间戳 → 左右图**共享**该时间戳；
- 左右图与 `CameraInfo` 由**同一份标定**推导（K 来自标定原图尺度，双目外参来自 `T`），
  深度分支仍用 `depth_core.rectified_camera_info()` 的 P1 结果，两者同源不重复实现；
- 同一节点内还会发 `/boom_birds/stereo/stitched`（调试用），语义仍是「唯一采集源」。

时间戳规则（三种模式都必须遵守）
--------------------------------
- `v4l2`：时间戳来自 V4L2 驱动（`VIDIOC_DQBUF` 的 `v4l2_buffer.timestamp`），
  经 `StereoFrameClock` 映射到 ROS 时间域。**绝不用 OpenCV 取帧返回时刻或 ROS 发布时间冒充**；
  时域不可核实即拒绝发布并给出诊断（`camera_timestamp.CameraTimestampError`）。
- `replay`：回放已保存的真实帧。这些帧**没有曝光时间戳**，所以时间戳是**回放器自己合成的
  单调时刻**（`ReplayFrameSource`），与真机采集走同一条解码/切分/发布路径，但不得据此宣称
  相机—IMU 已同步或曝光时刻已验证。
- `synth` / `file`：保留原有脱机链路（TEST-ONLY 合成 / 磁盘回放），时间戳沿用 ROS 当前时刻，
  仅用于软件行为验证。

`v4l2` 与 `replay` 都要求提供标定文件：拿不到标定就不发布 `CameraInfo`，
也不允许用占位内参把下游（深度/OpenVINS）引到错误尺度上。
"""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from .timebase import RosTimeBase
from .frames import quat_to_rot

QOS_IMAGE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST)

# 支持的输入模式；`camera_timestamp` 那套只在真机/回放模式里用到，按需导入。
MODES = ("v4l2", "replay", "synth", "file")


def _load_calibration(path: str) -> dict:
    """读取标定 npz；缺文件必须显式失败，不允许静默占位。"""
    calib = Path(path)
    if not calib.is_file():
        raise RuntimeError(
            f"标定文件不存在：{calib}。真机/回放模式必须提供标定，否则内参与基线不可信"
        )
    with np.load(calib) as archive:
        return {key: archive[key].copy() for key in archive.files}


def raw_camera_info(calibration: dict, side: str, frame_id: str, width: int, height: int,
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
    image_size = tuple(int(v) for v in calibration["image_size"])
    if side not in ("left", "right"):
        raise ValueError(f"side 只能是 left/right，收到 {side}")

    nominal = float(width) / float(image_size[0])
    if isinstance(scale, str):
        if scale != "auto":
            raise ValueError(f"scale 只能是 'auto' 或数值，收到 {scale!r}")
        scale_factor = nominal
    else:
        scale_factor = float(scale)
        if abs(scale_factor - 1.0) > 1e-9 and abs(nominal - scale_factor) > 1e-6:
            raise RuntimeError(
                f"显式 scale={scale_factor} 与采集/标定宽度比 {nominal:.6f} 不一致"
            )
        if abs(scale_factor - 1.0) <= 1e-9 and (width, height) != image_size:
            raise RuntimeError(
                f"采集尺寸 {width}x{height} 与标定 image_size {image_size} 不一致，"
                "且未允许缩放（scale=1.0）：原始内参不能跨分辨率复用"
            )
    scaled = abs(scale_factor - 1.0) > 1e-9

    k = np.asarray(calibration["K1" if side == "left" else "K2"], dtype=float).reshape(3, 3)
    d = np.asarray(calibration["D1" if side == "left" else "D2"], dtype=float).ravel()
    t = np.asarray(calibration["T"], dtype=float).ravel()
    baseline_m = float(np.linalg.norm(t))
    if scaled:
        # 纯降采样假设：fx/fy/cx/cy 与分辨率同比缩放；畸变系数不变。
        # **物理基线不随分辨率变化**（米制刚体量）。曾把 baseline_m 也乘了 scale，
        # 导致右目 P[3] = -fx·B 在 scale=0.5 时变成正确值的 1/4，已修正。
        k = k.copy()
        k[0, :] *= scale_factor
        k[1, :] *= scale_factor

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
    tx = 0.0 if side == "left" else -fx * baseline_m
    info.p = [fx, 0.0, cx, tx, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info, baseline_m, scale_factor


class StereoSourceNode(Node):
    """四种输入模式的唯一采集/发布节点。"""

    def __init__(self) -> None:
        super().__init__("boom_birds_stereo_source")

        # ---- 模式与输入 ----
        self.declare_parameter("mode", "synth")                 # v4l2 | replay | file | synth
        self.declare_parameter("path", "")                      # file/replay：拼接图或目录
        self.declare_parameter("rate_hz", 5.0)                  # 发布上限（也是合成/回放节奏）
        self.declare_parameter("loop", True)                    # 回放/文件模式是否循环
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
        self.declare_parameter("left_topic", "/boom_birds/stereo/left_raw")
        self.declare_parameter("right_topic", "/boom_birds/stereo/right_raw")
        self.declare_parameter("stitched_topic", "/boom_birds/stereo/stitched")
        # 左右 CameraInfo 各自与图像配对（ROS 惯例：一个相机一个 camera_info 话题）。
        # 留空则由对应的图像话题派生：<image_topic>/camera_info。
        self.declare_parameter("left_camera_info_topic", "")
        self.declare_parameter("right_camera_info_topic", "")
        self.declare_parameter("camera_info_topic_suffix", "camera_info")
        # 旧版把左右轮流发在同一个话题上。保留为**可选**（默认关闭），
        # 避免已有订阅者被静默破坏；新代码不要用它。
        self.declare_parameter("legacy_combined_camera_info_topic", "")
        self.declare_parameter("frame_id_left", "cam0")
        self.declare_parameter("frame_id_right", "cam1")
        self.declare_parameter("frame_id_stitched", "cam0")
        self.declare_parameter("publish_stitched", True)

        # ---- 合成模式（保持既有行为与参数名）----
        self.declare_parameter("synth_camera_y_offset_m", 0.0)
        self.declare_parameter("origin_offset_m", 0.0)
        self.declare_parameter("scene_x_offset_m", 0.0)
        self.declare_parameter("synthetic_calibration_out", "")
        self.declare_parameter("synth_pose_topic", "")  # TEST-ONLY：由运动仿真更新相机位置
        self.declare_parameter("synth_pose_timeout_s", 0.5)

        # ---- 诊断 ----
        self.declare_parameter("stats_topic", "/boom_birds/stereo/source_status")
        self.declare_parameter("stats_rate_hz", 1.0)

        self.mode = str(self.get_parameter("mode").value)
        if self.mode not in MODES:
            raise RuntimeError(f"未知 mode：{self.mode}（可用：{', '.join(MODES)}）")

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.frame_left = str(self.get_parameter("frame_id_left").value)
        self.frame_right = str(self.get_parameter("frame_id_right").value)
        self.frame_stitched = str(self.get_parameter("frame_id_stitched").value)
        self.bridge = CvBridge()

        # 采集与发布解耦：采集线程只保留最近 N 帧，处理慢时丢旧帧而不是无限积压。
        depth = max(int(self.get_parameter("publish_queue_depth").value), 1)
        self._queue: list = []
        self._queue_lock = threading.Lock()
        self._stop = threading.Event()
        self._reader_thread: threading.Thread | None = None

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
        self._queue_depth = depth

        # ---- 标定（v4l2/replay 必需）----
        self.calibration: dict | None = None
        self.baseline_m: float | None = None
        self._scale_warned = False
        if self.mode in ("v4l2", "replay"):
            calib_file = str(self.get_parameter("calibration_file").value)
            self.calibration = _load_calibration(calib_file)
            self.get_logger().info(f"已加载标定：{calib_file}")

        # ---- 发布器 ----
        self.pub_left = self.create_publisher(Image, self.get_parameter("left_topic").value, QOS_IMAGE)
        self.pub_right = self.create_publisher(Image, self.get_parameter("right_topic").value, QOS_IMAGE)
        self.pub_stitched = self.create_publisher(
            Image, self.get_parameter("stitched_topic").value, QOS_IMAGE
        )
        suffix = str(self.get_parameter("camera_info_topic_suffix").value).strip("/")

        def _info_topic(explicit_param: str, image_param: str) -> str:
            explicit = str(self.get_parameter(explicit_param).value).strip()
            if explicit:
                return explicit
            image = str(self.get_parameter(image_param).value).rstrip("/")
            return f"{image}/{suffix}" if suffix else image

        self.info_topic_left = _info_topic("left_camera_info_topic", "left_topic")
        self.info_topic_right = _info_topic("right_camera_info_topic", "right_topic")
        self.pub_info_left = self.create_publisher(CameraInfo, self.info_topic_left, QOS_IMAGE)
        self.pub_info_right = self.create_publisher(CameraInfo, self.info_topic_right, QOS_IMAGE)

        legacy = str(self.get_parameter("legacy_combined_camera_info_topic").value).strip()
        self.pub_info_legacy = (
            self.create_publisher(CameraInfo, legacy, QOS_IMAGE) if legacy else None
        )
        from std_msgs.msg import String

        self.pub_stats = self.create_publisher(String, self.get_parameter("stats_topic").value, 10)

        # ---- 输入源 ----
        self.frame_source = None
        self._synthetic = None
        self._file_pairs: list = []
        self._setup_source()
        self._synth_pose = None
        self._synth_pose_mono = None
        synth_pose_topic = str(self.get_parameter("synth_pose_topic").value)
        if synth_pose_topic:
            if self.mode != "synth":
                raise RuntimeError("synth_pose_topic 只允许在 synth 模式使用")
            self.create_subscription(PoseStamped, synth_pose_topic, self._on_synth_pose,
                                     qos_profile_sensor_data)

        self.timer = self.create_timer(1.0 / max(self.rate_hz, 0.1), self.tick)
        self.create_timer(1.0 / max(float(self.get_parameter("stats_rate_hz").value), 1e-3),
                          self.publish_stats)
        self.get_logger().info(
            f"stereo_source 启动：mode={self.mode} rate={self.rate_hz}Hz "
            f"left={self.pub_left.topic_name} right={self.pub_right.topic_name} "
            f"left_info={self.info_topic_left} right_info={self.info_topic_right}"
            + (f" legacy_info={self.pub_info_legacy.topic_name}"
               if self.pub_info_legacy is not None else "")
        )
        if self.mode in ("v4l2", "replay"):
            self.get_logger().warn(
                "v4l2/replay 模式的图像时间戳只保证「可追溯到采集/回放时刻」；"
                "V4L2 时间戳与真实曝光时刻的偏差仍需实机标定，不得当作已验证的曝光时间"
            )

    # ------------------------------------------------------------------ 输入源

    def _setup_source(self) -> None:
        if self.mode == "v4l2":
            self._setup_v4l2()
        elif self.mode == "replay":
            self._setup_replay()
        elif self.mode == "file":
            self._setup_file()
        else:
            self._setup_synth()

    def _setup_v4l2(self) -> None:
        from .camera_timestamp import CameraTimestampSource, StereoFrameClock
        from .stereo_capture import V4L2FrameSource

        width = int(self.get_parameter("capture_width").value)
        height = int(self.get_parameter("capture_height").value)
        device = str(self.get_parameter("device").value)
        source = CameraTimestampSource(
            device, width, height,
            fps=int(self.get_parameter("capture_fps").value),
            buffer_count=int(self.get_parameter("capture_buffers").value),
            allow_realtime=bool(self.get_parameter("allow_realtime_timestamp").value),
            realtime_uncertainty_limit_s=float(
                self.get_parameter("realtime_uncertainty_limit_s").value
            ),
        )
        clock = StereoFrameClock(source, RosTimeBase(self.get_clock()), stitched_width=width)
        self.frame_source = V4L2FrameSource(clock, timeout_s=float(
            self.get_parameter("capture_timeout_s").value
        ))
        self.frame_source.open()
        self.get_logger().info(
            f"V4L2 已打开：{device} {width}x{height} "
            f"协商结果={self.frame_source.describe().get('negotiated')}"
        )

    def _setup_replay(self) -> None:
        from .stereo_capture import ReplayFrameSource

        path = str(self.get_parameter("path").value)
        if not path:
            raise RuntimeError("replay 模式需要 path（拼接图、目录，或 _a/_b 图对所在目录）")
        start_mono = float(self.get_parameter("replay_start_mono_s").value)
        if start_mono < 0.0:
            start_mono = time.monotonic()
        self.frame_source = ReplayFrameSource(
            path,
            timebase=RosTimeBase(self.get_clock()),
            start_mono_s=start_mono,
            period_s=1.0 / max(float(self.get_parameter("replay_fps").value), 1e-3),
            loop=self.loop,
        )
        self.get_logger().info(
            f"回放源已加载：{path}（{self.frame_source.describe().get('frame_count')} 帧，"
            "时间戳由回放器合成，记录帧不含曝光时间戳）"
        )

    def _setup_file(self) -> None:
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
                self._file_pairs.append((left, right, c))
        else:
            self._file_pairs.append((path, path, path))
        if not self._file_pairs:
            raise RuntimeError("file 模式未找到可用输入（需拼接图 *.png/*.jpg 或目录）")
        self.get_logger().info(f"file 模式：{len(self._file_pairs)} 帧")

    def _setup_synth(self) -> None:
        from .synthetic import default_scene

        scene_x = float(self.get_parameter("scene_x_offset_m").value)
        self._synthetic = default_scene(world_x_offset=scene_x if scene_x else 0.0)
        calib_out = str(self.get_parameter("synthetic_calibration_out").value)
        if calib_out:
            from .synthetic import write_synth_calibration

            os.makedirs(os.path.dirname(calib_out) or ".", exist_ok=True)
            write_synth_calibration(calib_out)
            self.get_logger().info(f"合成标定已写出：{calib_out}")

    # ------------------------------------------------------------------ 采集线程

    def start_reader(self) -> None:
        """v4l2/replay 的取帧放到独立线程：`read_raw()` 会阻塞，不能占用执行器。"""
        if self.frame_source is None:
            return
        self._reader_thread = threading.Thread(target=self._reader_loop, name="stereo_reader",
                                               daemon=True)
        self._reader_thread.start()

    def _reader_loop(self) -> None:
        interval = 1.0 / max(float(self.get_parameter("capture_fps").value), 1.0)
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                frame = self.frame_source.next_frame()
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
            with self._queue_lock:
                self._queue.append(frame)
                while len(self._queue) > self._queue_depth:
                    self._queue.pop(0)
                    self.counters["dropped_backlog"] += 1
            # 真机采集按相机节奏推进；回放源自带节奏，不需要额外限速
            if self.mode == "v4l2":
                remaining = interval - (time.monotonic() - started)
                if remaining > 0:
                    time.sleep(remaining)

    def _decode_frame(self, frame):
        """按宽度对半切左右目。

        切分实现**只有一份**：采集源的 `decode()` 与 `camera_timestamp.decode_stitched`
        最终都走同一个 `StereoFrameClock.decode_stereo`。本节点不重复实现切分，
        真机与回放因此走完全相同的解码路径。
        """
        decode = getattr(self.frame_source, "decode", None)
        if callable(decode):
            return decode(frame)
        from .camera_timestamp import decode_stitched

        return decode_stitched(frame.stitched, frame.stitched_width, frame.height)

    def _take_frame(self):
        with self._queue_lock:
            return self._queue.pop(0) if self._queue else None

    # ------------------------------------------------------------------ 发布

    def tick(self) -> None:
        if self.mode in ("v4l2", "replay"):
            self._tick_captured()
        else:
            self._tick_offline()

    def _tick_captured(self) -> None:
        """真机/回放：只发布采集线程交付的帧，时间戳完全沿用采集时间戳。"""
        frame = self._take_frame()
        if frame is None:
            return
        try:
            left, right = self.frame_source.decode(frame)
        except Exception as exc:  # noqa: BLE001
            self.counters["capture_errors"] += 1
            self.counters["last_error"] = f"decode: {exc}"
            self.get_logger().warn(f"拼接帧解码失败（拒绝发布）：{exc}", throttle_duration_sec=2.0)
            return

        header = Header()
        header.stamp = _stamp_from_seconds(frame.capture_ros_s)
        header.frame_id = self.frame_left
        self.pub_left.publish(self.bridge.cv2_to_imgmsg(left, encoding="mono8", header=header))
        right_header = Header()
        right_header.stamp = header.stamp          # 同一帧：左右共享同一时间戳
        right_header.frame_id = self.frame_right
        self.pub_right.publish(self.bridge.cv2_to_imgmsg(right, encoding="mono8", header=right_header))
        if bool(self.get_parameter("publish_stitched").value):
            st = Header()
            st.stamp = header.stamp
            st.frame_id = self.frame_stitched
            self.pub_stitched.publish(
                self.bridge.cv2_to_imgmsg(np.hstack([left, right]), encoding="mono8", header=st)
            )
        self._publish_camera_info(header.stamp, left.shape[1], left.shape[0])
        self.counters["frames_published"] += 1

    def _tick_offline(self) -> None:
        """合成/文件：保持既有脱机语义（时间戳 = ROS 当前时刻）。"""
        try:
            left, right, stitched = self._load_offline()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"取帧失败：{exc}", throttle_duration_sec=2.0)
            return
        stamp = self.get_clock().now().to_msg()
        self.pub_left.publish(self.bridge.cv2_to_imgmsg(left, encoding="mono8",
                                                        header=self._header(stamp, self.frame_left)))
        self.pub_right.publish(self.bridge.cv2_to_imgmsg(right, encoding="mono8",
                                                         header=self._header(stamp, self.frame_right)))
        if bool(self.get_parameter("publish_stitched").value):
            self.pub_stitched.publish(self.bridge.cv2_to_imgmsg(
                stitched, encoding="mono8", header=self._header(stamp, self.frame_stitched)
            ))
        self._publish_camera_info(stamp, left.shape[1], left.shape[0])
        self.counters["frames_published"] += 1
        self.counters["frames_captured"] += 1
        self._offline_seq = getattr(self, "_offline_seq", 0) + 1

    def _on_synth_pose(self, msg: PoseStamped) -> None:
        """仿真相机位姿驱动双目几何；失效位姿不得进入渲染器。"""
        p = msg.pose.position
        q = msg.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not np.all(np.isfinite(values)):
            return
        if abs(sum(v * v for v in (q.x, q.y, q.z, q.w)) - 1.0) > 0.05:
            self._synth_pose_mono = None
            return
        self._synth_pose = (float(p.x), float(p.y), float(p.z))
        self._synth_rotation = quat_to_rot((q.x, q.y, q.z, q.w))
        self._synth_pose_mono = time.monotonic()

    def _load_offline(self):
        if self.mode == "synth":
            from .synthetic import render_stereo

            if str(self.get_parameter("synth_pose_topic").value):
                age = (None if self._synth_pose_mono is None
                       else time.monotonic() - self._synth_pose_mono)
                if age is None or age > float(self.get_parameter("synth_pose_timeout_s").value):
                    raise RuntimeError("合成运动位姿缺失或过期，停止发布双目帧")
                self._synthetic.camera_x, self._synthetic.camera_y_offset, self._synthetic.camera_z = self._synth_pose
                self._synthetic.camera_rotation = self._synth_rotation
                if self._synthetic.camera_x >= self._synthetic.wall_x - 0.2:
                    raise RuntimeError("合成相机已到场景墙面，停止发布双目帧")
            else:
                self._synthetic.camera_y_offset = float(self.get_parameter("synth_camera_y_offset_m").value)
            left, right, _ = render_stereo(self._synthetic)
            return left, right, np.hstack([left, right])
        seq = getattr(self, "_offline_seq", 0)
        if self.loop:
            left_path, right_path, stitched_path = self._file_pairs[seq % len(self._file_pairs)]
        else:
            if seq >= len(self._file_pairs):
                raise RuntimeError("文件回放已到末尾（loop=false）")
            left_path, right_path, stitched_path = self._file_pairs[seq]
        stitched = cv2.imread(stitched_path, cv2.IMREAD_GRAYSCALE)
        if stitched is None:
            raise RuntimeError(f"无法读取图像：{stitched_path}")
        if left_path == right_path:
            half = stitched.shape[1] // 2
            return stitched[:, :half], stitched[:, half:], stitched
        left = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)
        if left is None or right is None:
            raise RuntimeError(f"无法读取图像对：{left_path} / {right_path}")
        return left, right, np.hstack([left, right])

    def _publish_camera_info(self, stamp, width: int, height: int) -> None:
        """左右各发一条 CameraInfo，各自发到与图像配对的话题；缺标定时不发并给出诊断。"""
        if self.calibration is None:
            return
        for side, frame_id, pub in (
            ("left", self.frame_left, self.pub_info_left),
            ("right", self.frame_right, self.pub_info_right),
        ):
            try:
                info, baseline, scale = raw_camera_info(
                    self.calibration, side, frame_id, width, height,
                    scale=str(self.get_parameter("raw_info_scale").value),
                )
            except (RuntimeError, ValueError) as exc:
                self.counters["last_error"] = f"camera_info: {exc}"
                self.get_logger().warn(str(exc), throttle_duration_sec=10.0)
                return
            info.header.stamp = stamp
            pub.publish(info)
            if self.pub_info_legacy is not None:
                self.pub_info_legacy.publish(info)
            self.baseline_m = baseline
            self.counters["raw_info_scale"] = scale
            if abs(scale - 1.0) > 1e-9 and not self._scale_warned:
                self._scale_warned = True
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

    def publish_stats(self) -> None:
        import json

        from std_msgs.msg import String

        stats = dict(self.counters)
        stats["mode"] = self.mode
        stats["queue_len"] = len(self._queue)
        stats["baseline_m"] = self.baseline_m
        stats["left_camera_info_topic"] = self.info_topic_left
        stats["right_camera_info_topic"] = self.info_topic_right
        stats["legacy_combined_camera_info"] = (
            None if self.pub_info_legacy is None else self.pub_info_legacy.topic_name
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
        self.pub_stats.publish(msg)

    def shutdown(self) -> None:
        self._stop.set()
        if self._reader_thread is not None and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2.0)
        if self.frame_source is not None:
            try:
                self.frame_source.close()
            except Exception:  # noqa: BLE001
                pass


def _stamp_from_seconds(seconds: float):
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
    rclpy.init(args=argv)
    node = None
    try:
        node = StereoSourceNode()
        node.start_reader()
        rclpy.spin(node)
    except Exception as exc:  # noqa: BLE001
        print(f"[boom_birds_nav] stereo_source 启动失败：{exc}")
        raise
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
