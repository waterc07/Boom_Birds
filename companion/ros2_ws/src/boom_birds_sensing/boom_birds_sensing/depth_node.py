"""深度与完整 XYZ 的 ROS 2 节点（脱机）。

契约：
- 订阅左右两路原始图（唯一采集源语义），同帧合成后复用 StereoProcessor；
- 发布 32FC1 米制深度与主 XYZ；无效值保持 NaN；
- 16UC1 毫米兼容话题的无效值为整数 0；
- 发布保持 H×W 对应的主点云（is_dense=false）与附加的紧凑点云；
- 发布由同一标定 P1 推导的 CameraInfo，供地图使用，避免两套内参。
"""

from __future__ import annotations

from boom_birds_control.runtime_config import DEFAULTS

import threading

import cv2
import message_filters
import numpy as np
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from std_msgs.msg import Header

from boom_birds_sensing.depth_core import (
    annotate_validity,
    make_processor,
    process_stitched,
    rectified_camera_info,
    xyz_to_compact,
    xyz_to_structured,
)
from boom_birds_sensing.camera_geometry import scaled_camera_info
from boom_birds_sensing.ros_msg import cloud2_from_structured, cloud2_xyz_hw

QOS_IMAGE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)


class DepthNode(Node):
    def __init__(self) -> None:
        super().__init__("boom_birds_depth")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("input_transport", "raw")
        self.declare_parameter("mjpeg_topic", DEFAULTS.stereo_stitched_topic + "/compressed")
        self.declare_parameter("left_topic", DEFAULTS.stereo_left_topic)
        self.declare_parameter("right_topic", DEFAULTS.stereo_right_topic)
        self.declare_parameter("depth_topic", DEFAULTS.depth_topic)
        self.declare_parameter("xyz_topic", DEFAULTS.depth_xyz_topic)
        self.declare_parameter("xyz_valid_topic", DEFAULTS.depth_xyz_valid_topic)
        self.declare_parameter("camera_info_topic", DEFAULTS.camera_info_topic)
        self.declare_parameter("frame_id", DEFAULTS.camera_frame)
        self.declare_parameter("min_depth_m", 0.2)
        # 量程默认值只能有一个定义点（RuntimeConfig）；这里写死 5.0 就是第二份定义，
        # 改基准字段时它会静默落后（独立核验 F1 实测）。
        self.declare_parameter("max_depth_m", DEFAULTS.depth_max_range_m)
        self.declare_parameter("publish_xyz", True)
        self.declare_parameter("publish_compact_xyz", True)
        # 主话题保持契约要求的 NaN；另发一个 0 值兼容话题给只接受 uint16 毫米流的消费者。
        self.declare_parameter("depth_compat_topic", DEFAULTS.depth_compat_topic)
        self.declare_parameter("publish_depth_compat", True)
        self.declare_parameter("process_latest_only", False)
        self.declare_parameter("opencv_threads", 0)
        self.declare_parameter("image_queue_depth", 10)
        self.declare_parameter("sync_queue_depth", 10)
        self.declare_parameter("output_scale", 1.0)
        self.declare_parameter("publish_color_preview", False)

        calib = self.get_parameter("calibration_file").value
        if not calib:
            raise RuntimeError("必须显式提供 calibration_file（缺标定不允许静默使用占位值）")
        self.processor = make_processor(calib)
        self.frame_id = self.get_parameter("frame_id").value
        self.min_depth = float(self.get_parameter("min_depth_m").value)
        self.max_depth = float(self.get_parameter("max_depth_m").value)
        self.publish_xyz = bool(self.get_parameter("publish_xyz").value)
        self.publish_compact = bool(self.get_parameter("publish_compact_xyz").value)
        self.output_scale = float(self.get_parameter("output_scale").value)
        if not 0.5 <= self.output_scale <= 1.0:
            raise ValueError("output_scale 必须处于 0.5 到 1.0")

        info = rectified_camera_info(self.processor, self.frame_id)
        self.camera_info = scaled_camera_info(info, self.output_scale)
        self.get_logger().info(
            "深度内参（与标定 P1 同源）："
            f"fx={self.camera_info['fx']:.4f} fy={self.camera_info['fy']:.4f} "
            f"cx={self.camera_info['cx']:.4f} cy={self.camera_info['cy']:.4f} "
            f"baseline={info['baseline_m']:.6f} m "
            f"size={self.camera_info['width']}x{self.camera_info['height']}"
        )

        cv_threads = int(self.get_parameter("opencv_threads").value)
        if cv_threads < 0:
            raise ValueError("opencv_threads must be nonnegative")
        if cv_threads:
            cv2.setNumThreads(cv_threads)
        image_depth = int(self.get_parameter("image_queue_depth").value)
        sync_depth = int(self.get_parameter("sync_queue_depth").value)
        if image_depth < 1 or sync_depth < 1:
            raise ValueError("image_queue_depth/sync_queue_depth 必须为正整数")
        image_qos = QoSProfile(depth=image_depth, reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST)
        self.bridge = CvBridge()
        self.pub_depth = self.create_publisher(Image, self.get_parameter("depth_topic").value, image_qos)
        self.publish_color_preview = bool(self.get_parameter("publish_color_preview").value)
        self.pub_preview = self.create_publisher(
            Image, DEFAULTS.depth_preview_topic, image_qos)
        self.pub_xyz = self.create_publisher(PointCloud2, self.get_parameter("xyz_topic").value, image_qos)
        self.pub_xyz_valid = self.create_publisher(PointCloud2, self.get_parameter("xyz_valid_topic").value, image_qos)
        self.pub_info = self.create_publisher(CameraInfo, self.get_parameter("camera_info_topic").value, image_qos)
        self.publish_depth_compat = bool(self.get_parameter("publish_depth_compat").value)
        self.pub_depth_compat = self.create_publisher(
            Image, self.get_parameter("depth_compat_topic").value, image_qos
        )

        self.process_latest_only = bool(self.get_parameter("process_latest_only").value)
        self._depth_condition = threading.Condition()
        self._depth_stop = threading.Event()
        self._pending_pair = None
        self._worker_error = None
        self._depth_thread = None
        self.dropped_processing = 0
        transport = str(self.get_parameter("input_transport").value)
        if transport == "raw":
            self.sub_left = message_filters.Subscriber(
                self, Image, self.get_parameter("left_topic").value, qos_profile=image_qos)
            self.sub_right = message_filters.Subscriber(
                self, Image, self.get_parameter("right_topic").value, qos_profile=image_qos)
            # 左右目来自同一拼接帧，stamp 必须完全相同；近似配对会混入不同曝光的两目。
            self.sync = message_filters.TimeSynchronizer(
                [self.sub_left, self.sub_right], queue_size=sync_depth)
            self.sync.registerCallback(self.queue_pair if self.process_latest_only else self.on_pair)
        elif transport == "mjpeg":
            self.sub_packet = self.create_subscription(
                CompressedImage, self.get_parameter("mjpeg_topic").value,
                self.queue_packet if self.process_latest_only else self.on_packet, image_qos)
        else:
            raise ValueError("input_transport must be raw or mjpeg")

        self.frames = 0
        self.stats = {"valid_ratio_sum": 0.0, "compute_ms_sum": 0.0, "over_range": 0}
        self.report_timer = self.create_timer(10.0, self.report)
        if self.process_latest_only:
            self._depth_thread = threading.Thread(target=self._process_worker, name="depth_worker", daemon=True)
            self._depth_thread.start()

    def queue_pair(self, left_msg: Image, right_msg: Image) -> None:
        if self._worker_error is not None:
            raise RuntimeError("depth worker failed") from self._worker_error
        with self._depth_condition:
            if self._depth_stop.is_set():
                return
            if self._pending_pair is not None:
                self.dropped_processing += 1
            # 单槽只替换尚未开始计算的输入；正在处理的一对图像不受新帧影响。
            self._pending_pair = (left_msg, right_msg)
            self._depth_condition.notify()

    def _process_worker(self) -> None:
        try:
            while True:
                with self._depth_condition:
                    self._depth_condition.wait_for(
                        lambda: self._depth_stop.is_set() or self._pending_pair is not None)
                    if self._depth_stop.is_set():
                        return
                    pair = self._pending_pair
                    self._pending_pair = None
                if pair[1] is None:
                    self.on_packet(pair[0])
                else:
                    self.on_pair(*pair)
        except Exception as exc:
            self._worker_error = exc
            self._depth_stop.set()

    def shutdown(self) -> None:
        self._depth_stop.set()
        with self._depth_condition:
            self._pending_pair = None
            self._depth_condition.notify_all()
        if self._depth_thread is not None:
            self._depth_thread.join(timeout=2.0)
            if self._depth_thread.is_alive():
                raise RuntimeError("depth worker did not stop within 2 s")

    def queue_packet(self, msg: CompressedImage) -> None:
        self.queue_pair(msg, None)

    def on_packet(self, msg: CompressedImage) -> None:
        if "jpeg" not in msg.format.lower():
            raise ValueError("compressed stereo input must be JPEG")
        if not msg.data:
            raise ValueError("invalid stitched JPEG")
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None or image.shape[1] % 2:
            raise ValueError("invalid stitched JPEG")
        result = process_stitched(self.processor, image)
        self._publish_result(result, msg.header.stamp)

    def on_pair(self, left_msg: Image, right_msg: Image) -> None:
        try:
            left = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding="mono8")
            right = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding="mono8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"图像解码失败：{exc}")
            return
        if left.shape[0] != right.shape[0]:
            self.get_logger().error("左右图高度不一致，丢弃该帧")
            return
        stitched = np.hstack([left, right])
        try:
            result = process_stitched(self.processor, stitched)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"深度计算失败：{exc}")
            return

        self._publish_result(result, left_msg.header.stamp)

    def _publish_result(self, result, stamp) -> None:
        # 公共契约（任务要求）：主深度话题 32FC1、米制、**无效值保持 NaN**。
        # 另发一个 0 值兼容话题，供只接受 uint16 毫米流的消费者使用。
        depth = result.depth
        xyz = result.xyz
        valid = result.valid
        if self.output_scale != 1.0:
            size = (self.camera_info["width"], self.camera_info["height"])
            depth = cv2.resize(depth, size, interpolation=cv2.INTER_NEAREST)
            xyz = cv2.resize(xyz, size, interpolation=cv2.INTER_NEAREST)
            valid = cv2.resize(valid.astype(np.uint8), size, interpolation=cv2.INTER_NEAREST).astype(bool)
        depth_pub, invalid, over_range = annotate_validity(
            depth, valid, self.max_depth, self.min_depth
        )
        # min/max 量程筛查用于毫米兼容流和诊断；主深度保留算法匹配结果及 NaN。
        depth_nan = np.asarray(depth, dtype=np.float32).copy()   # 无效位置已是 NaN
        header = Header()
        header.stamp = stamp
        header.frame_id = self.frame_id
        self.pub_depth.publish(self.bridge.cv2_to_imgmsg(depth_nan, encoding="32FC1", header=header))
        if self.publish_color_preview:
            normalized = np.uint8(np.clip(
                (np.nan_to_num(depth_nan, nan=self.min_depth) - self.min_depth)
                / (self.max_depth - self.min_depth), 0, 1) * 255)
            preview = cv2.applyColorMap(255 - normalized, cv2.COLORMAP_TURBO)
            preview[~valid] = 0
            self.pub_preview.publish(self.bridge.cv2_to_imgmsg(preview, encoding="bgr8", header=header))
        if self.publish_depth_compat:
            # 兼容话题必须名副其实：16UC1、毫米、uint16、无效值 = 整数 0。
            # 主话题保持 32FC1、米、NaN（公共契约）。
            depth_mm = np.where(np.isfinite(depth_pub), depth_pub * 1000.0, 0.0)
            depth_mm = np.clip(np.rint(depth_mm), 0.0, 65535.0).astype(np.uint16)
            self.pub_depth_compat.publish(
                self.bridge.cv2_to_imgmsg(depth_mm, encoding="16UC1", header=header)
            )

        if self.publish_xyz:
            msg = cloud2_xyz_hw(xyz, valid, header)
            self.pub_xyz.publish(msg)
        if self.publish_compact:
            pts, dense = xyz_to_compact(xyz, valid)
            self.pub_xyz_valid.publish(cloud2_from_structured(pts, header, dense))

        info_msg = CameraInfo()
        info_msg.header = header
        info_msg.width = self.camera_info["width"]
        info_msg.height = self.camera_info["height"]
        info_msg.k = [
            self.camera_info["fx"], 0.0, self.camera_info["cx"],
            0.0, self.camera_info["fy"], self.camera_info["cy"],
            0.0, 0.0, 1.0,
        ]
        info_msg.p = [
            self.camera_info["fx"], 0.0, self.camera_info["cx"], 0.0,
            0.0, self.camera_info["fy"], self.camera_info["cy"], 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        self.pub_info.publish(info_msg)

        self.frames += 1
        self.stats["valid_ratio_sum"] += float(valid.mean())
        self.stats["compute_ms_sum"] += float(result.timings["compute_ms"])
        self.stats["over_range"] += int(over_range.sum())
        nan_ratio = float(np.isnan(depth_nan).mean())
        self.stats["nan_ratio_sum"] = self.stats.get("nan_ratio_sum", 0.0) + nan_ratio

    def _xyz_message(self, result, header: Header):
        return cloud2_xyz_hw(result.xyz, result.valid, header)

    def report(self) -> None:
        if self._worker_error is not None:
            raise RuntimeError("depth worker failed") from self._worker_error
        if self.frames == 0:
            self.get_logger().warn("尚无成功处理的深度帧")
            return
        self.get_logger().info(
            f"深度统计：frames={self.frames} 平均有效率={self.stats['valid_ratio_sum']/self.frames:.3f} "
            f"平均 compute={self.stats['compute_ms_sum']/self.frames:.1f} ms "
            f"超量程像素累计={self.stats['over_range']} "
            f"dropped_processing={self.dropped_processing} "
            f"主话题 NaN 比例={self.stats.get('nan_ratio_sum', 0.0)/self.frames:.3f}"
        )


def main(argv=None) -> None:
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    old_handlers = {sig: signal.signal(sig, signal.default_int_handler)
                    for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = DepthNode()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:  # noqa: BLE001
        print(f"[boom_birds_nav] depth_node 启动失败：{exc}")
        raise
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        # 幂等关闭：外部已经 shutdown（例如 Ctrl-C 或父进程信号）时不再重复调用，
        # 否则会在日志里留下 'rcl_shutdown already called' 的噪声，掩盖真正的失败原因。
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    main()
