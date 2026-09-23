"""深度与完整 XYZ 的 ROS 2 节点（脱机）。

契约：
- 订阅左右两路原始图（唯一采集源语义），同帧合成后复用 StereoProcessor；
- 发布 32FC1 米制深度：无效观测发布为 0.0（无回波），绝不发布 NaN 给地图；
  公共数组层仍保留 NaN 语义（主 XYZ 点云无效点为 NaN）；
- 发布保持 H×W 对应的主点云（is_dense=false）与附加的紧凑点云；
- 发布由同一标定 P1 推导的 CameraInfo，供地图使用，避免两套内参。
"""

from __future__ import annotations

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header

from .depth_core import (
    annotate_validity,
    make_processor,
    process_stitched,
    rectified_camera_info,
    xyz_to_compact,
    xyz_to_structured,
)
from .ros_msg import cloud2_from_structured, cloud2_xyz_hw

QOS_IMAGE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)


class DepthNode(Node):
    def __init__(self) -> None:
        super().__init__("boom_birds_depth")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("left_topic", "/boom_birds/stereo/left_raw")
        self.declare_parameter("right_topic", "/boom_birds/stereo/right_raw")
        self.declare_parameter("depth_topic", "/boom_birds/depth/image")
        self.declare_parameter("xyz_topic", "/boom_birds/depth/xyz")
        self.declare_parameter("xyz_valid_topic", "/boom_birds/depth/xyz_valid")
        self.declare_parameter("camera_info_topic", "/boom_birds/depth/camera_info")
        self.declare_parameter("frame_id", "cam0_rect")
        self.declare_parameter("min_depth_m", 0.2)
        self.declare_parameter("max_depth_m", 5.0)
        self.declare_parameter("publish_xyz", True)
        self.declare_parameter("publish_compact_xyz", True)
        # 主话题保持契约要求的 NaN；另发一个 0 值兼容话题给只接受 uint16 毫米流的消费者。
        self.declare_parameter("depth_compat_topic", "/boom_birds/depth/image_compat_uint16_mm")
        self.declare_parameter("publish_depth_compat", True)

        calib = self.get_parameter("calibration_file").value
        if not calib:
            raise RuntimeError("必须显式提供 calibration_file（缺标定不允许静默使用占位值）")
        self.processor = make_processor(calib)
        self.frame_id = self.get_parameter("frame_id").value
        self.min_depth = float(self.get_parameter("min_depth_m").value)
        self.max_depth = float(self.get_parameter("max_depth_m").value)
        self.publish_xyz = bool(self.get_parameter("publish_xyz").value)
        self.publish_compact = bool(self.get_parameter("publish_compact_xyz").value)

        info = rectified_camera_info(self.processor, self.frame_id)
        self.camera_info = info
        self.get_logger().info(
            "深度内参（与标定 P1 同源）："
            f"fx={info['fx']:.4f} fy={info['fy']:.4f} cx={info['cx']:.4f} cy={info['cy']:.4f} "
            f"baseline={info['baseline_m']:.6f} m size={info['width']}x{info['height']}"
        )

        self.bridge = CvBridge()
        self.pub_depth = self.create_publisher(Image, self.get_parameter("depth_topic").value, QOS_IMAGE)
        self.pub_xyz = self.create_publisher(PointCloud2, self.get_parameter("xyz_topic").value, QOS_IMAGE)
        self.pub_xyz_valid = self.create_publisher(PointCloud2, self.get_parameter("xyz_valid_topic").value, QOS_IMAGE)
        self.pub_info = self.create_publisher(CameraInfo, self.get_parameter("camera_info_topic").value, QOS_IMAGE)
        self.publish_depth_compat = bool(self.get_parameter("publish_depth_compat").value)
        self.pub_depth_compat = self.create_publisher(
            Image, self.get_parameter("depth_compat_topic").value, QOS_IMAGE
        )

        # 左右两路都必须订阅；使用与发布端一致的 reliable QoS。
        self.sub_left = message_filters.Subscriber(self, Image, self.get_parameter("left_topic").value, qos_profile=QOS_IMAGE)
        self.sub_right = message_filters.Subscriber(self, Image, self.get_parameter("right_topic").value, qos_profile=QOS_IMAGE)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.sub_left, self.sub_right], queue_size=10, slop=0.02)
        self.sync.registerCallback(self.on_pair)

        self.frames = 0
        self.stats = {"valid_ratio_sum": 0.0, "compute_ms_sum": 0.0, "over_range": 0}
        self.report_timer = self.create_timer(10.0, self.report)

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
        # 左图为单通道（mono8），深度算法通道无关；统一转成 3 通道 BGR 以满足
        # rectify_image 的入参契约（与 A/B 类测试的构造方式一致）。
        stitched = cv2.cvtColor(np.hstack([left, right]), cv2.COLOR_GRAY2BGR)
        try:
            result = process_stitched(self.processor, stitched)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"深度计算失败：{exc}")
            return

        # 公共契约（任务要求）：主深度话题 32FC1、米制、**无效值保持 NaN**。
        # 另发一个 0 值兼容话题，供只接受 uint16 毫米流的消费者使用。
        depth_pub, invalid, over_range = annotate_validity(
            result.depth, result.valid, self.max_depth, self.min_depth
        )
        depth_nan = np.asarray(result.depth, dtype=np.float32).copy()   # 无效位置已是 NaN
        header = Header()
        header.stamp = left_msg.header.stamp
        header.frame_id = self.frame_id
        self.pub_depth.publish(self.bridge.cv2_to_imgmsg(depth_nan, encoding="32FC1", header=header))
        if self.publish_depth_compat:
            # 兼容话题必须名副其实：16UC1、毫米、uint16、无效值 = 整数 0。
            # 主话题保持 32FC1、米、NaN（公共契约）。
            depth_mm = np.where(np.isfinite(depth_pub), depth_pub * 1000.0, 0.0)
            depth_mm = np.clip(np.rint(depth_mm), 0.0, 65535.0).astype(np.uint16)
            self.pub_depth_compat.publish(
                self.bridge.cv2_to_imgmsg(depth_mm, encoding="16UC1", header=header)
            )

        if self.publish_xyz:
            msg = self._xyz_message(result, header)
            self.pub_xyz.publish(msg)
        if self.publish_compact:
            pts, dense = xyz_to_compact(result.xyz, result.valid)
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
        self.stats["valid_ratio_sum"] += float(result.valid.mean())
        self.stats["compute_ms_sum"] += float(result.timings["compute_ms"])
        self.stats["over_range"] += int(over_range.sum())
        nan_ratio = float(np.isnan(depth_nan).mean())
        self.stats["nan_ratio_sum"] = self.stats.get("nan_ratio_sum", 0.0) + nan_ratio

    def _xyz_message(self, result, header: Header):
        return cloud2_xyz_hw(result.xyz, result.valid, header)

    def report(self) -> None:
        if self.frames == 0:
            self.get_logger().warn("尚无成功处理的深度帧")
            return
        self.get_logger().info(
            f"深度统计：frames={self.frames} 平均有效率={self.stats['valid_ratio_sum']/self.frames:.3f} "
            f"平均 compute={self.stats['compute_ms_sum']/self.frames:.1f} ms "
            f"超量程像素累计={self.stats['over_range']} "
            f"主话题 NaN 比例={self.stats.get('nan_ratio_sum', 0.0)/self.frames:.3f}"
        )


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = None
    try:
        node = DepthNode()
        rclpy.spin(node)
    except Exception as exc:  # noqa: BLE001
        print(f"[boom_birds_nav] depth_node 启动失败：{exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        # 幂等关闭：外部已经 shutdown（例如 Ctrl-C 或父进程信号）时不再重复调用，
        # 否则会在日志里留下 'rcl_shutdown already called' 的噪声，掩盖真正的失败原因。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
