"""唯一采集源语义的输入节点（脱机）。

职责：
- 提供左右原始图与拼接图，作为深度分支和 OpenVINS 分支的共同图像来源；
  同一帧的左右图共享同一时间戳，不允许两个算法各自打开相机。
- 两种输入模式：
  * file   ：回放磁盘上的拼接图像或左右图对（历史快照烟雾回放）；
  * synth  ：确定性合成双目（TEST-ONLY，供分层验证）。
- 不连接任何真实设备；真实相机采集仍由既有 depth_preview.py 独立程序负责，
  后续把采集线程抽象成同一接口即可复用本节点语义。
"""

from __future__ import annotations

import glob
import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .synthetic import default_scene, render_stereo

QOS_IMAGE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)


class StereoSource(Node):
    def __init__(self) -> None:
        super().__init__("boom_birds_stereo_source")
        self.declare_parameter("mode", "synth")                 # synth | file
        self.declare_parameter("path", "")                      # file 模式：拼接图或目录
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("left_topic", "/boom_birds/stereo/left_raw")
        self.declare_parameter("right_topic", "/boom_birds/stereo/right_raw")
        self.declare_parameter("stitched_topic", "/boom_birds/stereo/stitched")
        self.declare_parameter("frame_id_left", "cam0")
        self.declare_parameter("frame_id_right", "cam1")
        self.declare_parameter("synth_camera_y_offset_m", 0.0)
        # 世界坐标系原点偏移：与 vio_source 保持一致，使深度场景与位姿同处新坐标系
        self.declare_parameter("origin_offset_m", 0.0)
        # 场景可独立于相机原点平移：用于构造"换了坐标系、但相机贴近障碍"的场景，
        # 使观测距离仍落在 max_ray_length 内（否则地图不会被写入）。
        self.declare_parameter("scene_x_offset_m", 0.0)
        self.declare_parameter("synthetic_calibration_out", "")

        self.mode = self.get_parameter("mode").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.frame_left = self.get_parameter("frame_id_left").value
        self.frame_right = self.get_parameter("frame_id_right").value
        self.bridge = CvBridge()
        self.seq = 0
        self.file_pairs = []
        # Depth is measured from the camera. Shifting the world-frame origin
        # moves the published camera pose, not the stereo disparity/depth.
        # scene_x_offset_m is a separate *relative* scene shift for tests.
        self.origin_offset = float(self.get_parameter("origin_offset_m").value)
        self.scene_x_offset = float(self.get_parameter("scene_x_offset_m").value)
        self.synth_scene = default_scene(world_x_offset=self.scene_x_offset)
        self.synth_files = []

        self.pub_left = self.create_publisher(Image, self.get_parameter("left_topic").value, QOS_IMAGE)
        self.pub_right = self.create_publisher(Image, self.get_parameter("right_topic").value, QOS_IMAGE)
        self.pub_stitched = self.create_publisher(Image, self.get_parameter("stitched_topic").value, QOS_IMAGE)

        if self.mode == "file":
            self._collect_files(self.get_parameter("path").value)
            if not self.file_pairs:
                raise RuntimeError("file 模式未找到可用输入（需拼接图 *.png/*.jpg 或目录）")
            self.get_logger().info(f"file 模式：{len(self.file_pairs)} 帧")
        elif self.mode == "synth":
            y_off = float(self.get_parameter("synth_camera_y_offset_m").value)
            self.synth_scene = default_scene(world_x_offset=self.scene_x_offset if self.scene_x_offset else 0.0)
            self.synth_scene.camera_y_offset = y_off
            calib_out = self.get_parameter("synthetic_calibration_out").value
            if calib_out:
                from .synthetic import write_synth_calibration

                os.makedirs(os.path.dirname(calib_out) or ".", exist_ok=True)
                write_synth_calibration(calib_out)
                self.get_logger().info(f"合成标定已写出：{calib_out}")
        else:
            raise RuntimeError(f"未知 mode：{self.mode}")

        self.timer = self.create_timer(1.0 / max(self.rate_hz, 0.1), self.tick)
        self.get_logger().info(
            f"stereo_source 启动：mode={self.mode} rate={self.rate_hz}Hz "
            f"left={self.pub_left.topic_name} right={self.pub_right.topic_name}"
        )

    def _collect_files(self, path: str) -> None:
        if not path:
            return
        if os.path.isdir(path):
            cands = sorted(glob.glob(os.path.join(path, "*.png")) + glob.glob(os.path.join(path, "*.jpg")))
            for c in cands:
                base = os.path.splitext(c)[0]
                left = base + "_a.png" if os.path.exists(base + "_a.png") else c
                right = base + "_b.png" if os.path.exists(base + "_b.png") else c
                self.file_pairs.append((left, right, c))
        else:
            self.file_pairs.append((path, path, path))

    def _load_stitched(self, seq: int):
        if self.mode == "synth":
            y_off = float(self.get_parameter("synth_camera_y_offset_m").value)
            self.synth_scene.camera_y_offset = y_off
            left, right, _ = render_stereo(self.synth_scene)
            return left, right, np.hstack([left, right])
        if self.file_pairs:
            left_path, right_path, stitched_path = self.file_pairs[seq % len(self.file_pairs)]
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
        raise RuntimeError("没有可用的输入帧")

    def tick(self) -> None:
        try:
            left, right, stitched = self._load_stitched(self.seq)
        except Exception as exc:  # noqa: BLE001 - 需要把失败暴露到日志
            self.get_logger().error(f"取帧失败：{exc}")
            return
        # 同一帧的左右图与拼接图共享同一时间戳（唯一采集源语义）。
        stamp = self.get_clock().now().to_msg()
        self.pub_left.publish(self.bridge.cv2_to_imgmsg(left, encoding="mono8", header=self._header(stamp, self.frame_left)))
        self.pub_right.publish(self.bridge.cv2_to_imgmsg(right, encoding="mono8", header=self._header(stamp, self.frame_right)))
        self.pub_stitched.publish(
            self.bridge.cv2_to_imgmsg(stitched, encoding="mono8", header=self._header(stamp, self.frame_left))
        )
        self.seq += 1

    @staticmethod
    def _header(stamp, frame_id: str):
        from std_msgs.msg import Header

        h = Header()
        h.stamp = stamp
        h.frame_id = frame_id
        return h


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = None
    try:
        node = StereoSource()
        rclpy.spin(node)
    except Exception as exc:  # noqa: BLE001
        print(f"[boom_birds_nav] stereo_source 启动失败：{exc}")
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
