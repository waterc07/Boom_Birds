"""TEST-ONLY 合成双目输入；复用 sensing 的发布与 CameraInfo 实现。"""
from boom_birds_control.runtime_config import DEFAULTS
import os
import time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
from boom_birds_control.frames import quat_to_rot
from boom_birds_sensing.stereo_source import StereoSourceNode


class SyntheticStereoSourceNode(StereoSourceNode):
    MODES = ("synth",)

    def __init__(self):
        super().__init__(default_mode="synth")

    def _declare_source_parameters(self):
        # ---- 合成模式（保持既有行为与参数名）----
        self.declare_parameter("synth_camera_y_offset_m", 0.0)
        self.declare_parameter("origin_offset_m", 0.0)
        self.declare_parameter("scene_x_offset_m", 0.0)
        self.declare_parameter("synthetic_calibration_out", "")
        self.declare_parameter("synth_pose_topic", "")  # TEST-ONLY：由运动仿真更新相机位置
        self.declare_parameter("synth_pose_timeout_s", 0.5)
        self.declare_parameter("synth_map_topic", "")  # TEST-ONLY：EGO mockamap 全局点云
        self.declare_parameter("synth_map_resolution_m", 0.1)
        self.declare_parameter("synth_min_altitude_m", -1.0)
        self.declare_parameter("synth_ground_z_m", 0.0)
        self._synth_altitude_started = False


    def _setup_source_subscriptions(self):
        self._synth_pose = None
        self._synth_pose_mono = None
        self._synth_pose_stamp_s = None
        self._synth_map_points = None
        synth_pose_topic = str(self.get_parameter("synth_pose_topic").value)
        if synth_pose_topic:
            if self.mode != "synth":
                raise RuntimeError("synth_pose_topic 只允许在 synth 模式使用")
            self.create_subscription(PoseStamped, synth_pose_topic, self._on_synth_pose,
                                     qos_profile_sensor_data)
        synth_map_topic = str(self.get_parameter("synth_map_topic").value)
        self.pub_synth_map = None
        if synth_map_topic:
            if self.mode != "synth" or not synth_pose_topic:
                raise RuntimeError("synth_map_topic 只允许在位姿驱动的 synth 模式使用")
            self.create_subscription(PointCloud2, synth_map_topic, self._on_synth_map, 10)
            self.pub_synth_map = self.create_publisher(
                PointCloud2, DEFAULTS.sim_world_cloud_topic, 1)


    def _setup_source(self):
        self._setup_synth()

    def _setup_synth(self) -> None:
        from boom_birds_sim.synthetic import default_scene

        scene_x = float(self.get_parameter("scene_x_offset_m").value)
        self._synthetic = default_scene(world_x_offset=scene_x if scene_x else 0.0)
        calib_out = str(self.get_parameter("synthetic_calibration_out").value)
        if calib_out:
            from boom_birds_sim.synthetic import write_synth_calibration

            os.makedirs(os.path.dirname(calib_out) or ".", exist_ok=True)
            write_synth_calibration(calib_out)
            self.get_logger().info(f"合成标定已写出：{calib_out}")


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
        self._synth_pose_stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


    def _on_synth_map(self, msg: PointCloud2) -> None:
        """只接受仿真世界系 XYZ，点云缺失时不生成伪造的空地图帧。"""
        if msg.header.frame_id not in ("world", "global"):
            self.get_logger().error("合成地图坐标系应为 world/global", throttle_duration_sec=5.0)
            return
        from sensor_msgs_py import point_cloud2

        try:
            points = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"),
                                                    skip_nans=True)
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f"合成地图点云无有效 XYZ：{exc}", throttle_duration_sec=5.0)
            return
        if len(points) < 100:
            self.get_logger().error("合成地图点数不足，拒绝替换场景", throttle_duration_sec=5.0)
            return
        self._synth_map_points = np.asarray(points, dtype=np.float32)
        msg.header.frame_id = "global"  # 仅 SIH 测试中，mockamap world 与 global 共用原点
        self.pub_synth_map.publish(msg)


    def _load_offline(self):
        self._offline_capture_ros_s = self.get_clock().now().nanoseconds * 1e-9
        if self.mode == "synth":
            from boom_birds_sim.synthetic import render_stereo

            if str(self.get_parameter("synth_pose_topic").value):
                age = (None if self._synth_pose_mono is None
                       else time.monotonic() - self._synth_pose_mono)
                sample_stamp = self._synth_pose_stamp_s
                sample_age = (None if sample_stamp is None else self._offline_capture_ros_s - sample_stamp)
                timeout = float(self.get_parameter("synth_pose_timeout_s").value)
                if (age is None or age > timeout or sample_age is None
                        or not np.isfinite(sample_stamp) or sample_stamp <= 0
                        or not -1e-6 <= sample_age <= timeout):
                    raise RuntimeError("合成运动位姿缺失或过期，停止发布双目帧")
                self._offline_capture_ros_s = sample_stamp
                self._synthetic.camera_x, self._synthetic.camera_y_offset, self._synthetic.camera_z = self._synth_pose
                self._synthetic.camera_rotation = self._synth_rotation
                min_altitude = float(self.get_parameter("synth_min_altitude_m").value)
                ground_z = float(self.get_parameter("synth_ground_z_m").value)
                if not self._synth_altitude_started:
                    if min_altitude >= 0.0 and self._synthetic.camera_z - ground_z < min_altitude:
                        raise RuntimeError("SIH 尚未达到合成场景的起飞高度，停止发布双目帧")
                    self._synth_altitude_started = True
                if (not str(self.get_parameter("synth_map_topic").value)
                        and self._synthetic.camera_x >= self._synthetic.wall_x - 0.2):
                    raise RuntimeError("合成相机已到场景墙面，停止发布双目帧")
            else:
                self._synthetic.camera_y_offset = float(self.get_parameter("synth_camera_y_offset_m").value)
            if str(self.get_parameter("synth_map_topic").value):
                if self._synth_map_points is None:
                    raise RuntimeError("尚未收到 EGO mockamap 全局点云，停止发布双目帧")
                from boom_birds_sim.pointcloud_scene import depth_from_world_cloud
                from boom_birds_sim.synthetic import render_stereo_from_pointcloud_depth

                depth = depth_from_world_cloud(
                    self._synth_map_points, self._synthetic.camera_pose_world(),
                    resolution_m=float(self.get_parameter("synth_map_resolution_m").value))
                left, right = render_stereo_from_pointcloud_depth(depth)
            else:
                left, right, _ = render_stereo(self._synthetic)
            return left, right, np.hstack([left, right])
        return super()._load_offline()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = None
    try:
        node = SyntheticStereoSourceNode()
        node.start_reader()
        rclpy.spin(node)
    except Exception as exc:  # noqa: BLE001
        print(f"[boom_birds_sim] stereo_source 启动失败：{exc}")
        raise
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
