"""TEST-ONLY 合成双目输入；复用 sensing 的发布与 CameraInfo 实现。"""
from boom_birds_control.runtime_config import DEFAULTS
import os
import time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
from boom_birds_control.frames import quatToRot
from boom_birds_sensing.stereo_source import StereoSourceNode


class SyntheticStereoSourceNode(StereoSourceNode):
    MODES = ("synth",)

    def __init__(self):
        super().__init__(defaultMode="synth")

    def _declareSourceParameters(self):
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
        self._synthAltitudeStarted = False


    def _setupSourceSubscriptions(self):
        self._synthPose = None
        self._synthPoseMono = None
        self._synthPoseStampS = None
        self._synthMapPoints = None
        synthPoseTopic = str(self.get_parameter("synth_pose_topic").value)
        if synthPoseTopic:
            if self.mode != "synth":
                raise RuntimeError("synth_pose_topic 只允许在 synth 模式使用")
            self.create_subscription(PoseStamped, synthPoseTopic, self._onSynthPose,
                                     qos_profile_sensor_data)
        synthMapTopic = str(self.get_parameter("synth_map_topic").value)
        self.pubSynthMap = None
        if synthMapTopic:
            if self.mode != "synth" or not synthPoseTopic:
                raise RuntimeError("synth_map_topic 只允许在位姿驱动的 synth 模式使用")
            self.create_subscription(PointCloud2, synthMapTopic, self._onSynthMap, 10)
            self.pubSynthMap = self.create_publisher(
                PointCloud2, DEFAULTS.sim_world_cloud_topic, 1)


    def _setupSource(self):
        self._setupSynth()

    def _setupSynth(self) -> None:
        from boom_birds_sim.synthetic import defaultScene

        sceneX = float(self.get_parameter("scene_x_offset_m").value)
        self._synthetic = defaultScene(worldXOffset=sceneX if sceneX else 0.0)
        calibOut = str(self.get_parameter("synthetic_calibration_out").value)
        if calibOut:
            from boom_birds_sim.synthetic import writeSynthCalibration

            os.makedirs(os.path.dirname(calibOut) or ".", exist_ok=True)
            writeSynthCalibration(calibOut)
            self.get_logger().info(f"合成标定已写出：{calibOut}")


    def _onSynthPose(self, msg: PoseStamped) -> None:
        """仿真相机位姿驱动双目几何；失效位姿不得进入渲染器。"""
        p = msg.pose.position
        q = msg.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not np.all(np.isfinite(values)):
            return
        if abs(sum(v * v for v in (q.x, q.y, q.z, q.w)) - 1.0) > 0.05:
            self._synthPoseMono = None
            return
        self._synthPose = (float(p.x), float(p.y), float(p.z))
        self._synthRotation = quatToRot((q.x, q.y, q.z, q.w))
        self._synthPoseMono = time.monotonic()
        self._synthPoseStampS = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


    def _onSynthMap(self, msg: PointCloud2) -> None:
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
        self._synthMapPoints = np.asarray(points, dtype=np.float32)
        msg.header.frame_id = "global"  # 仅 SIH 测试中，mockamap world 与 global 共用原点
        self.pubSynthMap.publish(msg)


    def _loadOffline(self):
        self._offlineCaptureRosS = self.get_clock().now().nanoseconds * 1e-9
        if self.mode == "synth":
            from boom_birds_sim.synthetic import renderStereo

            if str(self.get_parameter("synth_pose_topic").value):
                age = (None if self._synthPoseMono is None
                       else time.monotonic() - self._synthPoseMono)
                sampleStamp = self._synthPoseStampS
                sampleAge = (None if sampleStamp is None else self._offlineCaptureRosS - sampleStamp)
                timeout = float(self.get_parameter("synth_pose_timeout_s").value)
                if (age is None or age > timeout or sampleAge is None
                        or not np.isfinite(sampleStamp) or sampleStamp <= 0
                        or not -1e-6 <= sampleAge <= timeout):
                    raise RuntimeError("合成运动位姿缺失或过期，停止发布双目帧")
                self._offlineCaptureRosS = sampleStamp
                self._synthetic.camera_x, self._synthetic.camera_y_offset, self._synthetic.camera_z = self._synthPose
                self._synthetic.camera_rotation = self._synthRotation
                minAltitude = float(self.get_parameter("synth_min_altitude_m").value)
                groundZ = float(self.get_parameter("synth_ground_z_m").value)
                if not self._synthAltitudeStarted:
                    if minAltitude >= 0.0 and self._synthetic.camera_z - groundZ < minAltitude:
                        raise RuntimeError("SIH 尚未达到合成场景的起飞高度，停止发布双目帧")
                    self._synthAltitudeStarted = True
                if (not str(self.get_parameter("synth_map_topic").value)
                        and self._synthetic.camera_x >= self._synthetic.wall_x - 0.2):
                    raise RuntimeError("合成相机已到场景墙面，停止发布双目帧")
            else:
                self._synthetic.camera_y_offset = float(self.get_parameter("synth_camera_y_offset_m").value)
            if str(self.get_parameter("synth_map_topic").value):
                if self._synthMapPoints is None:
                    raise RuntimeError("尚未收到 EGO mockamap 全局点云，停止发布双目帧")
                from boom_birds_sim.pointcloud_scene import depthFromWorldCloud
                from boom_birds_sim.synthetic import renderStereoFromPointcloudDepth

                depth = depthFromWorldCloud(
                    self._synthMapPoints, self._synthetic.cameraPoseWorld(),
                    resolutionM=float(self.get_parameter("synth_map_resolution_m").value))
                left, right = renderStereoFromPointcloudDepth(depth)
            else:
                left, right, _ = renderStereo(self._synthetic)
            return left, right, np.hstack([left, right])
        return super()._loadOffline()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = None
    try:
        node = SyntheticStereoSourceNode()
        node.startReader()
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
