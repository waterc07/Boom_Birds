"""合成/文件 VIO 与 IMU 源（TEST-ONLY，不连接任何真实飞控）。

用途：在无 OpenVINS、无飞控的情况下，为位姿适配与地图/规划测试提供**确定性**输入，
并可选注入时间偏移，用于验证时间容差与失效处理。

严格声明：这里发布的是合成数据，不是飞控 IMU，也不是 VIO 精度证据。
"""

from __future__ import annotations

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .frames import rot_to_quat
from .synthetic import default_scene


class SyntheticVioSource(Node):
    def __init__(self) -> None:
        super().__init__("boom_birds_synthetic_vio")
        self.declare_parameter("odom_topic", "/boom_birds/ov/odomimu")
        self.declare_parameter("imu_topic", "/boom_birds/imu")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("pose_lag_s", 0.0)        # 故意注入的时间偏移（秒）
        self.declare_parameter("y_motion_amplitude_m", 0.0)
        self.declare_parameter("origin_offset_m", 0.0)
        # 坐标系原点偏移（米）：用于验证"定位重置后换到新坐标系"时旧地图/旧障碍不残留。
        # 该偏移同时作用于合成位姿的平移与生成的深度场景，使环境在旧坐标系里整体平移。
        self.declare_parameter("y_motion_period_s", 10.0)
        self.declare_parameter("publish_imu", True)
        self.declare_parameter("imu_rate_hz", 200.0)
        self.declare_parameter("frame_world", "global")

        off = float(self.get_parameter("origin_offset_m").value) if self.has_parameter("origin_offset_m") else 0.0
        self.origin_offset = off
        # 场景随坐标系一起平移：相机相对环境不变，障碍在新坐标系里出现在新位置
        self.scene = default_scene(world_x_offset=off if off else 0.0)
        self.pub_odom = self.create_publisher(Odometry, self.get_parameter("odom_topic").value, 10)
        self.pub_imu = self.create_publisher(Imu, self.get_parameter("imu_topic").value, qos_profile_sensor_data)
        rate = float(self.get_parameter("rate_hz").value)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(1.0 / max(rate, 0.1), self.tick_odom)
        if bool(self.get_parameter("publish_imu").value):
            self.create_timer(1.0 / max(float(self.get_parameter("imu_rate_hz").value), 1.0), self.tick_imu)
        self.get_logger().info(
            "合成 VIO 源启动（TEST-ONLY，非飞控数据）："
            f"pose_lag={self.get_parameter('pose_lag_s').value}s rate={rate}Hz"
        )

    def _t_motion(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9 - self.t0

    def tick_odom(self) -> None:
        amp = float(self.get_parameter("y_motion_amplitude_m").value)
        period = max(float(self.get_parameter("y_motion_period_s").value), 1e-3)
        t = self._t_motion()
        y = amp * np.sin(2 * np.pi * t / period)
        vy = amp * (2 * np.pi / period) * np.cos(2 * np.pi * t / period) if amp else 0.0
        T = self.scene.camera_pose_world(y_offset=y)
        off = self.origin_offset
        if off:
            # 世界坐标原点平移：相机位置与场景一起搬到新坐标系（相对几何不变）
            T = T.copy()
            T[0, 3] += off
        # 合成源给出的是「IMU 位姿」；本轮测试令 IMU 与相机外参由配置单独给出，
        # 因此这里发布相机位姿并让配置里的 T_I_C0 为单位阵以外的测试值承担变换职责。
        q = rot_to_quat(T[:3, :3])
        msg = Odometry()
        lag = float(self.get_parameter("pose_lag_s").value)
        stamp = self.get_clock().now().nanoseconds * 1e-9 - lag
        msg.header.stamp.sec = int(stamp)
        msg.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
        msg.header.frame_id = self.get_parameter("frame_world").value
        msg.child_frame_id = "imu"
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = (
            float(T[0, 3]), float(T[1, 3]), float(T[2, 3]),
        )
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = (
            float(q[0]), float(q[1]), float(q[2]), float(q[3]),
        )
        # 世界系速度：dy/dt 沿世界 +y
        msg.twist.twist.linear.y = float(vy)
        self.pub_odom.publish(msg)

    def tick_imu(self) -> None:
        msg = Imu()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "imu"
        msg.linear_acceleration.z = -9.81
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        self.pub_imu.publish(msg)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = None
    try:
        node = SyntheticVioSource()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        # 幂等关闭：外部已经 shutdown（例如 Ctrl-C 或父进程信号）时不再重复调用，
        # 否则会在日志里留下 'rcl_shutdown already called' 的噪声，掩盖真正的失败原因。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
