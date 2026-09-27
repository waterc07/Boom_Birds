"""TEST-ONLY：SIH 启动时发送起点悬停设定点，随后接管 EGO 轨迹。"""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node


class SitlHoldRelay(Node):
    def __init__(self) -> None:
        super().__init__("boom_birds_sitl_hold_relay")
        self.declare_parameter("hold_min_altitude_m", 1.3)
        self.declare_parameter("handoff_max_distance_m", 0.5)
        self._hold: tuple[float, float, float] | None = None
        self._latest_pose: tuple[float, float, float] | None = None
        self._forwarding = False
        self._publisher = self.create_publisher(
            PositionCommand, "/boom_birds/ego/position_cmd", 10)
        self.create_subscription(Odometry, "/boom_birds/vio/odom_ego", self._on_odom, 10)
        self.create_subscription(PositionCommand, "/boom_birds/ego/position_cmd_raw",
                                 self._on_ego, 10)
        self.create_timer(0.05, self._publish_hold)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        pose = (float(p.x), float(p.y), float(p.z))
        if not all(math.isfinite(v) for v in pose):
            return
        self._latest_pose = pose
        if self._hold is None and pose[2] >= float(self.get_parameter("hold_min_altitude_m").value):
            self._hold = pose
            self.get_logger().info(f"SIH 起点悬停设定点：{pose}")

    def _on_ego(self, msg: PositionCommand) -> None:
        if self._hold is None or self._latest_pose is None:
            return
        if msg.trajectory_flag != PositionCommand.TRAJECTORY_STATUS_READY:
            if self._forwarding:
                self._publisher.publish(msg)
            return
        p = msg.position
        distance = math.dist((float(p.x), float(p.y), float(p.z)), self._latest_pose)
        if not self._forwarding and distance > float(self.get_parameter("handoff_max_distance_m").value):
            self.get_logger().warn(
                f"拒绝跳入已走过的 EGO 轨迹：首点距当前位置 {distance:.2f} m",
                throttle_duration_sec=2.0)
            return
        if not self._forwarding:
            self._forwarding = True
            self.get_logger().info(f"从悬停切换到 EGO 轨迹：首点距离 {distance:.2f} m")
        self._publisher.publish(msg)

    def _publish_hold(self) -> None:
        if self._hold is None or self._forwarding:
            return
        msg = PositionCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "global"
        msg.position.x, msg.position.y, msg.position.z = self._hold
        msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        msg.trajectory_id = 1
        self._publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = SitlHoldRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
