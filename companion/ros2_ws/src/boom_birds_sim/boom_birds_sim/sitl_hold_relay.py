"""TEST-ONLY（**旧路径**）：SIH 启动时发送起点悬停设定点，随后接管 EGO 轨迹。

已由 `lifecycle_node` + `ControlCommand` 会话协议取代：正式入口
`launch/px4_sih_mission.launch.py` 显式传 `hold_relay="false"`，只有旧的分段启动
`px4_sitl_motion.launch.py`（`bootstrap_only=true and hold_relay=true`）才启用。
保留它只为兼容旧的分段实验，它**不**参与会话/轨迹号/取消协议，因此不得作为
新链路的控制出口。高度门槛与接管距离都取自 RuntimeConfig，不另写数值。
"""

from __future__ import annotations

from boom_birds_control.runtime_config import DEFAULTS

import math

import rclpy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node
from boom_birds_control.handoff import DEFAULT_HANDOFF_DISTANCE_M, handoff_allowed


class SitlHoldRelay(Node):
    def __init__(self) -> None:
        super().__init__("boom_birds_sitl_hold_relay")
        # 字段名与参考系见 runtime_config.ALTITUDE_REFERENCE：相对起飞点地面。
        self.declare_parameter("hold_lock_min_altitude_agl_m",
                              DEFAULTS.hold_lock_min_altitude_agl_m)
        self.declare_parameter("handoff_max_distance_m", DEFAULT_HANDOFF_DISTANCE_M)
        self._hold: tuple[float, float, float] | None = None
        self._latest_pose: tuple[float, float, float] | None = None
        self._forwarding = False
        self._publisher = self.create_publisher(
            PositionCommand, "/boom_birds/ego/position_cmd", 10)
        self.create_subscription(Odometry, DEFAULTS.odom_topic, self._on_odom, 10)
        self.create_subscription(PositionCommand, "/boom_birds/ego/position_cmd_raw",
                                 self._on_ego, 10)
        self.create_timer(0.05, self._publish_hold)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        pose = (float(p.x), float(p.y), float(p.z))
        if not all(math.isfinite(v) for v in pose):
            return
        self._latest_pose = pose
        if self._hold is None and pose[2] >= float(
                self.get_parameter("hold_lock_min_altitude_agl_m").value):
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
        if not self._forwarding and not handoff_allowed(
                (float(p.x), float(p.y), float(p.z)), self._latest_pose,
                float(self.get_parameter("handoff_max_distance_m").value)):
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
