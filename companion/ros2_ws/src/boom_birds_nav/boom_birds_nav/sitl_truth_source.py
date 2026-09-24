"""TEST-ONLY：把 PX4 SIH 的局部位置回读为 ROS 仿真真值。

只连接回环 14550 的只读 MAVLink 链路；不发送 setpoint、模式或解锁命令。
这是仿真反馈，绝不能冒充 OpenVINS 里程计或真机相机/IMU 证据。
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import Imu

from .px4_backend import MavlinkPx4Backend
from .frames import rot_to_quat


def ned_to_ros_position(v):
    """本工程局部系约定：N→x, E→-y, D→-z。"""
    return float(v[0]), -float(v[1]), -float(v[2])


def px4_attitude_to_ros_rotation(roll: float, pitch: float, yaw: float):
    """PX4 NED/FRD 的 Euler 姿态转为工程世界 Z-up / 机体 FLU。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    r_ned_frd = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    axis = np.diag([1.0, -1.0, -1.0])
    return axis @ r_ned_frd @ axis


class SitlTruthSource(Node):
    def __init__(self):
        super().__init__("boom_birds_sitl_truth")
        self.declare_parameter("connection", "udpin:127.0.0.1:14550")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("max_state_age_s", 0.25)
        connection = str(self.get_parameter("connection").value)
        if connection != "udpin:127.0.0.1:14550":
            raise RuntimeError("SITL 真值源只允许连接本机 SIH 的 14550 端口")
        self.backend = MavlinkPx4Backend(
            connection=connection, dry_run=True, allow_arming=False,
            send_heartbeat=False, heartbeat_timeout_s=1.0,
        )
        self.backend.connect()
        self.first_epoch = None
        self.pub_odom = self.create_publisher(Odometry, "/boom_birds/ov/odomimu", 10)
        self.pub_body = self.create_publisher(Odometry, "/boom_birds/sitl/odom", 10)
        self.pub_path = self.create_publisher(Path, "/boom_birds/sitl/path", 10)
        self.pub_camera = self.create_publisher(PoseStamped, "/boom_birds/vio/camera_pose", 10)
        self.pub_ego = self.create_publisher(Odometry, "/boom_birds/vio/odom_ego", 10)
        self.pub_imu = self.create_publisher(Imu, "/boom_birds/imu", 10)
        self.path = Path()
        self.path.header.frame_id = "global"
        self._path_tick = 0
        rate = max(float(self.get_parameter("rate_hz").value), 1.0)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().warn("SITL 真值源：仅仿真，位置来自 PX4 EKF；不代表 VIO 初始化或真实 IMU")

    def tick(self):
        state = self.backend.read_vehicle_state()
        if self.first_epoch is None and state.connected:
            self.first_epoch = state.restart_epoch
        if self.first_epoch is not None and state.restart_epoch != self.first_epoch:
            self.get_logger().error("PX4 启动周期变化，真值源闭锁；需重启整条仿真链", once=True)
            return
        max_age = float(self.get_parameter("max_state_age_s").value)
        if (not state.connected or state.position_ned_m is None
                or state.velocity_ned_m_s is None or state.yaw_rad is None
                or state.roll_rad is None or state.pitch_rad is None
                or state.position_age_s is None or state.position_age_s > max_age
                or state.attitude_age_s is None or state.attitude_age_s > max_age):
            return
        x, y, z = ned_to_ros_position(state.position_ned_m)
        vx, vy, vz = ned_to_ros_position(state.velocity_ned_m_s)
        body_rotation = px4_attitude_to_ros_rotation(
            float(state.roll_rad), float(state.pitch_rad), float(state.yaw_rad))
        body_q = rot_to_quat(body_rotation)
        optical_from_body = np.array([
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ])
        camera_q = rot_to_quat(body_rotation @ optical_from_body)
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "global"
        odom.child_frame_id = "body"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
         odom.pose.pose.orientation.z, odom.pose.pose.orientation.w) = map(float, body_q)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        odom.twist.twist.angular.z = -float(state.yaw_rate_rad_s or 0.0)
        self.pub_odom.publish(odom)
        self.pub_body.publish(odom)
        self.pub_ego.publish(odom)
        self._path_tick += 1
        if self._path_tick % 3 == 0:
            pose = PoseStamped()
            pose.header = odom.header
            pose.pose = odom.pose.pose
            self.path.header.stamp = stamp
            self.path.poses.append(pose)
            self.path.poses = self.path.poses[-300:]
            self.pub_path.publish(self.path)

        # 光学系随机体姿态旋转；场景渲染器使用同一姿态作光线投影。
        camera = PoseStamped()
        camera.header.stamp = stamp
        camera.header.frame_id = "global"
        camera.pose.position.x = x
        camera.pose.position.y = y
        camera.pose.position.z = z
        (camera.pose.orientation.x, camera.pose.orientation.y,
         camera.pose.orientation.z, camera.pose.orientation.w) = map(float, camera_q)
        self.pub_camera.publish(camera)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "body"
        imu.orientation = odom.pose.pose.orientation
        imu.angular_velocity.z = odom.twist.twist.angular.z
        # 没有从 MAVLink 读到比力；用协方差 -1 显式标记不可用。
        imu.linear_acceleration_covariance[0] = -1.0
        self.pub_imu.publish(imu)

    def destroy_node(self):
        self.backend.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SitlTruthSource()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
