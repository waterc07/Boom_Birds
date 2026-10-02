"""兼容入口：启动 MAVROS IMU 适配器；MAVROS 连接须单独启动。"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package="boom_birds_sensing", executable="mavros_imu_node",
        name="boom_birds_mavros_imu", output="screen")])
