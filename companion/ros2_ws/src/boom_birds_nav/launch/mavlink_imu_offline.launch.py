"""启动 MAVLink IMU 接收节点（真实链路；默认连串口，可切换 UDP 做脱机联调）。

本 launch **不会**连接任何设备，除非用户显式给出可达的 connection：
- 真机：`connection:=serial:/dev/ttyAMA0 baud:=115200`
- 脱机 UDP 联调：`connection:=udpin:127.0.0.1:14555`，再用测试脚本或
  `mavlink_imu_replay` 发送模拟/记录的 MAVLink 消息。

注意：本 launch 只启动 IMU 上行接收。它不会启动 Offboard、不会解锁、不发送 setpoint。
同时不要与 TEST-ONLY 的 `vio_source` 一起运行：两者会争抢 /boom_birds/imu。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav_share = get_package_share_directory("boom_birds_nav")
    default_params = os.path.join(nav_share, "config", "mavlink_imu.yaml")

    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="节点参数文件（默认随包安装的 config/mavlink_imu.yaml）",
        ),
        DeclareLaunchArgument(
            "connection",
            default_value="serial:/dev/ttyAMA0",
            description="链路：serial:<设备> | udpin:ip:port | udpout:ip:port | tcp:ip:port",
        ),
        DeclareLaunchArgument("baud", default_value="115200", description="串口波特率（仅 serial:）"),
        DeclareLaunchArgument("stream_rate_hz", default_value="50.0",
                              description="请求的 HIGHRES_IMU 频率（实际达成看诊断）"),
        DeclareLaunchArgument("timesync_rate_hz", default_value="2.0"),
        DeclareLaunchArgument("sync_timeout_s", default_value="1.0"),
        DeclareLaunchArgument("max_rtt_s", default_value="0.02"),
        DeclareLaunchArgument("imu_topic", default_value="/boom_birds/imu"),
        DeclareLaunchArgument("apply_camera_imu_offset", default_value="false",
                              description="是否启用相机—IMU 时间偏移归算（未标定须为 false）"),
        DeclareLaunchArgument("camera_imu_offset_s", default_value="0.0",
                              description="offset = t_cam_ros - t_imu_ros"),
    ]

    return LaunchDescription(args + [
        LogInfo(msg=["[mavlink_imu] connection=", LaunchConfiguration("connection"),
                     " baud=", LaunchConfiguration("baud")]),
        Node(
            package="boom_birds_nav",
            executable="mavlink_imu_node",
            name="boom_birds_mavlink_imu",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "connection": LaunchConfiguration("connection"),
                    "baud": ParameterValue(LaunchConfiguration("baud"), value_type=int),
                    "stream_rate_hz": ParameterValue(LaunchConfiguration("stream_rate_hz"),
                                                     value_type=float),
                    "timesync_rate_hz": ParameterValue(LaunchConfiguration("timesync_rate_hz"),
                                                       value_type=float),
                    "sync_timeout_s": ParameterValue(LaunchConfiguration("sync_timeout_s"),
                                                     value_type=float),
                    "max_rtt_s": ParameterValue(LaunchConfiguration("max_rtt_s"), value_type=float),
                    "imu_topic": LaunchConfiguration("imu_topic"),
                    "apply_camera_imu_offset": ParameterValue(
                        LaunchConfiguration("apply_camera_imu_offset"), value_type=bool
                    ),
                    "camera_imu_offset_s": ParameterValue(
                        LaunchConfiguration("camera_imu_offset_s"), value_type=float
                    ),
                },
            ],
        ),
    ])
