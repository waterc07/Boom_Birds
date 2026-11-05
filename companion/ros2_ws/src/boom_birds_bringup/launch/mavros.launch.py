"""单一 MAVROS FCU 连接；默认回环，不解锁、不发送任务 setpoint。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = Path(get_package_share_directory("boom_birds_bringup")) / "config/mavros.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("fcu_url", default_value="udp://127.0.0.1:14540@127.0.0.1:14580"),
        DeclareLaunchArgument("config_file", default_value=str(config)),
        DeclareLaunchArgument("timesync_rate_hz",default_value="10.0"),
        DeclareLaunchArgument("convergence_samples",default_value="500"),
        Node(package="mavros", executable="mavros_node", output="screen",
            parameters=[LaunchConfiguration("config_file"), {"fcu_url": LaunchConfiguration("fcu_url"),
                "gcs_url": "", "tgt_system": 1, "tgt_component": 1, "fcu_protocol": "v2.0"}]),
        Node(package="boom_birds_sensing", executable="mavros_config_node", output="screen",
            parameters=[{"timesync_rate_hz":ParameterValue(LaunchConfiguration("timesync_rate_hz"),value_type=float),
                         "convergence_samples":ParameterValue(LaunchConfiguration("convergence_samples"),value_type=int)}]),
    ])
