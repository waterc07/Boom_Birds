"""兼容启动入口：实现位于 boom_birds_bringup/launch/px4_interface.launch.py。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    path = Path(get_package_share_directory("boom_birds_bringup")) / "launch/px4_interface.launch.py"
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)))])
