"""TEST-ONLY：双目深度/EGO/PX4 SIH 的运动仿真链。

需先单独启动已核实的 PX4 SIH -i 0，并生成合成标定。
本 launch 不启动飞控、不解锁、不切 OFFBOARD；只允许回环 MAVLink。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory("boom_birds_nav")
    ego_share = get_package_share_directory("ego_planner")
    calibration = LaunchConfiguration("calibration_file")
    ego_launch = os.path.join(ego_share, "launch", "boom_birds_offline.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument(
            "calibration_file",
            default_value="/tmp/boom_birds_synth/synthetic_candidate.npz",
        ),
        DeclareLaunchArgument("goal_x", default_value="2.5"),
        DeclareLaunchArgument("goal_y", default_value="1.2"),
        DeclareLaunchArgument("goal_z", default_value="1.2"),
        LogInfo(msg="[PX4 SITL MOTION] TEST-ONLY: synthetic stereo + PX4 EKF truth; no real camera/VIO"),
        Node(package="boom_birds_nav", executable="sitl_truth_source",
             name="boom_birds_sitl_truth", output="screen"),
        Node(package="boom_birds_nav", executable="stereo_source",
             name="boom_birds_stereo_source", output="screen",
             parameters=[{
                 "mode": "synth",
                 "rate_hz": 5.0,
                 "synth_pose_topic": "/boom_birds/vio/camera_pose",
                 "synth_pose_timeout_s": 0.5,
             }]),
        Node(package="boom_birds_nav", executable="depth_node",
             name="boom_birds_depth", output="screen",
             parameters=[{"calibration_file": calibration, "frame_id": "cam0_rect"}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ego_launch),
            launch_arguments={
                "goal_x": LaunchConfiguration("goal_x"),
                "goal_y": LaunchConfiguration("goal_y"),
                "goal_z": LaunchConfiguration("goal_z"),
            }.items(),
        ),
        Node(package="boom_birds_nav", executable="px4_interface_node",
             name="boom_birds_px4_interface", output="screen",
             parameters=[os.path.join(nav_share, "config", "px4_sitl_motion.yaml")]),
    ])
