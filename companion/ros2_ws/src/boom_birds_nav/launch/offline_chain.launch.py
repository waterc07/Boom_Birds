"""Boom_Birds 脱机链路启动（分段可启用）。

默认只启动链路本体：stereo_source → depth_node → pose_adapter(+合成 VIO 源)。
EGO 由 ego-planner-swarm 的 boom_birds_offline.launch.py 启动，通过话题接入。
所有资源路径经 ament share 或显式参数解析，不依赖当前工作目录。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory("boom_birds_nav")
    stereo_share = get_package_share_directory("stereo_depth")

    default_real_calib = os.path.join(stereo_share, "calibration", "live_20260916_210120_642136", "candidate.npz")

    args = [
        DeclareLaunchArgument("source_mode", default_value="synth", description="synth 或 file"),
        DeclareLaunchArgument("source_path", default_value="", description="file 模式的图像或目录"),
        DeclareLaunchArgument("source_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("calibration_file", default_value=default_real_calib,
                              description="深度节点使用的标定；合成验证请传合成标定"),
        DeclareLaunchArgument("extrinsics_file",
                              default_value=os.path.join(nav_share, "config", "extrinsics_synthetic_test.yaml")),
        DeclareLaunchArgument("camera_y_motion_amplitude_m", default_value="0.0"),
        DeclareLaunchArgument("pose_lag_s", default_value="0.0"),
        DeclareLaunchArgument("use_depth_gate", default_value="true"),
        DeclareLaunchArgument("enable_synthetic_vio", default_value="true"),
        DeclareLaunchArgument("enable_adapter", default_value="true"),
        DeclareLaunchArgument("max_depth_m", default_value="5.0"),
        DeclareLaunchArgument("min_depth_m", default_value="0.2"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    source = Node(
        package="boom_birds_nav",
        executable="stereo_source",
        name="boom_birds_stereo_source",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[{
            "mode": LaunchConfiguration("source_mode"),
            "path": LaunchConfiguration("source_path"),
            "rate_hz": LaunchConfiguration("source_rate_hz"),
            "synth_camera_y_offset_m": LaunchConfiguration("camera_y_motion_amplitude_m"),
        }],
    )

    vio = Node(
        package="boom_birds_nav",
        executable="vio_source",
        name="boom_birds_synthetic_vio",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_synthetic_vio")),
        parameters=[{
            "pose_lag_s": LaunchConfiguration("pose_lag_s"),
            "y_motion_amplitude_m": LaunchConfiguration("camera_y_motion_amplitude_m"),
            "y_motion_period_s": 12.0,
        }],
    )

    depth = Node(
        package="boom_birds_nav",
        executable="depth_node",
        name="boom_birds_depth",
        output="screen",
        parameters=[{
            "calibration_file": LaunchConfiguration("calibration_file"),
            "min_depth_m": LaunchConfiguration("min_depth_m"),
            "max_depth_m": LaunchConfiguration("max_depth_m"),
        }],
    )

    adapter = Node(
        package="boom_birds_nav",
        executable="pose_adapter",
        name="boom_birds_pose_adapter",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_adapter")),
        parameters=[{
            "extrinsics_file": LaunchConfiguration("extrinsics_file"),
            "calibration_file": LaunchConfiguration("calibration_file"),
            "use_depth_gate": LaunchConfiguration("use_depth_gate"),
        }],
    )

    return LaunchDescription(args + [source, vio, depth, adapter])
