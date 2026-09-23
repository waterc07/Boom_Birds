"""真机双目采集 / 回放启动（唯一采集源）。

本 launch 只启动**一个**采集节点：真机模式打开 `/dev/videoN`，回放模式读保存帧。
两种模式发布语义完全一致（同一拼接帧 → 同一采集时间戳 → 左右图共享），
因此深度与 OpenVINS 都订阅同一对话题，绝不允许各自再打开相机。

用法：
    # 真机（必须先核验设备、分辨率与标定一致）
    ros2 launch boom_birds_nav stereo_camera.launch.py \\
        mode:=v4l2 device:=/dev/video0 capture_width:=1280 capture_height:=480 \\
        calibration_file:=<标定.npz>

    # 回放保存帧（脱机；记录帧没有曝光时间戳，时间戳由回放器合成）
    ros2 launch boom_birds_nav stereo_camera.launch.py \\
        mode:=replay path:=<目录或拼接图> calibration_file:=<标定.npz>

边界：本 launch 不启动深度节点、不启动 OpenVINS、不连接飞控。真机命令只应在
确认设备与标定之后使用；本任务未在真机上运行过它。
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
    default_params = os.path.join(nav_share, "config", "stereo_camera.yaml")

    args = [
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="节点参数文件"),
        DeclareLaunchArgument("mode", default_value="v4l2",
                              description="v4l2（真机）| replay（保存帧）"),
        DeclareLaunchArgument("device", default_value="/dev/video0"),
        DeclareLaunchArgument("path", default_value="", description="replay 模式的输入路径"),
        DeclareLaunchArgument("capture_width", default_value="1280",
                              description="拼接帧宽度（左右两目之和）"),
        DeclareLaunchArgument("capture_height", default_value="480"),
        DeclareLaunchArgument("capture_fps", default_value="60"),
        DeclareLaunchArgument("rate_hz", default_value="30.0"),
        DeclareLaunchArgument("replay_fps", default_value="30.0"),
        DeclareLaunchArgument("calibration_file", default_value="",
                              description="标定 npz；v4l2/replay 必需"),
        DeclareLaunchArgument("left_topic", default_value="/boom_birds/stereo/left_raw"),
        DeclareLaunchArgument("right_topic", default_value="/boom_birds/stereo/right_raw"),
    ]

    return LaunchDescription(args + [
        LogInfo(msg=["[stereo_camera] mode=", LaunchConfiguration("mode"),
                     " device=", LaunchConfiguration("device"),
                     " size=", LaunchConfiguration("capture_width"), "x",
                     LaunchConfiguration("capture_height")]),
        Node(
            package="boom_birds_nav",
            executable="stereo_source",
            name="boom_birds_stereo_source",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "mode": LaunchConfiguration("mode"),
                    "device": LaunchConfiguration("device"),
                    "path": LaunchConfiguration("path"),
                    "capture_width": ParameterValue(LaunchConfiguration("capture_width"),
                                                    value_type=int),
                    "capture_height": ParameterValue(LaunchConfiguration("capture_height"),
                                                     value_type=int),
                    "capture_fps": ParameterValue(LaunchConfiguration("capture_fps"),
                                                  value_type=int),
                    "rate_hz": ParameterValue(LaunchConfiguration("rate_hz"), value_type=float),
                    "replay_fps": ParameterValue(LaunchConfiguration("replay_fps"),
                                                 value_type=float),
                    "calibration_file": LaunchConfiguration("calibration_file"),
                    "left_topic": LaunchConfiguration("left_topic"),
                    "right_topic": LaunchConfiguration("right_topic"),
                },
            ],
        ),
    ])
