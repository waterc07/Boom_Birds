"""第 2 层验证启动：合成双目 + 深度 + 位姿适配。

与 offline_chain 的区别：深度节点使用**合成标定**（TEST-ONLY），使几何断言有解析预期。
合成标定必须**在启动前**生成，否则 depth_node/pose_adapter 会按契约显式报错退出
（缺标定不允许静默使用占位值）：

    python3 -m boom_birds_nav.synthetic --write-calibration /tmp/boom_birds_synth/synthetic_candidate.npz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SYNTH_CALIB = os.path.join("/tmp", "boom_birds_synth", "synthetic_candidate.npz")


def generate_launch_description():
    nav_share = get_package_share_directory("boom_birds_nav")

    args = [
        DeclareLaunchArgument("synth_calibration", default_value=SYNTH_CALIB,
                              description="启动前必须已生成的合成标定 npz（TEST-ONLY）"),
        DeclareLaunchArgument("source_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("camera_y_motion_amplitude_m", default_value="0.0"),
        DeclareLaunchArgument("pose_lag_s", default_value="0.0"),
        DeclareLaunchArgument("use_depth_gate", default_value="true"),
        # 离线合成/回放的调度抖动可达数百毫秒：把超时做成参数。
        # 默认仍取契约值 0.15 s（真机 VIO 数据率下合适）；离线长跑可放宽，
        # 避免把"调度抖动"误判成"VIO 失效"而断掉地图更新。
        DeclareLaunchArgument("pose_timeout_s", default_value="0.15"),
        # 世界坐标系原点偏移仅作用于合成位姿；深度保留相机相对距离。
        DeclareLaunchArgument("origin_offset_m", default_value="0.0"),
        # 可单独移动合成场景相对相机的深度平面，默认 0。
        DeclareLaunchArgument("scene_x_offset_m", default_value="0.0"),
        DeclareLaunchArgument("depth_timeout_s", default_value="1.0"),
    ]

    return LaunchDescription(args + [
        LogInfo(msg=["[layer2] 使用合成标定：", LaunchConfiguration("synth_calibration")]),
        Node(
            package="boom_birds_nav",
            executable="stereo_source",
            name="boom_birds_stereo_source",
            output="screen",
            parameters=[{
                "mode": "synth",
                "rate_hz": LaunchConfiguration("source_rate_hz"),
                "synth_camera_y_offset_m": LaunchConfiguration("camera_y_motion_amplitude_m"),
                "origin_offset_m": ParameterValue(LaunchConfiguration("origin_offset_m"), value_type=float),
                "scene_x_offset_m": ParameterValue(LaunchConfiguration("scene_x_offset_m"), value_type=float),
            }],
        ),
        Node(
            package="boom_birds_nav",
            executable="vio_source",
            name="boom_birds_synthetic_vio",
            output="screen",
            parameters=[{
                "pose_lag_s": LaunchConfiguration("pose_lag_s"),
                "y_motion_amplitude_m": LaunchConfiguration("camera_y_motion_amplitude_m"),
                "y_motion_period_s": 12.0,
                "origin_offset_m": ParameterValue(LaunchConfiguration("origin_offset_m"), value_type=float),
                "scene_x_offset_m": ParameterValue(LaunchConfiguration("scene_x_offset_m"), value_type=float),
            }],
        ),
        Node(
            package="boom_birds_nav",
            executable="depth_node",
            name="boom_birds_depth",
            output="screen",
            parameters=[{
                "calibration_file": LaunchConfiguration("synth_calibration"),
                "frame_id": "cam0_rect",
            }],
        ),
        Node(
            package="boom_birds_nav",
            executable="pose_adapter",
            name="boom_birds_pose_adapter",
            output="screen",
            parameters=[{
                "calibration_file": LaunchConfiguration("synth_calibration"),
                "extrinsics_file": os.path.join(nav_share, "config", "extrinsics_synthetic_test.yaml"),
                "odom_imu_topic": "/boom_birds/ov/odomimu",
                "use_depth_gate": LaunchConfiguration("use_depth_gate"),
                "pose_timeout_s": ParameterValue(LaunchConfiguration("pose_timeout_s"), value_type=float),
                "depth_timeout_s": ParameterValue(LaunchConfiguration("depth_timeout_s"), value_type=float),
            }],
        ),
    ])
