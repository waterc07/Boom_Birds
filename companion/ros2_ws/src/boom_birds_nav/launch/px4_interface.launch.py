"""规划输出 → Px4Interface → PX4 的启动（默认全脱机安全值）。

默认行为：`backend:=fake`、`dry_run:=true`、`allow_arming:=false`
——启动后不连接任何飞控、不下发任何 setpoint，只把状态与失效原因发到
`/boom_birds/control/status`。

接 SITL 的推荐流程见本包 README「PX4 SITL 联调」一节；即便如此也只针对明确识别的
SITL 实例（回环地址），且本 launch 不会解锁、不会切模式。

    ros2 launch boom_birds_nav px4_interface.launch.py
    ros2 launch boom_birds_nav px4_interface.launch.py backend:=mavlink dry_run:=false
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
    default_params = os.path.join(nav_share, "config", "px4_interface.yaml")

    args = [
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("backend", default_value="fake", description="fake | mavlink"),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("allow_arming", default_value="false"),
        DeclareLaunchArgument("connection", default_value="udpin:127.0.0.1:14540",
                              description="仅回环地址；SITL 默认 UDP"),
        DeclareLaunchArgument("control_rate_hz", default_value="50.0"),
    ]

    return LaunchDescription(args + [
        LogInfo(msg=["[px4_interface] backend=", LaunchConfiguration("backend"),
                     " dry_run=", LaunchConfiguration("dry_run"),
                     " allow_arming=", LaunchConfiguration("allow_arming")]),
        Node(
            package="boom_birds_nav",
            executable="px4_interface_node",
            name="boom_birds_px4_interface",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "backend": LaunchConfiguration("backend"),
                    "dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
                    "allow_arming": ParameterValue(LaunchConfiguration("allow_arming"),
                                                   value_type=bool),
                    "connection": LaunchConfiguration("connection"),
                    "control_rate_hz": ParameterValue(LaunchConfiguration("control_rate_hz"),
                                                      value_type=float),
                },
            ],
        ),
    ])
