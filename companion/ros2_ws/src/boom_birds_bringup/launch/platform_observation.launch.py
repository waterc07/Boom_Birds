"""Image/rosbag input only; no capture or control processes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_file",default_value=""),
        DeclareLaunchArgument("test_only",default_value="false"),
        DeclareLaunchArgument("image_topic",default_value="/boom_birds/downward/image"),
        Node(package="boom_birds_sensing",executable="platform_observation_node",output="screen",
             parameters=[{"config_file":LaunchConfiguration("config_file"),
                          "test_only":ParameterValue(LaunchConfiguration("test_only"),value_type=bool),
                          "image_topic":LaunchConfiguration("image_topic")}]),
    ])
