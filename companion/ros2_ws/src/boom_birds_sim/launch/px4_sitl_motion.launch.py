"""TEST-ONLY：双目深度/EGO/PX4 SIH 的运动仿真链。

需先单独启动已核实的 PX4 SIH -i 0，并生成合成标定。
本 launch 不启动飞控、不解锁、不切 OFFBOARD；只允许回环 MAVLink。

配置来源（A2 单一来源）：

* 阈值、契约话题与坐标系一律取 ``boom_birds_nav.runtime_config``
  （``DEFAULTS`` / ``SCENES``），本文件不再写死这些数值；
* 本文件里剩下的数字是**仿真场景参数**（mockamap/random_forest 的形状、体素
  分辨率、目标点），既不参与高度/接管判定，也没有对应的 RuntimeConfig 字段；
* 节点自身参数（例如 stereo_source 的 ``rate_hz``、``synth_pose_timeout_s``）不在这里
  重复声明，直接使用节点代码里的默认值——重复声明等于又开一个定义点。
"""
from boom_birds_control.runtime_config import DEFAULTS

import os

from boom_birds_control.runtime_config import DEFAULTS, SCENES

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

#: 契约话题（config/contract.yaml）：合成相机位姿由运动仿真更新。RuntimeConfig
#: 没有这一项（它不是运行阈值，而是仿真链路话题），因此登记在这里唯一一处。
SIM_CAMERA_POSE_TOPIC = DEFAULTS.camera_pose_topic
#: forest_30m 场景名（runtime.yaml 的 scenes 段）。
REFERENCE_SCENE_NAME = "forest_30m"


def reference_scene():
    """forest_30m 场景：PX4 局部原点在 ROS global 下的位置。缺失即明确失败。"""
    try:
        return SCENES[REFERENCE_SCENE_NAME]
    except KeyError as exc:
        raise RuntimeError(
            f"runtime.yaml 缺少 {REFERENCE_SCENE_NAME} 场景，无法建立 SIH 参考原点") from exc


def planner_from_calibration(context, ego_launch):
    args = {"use_camera_info": "true"}
    args.update({key: LaunchConfiguration(key).perform(context) for key in ("goal_x", "goal_y", "goal_z")})
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(ego_launch), launch_arguments=args.items())]


def generate_launch_description():
    sim_share = get_package_share_directory("boom_birds_sim")
    ego_share = get_package_share_directory("ego_planner")
    calibration = LaunchConfiguration("calibration_file")
    ego_launch = os.path.join(ego_share, "launch", "boom_birds_offline.launch.py")
    return LaunchDescription([
        SetEnvironmentVariable("RMW_FASTRTPS_PUBLICATION_MODE", os.getenv("RMW_FASTRTPS_PUBLICATION_MODE", "ASYNCHRONOUS")),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("boom_birds_bringup"), "launch", "mavros.launch.py"))),
        DeclareLaunchArgument(
            "calibration_file",
            default_value="/tmp/boom_birds_synth/synthetic_candidate.npz",
        ),
        DeclareLaunchArgument("require_session", default_value="false"),
        DeclareLaunchArgument("control_mode", default_value="px4_position"),
        DeclareLaunchArgument("attitude_config_file", default_value=os.path.join(get_package_share_directory("boom_birds_control"), "config", "attitude_sih.yaml")),
        DeclareLaunchArgument("sih_pid", default_value="0"),
        DeclareLaunchArgument("platform_config_file", default_value=""),
        DeclareLaunchArgument("platform_test_only", default_value="false"),
        DeclareLaunchArgument("hold_relay", default_value="true"),
        DeclareLaunchArgument("goal_x", default_value="2.5"),
        DeclareLaunchArgument("output_scale", default_value="0.75"),
        DeclareLaunchArgument("ego_reference_scene", default_value="false"),
        DeclareLaunchArgument("forest_seed", default_value="1"),
        DeclareLaunchArgument("forest_obs_num", default_value="250"),
        DeclareLaunchArgument("forest_circle_num", default_value="250"),
        DeclareLaunchArgument("forest_x_size", default_value="26.0"),
        DeclareLaunchArgument("forest_center_x", default_value="15.0"),
        DeclareLaunchArgument("goal_y", default_value="1.2"),
        DeclareLaunchArgument("goal_z", default_value="1.2"),
        DeclareLaunchArgument("use_mockamap", default_value="false"),
        DeclareLaunchArgument("use_random_forest", default_value="false"),
        DeclareLaunchArgument("bootstrap_only", default_value="false"),
        DeclareLaunchArgument("synth_map_topic", default_value=""),
        DeclareLaunchArgument("synth_map_resolution_m", default_value="0.2"),
        # 合成输入发布下限：唯一来源是 RuntimeConfig.image_publish_min_altitude_agl_m。
        DeclareLaunchArgument("synth_min_altitude_m",
                              default_value=str(DEFAULTS.image_publish_min_altitude_agl_m)),
        # 深度可用量程由**场景**决定（见 RuntimeConfig.depth_max_range_m 的说明）：
        # 30 m 森林最近障碍约 10.4 m，用 5 m 量程时一个有效像素都没有、地图永不就绪。
        DeclareLaunchArgument("depth_max_range_m",
                              default_value=str(DEFAULTS.depth_max_range_m)),
        LogInfo(msg="[PX4 SITL MOTION] TEST-ONLY: synthetic stereo + PX4 EKF truth; no real camera/VIO"),
        Node(package="mockamap", executable="mockamap_node", name="boom_birds_mockamap",
             output="screen", condition=IfCondition(LaunchConfiguration("use_mockamap")),
             remappings=[("mock_map", DEFAULTS.sim_map_topic)],
             parameters=[{
                 "seed": 511,
                 "update_freq": 0.2,
                 "resolution": 0.2,
                 "x_length": 20,
                 "y_length": 20,
                 "z_length": 4,
                 "type": 2,
                 "width_min": 0.6,
                 "width_max": 1.5,
                 "obstacle_number": 25,
             }]),
        Node(package="map_generator", executable="random_forest", name="boom_birds_random_forest",
             output="screen", condition=IfCondition(LaunchConfiguration("use_random_forest")),
             remappings=[("/map_generator/global_cloud", DEFAULTS.sim_map_topic),
                         ("odometry", DEFAULTS.odom_topic)],
             parameters=[{
                 "map/x_size": ParameterValue(LaunchConfiguration("forest_x_size"), value_type=float),
                 "map/y_size": 20.0, "map/z_size": 3.0,
                 "map/center_x": ParameterValue(LaunchConfiguration("forest_center_x"), value_type=float),
                 "map/center_y": 0.0,
                 "map/seed": ParameterValue(LaunchConfiguration("forest_seed"), value_type=int),
                 "map/resolution": 0.1, "map/obs_num": ParameterValue(LaunchConfiguration("forest_obs_num"), value_type=int),
                 "map/circle_num": ParameterValue(LaunchConfiguration("forest_circle_num"), value_type=int),
                 "ObstacleShape/lower_rad": 0.5, "ObstacleShape/upper_rad": 0.7,
                 "ObstacleShape/lower_hei": 0.0, "ObstacleShape/upper_hei": 3.0,
                 "ObstacleShape/radius_l": 0.7, "ObstacleShape/radius_h": 0.5,
                 "ObstacleShape/z_l": 0.7, "ObstacleShape/z_h": 0.8,
                 "ObstacleShape/theta": 0.5, "pub_rate": 1.0, "min_distance": 0.8,
             }]),
        Node(package="boom_birds_sim", executable="sitl_truth_source",
             name="boom_birds_sitl_truth", output="screen",
             condition=UnlessCondition(LaunchConfiguration("ego_reference_scene")),
             parameters=[{"sih_pid": ParameterValue(LaunchConfiguration("sih_pid"), value_type=int)}]),
        Node(package="boom_birds_sim", executable="sitl_truth_source",
             name="boom_birds_sitl_truth", output="screen",
             condition=IfCondition(LaunchConfiguration("ego_reference_scene")),
             parameters=[{"sih_pid": ParameterValue(LaunchConfiguration("sih_pid"), value_type=int),
                          "world_origin_ros_m": list(reference_scene().origin)}]),
        Node(package="boom_birds_sim", executable="synthetic_stereo_source",
             name="boom_birds_stereo_source", output="screen",
             parameters=[{
                 "mode": "synth",
                 "synth_pose_topic": SIM_CAMERA_POSE_TOPIC,
                 "synth_map_topic": LaunchConfiguration("synth_map_topic"),
                 "synth_map_resolution_m": ParameterValue(
                     LaunchConfiguration("synth_map_resolution_m"), value_type=float),
                 "synth_ground_z_m": ParameterValue(PythonExpression([
                     str(reference_scene().origin[2]), " if '", LaunchConfiguration("ego_reference_scene"), "' == 'true' else 0.0"]), value_type=float),
                 "synth_min_altitude_m": ParameterValue(
                     LaunchConfiguration("synth_min_altitude_m"), value_type=float),
             }]),
        # SIH 的 IMU 由 sitl_truth_source 提供（它同时发布 /boom_birds/imu）。
        # 这里**不得**再起 vio_source：那会在同一话题上出现第二个发布者，违反契约
        # "合成替身与其它来源不得同时占用同一话题"。
        Node(package="boom_birds_sensing", executable="depth_node",
             name="boom_birds_depth", output="screen",
             parameters=[{"calibration_file": calibration, "frame_id": DEFAULTS.camera_frame,
                          "output_scale": ParameterValue(LaunchConfiguration("output_scale"), value_type=float),
                          "publish_color_preview": True,
                          "max_depth_m": ParameterValue(LaunchConfiguration("depth_max_range_m"), value_type=float)}]),
        Node(package="boom_birds_sim", executable="sitl_hold_relay",
             name="boom_birds_sitl_hold_relay", output="screen",
             condition=IfCondition(PythonExpression(["\'", LaunchConfiguration("bootstrap_only"), "\' == \'true\' and \'", LaunchConfiguration("hold_relay"), "\' == \'true\'"]))),
        OpaqueFunction(function=planner_from_calibration, args=[ego_launch],
                       condition=UnlessCondition(LaunchConfiguration("bootstrap_only"))),
        Node(package="boom_birds_control", executable="px4_interface_node",
             name="boom_birds_px4_interface", output="screen",
             condition=UnlessCondition(LaunchConfiguration("ego_reference_scene")),
             parameters=[os.path.join(sim_share, "config", "px4_sitl_motion.yaml"),
                         {"control_mode": LaunchConfiguration("control_mode"),
                          "attitude_config_file": LaunchConfiguration("attitude_config_file"),
                          "require_session": ParameterValue(LaunchConfiguration("require_session"), value_type=bool),
                          "sih_pid": ParameterValue(LaunchConfiguration("sih_pid"), value_type=int),
                          "platform_config_file": LaunchConfiguration("platform_config_file"),
                          "platform_test_only": ParameterValue(LaunchConfiguration("platform_test_only"), value_type=bool),
                          "allow_arming": ParameterValue(LaunchConfiguration("require_session"), value_type=bool)}]),
        Node(package="boom_birds_control", executable="px4_interface_node",
             name="boom_birds_px4_interface", output="screen",
             condition=IfCondition(LaunchConfiguration("ego_reference_scene")),
             parameters=[os.path.join(sim_share, "config", "px4_sitl_motion.yaml"),
                         {"control_mode": LaunchConfiguration("control_mode"),
                          "attitude_config_file": LaunchConfiguration("attitude_config_file"),
                          "require_session": ParameterValue(LaunchConfiguration("require_session"), value_type=bool),
                          "sih_pid": ParameterValue(LaunchConfiguration("sih_pid"), value_type=int),
                          "platform_config_file": LaunchConfiguration("platform_config_file"),
                          "platform_test_only": ParameterValue(LaunchConfiguration("platform_test_only"), value_type=bool),
                          "allow_arming": ParameterValue(LaunchConfiguration("require_session"), value_type=bool),
                          "frame_alignment": "declared",
                          "frame_alignment_translation_m": list(reference_scene().origin),
                          "frame_alignment_origin_note":
                              f"TEST-ONLY: PX4 SIH EKF origin maps to ROS {tuple(reference_scene().origin)}"}]),
    ])
