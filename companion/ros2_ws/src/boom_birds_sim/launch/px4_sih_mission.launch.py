"""TEST-ONLY SIH mission：launch 只负责起节点，Mission.START 才发起任务。

只用于本机 PX4 SIH；参数从 ``boom_birds_nav.runtime_config`` 取值（A2 单一来源），
本文件不重复写高度阈值。SIH 专用地显式打开 ``recovery_enabled``（实机默认关闭）。
"""
from boom_birds_control.runtime_config import DEFAULTS
from boom_birds_sim.forest_scenes import load_profile
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from boom_birds_nav.runtime_config import SCENES

#: 仿真地图话题与体素分辨率（不是运行阈值，属于仿真场景参数）。
#: **必须与各生成器自己的 resolution 一致**：mockamap 节点按 0.2 m 生成、
#: random_forest 按 0.1 m 生成。此前两者统一按 0.1 m 采样 mockamap 的 0.2 m 点云，
#: 会让合成深度看到与生成器不同的占据分布，EGO 于是把自己的轨迹判为
#: collision_free=0 并拒绝（SIH 实测：能进 OFFBOARD 但到不了目标）。
SIM_MAP_TOPIC = DEFAULTS.sim_map_topic
SIM_MAP_RESOLUTION_M = {"forest_30m": 0.1, "local": 0.2, "recovery_local": 0.2}
#: forest_30m 场景名（runtime.yaml 的 scenes 段）。
REFERENCE_SCENE_NAME = "forest_30m"


def nodes(context):
    scene = LaunchConfiguration("scene").perform(context)
    if scene not in SCENES: raise ValueError(f"unknown SIH scene: {scene}")
    config = SCENES[scene]
    mode = LaunchConfiguration("control_mode").perform(context)
    if mode not in ("px4_position", "companion_attitude"): raise ValueError("control_mode")
    profile = str(Path(get_package_share_directory("boom_birds_control")) / "config/attitude_sih.yaml")
    sim = Path(get_package_share_directory("boom_birds_sim")) / "launch"
    ego = Path(get_package_share_directory("ego_planner")) / "launch/boom_birds_offline.launch.py"
    args = {name: LaunchConfiguration(name).perform(context) for name in ("calibration_file", "output_scale", "sih_pid", "forest_seed", "platform_config_file")}
    if scene == REFERENCE_SCENE_NAME:
        args.update({key: str(value) for key, value in
                     load_profile(LaunchConfiguration("forest_profile").perform(context)).items()})
    args.update(platform_test_only="true" if args["platform_config_file"] else "false",control_mode=mode, attitude_config_file=profile, require_session="true", bootstrap_only="true", hold_relay="false",
        ego_reference_scene="true" if scene == REFERENCE_SCENE_NAME else "false",
        use_random_forest="true" if scene == REFERENCE_SCENE_NAME else "false",
        use_mockamap="false" if scene == REFERENCE_SCENE_NAME else "true",
        synth_map_topic=SIM_MAP_TOPIC,
        synth_map_resolution_m=str(SIM_MAP_RESOLUTION_M[scene]),
        synth_min_altitude_m="-1.0" if mode == "companion_attitude" else str(config.image_publish_min_altitude_agl_m),
        depth_max_range_m=str(config.depth_max_range_m))
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim / "px4_sitl_motion.launch.py")), launch_arguments=args.items()),
        # EGO 的 max_ray_length 就是"接受多远的深度"：invalid_depth_max_dist_ = 该值 + 0.1。
        # 与 depth_node 的 max_depth_m 取同一个场景量程，避免"发布了但被拒收"。
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(ego)),
                                 launch_arguments={"use_camera_info": "true", "require_session": "true",
                                                   "max_ray_length": str(config.depth_max_range_m),
                                                   "world_frame": config.world_frame,
                                                   "authorization_timeout_s": str(config.command_timeout_s),
                                                   "command_future_tolerance_s": str(config.command_future_tolerance_s),
                                                   "depth_topic": config.depth_topic,
                                                   "camera_info_topic": config.camera_info_topic,
                                                   "camera_pose_topic": config.camera_pose_topic,
                                                   "odom_topic": config.odom_topic,
                                                   "planner_request_topic": config.planner_request_topic,
                                                   "planner_status_topic": config.planner_status_topic,
                                                   "executor_status_topic": config.executor_status_topic,
                                                   "planner_command_topic": config.planner_command_topic,
                                                   "control_execution_topic": config.status_topic,
                                                   "compute_budget_s": str(config.planning_compute_budget_s),
                                                   "planning_horizon": str(config.planning_horizon_m),
                                                   "local_target_search_radius_m": str(config.local_target_search_radius_m),
                                                   "local_target_search_step_m": str(config.local_target_search_step_m),
                                                   "max_vel": str(config.planning_max_velocity_m_s),
                                                   "start_velocity_tolerance": str(config.handoff_max_speed_m_s),
                                                   "max_acc": str(config.planning_max_acceleration_m_s2),
                                                   "obstacle_clearance": str(config.planning_obstacle_clearance_m),
                                                   "obstacles_inflation": str(config.planning_obstacles_inflation_m),
                                                   "depth_pose_tolerance_s": str(config.depth_pose_sync_tolerance_s)}.items()),
        # TEST-ONLY：SIH 入口显式打开自动恢复；实机入口保持 RuntimeConfig 默认 false。
        Node(package="boom_birds_bringup", executable="lifecycle_node", output="screen",
             parameters=[{"scene": scene, "control_mode": mode, "attitude_config_file": profile,
                          "platform_landing_enabled": bool(args["platform_config_file"]),
                          "recovery_enabled": mode == "px4_position"}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="local"),
        DeclareLaunchArgument("control_mode", default_value="px4_position"),
        DeclareLaunchArgument("sih_pid", default_value="0"),
        DeclareLaunchArgument("platform_config_file", default_value=""),
        DeclareLaunchArgument("forest_seed", default_value="1"),
        DeclareLaunchArgument("forest_profile", default_value="reference_30m"),
        DeclareLaunchArgument("output_scale", default_value="0.75"),
        DeclareLaunchArgument("calibration_file", default_value="/tmp/boom_birds_synth/synthetic_candidate.npz"),
        OpaqueFunction(function=nodes),
    ])
