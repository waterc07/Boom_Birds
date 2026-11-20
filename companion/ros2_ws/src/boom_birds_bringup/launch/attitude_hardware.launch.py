# ******************************************************************************
# @file           : attitude_hardware.launch.py
# @brief          : 启动相机和 IMU 输入、OpenVINS、双目深度及姿态推力控制链。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

"""图像＋IMU→OpenVINS（按配置单目/双目），双目→深度；位置闭环→姿态＋推力；默认 dry-run，缺标定拒绝启动。"""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from boom_birds_control.runtime_config import DEFAULTS
from boom_birds_bringup.hardware_profile import validateProfile
from boom_birds_bringup.compute_profile import rawGeometry, openvinsParameters


def nodes(context):
    _defaults = {"shared_decode": "auto", "flight_mode": "auto", "recovery_enabled": "false", "recovery_mode": "off"}
    def get(k):
        return LaunchConfiguration(k, default=_defaults.get(k)).perform(context)
    def boolean(k):
        v=(get(k) or "false").lower()
        if v not in ("true","false"): raise ValueError(k+" 必须为 true/false")
        return v == "true"
    mapBounds = {k: LaunchConfiguration(k, default=v).perform(context) for k, v in
                  (("map_min_z", "-0.1"), ("map_size_z", "6.0"), ("virtual_ceil_z", "2.9"))}
    import math
    lower, height, ceiling = (float(mapBounds[k]) for k in ("map_min_z", "map_size_z", "virtual_ceil_z"))
    if (not all(math.isfinite(v) for v in (lower, height, ceiling))
            or height <= 0 or not lower < ceiling <= lower + height):
        raise ValueError("Require finite map_min_z < virtual_ceil_z <= map_min_z + map_size_z")
    flightMode = get("flight_mode") or "auto"
    if flightMode not in ("auto", "ground", "dry_run", "trial", "live"):
        raise ValueError("flight_mode")
    if flightMode == "auto":
        dryRun, allowArming = boolean("dry_run"), boolean("allow_arming")
        allowNonLoopback, allowHardware = boolean("allow_non_loopback"), boolean("allow_hardware_actions")
    else:
        dryRun = flightMode in ("ground", "dry_run")
        allowArming = flightMode in ("trial", "live")
        allowNonLoopback = flightMode == "live"
        allowHardware = flightMode in ("trial", "live")

    args={k:get(k) for k in ("fcu_url","calibration_file","extrinsics_file","vio_config_file","attitude_config_file")}
    validateProfile(**args, live=not dryRun)
    bringup=Path(get_package_share_directory("boom_birds_bringup"))
    sensing=Path(get_package_share_directory("boom_birds_sensing"))
    profile = get("compute_profile")
    if profile not in ("mono_budget", "calibrated"):
        raise ValueError("compute_profile 必须为 mono_budget 或 calibrated")
    budget = profile == "mono_budget"
    vioParams = openvinsParameters(args["vio_config_file"], budget=budget, sourceRateLimited=True)
    geometry = rawGeometry(args["vio_config_file"], int(get("capture_width")), int(get("capture_height")))
    paramsFile = str(bringup/"config/mono_budget.yaml")
    sourceParams = [str(sensing/"config/stereo_camera.yaml")]
    if budget:
        sourceParams.append(paramsFile)
    sourceParams.append({"mode": "v4l2", "device": get("camera_device"),
        "capture_width": int(get("capture_width")), "capture_height": int(get("capture_height")),
        "capture_fps": int(get("capture_fps")), "calibration_file": args["calibration_file"], **geometry})
    depthParams = [paramsFile] if budget else []
    depthParams.append({"calibration_file": args["calibration_file"],
        "output_scale": float(get("output_scale")), "max_depth_m": DEFAULTS.depth_max_range_m})
    shared = True if get("shared_decode") == "auto" else boolean("shared_decode")
    if budget and not shared:
        sourceParams[-1]["publish_mjpeg"] = True
    if shared:
        sourceParams[-1]["publish_mjpeg"] = False
        sourceParams[-1]["publish_stitched"] = False
        depthParams[-1].update(process_latest_only=True, opencv_threads=2,
                               image_queue_depth=2, sync_queue_depth=2)
    sensingNodes = ([Node(package="boom_birds_sensing", executable="shared_stereo_depth", output="screen",
        parameters=sourceParams[:-1] + [{**sourceParams[-1], **depthParams[-1]}])] if shared else [
        Node(package="boom_birds_sensing", executable="stereo_source", output="screen", parameters=sourceParams),
        Node(package="boom_birds_sensing", executable="depth_node", output="screen", parameters=depthParams)])

    plannerParams = ({"resolution": "0.2", "skip_pixel": "2", "use_distinctive_trajs": "false"}
                      if budget else {})
    ego=Path(get_package_share_directory("ego_planner"))/"launch/boom_birds_offline.launch.py"
    include=lambda path,kw: IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),launch_arguments=kw.items())
    return [
        include(bringup/"launch/mavros.launch.py",{"fcu_url":args["fcu_url"]}),
        Node(package="boom_birds_sensing",executable="mavros_imu_node",output="screen",
            parameters=[str(sensing/"config/mavros_imu.yaml")]),
        *sensingNodes,
        Node(package="ov_msckf", executable="run_subscribe_msckf", namespace="ov_msckf",
            output="screen", parameters=[vioParams],
            remappings=[("/ov_msckf/odomimu", DEFAULTS.odom_imu_topic)]),
        Node(package="boom_birds_sensing",executable="pose_adapter",output="screen",
            parameters=[{"extrinsics_file":args["extrinsics_file"],"calibration_file":args["calibration_file"],
                         "rate_hz":50.0}]),
        include(ego,{**plannerParams,**mapBounds,"use_camera_info":"true","require_session":"true",
            "world_frame":DEFAULTS.world_frame,"max_ray_length":str(DEFAULTS.depth_max_range_m),
            "compute_budget_s":str(DEFAULTS.planning_compute_budget_s),
            "authorization_timeout_s":str(DEFAULTS.command_timeout_s),
            "command_future_tolerance_s":str(DEFAULTS.command_future_tolerance_s),
            "planning_horizon":str(DEFAULTS.planning_horizon_m),
            "obstacle_clearance":str(DEFAULTS.planning_obstacle_clearance_m),
            "obstacles_inflation":str(DEFAULTS.planning_obstacles_inflation_m),
            "depth_pose_tolerance_s":str(DEFAULTS.depth_pose_sync_tolerance_s),
            "max_vel":str(DEFAULTS.planning_max_velocity_m_s),"max_acc":str(DEFAULTS.planning_max_acceleration_m_s2)}),
        Node(package="boom_birds_control",executable="px4_interface_node",output="screen",
            parameters=[{"control_mode":"companion_attitude","attitude_config_file":args["attitude_config_file"],
                "backend":"mavros","fcu_url":args["fcu_url"],"require_session":True,"dry_run":dryRun,
                "allow_arming":allowArming,"allow_non_loopback":allowNonLoopback,
                "allow_hardware_actions":allowHardware,"recovery_mode":get("recovery_mode") or "off",
                "hardware_validation_note":get("hardware_validation_note")}]),
        Node(package="boom_birds_bringup",executable="lifecycle_node",output="screen",
            parameters=[{"control_mode":"companion_attitude","attitude_config_file":args["attitude_config_file"],
                         "recovery_enabled":boolean("recovery_enabled")}]),
    ]


def generate_launch_description():
    defaults={"fcu_url":"","calibration_file":"","extrinsics_file":"","vio_config_file":"",
        "attitude_config_file":"","camera_device":"/dev/video0","capture_width":"2560",
        "capture_height":"960","capture_fps":"60","output_scale":"1.0","compute_profile":"mono_budget","shared_decode":"auto","dry_run":"true",
        "allow_arming":"false","allow_non_loopback":"false","allow_hardware_actions":"false",
        "hardware_validation_note":"","flight_mode":"auto","recovery_enabled":"false","recovery_mode":"off",
        "map_min_z":"-0.1","map_size_z":"6.0","virtual_ceil_z":"2.9"}
    return LaunchDescription([SetEnvironmentVariable("RMW_FASTRTPS_PUBLICATION_MODE", os.getenv("RMW_FASTRTPS_PUBLICATION_MODE", "ASYNCHRONOUS"))]+[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]
                             +[OpaqueFunction(function=nodes)])
