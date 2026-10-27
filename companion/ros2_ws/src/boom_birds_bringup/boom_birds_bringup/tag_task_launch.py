"""Launch the selected task with stereo VIO, Tag observation and one controller."""
import json
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from boom_birds_control.runtime_config import DEFAULTS
from boom_birds_control.platform_model import validateProfile as validateTag
from .hardware_profile import validateProfile, readYaml
from .compute_profile import rawGeometry, openvinsParameters
from .tag_flight_task import loadTask


def parameterValue(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return repr(value)
    return str(value)


def commandSpec(parameterList):
    return json.dumps([str(item) for item in parameterList])


def tagComputeProfile(name):
    if name == "configured":
        return dict(vio={},stereo={},depth={},downward={})
    if name != "balanced":
        raise ValueError("tag_compute_profile")
    return dict(vio=dict(max_clones=4,num_pts=80,max_msckf_in_update=10,
                         use_multi_threading_subs=False),
                stereo=dict(rate_hz=10.),depth=dict(max_processing_rate_hz=8.),
                downward=dict(publish_rate_hz=10.))


def taskNodes(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    def boolean(name):
        value = get(name).lower()
        if value not in ("true", "false"):
            raise ValueError(name+" must be true/false")
        return value == "true"
    dryRun, testOnly = boolean("dry_run"), boolean("tag_test_only")
    handover, startSensing = boolean("handover"), boolean("start_sensing")
    compute = tagComputeProfile(get("tag_compute_profile"))
    featureTracking = not handover or boolean("feature_tracking_diagnostics")
    if get("tag_compute_profile") == "balanced" and not startSensing:
        raise ValueError("tag_balanced_requires_owned_sensing")
    if testOnly and not dryRun:
        raise ValueError("tag_test_only_requires_dry_run")
    # 平台速度交接需要已核验的速度确认适配器；当前仅允许 dry-run/本机 SIH 证据。
    if handover and not dryRun:
        raise ValueError("tag_handover_requires_dry_run_until_hardware_verified")
    if handover and not startSensing:
        raise ValueError("tag_handover_requires_owned_vio_processes")
    args = {k: get(k) for k in ("fcu_url", "calibration_file", "extrinsics_file",
                                "vio_config_file", "attitude_config_file")}
    validateProfile(**args, live=not dryRun)
    vio = readYaml(args["vio_config_file"])
    if vio.get("max_cameras") != 2 or vio.get("use_stereo") is not True:
        raise ValueError("tag_tasks_require_stereo_vio")
    downwardInput,rangeInput=get("downward_input"),get("range_input")
    if downwardInput not in ("external","v4l2") or rangeInput not in ("external","mavros"):
        raise ValueError("tag_sensor_input_mode")
    if downwardInput=="v4l2":
        device=get("downward_device")
        if not device or Path(device).resolve()==Path(get("camera_device")).resolve():
            raise ValueError("downward_requires_distinct_explicit_device")
    loadTask(get("flight_task_file"))
    from .task_depth_guard import loadObstacleConfig
    loadObstacleConfig(get("obstacle_config_file"),live=not dryRun)
    tag = yaml.safe_load(Path(get("tag_config_file")).read_text(encoding="utf-8"))
    validateTag(tag, test_only=testOnly)
    bringup = Path(get_package_share_directory("boom_birds_bringup"))
    sensing = Path(get_package_share_directory("boom_birds_sensing"))
    controller = Node(package="boom_birds_control", executable="px4_interface_node", output="screen",
        parameters=[{"control_mode": "companion_attitude", "attitude_config_file": args["attitude_config_file"],
            "backend": "mavros", "fcu_url": args["fcu_url"], "require_session": True,
            "odom_topic":DEFAULTS.odom_body_topic,
            "dry_run": dryRun, "allow_arming": boolean("allow_arming"),
            "allow_non_loopback": boolean("allow_non_loopback"),
            "allow_hardware_actions": boolean("allow_hardware_actions"),
            "hardware_validation_note": get("hardware_validation_note"),
            "camera_health_topic": DEFAULTS.stereo_left_topic,
            # 交接/平台速度引导只用下视板几何 + 测距 + PX4 反馈，不依赖 VIO 世界系。
            "platform_config_file": get("tag_config_file") if handover else "",
            "platform_test_only": True,
            "platform_trace_directory": get("platform_trace_directory")}])
    mission = Node(package="boom_birds_bringup", executable="tag_flight_task", output="screen",
        parameters=[{"control_mode": "companion_attitude", "attitude_config_file": args["attitude_config_file"],
            "flight_task_file": get("flight_task_file"), "tag_config_file": get("tag_config_file"),
            "obstacle_config_file":get("obstacle_config_file"),
            "tag_test_only": testOnly, "dry_run": dryRun, "start": boolean("start"),
            "feature_tracking_enabled":featureTracking,"require_landing_inputs":True,
            "downward_image_topic":get("downward_image_topic"),
            "recovery_enabled": False,
            "platform_landing_enabled": handover, "handover_enabled": handover,
            "platform_status_timeout_s": float(get("platform_status_timeout_s"))}])
    detector = Node(package="boom_birds_sensing", executable="platform_observation_node", output="screen",
        parameters=[{"config_file": get("tag_config_file"), "test_only": testOnly,
                     "image_topic": get("downward_image_topic"),"image_frame_id":get("downward_frame_id"),"feature_tracking_enabled":featureTracking}])
    result = [controller, mission, detector]
    if downwardInput=="v4l2":
        result.append(Node(package="boom_birds_sensing",executable="downward_source",output="screen",
            parameters=[{"device":get("downward_device"),"config_file":get("downward_camera_config_file") or get("tag_config_file"),
                "native_config_file":get("downward_native_config_file"),
                "raw10_native_library":get("downward_raw10_native_library"),
                "media_device":get("downward_media_device"),"sensor_entity":get("downward_sensor_entity"),
                "pixel_format":get("downward_pixel_format"),"capture_fps":int(get("downward_capture_fps")),
                "image_topic":get("downward_image_topic"),"frame_id":get("downward_frame_id"),**compute["downward"]}]))
    if rangeInput=="mavros":
        result.append(Node(package="boom_birds_sensing",executable="mavros_range_node",output="screen",
            parameters=[{"sensor_id":int(get("range_sensor_id")),"target_system":int(get("range_target_system")),
                "target_component":int(get("range_target_component"))}]))
    if startSensing:
        geometry = rawGeometry(args["vio_config_file"], int(get("capture_width")), int(get("capture_height")))
        result += [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup/"launch/mavros.launch.py")),
                launch_arguments={"fcu_url": args["fcu_url"]}.items()),
            Node(package="boom_birds_sensing", executable="mavros_imu_node", output="screen",
                parameters=[str(sensing/"config/mavros_imu.yaml")]),
            Node(package="boom_birds_sensing", executable="stereo_source", output="screen",
                parameters=[str(sensing/"config/stereo_camera.yaml"),
                    {"mode": "v4l2", "device": get("camera_device"),
                     "capture_width": int(get("capture_width")), "capture_height": int(get("capture_height")),
                     "capture_fps": int(get("capture_fps")), "calibration_file": args["calibration_file"],
                     "publish_stitched": False, "publish_mjpeg": False, **geometry, **compute["stereo"]}]),
            Node(package="boom_birds_sensing",executable="depth_node",output="screen",
                parameters=[{"calibration_file":args["calibration_file"],"input_transport":"raw",
                    "process_latest_only":True,"max_processing_rate_hz":15.,
                    "opencv_threads":2,"image_queue_depth":2,"sync_queue_depth":2,
                    "publish_xyz":False,"publish_compact_xyz":False,"publish_depth_compat":False,**compute["depth"]}]),
        ]
        if handover:
            # VIO 与姿态适配器由任务自有的监管节点启动并持有句柄；交接确认后
            # 只有该节点能停止它们，退出回读与 CPU 释放写入 /boom_birds/compute/status。
            vioParameters = {**openvinsParameters(args["vio_config_file"], budget=False), **compute["vio"]}
            vioCommand = ["ros2", "run", "ov_msckf", "run_subscribe_msckf", "--ros-args",
                          "-r", "__ns:=/ov_msckf",
                          "-r", "/ov_msckf/odomimu:="+DEFAULTS.odom_imu_topic]
            for key, value in vioParameters.items():
                vioCommand += ["-p", key+":="+parameterValue(value)]
            adapterCommand = ["ros2", "run", "boom_birds_sensing", "pose_adapter", "--ros-args",
                              "-p", "extrinsics_file:="+args["extrinsics_file"],
                              "-p", "calibration_file:="+args["calibration_file"],
                              "-p", "rate_hz:=50.0"]
            commands = [commandSpec(vioCommand), commandSpec(adapterCommand)]
            result.append(Node(package="boom_birds_bringup", executable="compute_supervisor", output="screen",
                parameters=[{"commands": commands,
                             "log_directory": get("compute_log_directory"),
                             "test_only": True, "dry_run": dryRun,
                             "status_rate_hz": 2.}]))
        else:
            result += [
                Node(package="ov_msckf", executable="run_subscribe_msckf", namespace="ov_msckf",
                    output="screen", parameters=[{**openvinsParameters(args["vio_config_file"], budget=False), **compute["vio"]}],
                    remappings=[("/ov_msckf/odomimu", DEFAULTS.odom_imu_topic)]),
                Node(package="boom_birds_sensing", executable="pose_adapter", output="screen",
                    parameters=[{"extrinsics_file": args["extrinsics_file"],
                                 "calibration_file": args["calibration_file"], "rate_hz": 50.}]),
            ]
    # A dead critical process must terminate the stack; stale setpoints still expire in PX4.
    for process in [item for item in result if isinstance(item, Node)]:
        result.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason="tag task process exited"))])))
    return result


def taskLaunch(name):
    share = Path(get_package_share_directory("boom_birds_bringup"))
    defaults = {"flight_task_file": str(share/"config/flight_tasks"/(name+".yaml")),
        "fcu_url": "", "calibration_file": "", "extrinsics_file": "", "vio_config_file": "",
        "attitude_config_file": "", "tag_config_file": "", "tag_test_only": "false",
        "obstacle_config_file":str(share/"config/tag_obstacle_guard.yaml"),
        "camera_device": "/dev/video0", "capture_width": "2560", "capture_height": "960",
        "capture_fps": "60", "downward_image_topic": "/boom_birds/downward/image",
        "downward_input":"external","downward_device":"","downward_pixel_format":"MJPG",
        "downward_capture_fps":"30","downward_frame_id":"downward_optical",
        "downward_camera_config_file":"","downward_native_config_file":"",
        "downward_media_device":"","downward_sensor_entity":"","downward_raw10_native_library":"",
        "range_input":"mavros","range_sensor_id":"-1","range_target_system":"1","range_target_component":"1",
        "start_sensing": "true", "start": "false", "dry_run": "true", "allow_arming": "false",
        "allow_non_loopback": "false", "allow_hardware_actions": "false", "hardware_validation_note": "",
        "handover": "true", "platform_status_timeout_s": "0.5",
        "tag_compute_profile": "configured", "feature_tracking_diagnostics": "false",
        "compute_log_directory": "", "platform_trace_directory": ""}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()]
                             + [OpaqueFunction(function=taskNodes)])
