"""Gazebo 传感器基准：力驱动刚体、同步双目和模拟 IMU，不接飞行控制。"""

from pathlib import Path
import json
import hashlib
import math
import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp
from boom_birds_control.runtime_config import DEFAULTS

WIDTH, HEIGHT = 640, 480
FX = FY = 420.0
CX, CY = 319.5, 239.5
BASELINE = 0.0676718
# 光学系 x右/y下/z前 -> 刚体系 x前/y左/z上。
R_CAMERA_IMU = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
LEFT_POSITION = np.array([0.1, BASELINE / 2, 0.0])
CAMERA_TOPICS = (DEFAULTS.stereo_left_topic, DEFAULTS.stereo_right_topic)
MOTION_PROFILES = {
    "nominal": ((0.35, 0.28, 0.16, 0.04, 0.05, 0.10), (0.9, 0.8, 1.2, 0.7, 0.5, 0.4)),
    "gentle": ((0.18, 0.16, 0.07, 0.04, 0.05, 0.10), (0.45, 0.6, 0.8, 0.7, 0.5, 0.4)),
}
SENSOR_GATES = dict(image_rate_min_hz=19.0, imu_rate_min_hz=195.0)


def motion(t, profile="nominal"):
    """前 5 s 静止；随后施加平移和三轴转动，输出世界系线速度和机体系角速度。"""
    if t < 5 or t >= 40:
        return (0.0,) * 6
    u = t - 5
    ramp = min(u / 2, 1.0, (40 - t) / 2)
    return tuple(ramp * a * math.sin(f * u) for a, f in zip(*MOTION_PROFILES[profile]))


def write_camera_config(path, cameras):
    # OpenCV FileStorage 要求矩阵行使用 flow sequence，不能用 PyYAML 的 indentless 列表。
    lines = ["%YAML:1.0"]
    for name, fields in cameras.items():
        lines.append(name + ":")
        for key, value in fields.items():
            if key == "T_imu_cam":
                lines.append("  " + key + ":")
                lines.extend("    - " + json.dumps(row) for row in value)
            else:
                text = json.dumps(value) if isinstance(value, list) else str(value)
                lines.append("  " + key + ": " + text)
    Path(path).write_text("\n".join(lines) + "\n")
    file = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not file.isOpened():
        raise ValueError("OpenCV cannot read generated camera calibration")
    file.release()


def prepare(
    out,
    estimator_template,
    imu_template,
    motion_plugin,
    scene_name="range_3m",
    motion_profile="nominal",
    duration_sim_s=45.0,
):
    if not math.isfinite(duration_sim_s) or duration_sim_s <= 0:
        raise ValueError("duration_sim_s must be finite and positive")
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    motion_plugin = Path(motion_plugin).resolve()
    if not motion_plugin.is_file():
        raise FileNotFoundError(motion_plugin)
    if scene_name not in ("range_3m", "range_5m"):
        raise ValueError(scene_name)
    amplitudes, frequencies = MOTION_PROFILES[motion_profile]
    k = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]])
    np.savez(
        out / "stereo.npz",
        image_size=np.array([WIDTH, HEIGHT]),
        K1=k,
        K2=k,
        D1=np.zeros(5),
        D2=np.zeros(5),
        R=np.eye(3),
        T=np.array([-BASELINE, 0.0, 0.0]),
    )
    manifest = dict(
        test_only=True,
        model="force_driven_sensor_rig",
        scene=scene_name,
        motion_profile=motion_profile,
        motion_amplitudes=amplitudes,
        motion_frequencies=frequencies,
        sensor_noise="disabled baseline",
        seed=12,
        width=WIDTH,
        height=HEIGHT,
        fx=FX,
        fy=FY,
        cx=CX,
        cy=CY,
        baseline_m=BASELINE,
        image_rate_hz=20,
        imu_rate_hz=200,
        gravity_m_s2=9.81,
        real_time_factor_target=0.2,
        camera_to_imu_rotation=R_CAMERA_IMU.tolist(),
        left_camera_position_in_imu_m=LEFT_POSITION.tolist(),
        input_imu="Gazebo sensor; not MAVROS FCU upload",
        ground_truth_role="evaluation only",
        estimator_profile="noiseless_budget",
        duration_sim_s=float(duration_sim_s),
    )
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2))
    # 独立模拟标定，不读取或覆盖真实标定。
    cam = {}
    for i, y in enumerate((BASELINE / 2, -BASELINE / 2)):
        transform = np.eye(4)
        transform[:3, :3] = R_CAMERA_IMU
        transform[:3, 3] = [0.1, y, 0.0]
        cam[f"cam{i}"] = dict(
            T_imu_cam=transform.tolist(),
            cam_overlaps=[],
            camera_model="pinhole",
            distortion_coeffs=[0.0] * 4,
            distortion_model="radtan",
            intrinsics=[FX, FY, CX, CY],
            resolution=[WIDTH, HEIGHT],
            rostopic=CAMERA_TOPICS[i],
        )
    write_camera_config(out / "kalibr_imucam_chain.yaml", cam)
    text = Path(estimator_template).read_text()
    replacements = {
        "use_stereo: true": "use_stereo: false",
        "max_cameras: 2": "max_cameras: 1",
        "calib_cam_extrinsics: true": "calib_cam_extrinsics: false",
        "calib_cam_intrinsics: true": "calib_cam_intrinsics: false",
        "calib_cam_timeoffset: true": "calib_cam_timeoffset: false",
        "max_clones: 11": "max_clones: 6",
        "max_slam: 50": "max_slam: 0",
        "num_pts: 200": "num_pts: 100",
        "num_opencv_threads: 4": "num_opencv_threads: 2",
        "track_frequency: 21.0": "track_frequency: 21.0",
        "try_zupt: false": "try_zupt: true",
        "zupt_only_at_beginning: false": "zupt_only_at_beginning: true",
        "zupt_max_disparity: 0.5": "zupt_max_disparity: 0.001",
    }
    for a, b in replacements.items():
        text = text.replace(a, b)
    # 无噪声模拟用更小的静止阈值；6克隆窗口保留弱视差特征。
    text += "\nfi_max_baseline: 200\nfi_max_cond_number: 100000\n"
    (out / "estimator_config.yaml").write_text(text)
    text = Path(imu_template).read_text().replace("rostopic: /imu0", "rostopic: /boom_birds/imu")
    (out / "kalibr_imu_chain.yaml").write_text(text)
    rng = np.random.default_rng(12)
    image = np.full((1024, 1024, 3), 180, dtype=np.uint8)
    for _ in range(2500):
        x, y = rng.integers(0, 1000, 2)
        size = int(rng.integers(5, 28))
        color = tuple(int(v) for v in rng.integers(0, 256, 3))
        cv2.rectangle(image, (int(x), int(y)), (int(x + size), int(y + size)), color, -1)
    cv2.imwrite(str(out / "texture.png"), image)
    material = f'<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>{out / "texture.png"}</albedo_map><metalness>0</metalness><roughness>1</roughness></metal></pbr></material>'

    def box(name, pose, size):
        geometry = f"<geometry><box><size>{size}</size></box></geometry>"
        return f'<model name="{name}"><static>true</static><pose>{pose} 0 0 0</pose><link name="link"><collision name="collision">{geometry}</collision><visual name="visual">{geometry}{material}</visual></link></model>'

    scene = box("floor", "0 0 -0.05", "20 20 0.1")
    scene += box("front_wall", "3.2 0 2.5" if scene_name == "range_3m" else "5 0 2.5", "0.1 12 5")
    scene += box("left_wall", "0 5 2.5", "12 0.1 5")
    scene += box("right_wall", "0 -5 2.5", "12 0.1 5")
    obstacles = (
        (
            (2.2, 1.1, 1, 0.5, 0.6, 2),
            (2.4, -1.2, 0.8, 0.7, 0.5, 1.6),
            (2.8, 0.2, 2.6, 0.5, 0.7, 0.5),
        )
        if scene_name == "range_3m"
        else ((3, 1.2, 1, 0.5, 0.8, 2), (3, -1.5, 0.8, 0.7, 0.5, 1.6), (4, 0.2, 2.6, 0.5, 0.7, 0.5))
    )
    for i, (x, y, z, sx, sy, sz) in enumerate(obstacles):
        scene += box(f"obstacle_{i}", f"{x} {y} {z}", f"{sx} {sy} {sz}")
    cameras = ""
    for name, y, kind in [
        ("left", BASELINE / 2, "camera"),
        ("right", -BASELINE / 2, "camera"),
        ("depth", BASELINE / 2, "depth_camera"),
    ]:
        cameras += f'<sensor name="{name}" type="{kind}"><pose>0.1 {y} 0 0 0 0</pose><topic>/gz/bb/{name}/image</topic><gz_frame_id>bb_{name}_optical</gz_frame_id><update_rate>{20 if kind == "camera" else 10}</update_rate><always_on>true</always_on><camera><horizontal_fov>{2*math.atan(WIDTH/(2*FX))}</horizontal_fov><image><width>{WIDTH}</width><height>{HEIGHT}</height><format>R8G8B8</format></image><lens><intrinsics><fx>{FX}</fx><fy>{FY}</fy><cx>{CX}</cx><cy>{CY}</cy><s>0</s></intrinsics></lens><clip><near>0.1</near><far>15</far></clip></camera></sensor>'
    rig = f"""<model name="bb_rig"><pose>0 0 1.5 0 0 0</pose><link name="imu_link"><gravity>false</gravity><inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial>{cameras}<sensor name="imu" type="imu"><always_on>true</always_on><update_rate>200</update_rate><topic>/gz/bb/imu</topic><gz_frame_id>bb_imu</gz_frame_id></sensor></link><plugin filename="{motion_plugin}" name="boom_birds::SensorMotion"><amplitudes>{" ".join(map(str,amplitudes))}</amplitudes><frequencies>{" ".join(map(str,frequencies))}</frequencies><start_s>5</start_s><stop_s>40</stop_s><ramp_s>2</ramp_s></plugin><plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher"><odom_publish_frequency>100</odom_publish_frequency><odom_topic>/gz/bb/truth</odom_topic><odom_frame>world</odom_frame><robot_base_frame>bb_imu</robot_base_frame><dimensions>3</dimensions></plugin></model>"""
    world = f"""<?xml version="1.0"?><sdf version="1.9"><world name="bb_sensor"><physics name="physics" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>0.2</real_time_factor></physics><gravity>0 0 -9.81</gravity><plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/><plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/><plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/><plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/><plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin><scene><ambient>0.8 0.8 0.8 1</ambient><background>0.5 0.5 0.5 1</background></scene><light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><direction>-0.5 0 -1</direction><diffuse>0.8 0.8 0.8 1</diffuse><specular>0.1 0.1 0.1 1</specular></light>{scene}{rig}</world></sdf>"""
    (out / "sensor_world.sdf").write_text(world)
    specs = [
        ("/clock", "/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock", "GZ_TO_ROS"),
        (
            "/gz/bb/left/image",
            "/gazebo/left",
            "sensor_msgs/msg/Image",
            "gz.msgs.Image",
            "GZ_TO_ROS",
        ),
        (
            "/gz/bb/right/image",
            "/gazebo/right",
            "sensor_msgs/msg/Image",
            "gz.msgs.Image",
            "GZ_TO_ROS",
        ),
        (
            "/gz/bb/depth/image",
            "/gazebo/depth",
            "sensor_msgs/msg/Image",
            "gz.msgs.Image",
            "GZ_TO_ROS",
        ),
        (
            "/gz/bb/left/camera_info",
            "/gazebo/left_info",
            "sensor_msgs/msg/CameraInfo",
            "gz.msgs.CameraInfo",
            "GZ_TO_ROS",
        ),
        (
            "/gz/bb/right/camera_info",
            "/gazebo/right_info",
            "sensor_msgs/msg/CameraInfo",
            "gz.msgs.CameraInfo",
            "GZ_TO_ROS",
        ),
        ("/gz/bb/imu", "/gazebo/imu", "sensor_msgs/msg/Imu", "gz.msgs.IMU", "GZ_TO_ROS"),
        ("/gz/bb/truth", "/gazebo/truth", "nav_msgs/msg/Odometry", "gz.msgs.Odometry", "GZ_TO_ROS"),
    ]
    entries = [
        dict(
            gz_topic_name=g,
            ros_topic_name=r,
            ros_type_name=rt,
            gz_type_name=gt,
            direction=d,
            qos_profile=(
                "SYSTEM_DEFAULT"
                if rt
                in (
                    "sensor_msgs/msg/Image",
                    "sensor_msgs/msg/CameraInfo",
                    "geometry_msgs/msg/Twist",
                )
                else "SENSOR_DATA"
            ),
        )
        for g, r, rt, gt, d in specs
    ]
    (out / "bridge.yaml").write_text(yaml.safe_dump(entries))
    manifest["sha256"] = {
        p.name: hashlib.sha256(p.read_bytes()).hexdigest()
        for p in out.iterdir()
        if p.is_file() and p.name != "manifest.json"
    }
    manifest["motion_plugin_sha256"] = hashlib.sha256(motion_plugin.read_bytes()).hexdigest()
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2))
    return manifest


def check_imu_truth(truth, imu):
    """用真值位姿差分核验 IMU 比力/角速度；只供数据评估。"""
    truth, imu = np.asarray(truth, float), np.asarray(imu, float)
    if len(truth) < 3 or len(imu) < 3:
        return dict(pass_check=False, reason="insufficient IMU/pose samples")
    times = truth[:, 0]
    mask = (imu[:, 0] > times[1]) & (imu[:, 0] < times[-2])
    imu = imu[mask]
    rotation = Rotation.from_quat(truth[:, 4:8])
    at_imu = Slerp(times, rotation)(imu[:, 0])
    velocity = np.gradient(truth[:, 1:4], times, axis=0)
    acceleration = np.gradient(velocity, times, axis=0)
    reference = np.column_stack([np.interp(imu[:, 0], times, acceleration[:, i]) for i in range(3)])
    measured = at_imu.apply(imu[:, 1:4]) + [0.0, 0.0, -9.81]
    accel_error = np.linalg.norm(measured - reference, axis=1)
    angular = (rotation[:-1].inv() * rotation[1:]).as_rotvec() / np.diff(times)[:, None]
    mids = (times[1:] + times[:-1]) / 2
    ref_gyro = np.column_stack([np.interp(imu[:, 0], mids, angular[:, i]) for i in range(3)])
    gyro_error = np.linalg.norm(imu[:, 4:7] - ref_gyro, axis=1)
    result = dict(
        accel_error_p95_m_s2=float(np.percentile(accel_error, 95)),
        gyro_error_p95_rad_s=float(np.percentile(gyro_error, 95)),
        measured_world_accel_range_m_s2=np.ptp(measured, axis=0).tolist(),
        truth_position_range_m=np.ptp(truth[:, 1:4], axis=0).tolist(),
    )
    result["pass_check"] = bool(
        result["accel_error_p95_m_s2"] <= 0.05
        and result["gyro_error_p95_rad_s"] <= 0.02
        and max(result["measured_world_accel_range_m_s2"]) > 0.05
        and result["truth_position_range_m"][2] > 0.01
    )
    return result
