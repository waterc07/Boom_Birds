"""生成 340 g 机体及传感器配置，启动独立 Gazebo/PX4 SITL。"""

import argparse
from bisect import bisect_left
from contextlib import ExitStack
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
from threading import Lock
import time
import xml.etree.ElementTree as ET

import yaml

WS = Path(__file__).resolve().parents[1]
SIM = WS / "src/boom_birds_sim"
AIRCRAFT = "aircraft_340g"
ACTUATOR_SCALE = 1000
REFERENCE_SPEED = 1000.0  # Equivalent Gazebo speed at full thrust. No RPM measurement.
SENSOR_LINK_MASS = 0.00001
ROTOR_MASS = 0.0001  # Numerical inertia for links. Included in the 340 g total.
ROTORS = (
    ("front_right", "ccw"),
    ("rear_left", "ccw"),
    ("front_left", "cw"),
    ("rear_right", "cw"),
)
PLUGIN = "gz-sim-multicopter-motor-model-system"
COMMAND = f"/{AIRCRAFT}/command/motor_speed"
CALIBRATED_COMMAND = f"/{AIRCRAFT}/command/calibrated_motor_speed"


class ThrustCurve:
    def __init__(self, path):
        config = yaml.safe_load(path.read_text(encoding="utf-8"))
        self.throttles = [float(point["throttle"]) for point in config["curve"]]
        self.forces = [float(point["thrust_n"]) for point in config["curve"]]
        if not (
            config["interpolation"] == "linear"
            and len(self.throttles) >= 2
            and all(math.isfinite(value) for value in self.throttles + self.forces)
            and self.throttles[0] == 0 and self.throttles[-1] == 1
            and self.forces[0] == 0
            and all(a < b for a, b in zip(self.throttles, self.throttles[1:]))
            and all(a < b for a, b in zip(self.forces, self.forces[1:]))
        ):
            raise ValueError("Expected increasing throttle and thrust starting at zero")

    def force(self, throttle):
        throttle = max(0.0, min(1.0, throttle))
        right = max(1, bisect_left(self.throttles, throttle))
        left = right - 1
        fraction = (throttle - self.throttles[left]) / (self.throttles[right] - self.throttles[left])
        return self.forces[left] + fraction * (self.forces[right] - self.forces[left])

    @property
    def motorConstant(self):
        return self.forces[-1] / REFERENCE_SPEED**2

    def speed(self, actuator):
        return math.sqrt(self.force(actuator / ACTUATOR_SCALE) / self.motorConstant)


def put(parent, tag, value=None, **attrs):
    item = ET.SubElement(parent, tag, attrs)
    if value is not None:
        item.text = str(value)
    return item


def addRotor(model, index, xy, direction, curve):
    name = f"rotor_{index}"
    rotor = put(model, "link", name=name)
    put(rotor, "pose", f"{xy[0]} {xy[1]} -0.0008983 0 0 0")
    inertial = put(rotor, "inertial")
    put(inertial, "mass", ROTOR_MASS)
    tensor = put(inertial, "inertia")
    for axis in ("ixx", "iyy", "izz"):
        put(tensor, axis, 1e-8)
    joint = put(model, "joint", name=f"{name}_joint", type="revolute")
    put(joint, "parent", "base_link")
    put(joint, "child", name)
    axis = put(joint, "axis")
    put(axis, "xyz", "0 0 1")
    limit = put(axis, "limit")
    put(limit, "lower", -1e16)
    put(limit, "upper", 1e16)
    plugin = put(model, "plugin", filename=PLUGIN, name="gz::sim::systems::MulticopterMotorModel")
    for tag, value in {
        "jointName": f"{name}_joint",
        "linkName": name,
        "turningDirection": direction,
        "actuator_number": index,
        "commandSubTopic": "command/calibrated_motor_speed",
        "motorType": "velocity",
        "motorConstant": curve.motorConstant,
        "maxRotVelocity": REFERENCE_SPEED,
        "momentConstant": 0.016,
        "timeConstantUp": 0.0125,
        "timeConstantDown": 0.025,
        "rotorVelocitySlowdownSim": 10,
    }.items():
        put(plugin, tag, value)



def vector(values):
    return " ".join(str(value) for value in values)


def sensorTopic(name):
    return f"/{AIRCRAFT}/{name}"


def addCamera(body, name, pose, config, opticalFrame, baseline=0.0):
    width, height = config["resolution"]
    fx, fy, cx, cy = config["intrinsics"]
    near, far = config["clip_m"]
    topic = sensorTopic(name)
    sensor = put(body, "sensor", name=name, type="camera")
    put(sensor, "pose", vector(pose))
    put(sensor, "topic", f"{topic}/image")
    put(sensor, "gz_frame_id", opticalFrame)
    put(sensor, "always_on", 1)
    put(sensor, "update_rate", config["rate_hz"])
    camera = put(sensor, "camera", name=name)
    put(camera, "camera_info_topic", f"{topic}/camera_info")
    put(camera, "horizontal_fov", 2 * math.atan(width / (2 * fx)))
    image = put(camera, "image")
    put(image, "width", width)
    put(image, "height", height)
    put(image, "format", "L8")
    put(image, "anti_aliasing", 0)
    lens = put(camera, "lens")
    intrinsics = put(lens, "intrinsics")
    for key, value in (("fx", fx), ("fy", fy), ("cx", cx), ("cy", cy), ("s", 0)):
        put(intrinsics, key, value)
    projection = put(lens, "projection")
    for key, value in (("p_fx", fx), ("p_fy", fy), ("p_cx", cx), ("p_cy", cy), ("tx", -fx * baseline)):
        put(projection, key, value)
    clip = put(camera, "clip")
    put(clip, "near", near)
    put(clip, "far", far)


def stereoPositions(config):
    x, y, z = config["center_m"]
    half = config["baseline_m"] / 2
    return (x, y + half, z), (x, y - half, z)


def sensorMount(model, name, pose):
    mount = put(model, "link", name=name)
    put(mount, "pose", vector(pose), relative_to="base_link")
    inertial = put(mount, "inertial")
    put(inertial, "mass", SENSOR_LINK_MASS)
    tensor = put(inertial, "inertia")
    for axis in ("ixx", "iyy", "izz"):
        put(tensor, axis, 1e-9)
    joint = put(model, "joint", name=f"{name}_joint", type="fixed")
    put(joint, "parent", "base_link")
    put(joint, "child", name)
    return mount


def addOpticalFlow(model, config):
    mount = sensorMount(model, "flow_link", config["pose"])
    sensor = put(mount, "sensor", name="flow_camera", type="camera")
    put(sensor, "always_on", 1)
    put(sensor, "update_rate", config["rate_hz"])
    camera = put(sensor, "camera")
    put(camera, "horizontal_fov", config["horizontal_fov_rad"])
    image = put(camera, "image")
    put(image, "width", config["resolution"][0])
    put(image, "height", config["resolution"][1])
    put(image, "format", "R8G8B8")
    clip = put(camera, "clip")
    put(clip, "near", config["clip_m"][0])
    put(clip, "far", config["clip_m"][1])

    flow = put(mount, "sensor", name="optical_flow", type="custom", **{"gz:type": "optical_flow"})
    put(flow, "always_on", 1)
    put(flow, "update_rate", config["rate_hz"])


def addSensors(model, body, config):
    stereo = config["stereo"]
    for index, (name, position) in enumerate(zip(("left", "right"), stereoPositions(stereo))):
        addCamera(body, name, (*position, 0, 0, 0), stereo, f"cam{index}", index * stereo["baseline_m"])
    downward = config["downward"]
    mount = sensorMount(model, "downward_camera_link", downward["pose"])
    addCamera(mount, "downward", (0, 0, 0, 0, 0, 0), downward, "downward_optical")

    module = config["flow_ranging_module"]
    addOpticalFlow(model, module["optical_flow"])
    ranging = module["rangefinder"]
    sensor = put(body, "sensor", name="down_range", type="gpu_lidar")
    put(sensor, "pose", vector(ranging["pose"]))
    put(sensor, "topic", sensorTopic("down_range_raw"))
    put(sensor, "gz_frame_id", "range_sensor")
    put(sensor, "always_on", 1)
    put(sensor, "update_rate", ranging["rate_hz"])
    lidar = put(sensor, "lidar")
    scan = put(lidar, "scan")
    for dimension in ("horizontal", "vertical"):
        direction = put(scan, dimension)
        for key, value in (("samples", 1), ("resolution", 1), ("min_angle", 0), ("max_angle", 0)):
            put(direction, key, value)
    limits = put(lidar, "range")
    rayNear = float(ranging.get("ray_near_m", 0.0001))
    physicalMin, physicalMax = ranging["limits_m"]
    if not 0 < rayNear < physicalMin < physicalMax < math.inf:
        raise ValueError("Expected ray_near_m < minimum ToF range < maximum ToF range")
    put(limits, "min", rayNear)
    put(limits, "max", physicalMax)
    put(limits, "resolution", 0.001)


def bridgeConfig(world):
    rows = []

    def add(gazebo, ros, rosType, gazeboType):
        rows.append(dict(gz_topic_name=gazebo, ros_topic_name=ros, ros_type_name=rosType,
                         gz_type_name=f"gz.msgs.{gazeboType}", direction="GZ_TO_ROS"))

    add(f"/world/{world}/clock", "/clock", "rosgraph_msgs/msg/Clock", "Clock")
    for name, image, info in (
        ("left", "/boom_birds/stereo/left_raw", "/boom_birds/stereo/left/camera_info"),
        ("right", "/boom_birds/stereo/right_raw", "/boom_birds/stereo/right/camera_info"),
        ("downward", "/boom_birds/downward/image", "/boom_birds/downward/camera_info"),
    ):
        add(f"{sensorTopic(name)}/image", image, "sensor_msgs/msg/Image", "Image")
        add(f"{sensorTopic(name)}/camera_info", info, "sensor_msgs/msg/CameraInfo", "CameraInfo")
    add(sensorTopic("down_range"), "/boom_birds/platform/range", "sensor_msgs/msg/Range", "LaserScan")
    imu = f"/world/{world}/model/{AIRCRAFT}/link/base_link/sensor/imu_sensor/imu"
    add(imu, "/boom_birds/sim/imu_flu", "sensor_msgs/msg/Imu", "IMU")
    return rows


def saveStereoCalibration(out, config):
    import numpy as np

    fx, fy, cx, cy = config["intrinsics"]
    k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    rotation = np.eye(3)
    translation = np.array([[-config["baseline_m"]], [0.0], [0.0]])
    projection = np.column_stack((k, np.zeros(3)))
    rightProjection = projection.copy()
    rightProjection[0, 3] = -fx * config["baseline_m"]
    np.savez(
        out / "stereo_sim.npz", image_size=np.array(config["resolution"]),
        K1=k, K2=k, D1=np.zeros(5), D2=np.zeros(5), R=rotation, T=translation,
        R1=rotation, R2=rotation, P1=projection, P2=rightProjection,
    )
    # T_imu_cam expresses the optical axes in the FRD IMU frame at the body origin.
    lines = ["%YAML:1.0", "---"]
    for index, (x, y, z) in enumerate(stereoPositions(config)):
        topic = ("left_raw", "right_raw")[index]
        lines.extend((
            f"cam{index}:",
            "  T_imu_cam:",
            f"    - [0.0, 0.0, 1.0, {x}]",
            f"    - [1.0, 0.0, 0.0, {-y}]",
            f"    - [0.0, 1.0, 0.0, {-z}]",
            "    - [0.0, 0.0, 0.0, 1.0]",
            "  camera_model: pinhole",
            "  distortion_model: radtan",
            "  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]",
            f"  intrinsics: {json.dumps(config['intrinsics'])}",
            f"  resolution: {json.dumps(config['resolution'])}",
            f"  rostopic: /boom_birds/stereo/{topic}",
        ))
    (out / "stereo_imu_frd.yaml").write_text("\n".join(lines) + "\n")

def buildModel(px4, assets, curve, config):
    model = ET.parse(assets / "models/aircraft_340g_collision/model.sdf").getroot().find("model")
    model.set("name", AIRCRAFT)
    body = model.find("link")
    mass = body.find("inertial/mass")
    mass.text = str(float(mass.text) - len(ROTORS) * ROTOR_MASS - 2 * SENSOR_LINK_MASS)
    x500 = ET.parse(px4 / "Tools/simulation/gz/models/x500_base/model.sdf")
    sensor_names = {"imu_sensor", "air_pressure_sensor", "magnetometer_sensor", "navsat_sensor"}
    sensors = x500.findall("./model/link[@name='base_link']/sensor")
    for sensor in sensors:
        if sensor.get("name") in sensor_names:
            body.append(deepcopy(sensor))
    if {sensor.get("name") for sensor in body.findall("sensor")} != sensor_names:
        raise RuntimeError("X500 simulator sensor definitions were not found")
    addSensors(model, body, config)
    shapes = json.loads((assets / "collision-geometry.json").read_text())["shapes"]
    params = {"MPC_THR_HOVER": 0.55, "THR_MDL_FAC": 0}
    for index, (location, direction) in enumerate(ROTORS):
        x, y, _ = shapes[f"guard_{location}"]["center_m"]
        addRotor(model, index, (x, y), direction, curve)
        params.update({
            f"CA_ROTOR{index}_PX": x,
            f"CA_ROTOR{index}_PY": -y,
            f"CA_ROTOR{index}_PZ": 0.0008983,
            f"CA_ROTOR{index}_KM": 0.016 if direction == "ccw" else -0.016,
            f"SIM_GZ_EC_MIN{index+1}": 0,
            f"SIM_GZ_EC_MAX{index+1}": ACTUATOR_SCALE,
        })
    put(model, "plugin", filename="gz-sim-joint-state-publisher-system",
        name="gz::sim::systems::JointStatePublisher")
    return model, params


class ThrustRelay:
    def __init__(self, node, curve):
        from gz.msgs10.actuators_pb2 import Actuators
        self.messageType = Actuators
        self.publisher = node.advertise(CALIBRATED_COMMAND, Actuators)
        self.curve = curve
        self.lock = Lock()
        self.latest = None
        if not node.subscribe(Actuators, COMMAND, self.receive):
            raise RuntimeError(f"Could not subscribe to {COMMAND}")

    def receive(self, message):
        values = list(message.velocity)
        valid = len(values) == len(ROTORS) and all(math.isfinite(value) for value in values)
        with self.lock:
            self.latest = (time.monotonic(), values) if valid else None

    def tick(self):
        with self.lock:
            data = self.latest
        speeds = [0.0] * len(ROTORS)
        if data is not None and time.monotonic() - data[0] <= 0.3:
            speeds = [self.curve.speed(raw) for raw in data[1]]
        message = self.messageType()
        message.velocity.extend(speeds)
        self.publisher.publish(message)


class DownRangeRelay:
    def __init__(self, node, limits):
        from gz.msgs10.laserscan_pb2 import LaserScan

        self.minimum, self.maximum = map(float, limits)
        self.publisher = node.advertise(sensorTopic("down_range"), LaserScan)
        if not node.subscribe(LaserScan, sensorTopic("down_range_raw"), self.receive):
            raise RuntimeError("Could not subscribe to the raw downward laser scan")

    def receive(self, scan):
        scan.range_min = self.minimum
        scan.range_max = self.maximum
        # The nearest surface still occludes the background inside the ToF blind zone.
        for index, distance in enumerate(scan.ranges):
            if distance < self.minimum:
                scan.ranges[index] = -math.inf
            elif distance > self.maximum:
                scan.ranges[index] = math.inf
        self.publisher.publish(scan)


def addDeckTag(link, config):
    # Use the checked-in print master as geometry, with no texture or collision.
    svg = ET.parse(WS.parents[1] / config["apriltag_svg"]).getroot()
    x0, y0, width, height = map(float, svg.get("viewBox").split())
    deckX, deckY, deckHeight = config["size_m"]
    if width / 1000 > deckX or height / 1000 > deckY:
        raise ValueError("AprilTag paper does not fit the landing deck")
    rects = svg.findall("{http://www.w3.org/2000/svg}rect")
    ink = [rect for rect in rects if rect.get("fill") == "black"]
    if not ink:
        raise ValueError("AprilTag SVG has no black cells")
    for index, item in enumerate(rects):
        color = item.get("fill")
        if color not in ("white", "black"):
            continue
        x = float(item.get("x", x0)) - x0
        y = float(item.get("y", y0)) - y0
        w, h = float(item.get("width")), float(item.get("height"))
        xy = ((x + w / 2 - width / 2) / 1000, (height / 2 - y - h / 2) / 1000)
        # Keep the black ink above the white layer to avoid coplanar rendering.
        heightOffset = 0.0001 if color == "white" else 0.0003
        visual = put(link, "visual", name=f"tag_cell_{index}")
        put(visual, "pose", vector([*xy, deckHeight / 2 + heightOffset, 0, 0, 0]))
        box = put(put(visual, "geometry"), "box")
        put(box, "size", vector([w / 1000, h / 1000, 0.0002]))
        material = put(visual, "material")
        rgba = "1 1 1 1" if color == "white" else "0 0 0 1"
        put(material, "ambient", rgba)
        put(material, "diffuse", rgba)
        put(material, "emissive", rgba)


def addLandingDeck(world, config):
    deck = put(world, "model", name="aircraft_landing_deck")
    put(deck, "static", "true")
    put(deck, "pose", vector(config["pose"]))
    link = put(deck, "link", name="deck")
    collision = put(link, "collision", name="surface")
    visual = put(link, "visual", name="surface_visual")
    for item in (collision, visual):
        geometry = put(item, "geometry")
        put(put(geometry, "box"), "size", vector(config["size_m"]))
    material = put(visual, "material")
    put(material, "ambient", "0.16 0.20 0.24 1")
    put(material, "diffuse", "0.25 0.32 0.39 1")
    if "apriltag_svg" in config:
        addDeckTag(link, config)


def writeWorld(source, target, model, sceneConfig):
    root = ET.parse(source).getroot()
    root.set("xmlns:gz", "http://gazebosim.org")
    world = root.find("world")
    if world is None:
        raise ValueError("Expected a Gazebo world")
    if "landing_deck" in sceneConfig:
        addLandingDeck(world, sceneConfig["landing_deck"])
    put(model, "pose", vector(sceneConfig["spawn_pose"]))
    if world.findall("plugin") and world.find("plugin[@name='gz::sim::systems::Sensors']") is None:
        sensors = put(world, "plugin", filename="gz-sim-sensors-system", name="gz::sim::systems::Sensors")
        put(sensors, "render_engine", "ogre2")
    if world.findall("plugin"):
        put(world, "plugin", filename="libOpticalFlowSystem.so", name="custom::OpticalFlowSystem")
    world.append(model)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(target, encoding="utf-8", xml_declaration=True)
    return world.get("name")


def start(stack, args, logfile, cwd):
    log = stack.enter_context(logfile.open("w"))
    process = subprocess.Popen(args, cwd=cwd, stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)

    def stop():
        if process.poll() is not None:
            return
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)

    stack.callback(stop)
    return process


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4", type=Path, default=Path("/home/waterc/PX4-Autopilot"))
    parser.add_argument("--assets", type=Path, required=True, help="exported aircraft model directory")
    parser.add_argument("--out", type=Path, required=True, help="new run directory")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--config", type=Path, default=SIM / "config/aircraft_340g.yaml")
    parser.add_argument("--scene", choices=("empty", "rmuc2027"), default="empty")
    parser.add_argument("--ros-domain", type=int, default=193,
                        help="isolated local ROS domain used by the Gazebo/ROS bridge")
    parser.add_argument("--no-ros", action="store_true", help="skip ROS bridge and MAVROS")
    parser.add_argument("--generate-only", action="store_true", help="only write the runtime configuration")
    args = parser.parse_args()
    px4 = args.px4.resolve()
    assets = args.assets.resolve()
    build = px4 / "build/px4_sitl_default"
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    modelPath = assets / "models/aircraft_340g_collision"
    curve = ThrustCurve(modelPath / "motor_thrust.yaml")
    configText = args.config.read_text(encoding="utf-8")
    config = yaml.safe_load(configText)
    model, params = buildModel(px4, assets, curve, config)
    sourceWorld = (SIM / "worlds/rmuc2027.sdf" if args.scene == "rmuc2027"
                   else px4 / "Tools/simulation/gz/worlds/default.sdf")
    world = writeWorld(sourceWorld, out / "world.sdf", model, config["scenes"][args.scene])
    (out / "bridge.yaml").write_text(yaml.safe_dump(bridgeConfig(world), sort_keys=False))
    (out / "aircraft-config.yaml").write_text(configText, encoding="utf-8")
    saveStereoCalibration(out, config["stereo"])
    print(out, flush=True)
    (out / "px4-params.json").write_text(json.dumps(params, indent=2) + "\n")
    if args.generate_only:
        return
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    for key in tuple(os.environ):
        if key.startswith("PX4_PARAM_"):
            del os.environ[key]
    os.environ.update({
        "GZ_PARTITION": f"bb_340g_{os.getpid()}",
        "GZ_IP": "127.0.0.1",
        "GZ_SIM_RESOURCE_PATH": ":".join(str(path) for path in (
            assets / "models", px4 / "Tools/simulation/gz/models", SIM / "assets/rmuc2027/models",
        )),
        "ROS_DOMAIN_ID": str(args.ros_domain),
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        "GZ_SIM_SERVER_CONFIG_PATH": str(px4 / "src/modules/simulation/gz_bridge/server.config"),
        "GZ_SIM_SYSTEM_PLUGIN_PATH": str(build / "src/modules/simulation/gz_plugins"),
        "PX4_GZ_STANDALONE": "1",
        "PX4_GZ_WORLD": world,
        "PX4_GZ_MODEL_NAME": AIRCRAFT,
        "PX4_SYS_AUTOSTART": "4001",
        "PX4_SIM_MODEL": "gz_x500",
        "HEADLESS": "1",
        **{f"PX4_PARAM_{key}": str(value) for key, value in params.items()},
    })
    (out / "gazebo-partition.txt").write_text(os.environ["GZ_PARTITION"] + "\n")
    from gz.transport13 import Node
    node = Node()
    relay = ThrustRelay(node, curve)
    rangeRelay = DownRangeRelay(node, config["flow_ranging_module"]["rangefinder"]["limits_m"])
    try:
        with ExitStack() as stack:
            serverArgs = ["gz", "sim", "-s", "-r"]
            if not args.gui:
                serverArgs.append("--headless-rendering")
            server = start(stack, serverArgs + [str(out / "world.sdf")], out / "gazebo.log", out)
            if args.gui:
                start(stack, ["gz", "sim", "-g"], out / "gazebo-gui.log", out)
            autopilot = start(stack, [str(build / "bin/px4"), str(build / "etc"), "-d", "-i", "0"],
                              out / "px4.log", out)
            processes = [server, autopilot]
            if not args.no_ros:
                rosEnv = ["bash", "-c", 'source /opt/ros/jazzy/setup.bash && exec "$@"', "ros-env"]
                bridge = start(stack, rosEnv + [
                    "ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
                    "-p", f"config_file:={out / 'bridge.yaml'}",
                ], out / "bridge.log", out)
                mavros = start(stack, rosEnv + [
                    "ros2", "run", "mavros", "mavros_node", "--ros-args",
                    "--params-file", str(WS / "src/boom_birds_bringup/config/mavros.yaml"),
                    "-p", "fcu_url:=udp://127.0.0.1:14540@127.0.0.1:14580",
                    "-p", "use_sim_time:=true",
                    "-p", "tgt_system:=1", "-p", "tgt_component:=1", "-p", 'gcs_url:=""',
                ], out / "mavros.log", out)
                processes.extend((bridge, mavros))
            while True:
                if any(process.poll() is not None for process in processes):
                    raise RuntimeError(f"A simulator or bridge process stopped. Logs: {out}")
                relay.tick()
                time.sleep(0.01)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
