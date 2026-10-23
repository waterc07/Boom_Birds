"""仅回放相机/IMU给算法；真值留在原 bag，评估时离线读取。"""

import argparse
import json
import os
import re
import hashlib
import shutil
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
import rosbag2_py
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
from .gazebo_capture import LEFT, RIGHT, ns
from .gazebo_run import Processes, claimRosDomain
from .gazebo_fixture import SENSOR_GATES

POSE = "/ov_msckf/poseimu"
DEPTH = "/boom_birds/depth/image"
ODOM = "/ov_msckf/odomimu"
# 接口/质量门限在首次算法评估前固定；仅覆盖此模拟场景。
GATES = dict(
    **SENSOR_GATES,
    init_max_s=15.0,
    pose_min_count=300,
    pose_coverage_min=0.90,
    position_rmse_max_m=0.15,
    endpoint_error_max_m=0.20,
    orientation_p95_max_deg=5.0,
    depth_pair_min_count=100,
    depth_median_abs_max_m=0.10,
    depth_p95_abs_max_m=0.30,
    depth_valid_min_ratio=0.40,
    require_visual_updates=True,
)


def prepareReplayConfig(source, out, zupt_mode, feature_profile="source"):
    """保留输入标定，回放配置和诊断状态写入本轮目录。"""
    source, out = Path(source), Path(out)
    text = (source / "estimator_config.yaml").read_text()
    for name in ("kalibr_imu_chain.yaml", "kalibr_imucam_chain.yaml"):
        shutil.copyfile(source / name, out / name)
    settings = {
        "relative_config_imu": '"kalibr_imu_chain.yaml"',
        "relative_config_imucam": '"kalibr_imucam_chain.yaml"',
        "save_total_state": "true",
        "filepath_est": f'"{out / "state-estimate.txt"}"',
        "filepath_std": f'"{out / "state-std.txt"}"',
        "filepath_gt": f'"{out / "unused-groundtruth.txt"}"',
    }
    if zupt_mode == "imu":
        settings.update(zupt_chi2_multipler="1", zupt_max_disparity="0")
    elif zupt_mode == "disparity-strict":
        settings.update(zupt_chi2_multipler="0", zupt_max_disparity="0.05")
    elif zupt_mode == "noiseless":
        settings.update(zupt_chi2_multipler="0", zupt_max_disparity="0.001")
    if feature_profile == "low-parallax":
        settings.update(fi_max_baseline="200", fi_max_cond_number="100000")
    for key, value in settings.items():
        text, count = re.subn(rf"^{key}:.*$", f"{key}: {value}", text, flags=re.MULTILINE)
        if count == 0 and key.startswith("fi_"):
            text += f"\n{key}: {value}\n"
            continue
        if count != 1:
            raise ValueError(f"expected one config field: {key}, got {count}")
    target = out / "estimator_config.yaml"
    target.write_text(text)
    return target


class Outputs(Node):
    def __init__(self, out):
        super().__init__("gazebo_algorithm_outputs")
        self.out = Path(out)
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(uri=str(self.out / "outputs"), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""),
        )
        self.counts = {POSE: 0, DEPTH: 0, ODOM: 0}
        self.subs = []
        for topic, cls in ((POSE, PoseWithCovarianceStamped), (DEPTH, Image), (ODOM, Odometry)):
            typename = f'{cls.__module__.split(".")[0]}/msg/{cls.__name__}'
            self.writer.create_topic(
                rosbag2_py.TopicMetadata(
                    id=0, name=topic, type=typename, serialization_format="cdr"
                )
            )
            self.subs.append(
                self.create_subscription(cls, topic, lambda m, t=topic: self.receive(t, m), 20)
            )
        self.subs.append(self.create_subscription(Image, "/ov_msckf/trackhist", self.tracks, 10))
        self.bridge = CvBridge()
        self.trackSaved = False

    def receive(self, topic, msg):
        self.writer.write(topic, serialize_message(msg), ns(msg.header.stamp))
        self.counts[topic] += 1

    def tracks(self, msg):
        if not self.trackSaved and self.counts[POSE] > 100:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imwrite(str(self.out / "tracks.png"), image)
            self.trackSaved = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--rate", type=float, default=0.5)
    parser.add_argument(
        "--zupt-mode", choices=["source", "imu", "disparity-strict", "noiseless"], default="source"
    )
    parser.add_argument("--depth-profile", choices=["budget", "fullres"], default="budget")
    parser.add_argument("--feature-profile", choices=["source", "low-parallax"], default="source")
    parser.add_argument("--verbosity", choices=["INFO", "DEBUG"], default="DEBUG")
    args = parser.parse_args()
    if args.rate <= 0:
        parser.error("rate must be positive")
    source = Path(args.input).resolve()
    out = Path(args.out).resolve()
    if out.exists() and any(out.iterdir()):
        parser.error("out must be new or empty")
    out.mkdir(parents=True, exist_ok=True)
    (out / "gates.json").write_text(json.dumps(GATES, indent=2))
    config = prepareReplayConfig(source, out, args.zupt_mode, args.feature_profile)
    domain, domainLock = claimRosDomain()
    os.environ.update(
        PYTHONNOUSERSITE="1",
        ROS_DOMAIN_ID=str(domain),
        ROS_LOCALHOST_ONLY="1",
        OPENBLAS_NUM_THREADS="1",
    )
    startedWall = time.monotonic()
    rclpy.init()
    node = Outputs(out)
    procs = Processes(out)
    code = 1
    try:
        estimator = procs.start(
            "openvins",
            [
                "ros2",
                "run",
                "ov_msckf",
                "run_subscribe_msckf",
                str(config),
                "--ros-args",
                "-r",
                "__ns:=/ov_msckf",
                "-p",
                f"verbosity:={args.verbosity}",
                "-p",
                "use_sim_time:=true",
                "-p",
                "graceful_shutdown:=true",
                "-p",
                "save_total_state:=true",
                "-p",
                f'filepath_est:={out/"state-estimate.txt"}',
                "-p",
                f'filepath_std:={out/"state-std.txt"}',
            ],
        )
        depthOptions = (
            []
            if args.depth_profile == "budget"
            else [
                "-p",
                "experimental:=true",
                "-p",
                "matching_size:=[640,480]",
                "-p",
                "output_scale:=0.5",
                "-p",
                "sgbm_block_size:=3",
            ]
        )
        depthProcess = procs.start(
            "depth",
            [
                "ros2",
                "run",
                "boom_birds_sensing",
                "depth_node",
                "--ros-args",
                "-p",
                f"left_topic:={LEFT}",
                "-p",
                f"right_topic:={RIGHT}",
                "-p",
                f'calibration_file:={source/"stereo.npz"}',
                "-p",
                "use_sim_time:=true",
                "-p",
                "num_disparities:=64",
                "-p",
                "process_latest_only:=true",
                "-p",
                "max_processing_rate_hz:=17.0",
                "-p",
                "opencv_threads:=2",
                *depthOptions,
            ],
        )
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        topics = [
            LEFT,
            RIGHT,
            "/boom_birds/imu",
            "/boom_birds/stereo/left_raw/camera_info",
            "/boom_birds/stereo/right_raw/camera_info",
        ]
        player = procs.start(
            "playback",
            [
                "ros2",
                "bag",
                "play",
                str(source / "inputs"),
                "--clock",
                "100",
                "--rate",
                str(args.rate),
                "--topics",
                *topics,
            ],
        )
        deadline = time.monotonic() + 240
        while player.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if estimator.poll() is not None or depthProcess.poll() is not None:
                raise RuntimeError("algorithm process exited before playback completed")
        code = player.poll()
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        (out / "replay.json").write_text(
            json.dumps(
                dict(
                    player_exit=code,
                    counts=node.counts,
                    selected_input_topics=topics,
                    truth_published=False,
                    zupt_mode=args.zupt_mode,
                    feature_profile=args.feature_profile,
                    depth_profile=args.depth_profile,
                    matching_size=[640, 480] if args.depth_profile == "fullres" else [320, 240],
                    sgbm_block_size=3 if args.depth_profile == "fullres" else 5,
                    estimator_config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(),
                    playback_rate=args.rate,
                    ros_domain_id=domain,
                    wall_duration_s=time.monotonic() - startedWall,
                ),
                indent=2,
            )
        )
    finally:
        procs.close()
        node.writer.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if code == 0 and all(node.counts.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
