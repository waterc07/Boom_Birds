"""记录 Gazebo 传感器输入；双目仅按原始相同采样时间配对。"""

import argparse
from collections import defaultdict
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import Image, Imu, CameraInfo
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from cv_bridge import CvBridge
from .gazebo_fixture import WIDTH, HEIGHT, FX, FY, CX, CY, BASELINE, SENSOR_GATES, CAMERA_TOPICS

LEFT, RIGHT = CAMERA_TOPICS
TRUTH = "/gazebo/evaluation/odometry"
DEPTH_TRUTH = "/gazebo/evaluation/depth"


def ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


class Capture(Node):
    def __init__(self, out, duration):
        super().__init__("gazebo_sensor_capture")
        self.out, self.duration = Path(out), duration
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(uri=str(self.out / "inputs"), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""),
        )
        self.counts, self.times = defaultdict(int), defaultdict(list)
        self.types = {}
        self.pubs, self.subs = {}, []
        self.bridge = CvBridge()
        self.pending = [{}, {}]
        self.raw_stamps = [[], []]
        self.started = None
        self.started_wall = None
        self.now_ns = 0
        self.last_clock_recorded = -1
        self.done = False
        self.errors = []
        self.native_info = {}
        self.unpaired = 0
        for topic, cls in (
            (LEFT, Image),
            (RIGHT, Image),
            ("/boom_birds/imu", Imu),
            (TRUTH, Odometry),
            (DEPTH_TRUTH, Image),
            ("/clock", Clock),
            ("/boom_birds/stereo/left_raw/camera_info", CameraInfo),
            ("/boom_birds/stereo/right_raw/camera_info", CameraInfo),
        ):
            name = f'{cls.__module__.split(".")[0]}/msg/{cls.__name__}'
            self.types[topic] = name
            self.writer.create_topic(
                rosbag2_py.TopicMetadata(id=0, name=topic, type=name, serialization_format="cdr")
            )
            if topic != "/clock":
                self.pubs[topic] = self.create_publisher(cls, topic, 20)
        for name in ("left", "right"):
            self.subs.append(
                self.create_subscription(
                    CameraInfo,
                    f"/gazebo/{name}_info",
                    lambda m, n=name: self.check_info(n, m),
                    QoSProfile(depth=30),
                )
            )
        self.subs.append(
            self.create_subscription(Clock, "/clock", self.clock, qos_profile_sensor_data)
        )
        self.subs.append(
            self.create_subscription(
                Image, "/gazebo/left", lambda m: self.image(m, 0), QoSProfile(depth=30)
            )
        )
        self.subs.append(
            self.create_subscription(
                Image, "/gazebo/right", lambda m: self.image(m, 1), QoSProfile(depth=30)
            )
        )
        for source, target, cls in (
            ("/gazebo/imu", "/boom_birds/imu", Imu),
            ("/gazebo/truth", TRUTH, Odometry),
            ("/gazebo/depth", DEPTH_TRUTH, Image),
        ):
            self.subs.append(
                self.create_subscription(
                    cls,
                    source,
                    lambda m, t=target: self.forward(t, m),
                    QoSProfile(depth=30) if cls is Image else qos_profile_sensor_data,
                )
            )

    def check_info(self, name, msg):
        self.native_info[name] = dict(width=msg.width, height=msg.height, k=list(msg.k))
        expected = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        if (msg.width, msg.height) != (WIDTH, HEIGHT) or not np.allclose(
            msg.k, expected, atol=1e-3
        ):
            if "native CameraInfo mismatch" not in self.errors:
                self.errors.append("native CameraInfo mismatch")
            self.done = True

    def write(self, topic, msg, stamp):
        self.writer.write(topic, serialize_message(msg), stamp)
        self.counts[topic] += 1
        self.times[topic].append(stamp)

    def clock(self, msg):
        stamp = ns(msg.clock)
        if stamp < self.now_ns:
            self.errors.append("simulation clock moved backwards")
            self.done = True
        self.now_ns = stamp
        if stamp - self.last_clock_recorded >= 5000000:
            self.write("/clock", msg, stamp)
            self.last_clock_recorded = stamp
        if self.started is not None:
            elapsed = (stamp - self.started) * 1e-9
            if elapsed >= self.duration:
                self.done = True

    def forward(self, topic, msg):
        if self.done:
            return
        msg.header.frame_id = "bb_imu" if isinstance(msg, Imu) else msg.header.frame_id
        self.write(topic, msg, ns(msg.header.stamp))
        self.pubs[topic].publish(msg)

    def image(self, msg, index):
        if self.done:
            return
        stamp = ns(msg.header.stamp)
        self.raw_stamps[index].append(stamp)
        self.pending[index][stamp] = msg
        if stamp in self.pending[1 - index]:
            left = self.pending[0].pop(stamp)
            right = self.pending[1].pop(stamp)
            if self.started is None:
                self.started = stamp
                self.started_wall = time.monotonic()
            for i, m in enumerate((left, right)):
                if (m.width, m.height) != (WIDTH, HEIGHT):
                    self.errors.append("camera dimensions differ from calibration")
                    self.done = True
                    return
                gray = self.bridge.imgmsg_to_cv2(m, desired_encoding="mono8")
                output = self.bridge.cv2_to_imgmsg(gray, encoding="mono8")
                output.header = m.header
                output.header.frame_id = f'bb_{"left" if i == 0 else "right"}_optical'
                topic = LEFT if i == 0 else RIGHT
                self.write(topic, output, stamp)
                self.pubs[topic].publish(output)
                info = CameraInfo()
                info.header = output.header
                info.width, info.height = WIDTH, HEIGHT
                info.distortion_model = "plumb_bob"
                info.d = [0.0] * 5
                info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
                info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                info.p = [
                    FX,
                    0.0,
                    CX,
                    0.0 if i == 0 else -FX * BASELINE,
                    0.0,
                    FY,
                    CY,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                ]
                topic = topic + "/camera_info"
                self.write(topic, info, stamp)
                self.pubs[topic].publish(info)
        for pending in self.pending:
            while len(pending) > 30:
                pending.pop(next(iter(pending)))
                self.unpaired += 1

    def finish(self):
        report = dict(
            counts=dict(self.counts),
            errors=self.errors,
            native_camera_info=self.native_info,
            unpaired_dropped=self.unpaired,
            pending_unpaired=sum(map(len, self.pending)),
            completed_duration=self.done and not self.errors,
            rates_hz={},
            first_sim_s=None if self.started is None else self.started * 1e-9,
            last_sim_s=self.now_ns * 1e-9,
            wall_duration_s=(
                None if self.started_wall is None else time.monotonic() - self.started_wall
            ),
        )
        for topic, stamps in self.times.items():
            if len(stamps) > 1:
                a = np.asarray(stamps, dtype=np.int64)
                dt = np.diff(a) * 1e-9
                report["rates_hz"][topic] = dict(
                    mean=(len(a) - 1) / ((a[-1] - a[0]) * 1e-9),
                    max_gap_s=float(dt.max()),
                    non_increasing=int((dt <= 0).sum()),
                )
        report["effective_real_time_factor"] = (
            None
            if not report["wall_duration_s"]
            else (report["last_sim_s"] - report["first_sim_s"]) / report["wall_duration_s"]
        )
        report["pass"] = bool(
            report["completed_duration"]
            and not self.unpaired
            and len(self.native_info) == 2
            and report["rates_hz"].get(LEFT, {}).get("mean", 0) >= SENSOR_GATES["image_rate_min_hz"]
            and report["rates_hz"].get("/boom_birds/imu", {}).get("mean", 0)
            >= SENSOR_GATES["imu_rate_min_hz"]
            and all(
                self.counts[t] > 0 for t in (LEFT, RIGHT, "/boom_birds/imu", TRUTH, DEPTH_TRUTH)
            )
        )
        (self.out / "raw-image-stamps.json").write_text(json.dumps(self.raw_stamps))
        self.writer.close()
        (self.out / "capture.json").write_text(json.dumps(report, indent=2))
        return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--duration", type=float, default=45)
    parser.add_argument("--wall-timeout", type=float, default=300)
    args = parser.parse_args()
    rclpy.init()
    node = Capture(args.out, args.duration)
    deadline = time.monotonic() + args.wall_timeout
    try:
        while rclpy.ok() and not node.done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not node.done:
            node.errors.append("wall timeout before requested simulation duration")
    finally:
        report = node.finish()
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["pass"] else 1)


if __name__ == "__main__":
    main()
