"""SIH 证据记录器：把执行状态、控制状态与任务状态**完整**落盘为 JSONL。

为什么不用 `ros2 topic echo`：它对长消息会按终端宽度折行/截断，之前正是因此
丢掉了 `reasons`（失效原因码），排查时只能看到 `sending=false` 而看不到原因。
"""
from boom_birds_control.runtime_config import DEFAULTS

import json
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Imu, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from pathlib import Path
from geometry_msgs.msg import PoseStamped

RELIABLE = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST)


class Recorder(Node):
    def __init__(self, out_dir):
        super().__init__("boom_birds_sih_recorder")
        self.out_dir = out_dir
        self.last = {}
        self.cloud_saved = False
        self.create_subscription(PointCloud2, DEFAULTS.sim_map_topic, self._on_cloud,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        for topic, name in ((DEFAULTS.mission_status_topic, "mission"),
                            (DEFAULTS.control_status_topic, "control")):
            self.create_subscription(
                String, topic, lambda msg, n=name: self._on(n, msg), RELIABLE)
        # 诊断：/boom_birds/imu 到底有没有数据、有几个发布者
        self._imu_count = 0
        self._depth_count = 0
        self._depth_total = 0
        self._depth_valid = 0
        self._depth_in_range = 0
        self._depth_min = float("inf")
        self._depth_max = 0.0
        self._depth_err = None
        self._cam_z = []
        # 流间隔统计：EGO 的 odom_depth_timeout 是 1.0 s，因此要能量出
        # "哪一路在什么时候出现了 >1 s 的空洞"，而不是只看计数。
        self._gaps = {}
        self._last_seen = {}
        self.create_subscription(Imu, DEFAULTS.imu_topic, self._on_imu, 50)
        self.create_subscription(Image, DEFAULTS.depth_topic, self._on_depth, 50)
        self.create_subscription(PoseStamped, DEFAULTS.camera_pose_topic, self._on_cam, 50)
        from nav_msgs.msg import Odometry
        from boom_birds_interfaces.msg import ExecutionStatus, ControlCommand, PlannerStatus
        self.create_subscription(Odometry, DEFAULTS.odom_topic,
                                 lambda m: self._mark("odom_ego"), 50)
        self.create_subscription(ExecutionStatus, DEFAULTS.status_topic, self._on_execution, RELIABLE)
        for topic, name in ((DEFAULTS.planner_status_topic, "planner_instance"),
                            (DEFAULTS.executor_status_topic, "executor_instance")):
            self.create_subscription(PlannerStatus, topic, lambda m, n=name: self._on_instance(n, m), RELIABLE)
        for topic, name in ((DEFAULTS.planner_command_topic, "planner_command"),
                            (DEFAULTS.command_topic, "control_command")):
            self.create_subscription(ControlCommand, topic, lambda m, n=name: self._on_command(n, m), RELIABLE)
        self.create_timer(0.5, self._flush)

    def _on_cloud(self, msg):
        if self.cloud_saved:
            return
        points = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        if len(points) == 0:
            return
        np.save(Path(self.out_dir) / "scene_cloud.npy", points)
        (Path(self.out_dir) / "scene_cloud.json").write_text(json.dumps(
            dict(frame_id=msg.header.frame_id, point_count=len(points),
                 stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)))
        self.cloud_saved = True

    def _on(self, name, msg):
        try:
            self.last[name] = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            self.last[name] = {"raw": msg.data}

    def _on_instance(self, name, msg):
        self.last[name] = dict(session_id=msg.session_id, producer_session_id=msg.producer_session_id,
                              stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def _on_command(self, name, msg):
        vector = lambda p: [p.x, p.y, p.z]
        self.last[name] = dict(session_id=msg.session_id, planner_session_id=msg.planner_session_id,
            executor_session_id=msg.executor_session_id, trajectory_id=msg.trajectory_id,
            sequence=msg.sequence, command_type=msg.command_type, frame=msg.header.frame_id,
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            valid_for_s=msg.valid_for.sec + msg.valid_for.nanosec * 1e-9,
            position=vector(msg.position), velocity=vector(msg.velocity), acceleration=vector(msg.acceleration),
            yaw=msg.yaw, yaw_rate=msg.yaw_rate)
        self._write(name)

    def _on_execution(self, msg):
        self.last["execution"] = {
            "session_id": msg.session_id, "trajectory_id": int(msg.trajectory_id),
            "sequence": int(msg.sequence), "accepted": bool(msg.accepted),
            "sending": bool(msg.sending), "offboard_confirmed": bool(msg.offboard_confirmed),
            "connected": bool(msg.connected), "armed": bool(msg.armed),
            "landed_known": bool(msg.landed_known), "landed_state": int(msg.landed_state),
            "mode": msg.mode, "mode_detail": msg.mode_detail,
            "current_mode_detail": msg.current_mode_detail,
            "intended_mode_detail": msg.intended_mode_detail,
            "current_mode_age_s": msg.current_mode_age_s,
            "frame_reset_epoch": msg.frame_reset_epoch,
            "frame_reset_known": msg.frame_reset_known, "frame_reset_age_s": msg.frame_reset_age_s,
            "attitude_known": msg.attitude_known, "yaw_ned_rad": msg.yaw_ned_rad,
            "attitude_age_s": msg.attitude_age_s,
            "px4_safety_mode": msg.px4_safety_mode,
            "px4_failsafe_cause": msg.px4_failsafe_cause,
            "px4_safety_age_s": msg.px4_safety_age_s,
            "custom_main_mode": int(msg.custom_main_mode),
            "custom_sub_mode": int(msg.custom_sub_mode),
            "restart_epoch": int(msg.restart_epoch),
            "heartbeat_age_s": msg.heartbeat_age_s, "position_age_s": msg.position_age_s,
            "position_known": bool(msg.position_known),
            "position_ned": [msg.position_ned.x, msg.position_ned.y, msg.position_ned.z],
            "velocity_ned": [msg.velocity_ned.x, msg.velocity_ned.y, msg.velocity_ned.z],
            "sensors_ready": bool(msg.sensors_ready),
            "alignment_valid": bool(msg.alignment_valid),
            "alignment_yaw_offset_rad": msg.alignment_yaw_offset_rad,
            "alignment_translation_m": [msg.alignment_translation_m.x, msg.alignment_translation_m.y, msg.alignment_translation_m.z],
            "reasons": list(msg.reasons),
        }

    def _write(self, name):
        value = self.last.pop(name, None)
        if value is None:
            return
        with open(f"{self.out_dir}/recorder_{name}.jsonl", "a", encoding="utf-8") as fh:
            fh.write(json.dumps({"t": time.time(), "data": value}, ensure_ascii=False) + "\n")

    def _mark(self, name):
        now = time.time()
        prev = self._last_seen.get(name)
        if prev is not None:
            gap = now - prev
            rec = self._gaps.setdefault(name, {"max_gap": 0.0, "max_gap_at": 0.0, "n": 0, "over_1s": 0})
            rec["n"] += 1
            if gap > rec["max_gap"]:
                rec["max_gap"] = gap
                rec["max_gap_at"] = now
            if gap > 1.0:
                rec["over_1s"] += 1
        self._last_seen[name] = now

    def _on_imu(self, msg):
        self._imu_count += 1
        self._mark("imu")

    def _on_depth(self, msg):
        self._depth_count += 1
        self._mark("depth")
        # 直接量测"有没有落在 EGO 可见量程内的深度像素"：EGO 的 mapReady 要求
        # 至少一个 0 < z <= invalid_depth_max_dist_ 的像素，depth_node 的 max_depth_m=5.0。
        try:
            if msg.encoding == "32FC1" and msg.height and msg.width:
                arr = np.frombuffer(bytes(msg.data), dtype=np.float32)
                arr = arr[: msg.height * msg.width].reshape(msg.height, msg.width)
                finite = np.isfinite(arr) & (arr > 0.0)
                self._depth_total += arr.size
                self._depth_valid += int(finite.sum())
                if finite.any():
                    self._depth_min = min(self._depth_min, float(arr[finite].min()))
                    self._depth_max = max(self._depth_max, float(arr[finite].max()))
                    self._depth_in_range += int((finite & (arr <= 5.0)).sum())
        except Exception as exc:  # noqa: BLE001
            self._depth_err = str(exc)

    def _on_cam(self, msg):
        self._mark("camera_pose")
        self._cam_z.append(float(msg.pose.position.z))
        self._cam_z = self._cam_z[-400:]

    def _topic_diag(self):
        diag = {"imu_messages": self._imu_count, "depth_messages": self._depth_count,
                "depth_stats": {
                    "pixels_total": self._depth_total,
                    "pixels_finite_positive": self._depth_valid,
                    "fraction_finite": (round(self._depth_valid / self._depth_total, 4)
                                        if self._depth_total else None),
                    "pixels_within_5m": self._depth_in_range,
                    "min_finite_m": (None if self._depth_min == float("inf") else round(self._depth_min, 3)),
                    "max_finite_m": (None if self._depth_max == 0.0 else round(self._depth_max, 3)),
                    "decode_error": self._depth_err},
                "stream_gaps": {k: dict(v) for k, v in self._gaps.items()}}
        if self._cam_z:
            diag["camera_pose_z"] = {
                "n": len(self._cam_z), "min": min(self._cam_z),
                "max": max(self._cam_z), "last": self._cam_z[-1]}
        try:
            pubs = self.get_publishers_info_by_topic(DEFAULTS.imu_topic)
            diag["imu_publishers"] = sorted(
                f"{p.node_name}@{p.topic_type.split('/')[-1]}" for p in pubs)
            diag["imu_publisher_count"] = len(pubs)
        except Exception as exc:  # noqa: BLE001
            diag["imu_publishers_error"] = str(exc)
        self.last["topics"] = diag

    def _flush(self):
        self._topic_diag()
        for name in list(self.last):
            self._write(name)


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    rclpy.init()
    node = Recorder(out_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()