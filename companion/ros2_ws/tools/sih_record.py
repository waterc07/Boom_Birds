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
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
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
        self._statusKeys = {}
        self.cloud_saved = False
        self.create_subscription(PointCloud2, DEFAULTS.sim_map_topic, self._onCloud,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        for topic, name in ((DEFAULTS.mission_status_topic, "mission"),
                            (DEFAULTS.control_status_topic, "control")):
            self.create_subscription(
                String, topic, lambda msg, n=name: self._on(n, msg), RELIABLE)
        # 诊断：/boom_birds/imu 到底有没有数据、有几个发布者
        self._imuCount = 0
        self._depthCount = 0
        self._depthTotal = 0
        self._depthValid = 0
        self._depthInRange = 0
        self._depthMin = float("inf")
        self._depthMax = 0.0
        self._depthErr = None
        self._camZ = []
        # 流间隔统计：EGO 的 odom_depth_timeout 是 1.0 s，因此要能量出
        # "哪一路在什么时候出现了 >1 s 的空洞"，而不是只看计数。
        self._gaps = {}
        self._lastSeen = {}
        # 传感器诊断不要求补齐历史帧，避免记录器向输入发布者施加可靠传输回压。
        self.create_subscription(Imu, DEFAULTS.imu_topic, self._onImu, qos_profile_sensor_data)
        self.create_subscription(Image, DEFAULTS.depth_topic, self._onDepth, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, DEFAULTS.camera_pose_topic, self._onCam, qos_profile_sensor_data)
        from nav_msgs.msg import Odometry
        from boom_birds_interfaces.msg import ExecutionStatus, ControlCommand, PlannerStatus
        self.create_subscription(Odometry, DEFAULTS.odom_topic,
                                 lambda m: self._mark("odom_ego"), qos_profile_sensor_data)
        self.create_subscription(ExecutionStatus, DEFAULTS.status_topic, self._onExecution, RELIABLE)
        for topic, name in ((DEFAULTS.planner_status_topic, "planner_instance"),
                            (DEFAULTS.executor_status_topic, "executor_instance")):
            self.create_subscription(PlannerStatus, topic, lambda m, n=name: self._onInstance(n, m), RELIABLE)
        for topic, name in ((DEFAULTS.planner_command_topic, "planner_command"),
                            (DEFAULTS.command_topic, "control_command")):
            self.create_subscription(ControlCommand, topic, lambda m, n=name: self._onCommand(n, m), RELIABLE)
        self.create_timer(0.5, self._flush)

    def _onCloud(self, msg):
        if self.cloud_saved:
            return
        points = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=False)
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
        data = self.last[name]
        key = (data.get("state"), data.get("reason"), data.get("allow_setpoint"))
        changed = self._statusKeys.get(name) != key
        self._statusKeys[name] = key
        # 周期快照之外保留每次状态变化及全部控制拒绝，避免漏掉短暂失效原因。
        if changed or (name == "control" and not data.get("allow_setpoint", False)):
            self._write(name)

    def _onInstance(self, name, msg):
        self.last[name] = dict(session_id=msg.session_id, producer_session_id=msg.producer_session_id,
                              stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def _onCommand(self, name, msg):
        vector = lambda p: [p.x, p.y, p.z]
        self.last[name] = dict(session_id=msg.session_id, planner_session_id=msg.planner_session_id,
            executor_session_id=msg.executor_session_id, trajectory_id=msg.trajectory_id,
            sequence=msg.sequence, command_type=msg.command_type, frame=msg.header.frame_id,
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            valid_for_s=msg.valid_for.sec + msg.valid_for.nanosec * 1e-9,
            position=vector(msg.position), velocity=vector(msg.velocity), acceleration=vector(msg.acceleration),
            landing=msg.landing, ground_z_world_m=msg.ground_z_world_m,
            yaw=msg.yaw, yaw_rate=msg.yaw_rate)
        self._write(name)

    def _onExecution(self, msg):
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
            "control_mode": msg.control_mode, "control_pose_known": msg.control_pose_known,
            "control_position_world": [msg.control_position_world.x,msg.control_position_world.y,msg.control_position_world.z],
            "control_velocity_world": [msg.control_velocity_world.x,msg.control_velocity_world.y,msg.control_velocity_world.z],
            "control_pose_age_s": msg.control_pose_age_s, "control_yaw_world_rad": msg.control_yaw_world_rad,
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
        key = (msg.sensors_ready, msg.sending, tuple(msg.reasons))
        changed = self._statusKeys.get("execution") != key
        self._statusKeys["execution"] = key
        if changed or not msg.sensors_ready or msg.position_age_s > DEFAULTS.pose_timeout_s:
            self._write("execution")

    def _write(self, name):
        value = self.last.pop(name, None)
        if value is None:
            return
        with open(f"{self.out_dir}/recorder_{name}.jsonl", "a", encoding="utf-8") as fh:
            fh.write(json.dumps({"t": time.time(), "t_mono": time.monotonic(), "data": value}, ensure_ascii=False) + "\n")

    def _mark(self, name):
        now = time.monotonic()
        prev = self._lastSeen.get(name)
        if prev is not None:
            gap = now - prev
            rec = self._gaps.setdefault(name, {"max_gap": 0.0, "max_gap_at": 0.0, "n": 0, "over_1s": 0})
            rec["n"] += 1
            if gap > rec["max_gap"]:
                rec["max_gap"] = gap
                rec["max_gap_at"] = time.time()
            if gap > 1.0:
                rec["over_1s"] += 1
        self._lastSeen[name] = now

    def _onImu(self, msg):
        self._imuCount += 1
        self._mark("imu")

    def _onDepth(self, msg):
        self._depthCount += 1
        self._mark("depth")
        # 直接量测"有没有落在 EGO 可见量程内的深度像素"：EGO 的 mapReady 要求
        # 至少一个 0 < z <= invalid_depth_max_dist_ 的像素，depth_node 的 max_depth_m=5.0。
        try:
            if msg.encoding == "32FC1" and msg.height and msg.width:
                arr = np.frombuffer(bytes(msg.data), dtype=np.float32)
                arr = arr[: msg.height * msg.width].reshape(msg.height, msg.width)
                finite = np.isfinite(arr) & (arr > 0.0)
                self._depthTotal += arr.size
                self._depthValid += int(finite.sum())
                if finite.any():
                    self._depthMin = min(self._depthMin, float(arr[finite].min()))
                    self._depthMax = max(self._depthMax, float(arr[finite].max()))
                    self._depthInRange += int((finite & (arr <= 5.0)).sum())
        except Exception as exc:  # noqa: BLE001
            self._depthErr = str(exc)

    def _onCam(self, msg):
        self._mark("camera_pose")
        self._camZ.append(float(msg.pose.position.z))
        self._camZ = self._camZ[-400:]

    def _topicDiag(self):
        diag = {"imu_messages": self._imuCount, "depth_messages": self._depthCount,
                "depth_stats": {
                    "pixels_total": self._depthTotal,
                    "pixels_finite_positive": self._depthValid,
                    "fraction_finite": (round(self._depthValid / self._depthTotal, 4)
                                        if self._depthTotal else None),
                    "pixels_within_5m": self._depthInRange,
                    "min_finite_m": (None if self._depthMin == float("inf") else round(self._depthMin, 3)),
                    "max_finite_m": (None if self._depthMax == 0.0 else round(self._depthMax, 3)),
                    "decode_error": self._depthErr},
                "stream_gaps": {k: dict(v) for k, v in self._gaps.items()}}
        if self._camZ:
            diag["camera_pose_z"] = {
                "n": len(self._camZ), "min": min(self._camZ),
                "max": max(self._camZ), "last": self._camZ[-1]}
        try:
            pubs = self.get_publishers_info_by_topic(DEFAULTS.imu_topic)
            diag["imu_publishers"] = sorted(
                f"{p.node_name}@{p.topic_type.split('/')[-1]}" for p in pubs)
            diag["imu_publisher_count"] = len(pubs)
        except Exception as exc:  # noqa: BLE001
            diag["imu_publishers_error"] = str(exc)
        self.last["topics"] = diag

    def _flush(self):
        self._topicDiag()
        for name in list(self.last):
            self._write(name)


def main():
    outDir = sys.argv[1] if len(sys.argv) > 1 else "."
    rclpy.init()
    node = Recorder(outDir)
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