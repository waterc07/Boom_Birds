"""MAVROS IMU 时间与坐标适配；输入为同步 ROS 时间和 FLU。"""
import math


class ImuGate:
    def __init__(self, *, max_rtt_s=.02, sync_timeout_s=1., converge_samples=5,
                 max_offset_deviation_s=.05, max_sample_age_s=1.):
        self.max_rtt_s, self.sync_timeout_s = max_rtt_s, sync_timeout_s
        self.converge_samples, self.max_deviation = converge_samples, max_offset_deviation_s
        self.max_age = max_sample_age_s
        self.count = 0
        self.last_sync = self.last_remote = self.last_stamp = self.ros_offset = None
        self.offset_ns = 0
        self.reason = "no_timesync_sample"
        self.published = self.rejected = self.restarts = 0

    def reset(self, reason):
        self.count = 0
        self.last_sync = self.last_stamp = None
        self.reason = reason

    def on_sync(self, msg, now_mono, now_ros):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        rtt = float(msg.round_trip_time_ms) * .001
        remote = int(msg.remote_timestamp_ns)
        if self.last_remote is not None and remote + 1000000000 < self.last_remote:
            self.reset("px4_boot_time_regressed")
            self.restarts += 1
        self.last_remote = remote
        ros_offset = now_ros - now_mono
        if self.ros_offset is not None and abs(ros_offset - self.ros_offset) > .05:
            self.reset("ros_clock_changed")
        self.ros_offset = ros_offset
        if not math.isfinite(rtt) or not 0 <= rtt <= self.max_rtt_s:
            self.reset("timesync_rtt_rejected")
            return False
        if not 0 <= now_ros - stamp <= self.sync_timeout_s or remote <= 0:
            self.reset("timesync_stale")
            return False
        if abs(int(msg.observed_offset_ns) - int(msg.estimated_offset_ns)) * 1e-9 > self.max_deviation:
            self.reset("timesync_offset_unsettled")
            return False
        self.count += 1
        self.offset_ns = int(msg.estimated_offset_ns)
        self.last_sync = now_mono - (now_ros - stamp)
        self.reason = "no_clock_mapping" if self.count < self.converge_samples else "ready"
        return True

    def accept(self, msg, *, now_mono, now_ros, connected, camera_offset_s=0.):
        reason = None
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not connected: reason = "heartbeat_stale"
        elif self.last_sync is None or not 0 <= now_mono - self.last_sync <= self.sync_timeout_s:
            reason = "timesync_stale"
        elif self.count < self.converge_samples: reason = "no_clock_mapping"
        elif self.ros_offset is None or abs(now_ros - now_mono - self.ros_offset) > .05:
            self.reset("ros_clock_changed")
            reason = self.reason
        elif stamp <= 0 or not 0 <= now_ros - stamp <= self.max_age: reason = "imu_sample_age"
        elif self.last_stamp is not None and stamp <= self.last_stamp: reason = "imu_nonmonotonic"
        elif msg.angular_velocity_covariance[0] < 0 or msg.linear_acceleration_covariance[0] < 0:
            reason = "imu_fields_unavailable"
        values = tuple(float(getattr(v, a)) for v in (msg.angular_velocity, msg.linear_acceleration) for a in ("x", "y", "z"))
        if not all(math.isfinite(v) for v in values): reason = "imu_nonfinite"
        if reason:
            self.rejected += 1
            self.reason = reason
            return None
        self.last_stamp = stamp
        self.published += 1
        self.reason = "ready"
        # 保留项目 FRD 契约；MAVROS imu/data_raw 已经从 PX4 FRD 转为 FLU。
        return (stamp + camera_offset_s, (values[0], -values[1], -values[2]),
                (values[3], -values[4], -values[5]))
