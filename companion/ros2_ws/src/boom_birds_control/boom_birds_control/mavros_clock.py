"""MAVROS 时间同步与 IMU 门控；输入为同步 ROS 时间和 FLU。"""
import math
from collections import Counter


class ImuGate:
    def __init__(self, *, max_rtt_s=.02, sync_timeout_s=1., converge_samples=5,
                 max_offset_deviation_s=.05, max_sample_age_s=1., hold_last_sync_on_rtt=False):
        self.max_rtt_s, self.sync_timeout_s = max_rtt_s, sync_timeout_s
        self.converge_samples, self.maxDeviation = converge_samples, max_offset_deviation_s
        self.maxAge = max_sample_age_s
        self.hold_last_sync_on_rtt = hold_last_sync_on_rtt
        self.syncRejections = Counter()
        self.rejectionCounts = Counter()
        self.last_sync_rejection = None
        self.lastSampleAgeS = None
        self.count = 0
        self.lastSync = self.lastRemote = self.lastStamp = self.rosOffset = None
        self.offsetNs = 0
        self.reason = "no_timesync_sample"
        self.published = self.rejected = self.restarts = 0

    def reset(self, reason):
        self.count = 0
        self.lastSync = self.lastStamp = None
        self.reason = reason

    def reject(self, reason):
        self.rejected += 1
        self.rejectionCounts[reason] += 1
        self.reason = reason

    def _rejectSync(self, reason, *, keepMapping=False):
        self.syncRejections[reason] += 1
        self.last_sync_rejection = reason
        if not keepMapping:
            self.reset(reason)
        return False

    def onSync(self, msg, nowMono, nowRos):
        # 过期映射必须重新收敛，不能由一条迟到的合格样本直接恢复。
        if self.lastSync is not None and not 0 <= nowMono - self.lastSync <= self.sync_timeout_s:
            self.reset("timesync_stale")
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        rtt = float(msg.round_trip_time_ms) * .001
        remote = int(msg.remote_timestamp_ns)
        if self.lastRemote is not None and remote + 1000000000 < self.lastRemote:
            self.reset("px4_boot_time_regressed")
            self.restarts += 1
        self.lastRemote = remote
        rosOffset = nowRos - nowMono
        if self.rosOffset is not None and abs(rosOffset - self.rosOffset) > .05:
            self.reset("ros_clock_changed")
        self.rosOffset = rosOffset
        if not math.isfinite(rtt) or not 0 <= rtt <= self.max_rtt_s:
            # 超限样本不更新偏移或有效期；启用保留策略时仅沿用尚未过期的已收敛映射。
            keep = (self.hold_last_sync_on_rtt and self.lastSync is not None
                    and self.count >= self.converge_samples)
            return self._rejectSync("timesync_rtt_rejected", keepMapping=keep)
        if not 0 <= nowRos - stamp <= self.sync_timeout_s or remote <= 0:
            return self._rejectSync("timesync_stale")
        if abs(int(msg.observed_offset_ns) - int(msg.estimated_offset_ns)) * 1e-9 > self.maxDeviation:
            return self._rejectSync("timesync_offset_unsettled")
        self.count += 1
        self.offsetNs = int(msg.estimated_offset_ns)
        self.lastSync = nowMono - (nowRos - stamp)
        self.reason = "no_clock_mapping" if self.count < self.converge_samples else "ready"
        return True

    def accept(self, msg, *, nowMono, nowRos, connected, cameraOffsetS=0.):
        reason = None
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.lastSampleAgeS = nowRos - stamp
        if not connected: reason = "heartbeat_stale"
        elif self.lastSync is None or not 0 <= nowMono - self.lastSync <= self.sync_timeout_s:
            reason = "timesync_stale"
        elif self.count < self.converge_samples: reason = "no_clock_mapping"
        elif self.rosOffset is None or abs(nowRos - nowMono - self.rosOffset) > .05:
            self.reset("ros_clock_changed")
            reason = self.reason
        elif stamp <= 0 or not 0 <= nowRos - stamp <= self.maxAge: reason = "imu_sample_age"
        elif self.lastStamp is not None and stamp <= self.lastStamp: reason = "imu_nonmonotonic"
        elif msg.angular_velocity_covariance[0] < 0 or msg.linear_acceleration_covariance[0] < 0:
            reason = "imu_fields_unavailable"
        values = tuple(float(getattr(v, a)) for v in (msg.angular_velocity, msg.linear_acceleration) for a in ("x", "y", "z"))
        if not all(math.isfinite(v) for v in values): reason = "imu_nonfinite"
        if reason:
            self.reject(reason)
            return None
        self.lastStamp = stamp
        self.published += 1
        self.reason = "ready"
        # 保留项目 FRD 契约；MAVROS imu/data_raw 已经从 PX4 FRD 转为 FLU。
        return (stamp + cameraOffsetS, (values[0], -values[1], -values[2]),
                (values[3], -values[4], -values[5]))
