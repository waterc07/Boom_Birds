"""MAVROS router HIGHRES_IMU + sys_time 偏移 → 项目 FRD IMU；未同步时拒发。"""
import copy
import struct
import json
import time
from collections import deque
import rclpy
import signal
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from mavros_msgs.msg import State, TimesyncStatus, Mavlink
from mavros_msgs.srv import CommandLong
from .mavros_imu import ImuGate


class MavrosImuNode(Node):
    def __init__(self, **kwargs):
        super().__init__("boom_birds_mavros_imu", **kwargs)
        defaults = dict(mavros_namespace="/mavros", imu_topic="/boom_birds/imu",
            stats_topic="/boom_birds/imu/mavlink_status", heartbeat_timeout_s=3.,
            max_rtt_s=.02, sync_timeout_s=1., converge_samples=5,
            max_offset_deviation_s=.05, max_sample_age_s=1., stream_rate_hz=50.,
            camera_imu_offset_s=0., apply_camera_imu_offset=False, hold_last_sync_on_rtt=True,
            max_future_wait_s=.02)
        for k, v in defaults.items(): self.declare_parameter(k, v)
        get = lambda k: self.get_parameter(k).value
        self.gate = ImuGate(**{k:get(k) for k in ("max_rtt_s", "sync_timeout_s", "converge_samples",
            "max_offset_deviation_s", "max_sample_age_s", "hold_last_sync_on_rtt")})
        if not 0. <= float(get("max_future_wait_s")) <= .02:
            raise ValueError("max_future_wait_s must be between 0 and 0.02 seconds")
        ns = str(get("mavros_namespace")).rstrip("/")
        self._stateTime = None
        self._connected = False
        self._pendingImu = deque()
        self._futureBuffered = 0
        self._rawReceived = 0
        self._sampleTimesUs = deque(maxlen=200)
        self._streamFuture = None
        self._streamRequested = False
        self.pub = self.create_publisher(Imu, str(get("imu_topic")), qos_profile_sensor_data)
        self.stats = self.create_publisher(String, str(get("stats_topic")), 10)
        self.create_subscription(State, ns + "/state", self._state, qos_profile_sensor_data)
        self.create_subscription(TimesyncStatus, ns + "/timesync_status", self._sync, qos_profile_sensor_data)
        self.create_subscription(Mavlink, "/uas1/mavlink_source", self._rawImu,
            QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._command = self.create_client(CommandLong, ns + "/cmd/command")
        # 状态发布需短于 1 s 同步有效期，避免采集器因旧快照误报过期。
        self.create_timer(.2, self._report)
        self._pendingTimer = self.create_timer(.005, self._drainPendingImu)
        self._pendingTimer.cancel()

    def _state(self, msg):
        nowRos = self.get_clock().now().nanoseconds * 1e-9
        age = nowRos - msg.header.stamp.sec - msg.header.stamp.nanosec * 1e-9
        self._connected = bool(msg.connected) and 0 <= age <= float(self.get_parameter("heartbeat_timeout_s").value)
        self._stateTime = time.monotonic() - max(age, 0.)
        if not self._connected:
            self.gate.reset("heartbeat_stale")
            self._pendingImu.clear()
            self._pendingTimer.cancel()
            self._streamRequested = False

    def _sync(self, msg):
        old = self.gate.restarts
        self.gate.onSync(msg, time.monotonic(), self.get_clock().now().nanoseconds * 1e-9)
        if self.gate.lastSync is None:
            self._pendingImu.clear()
            self._pendingTimer.cancel()
        if old != self.gate.restarts: self._streamRequested = False

    def _healthy(self, now):
        return (self._connected and self._stateTime is not None and
                0 <= now - self._stateTime <= float(self.get_parameter("heartbeat_timeout_s").value))

    def _rawImu(self, raw):
        # MAVROS router 负责校验帧；只接受目标 PX4 的完整 HIGHRES_IMU 采样。
        if raw.framing_status != 1 or raw.msgid != 105 or raw.sysid != 1 or raw.compid != 1: return
        data = b"".join(struct.pack("<Q", int(v)) for v in raw.payload64)[:raw.len]
        if len(data) < 61: return  # MAVLink 2 可截断 fields_updated 的高位零字节。
        fields = struct.unpack("<Q13fH", data[:62].ljust(62, b"\0"))
        if fields[-1] & 63 != 63:
            self.gate.reject("imu_fields_unavailable")
            return
        self._rawReceived += 1
        sampleUs = int(fields[0])
        if self._sampleTimesUs and sampleUs <= self._sampleTimesUs[-1]:
            self._sampleTimesUs.clear()
        self._sampleTimesUs.append(sampleUs)
        if self.gate.lastSync is None: return
        msg = Imu()
        ns = int(fields[0]) * 1000 + self.gate.offsetNs
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1000000000)
        # Gate 接口为 FLU；数据原本为 PX4 FRD。
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = fields[1], -fields[2], -fields[3]
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = fields[4], -fields[5], -fields[6]
        self._bufferImu(msg)

    def _bufferImu(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        nowRos = self.get_clock().now().nanoseconds * 1e-9
        wait = float(self.get_parameter("max_future_wait_s").value)
        if 0 < stamp-nowRos <= wait or self._pendingImu:
            if len(self._pendingImu) >= 64:
                self.gate.reject("imu_pending_overflow")
                return
            if stamp > nowRos: self._futureBuffered += 1
            self._pendingImu.append((msg, time.monotonic()))
            if self._pendingTimer.isCanceled(): self._pendingTimer.reset()
            self._drainPendingImu()
        else:
            self._imu(msg)

    def _drainPendingImu(self):
        # 不提前发布、不改时间戳；最多等待 20 ms，超时仍交给门控拒绝。
        wait = float(self.get_parameter("max_future_wait_s").value)
        while self._pendingImu:
            msg, queuedAt = self._pendingImu[0]
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            nowRos = self.get_clock().now().nanoseconds * 1e-9
            if stamp > nowRos and time.monotonic()-queuedAt < wait: return
            self._pendingImu.popleft()
            self._imu(msg)
        self._pendingTimer.cancel()

    def _imu(self, msg):
        now = time.monotonic()
        offset = float(self.get_parameter("camera_imu_offset_s").value) if self.get_parameter("apply_camera_imu_offset").value else 0.
        sample = self.gate.accept(msg, nowMono=now, nowRos=self.get_clock().now().nanoseconds * 1e-9,
                                  connected=self._healthy(now), cameraOffsetS=offset)
        if sample is None: return
        stamp, angular, acceleration = sample
        out = copy.deepcopy(msg)
        ns = round(stamp * 1e9)
        out.header.stamp.sec, out.header.stamp.nanosec = divmod(ns, 1000000000)
        out.header.frame_id = "imu"
        out.orientation.x = out.orientation.y = out.orientation.z = 0.
        out.orientation.w = 1.
        out.orientation_covariance = [-1.] + [0.] * 8
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = angular
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = acceleration
        # Covariance: diag(1,-1,-1) C diag(1,-1,-1).
        signs = (1., -1., -1.)
        for field in ("angular_velocity_covariance", "linear_acceleration_covariance"):
            c = getattr(out, field)
            setattr(out, field, [c[i*3+j] * signs[i] * signs[j] for i in range(3) for j in range(3)])
        self.pub.publish(out)

    def _report(self):
        now = time.monotonic()
        if (self._healthy(now) and not self._streamRequested and self._streamFuture is None
                and self._command.serviceIsReady()):
            rate = float(self.get_parameter("stream_rate_hz").value)
            if rate > 0:
                self._streamFuture = self._command.callAsync(CommandLong.Request(command=511,
                    param1=105., param2=1e6/rate))
                def done(future):
                    self._streamFuture = None
                    try: self._streamRequested = bool(future.result().success)
                    except Exception: self._streamRequested = False
                self._streamFuture.add_done_callback(done)
        samples = self._sampleTimesUs
        sensorHz = ((len(samples)-1)*1e6/(samples[-1]-samples[0])
                     if len(samples) > 1 and samples[-1] > samples[0] else None)
        self.stats.publish(String(data=json.dumps(dict(backend="mavros", connected=self._healthy(now),
            imu_received=self._rawReceived, imu_sensor_rate_hz=sensorHz,
            imu_future_buffered=self._futureBuffered, imu_pending=len(self._pendingImu),
            imu_rejection_counts=dict(self.gate.rejectionCounts),
            sync_rejection_counts=dict(self.gate.syncRejections),
            last_sync_rejection=self.gate.last_sync_rejection,
            last_imu_age_s=self.gate.lastSampleAgeS,
            last_good_sync_age_s=None if self.gate.lastSync is None else now-self.gate.lastSync,
            stream_request_accepted=self._streamRequested,
            imu_published=self.gate.published, imu_rejected=self.gate.rejected,
            sync_samples=self.gate.count, estimated_offset_ns=self.gate.offsetNs,
            last_mapping_reason=self.gate.reason, px4_restarts=self.gate.restarts))))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    handlers = {sig: signal.signal(sig, signal.default_int_handler)
                for sig in (signal.SIGINT, signal.SIGTERM)}
    node = MavrosImuNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
