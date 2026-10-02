"""MAVROS router HIGHRES_IMU + sys_time 偏移 → 项目 FRD IMU；未同步时拒发。"""
import copy
import struct
import json
import time
import rclpy
from rclpy.node import Node
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
            camera_imu_offset_s=0., apply_camera_imu_offset=False)
        for k, v in defaults.items(): self.declare_parameter(k, v)
        get = lambda k: self.get_parameter(k).value
        self.gate = ImuGate(**{k:get(k) for k in ("max_rtt_s", "sync_timeout_s", "converge_samples",
            "max_offset_deviation_s", "max_sample_age_s")})
        ns = str(get("mavros_namespace")).rstrip("/")
        self._state_time = None
        self._connected = False
        self._stream_future = None
        self._stream_requested = False
        self.pub = self.create_publisher(Imu, str(get("imu_topic")), qos_profile_sensor_data)
        self.stats = self.create_publisher(String, str(get("stats_topic")), 10)
        self.create_subscription(State, ns + "/state", self._state, qos_profile_sensor_data)
        self.create_subscription(TimesyncStatus, ns + "/timesync_status", self._sync, qos_profile_sensor_data)
        self.create_subscription(Mavlink, "/uas1/mavlink_source", self._raw_imu,
            QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._command = self.create_client(CommandLong, ns + "/cmd/command")
        self.create_timer(1., self._report)

    def _state(self, msg):
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        age = now_ros - msg.header.stamp.sec - msg.header.stamp.nanosec * 1e-9
        self._connected = bool(msg.connected) and 0 <= age <= float(self.get_parameter("heartbeat_timeout_s").value)
        self._state_time = time.monotonic() - max(age, 0.)
        if not self._connected:
            self.gate.reset("heartbeat_stale")
            self._stream_requested = False

    def _sync(self, msg):
        old = self.gate.restarts
        self.gate.on_sync(msg, time.monotonic(), self.get_clock().now().nanoseconds * 1e-9)
        if old != self.gate.restarts: self._stream_requested = False

    def _healthy(self, now):
        return (self._connected and self._state_time is not None and
                0 <= now - self._state_time <= float(self.get_parameter("heartbeat_timeout_s").value))

    def _raw_imu(self, raw):
        # MAVROS router 负责校验帧；只接受目标 PX4 的完整 HIGHRES_IMU 采样。
        if raw.framing_status != 1 or raw.msgid != 105 or raw.sysid != 1 or raw.compid != 1: return
        data = b"".join(struct.pack("<Q", int(v)) for v in raw.payload64)[:raw.len]
        if len(data) < 61: return  # MAVLink 2 可截断 fields_updated 的高位零字节。
        fields = struct.unpack("<Q13fH", data[:62].ljust(62, b"\0"))
        if fields[-1] & 63 != 63:
            self.gate.rejected += 1
            self.gate.reason = "imu_fields_unavailable"
            return
        if self.gate.last_sync is None: return
        msg = Imu()
        ns = int(fields[0]) * 1000 + self.gate.offset_ns
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1000000000)
        # Gate 接口为 FLU；数据原本为 PX4 FRD。
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = fields[1], -fields[2], -fields[3]
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = fields[4], -fields[5], -fields[6]
        self._imu(msg)

    def _imu(self, msg):
        now = time.monotonic()
        offset = float(self.get_parameter("camera_imu_offset_s").value) if self.get_parameter("apply_camera_imu_offset").value else 0.
        sample = self.gate.accept(msg, now_mono=now, now_ros=self.get_clock().now().nanoseconds * 1e-9,
                                  connected=self._healthy(now), camera_offset_s=offset)
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
        if (self._healthy(now) and not self._stream_requested and self._stream_future is None
                and self._command.service_is_ready()):
            rate = float(self.get_parameter("stream_rate_hz").value)
            if rate > 0:
                self._stream_future = self._command.call_async(CommandLong.Request(command=511,
                    param1=105., param2=1e6/rate))
                def done(future):
                    self._stream_future = None
                    try: self._stream_requested = bool(future.result().success)
                    except Exception: self._stream_requested = False
                self._stream_future.add_done_callback(done)
        self.stats.publish(String(data=json.dumps(dict(backend="mavros", connected=self._healthy(now),
            imu_published=self.gate.published, imu_rejected=self.gate.rejected,
            sync_samples=self.gate.count, estimated_offset_ns=self.gate.offset_ns,
            last_mapping_reason=self.gate.reason, px4_restarts=self.gate.restarts))))


def main(args=None):
    rclpy.init(args=args)
    node = MavrosImuNode()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
