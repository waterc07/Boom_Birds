"""PX4 通信经 ROS 2 MAVROS；后端接口仍使用 NED/FRD。"""
from dataclasses import replace
import math
import time
from types import SimpleNamespace
from .px4_backend import MavlinkPx4Backend, MAV_CMD_COMPONENT_ARM_DISARM
from .mavros_observation import decode_observation


def ned_to_enu(v):
    return (float(v[1]), float(v[0]), -float(v[2]))


class MavrosPx4Backend(MavlinkPx4Backend):
    """复用既有状态缓存和 setpoint 校验；不建立 socket、不导入 pymavlink。

    MAVROS raw 状态补齐启动时钟、CURRENT_MODE 和 ODOMETRY reset_counter。
    下行只使用 PositionTarget、CommandBool、CommandLong。
    """
    def __init__(self, node, *, fcu_url="udp://127.0.0.1:14540@127.0.0.1:14580",
                 namespace="/mavros", mavros_node="/mavros_node", target_system=1,
                 target_component=1, **kwargs):
        # 父类仅初始化无 I/O 的缓存与校验器；其 legacy connect 从不调用。
        super().__init__(connection="udpin:127.0.0.1:14540", target_system=target_system,
                         target_component=target_component, **kwargs)
        self._node = node
        self._fcu_url = str(fcu_url)
        self._namespace = namespace.rstrip("/")
        self._mavros_node = mavros_node
        self._endpoint_verified = False
        self._resources = []
        self._pending = {}
        self._last_endpoint_query = -math.inf
        self._last_stream_request = -math.inf
        self._source_age_limit = self.heartbeat_timeout_s
        self._ros_offset = None
        self._closed = False
        self._guard_reason = None
        self._guard_detail = ""
        # 真机 URL 必须显式授权；项目动作服务仍另核验 SIH 进程。
        from urllib.parse import urlsplit
        from .px4_backend import _is_loopback_host
        try:
            u = urlsplit(self._fcu_url)
            bind, peer = u.netloc.split("@", 1)
            host = peer.rsplit(":", 1)[0]
            bind_host = bind.rsplit(":", 1)[0]
            self._link_is_loopback = (u.scheme == "udp" and _is_loopback_host(host)
                                      and _is_loopback_host(bind_host))
        except ValueError:
            self._link_is_loopback = False
        if not self._link_is_loopback and not self.allow_non_loopback:
            self._guard_reason = "non_loopback_fcu_url_refused"
        self._connection = self._fcu_url
        self._kind = "mavros"

    def connect(self):
        self._bump("connect_attempts")
        if self._guard_reason:
            self._refuse(self._guard_reason, self._fcu_url, "connect")
            return False
        if self._resources:
            return True
        from mavros_msgs.msg import Mavlink, PositionTarget
        from mavros_msgs.srv import CommandBool, CommandLong
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
        from rclpy.parameter_client import AsyncParameterClient
        self._position_type = PositionTarget
        self._arm_type, self._command_type = CommandBool, CommandLong
        self._pub = self._node.create_publisher(PositionTarget,
            self._namespace + "/setpoint_raw/local", qos_profile_sensor_data)
        self._arm_client = self._node.create_client(CommandBool, self._namespace + "/cmd/arming")
        self._command_client = self._node.create_client(CommandLong, self._namespace + "/cmd/command")
        self._parameters = AsyncParameterClient(self._node, self._mavros_node)
        self._raw_sub = self._node.create_subscription(Mavlink,
            f"/uas{self.target_system}/mavlink_source", self._on_raw,
            QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._timer = self._node.create_timer(.2, self._poll)
        self._resources = [self._raw_sub, self._timer, self._pub, self._arm_client, self._command_client]
        self._conn = self._node  # 缓存层的连接标志，不是 MAVLink socket。
        self._closed = False
        return True

    def close(self):
        self._closed = True
        for future, _ in list(self._pending.values()): future.cancel()
        self._pending.clear()
        for resource in self._resources:
            if resource is self._timer: self._node.destroy_timer(resource)
            elif resource is self._raw_sub: self._node.destroy_subscription(resource)
            elif resource is self._pub: self._node.destroy_publisher(resource)
            else: self._node.destroy_client(resource)
        self._resources.clear()
        self._conn = None
        self._endpoint_verified = False

    def _poll(self):
        now = self._clock()
        for key, (future, sent) in list(self._pending.items()):
            if now - sent > 3.:
                future.cancel()
                self._pending.pop(key, None)
                self._refuse("mavros_service_timeout", str(key), "command")
        if now - self._last_endpoint_query < 1. or not self._parameters.services_are_ready():
            return
        self._last_endpoint_query = now
        self._parameters.get_parameters(["fcu_url", "tgt_system", "tgt_component"]).add_done_callback(self._endpoint_result)

    def _endpoint_result(self, future):
        if self._closed: return
        try:
            values = future.result().values
            valid = (values[0].string_value == self._fcu_url and
                     values[1].integer_value == self.target_system and
                     values[2].integer_value == self.target_component)
        except Exception:
            valid = False
        if self._endpoint_verified and not valid:
            self._mark_restart("mavros_endpoint_changed", self._clock(), self._boot_ms or 0, 0)
        self._endpoint_verified = valid
        if not valid: self._refuse("mavros_endpoint_mismatch", self._fcu_url, "connect")

    def _on_raw(self, ros_msg):
        if not self._endpoint_verified: return
        now = self._clock()
        ros_now = self._node.get_clock().now().nanoseconds * 1e-9
        offset = ros_now - now
        if self._ros_offset is not None and abs(offset - self._ros_offset) > .05:
            self._mark_restart("ros_clock_changed", now, self._boot_ms or 0, 0)
        self._ros_offset = offset
        stamp = ros_msg.header.stamp.sec + ros_msg.header.stamp.nanosec * 1e-9
        age = ros_now - stamp
        if not math.isfinite(age) or age < -.01 or age > self._source_age_limit:
            self._bump("stale_mavros_observations")
            return
        try:
            msg = decode_observation(ros_msg)
            if msg is None: return
            if not self._accept_source(msg.get_srcSystem(), msg.get_srcComponent()):
                self._bump("rejected_source")
                return
            observed = now - max(age, 0.)
            mid = msg.get_msgId()
            if mid == 30: self._handle_attitude(msg, observed)
            elif mid == 32: self._handle_local_position(msg, observed)
            elif mid == 0: self._handle_heartbeat(msg, observed)
            elif mid == 77: self._handle_command_ack(msg, observed)
            else:
                self._handle_message(msg)
                if mid == 436: self._current_mode_mono = observed
                elif mid == 331: self._odom_reset_mono = observed
                elif mid == 245: self._landed_mono = observed
            self._peer_seen = True
        except (ValueError, TypeError, AttributeError) as exc:
            self._bump("parse_errors")
            self.last_error = f"mavros_observation_invalid: {exc}"

    def is_connected(self):
        return self._endpoint_verified and super().is_connected()

    def _tx_path_ready(self):
        if self._closed or not self._endpoint_verified:
            return False, "MAVROS FCU URL / target 未核实"
        if not self.is_connected(): return False, "PX4 心跳过期"
        return True, ""

    def _send_command_long(self, name, command, *, is_arming=False, commanded_armed=None,
                           commanded_mode_name=None, **params):
        if is_arming and not self._arming_allowed()[0]:
            self._refuse("arming_not_allowed", self._arming_allowed()[1], name)
            return False
        if self.dry_run:
            self._refuse("dry_run_command_suppressed", name, name)
            return False
        ready, why = self._tx_path_ready()
        if not ready:
            self._refuse("tx_path_not_ready", why, name)
            return False
        if command in self._pending:
            self._refuse("mavros_command_pending", str(command), name)
            return False
        client = self._arm_client if command == MAV_CMD_COMPONENT_ARM_DISARM else self._command_client
        if not client.service_is_ready():
            self._refuse("mavros_service_unavailable", name, name)
            return False
        if command == MAV_CMD_COMPONENT_ARM_DISARM:
            request = self._arm_type.Request(value=bool(commanded_armed))
        else:
            request = self._command_type.Request()
            request.broadcast, request.command, request.confirmation = False, int(command), 0
            for i in range(1, 8): setattr(request, f"param{i}", float(params.get(f"param{i}", 0.)))
        epoch = self._restart_epoch
        try:
            future = client.call_async(request)
        except Exception as exc:
            self._refuse("mavros_service_failed", str(exc), name)
            return False
        self.commanded_armed = commanded_armed if commanded_armed is not None else self.commanded_armed
        self.commanded_mode_name = commanded_mode_name or self.commanded_mode_name
        self._commands[command] = dict(command=command, name=name, sent_mono_s=self._clock(),
            ack_result=None, ack_result_name=None, ack_mono_s=None)
        self._pending[command] = (future, self._clock())
        def done(result):
            pending = self._pending.get(command)
            if self._closed or epoch != self._restart_epoch or pending is None or pending[0] is not result: return
            self._pending.pop(command, None)
            try:
                response = result.result()
                ack = int(response.result)
                if not response.success and ack == 0: ack = 4
                self._handle_command_ack(SimpleNamespace(command=command, result=ack), self._clock())
            except Exception as exc:
                self._refuse("mavros_service_failed", str(exc), name)
        future.add_done_callback(done)
        self._bump("commands_sent")
        return True

    def request_observation_streams(self):
        # MAVROS 只有 common 消息插件；CURRENT_MODE 通过 router 的原始观测读取。
        if self.dry_run or self._observation_streams_requested or not self.is_connected(): return
        if self._clock() - self._last_stream_request < 5.: return
        self._last_stream_request = self._clock()
        ids = iter((30, 32, 245, 436, 331))
        def next_stream(previous=None):
            if self._closed: return
            if previous is not None:
                try:
                    if not previous.result().success: return
                except Exception: return
            mid = next(ids, None)
            if mid is None:
                self._observation_streams_requested = True
                return
            if not self._command_client.service_is_ready(): return
            req = self._command_type.Request(command=511, param1=float(mid),
                param2=100000. if mid != 245 else 1000000.)
            self._command_client.call_async(req).add_done_callback(next_stream)
        next_stream()

    def send_setpoint(self, setpoint, type_mask=None):
        if self.sih_setpoint_inhibited: return False
        values, mask, reason, detail = self._extract_setpoint(setpoint, type_mask)
        if reason:
            self._refuse(reason, detail, "send_setpoint")
            return False
        self._record_last_setpoint(values, mask, self._current_boot_time_ms())
        self._bump("setpoints_built")
        if self.dry_run:
            self._bump("setpoints_suppressed_dry_run")
            return True
        ready, why = self._tx_path_ready()
        if not ready or self._pub.get_subscription_count() == 0:
            self._refuse("tx_path_not_ready", why or "MAVROS setpoint 订阅者缺失", "send_setpoint")
            return False
        msg = self._position_type()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.coordinate_frame = 1
        msg.type_mask = mask  # MAVROS 原样转发 mask；保持 PX4 NED 轴语义。
        for attr, names in (("position", ("x", "y", "z")), ("velocity", ("vx", "vy", "vz")),
                            ("acceleration_or_force", ("afx", "afy", "afz"))):
            target = getattr(msg, attr)
            target.x, target.y, target.z = ned_to_enu(tuple(values[k] for k in names))
        msg.yaw = math.atan2(math.sin(math.pi / 2 - values["yaw"]), math.cos(math.pi / 2 - values["yaw"]))
        msg.yaw_rate = -values["yaw_rate"]
        self._pub.publish(msg)
        self._bump("setpoints_sent")
        return True

    def read_vehicle_state(self):
        return replace(super().read_vehicle_state(), source="mavros")

    def stream_diagnostics(self):
        out = super().stream_diagnostics()
        out.update(backend="mavros", endpoint_verified=self._endpoint_verified,
                   mavros_namespace=self._namespace, live_transport="ROS 2 MAVROS")
        return out
