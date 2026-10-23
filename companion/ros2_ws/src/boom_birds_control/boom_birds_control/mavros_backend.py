"""PX4 通信经 ROS 2 MAVROS；后端接口仍使用 NED/FRD。"""
from dataclasses import replace
import math
from types import SimpleNamespace
from .px4_backend import MavlinkPx4Backend, MAV_CMD_COMPONENT_ARM_DISARM
from .mavros_observation import decodeObservation


def nedToEnu(v):
    return (float(v[1]), float(v[0]), -float(v[2]))


class MavrosPx4Backend(MavlinkPx4Backend):
    """复用既有状态缓存和 setpoint 校验；不建立 socket、不导入 pymavlink。

    MAVROS raw 状态补齐启动时钟、CURRENT_MODE 和 ODOMETRY reset_counter。
    下行使用互斥的 PositionTarget / AttitudeTarget，以及 CommandBool、CommandLong。
    """
    def __init__(self, node, *, fcu_url="udp://127.0.0.1:14540@127.0.0.1:14580",
                 namespace="/mavros", mavros_node="/mavros_node", target_system=1,
                 target_component=1, control_mode="px4_position", sih_pid=0, **kwargs):
        # 父类仅初始化无 I/O 的缓存与校验器；其 legacy connect 从不调用。
        super().__init__(connection="udpin:127.0.0.1:14540", target_system=target_system,
                         target_component=target_component, **kwargs)
        self._node = node
        self._fcuUrl = str(fcu_url)
        self._namespace = namespace.rstrip("/")
        self._mavrosNode = mavros_node
        self._endpointVerified = False
        self._resources = []
        self._pending = {}
        self._lastEndpointQuery = -math.inf
        self._lastStreamRequest = -math.inf
        self._sourceAgeLimit = self.heartbeat_timeout_s
        self._rosOffset = None
        self._closed = False
        from .sih_guard import verifySihProcess
        self._sihReceiptTime = verifySihProcess(int(sih_pid)) if sih_pid else False
        self.control_mode = control_mode
        self._attitudeScalingReady = False
        self._lastScalingQuery = -math.inf
        self._scalingFuture = None
        from collections import deque
        from .mavros_clock import ImuGate
        # RTT 异常不刷新映射；姿态路线可沿用尚未过期的已收敛映射。
        self._timeGate = ImuGate(hold_last_sync_on_rtt=self.control_mode == "companion_attitude")
        self._attitudeHistory = deque(maxlen=200)
        self._pendingObservations = deque()
        self._guardReason = None
        self._guardDetail = ""
        # 真机 URL 必须显式授权；项目动作服务仍另核验 SIH 进程。
        from urllib.parse import urlsplit
        from .px4_backend import _isLoopbackHost
        try:
            u = urlsplit(self._fcuUrl)
            bind, peer = u.netloc.split("@", 1)
            host = peer.rsplit(":", 1)[0]
            bindHost = bind.rsplit(":", 1)[0]
            self._linkIsLoopback = (u.scheme == "udp" and _isLoopbackHost(host)
                                      and _isLoopbackHost(bindHost))
        except ValueError:
            self._linkIsLoopback = False
        if not self._linkIsLoopback and not self.allow_non_loopback:
            self._guardReason = "non_loopback_fcu_url_refused"
        self._connection = self._fcuUrl
        self._kind = "mavros"

    def connect(self):
        self._bump("connect_attempts")
        if self._guardReason:
            self._refuse(self._guardReason, self._fcuUrl, "connect")
            return False
        if self._resources:
            return True
        from mavros_msgs.msg import Mavlink, PositionTarget, AttitudeTarget, TimesyncStatus
        from mavros_msgs.srv import CommandBool, CommandLong
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
        from rclpy.parameter_client import AsyncParameterClient
        self._positionType = PositionTarget
        self._attitudeType = AttitudeTarget
        self._attitudePub = (self._node.create_publisher(AttitudeTarget,
            self._namespace + "/setpoint_raw/attitude", qos_profile_sensor_data)
            if self.control_mode == "companion_attitude" else None)
        self._scalingParameters = AsyncParameterClient(self._node, self._namespace + "/setpoint_raw")
        self._armType, self._commandType = CommandBool, CommandLong
        self._pub = (self._node.create_publisher(PositionTarget,
            self._namespace + "/setpoint_raw/local", qos_profile_sensor_data)
            if self.control_mode == "px4_position" else None)
        self._foreignPosition = False
        def positionConflict(_):
            self._foreignPosition = True
        self._positionWatch = (self._node.create_subscription(PositionTarget,
            self._namespace + "/setpoint_raw/local", positionConflict, qos_profile_sensor_data)
            if self.control_mode == "companion_attitude" else None)
        self._armClient = self._node.create_client(CommandBool, self._namespace + "/cmd/arming")
        self._commandClient = self._node.create_client(CommandLong, self._namespace + "/cmd/command")
        self._parameters = AsyncParameterClient(self._node, self._mavrosNode)
        self._rawSub = self._node.create_subscription(Mavlink,
            f"/uas{self.target_system}/mavlink_source", self._onRaw,
            QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._syncSub = self._node.create_subscription(TimesyncStatus, self._namespace + "/timesync_status", self._onSync, qos_profile_sensor_data)
        self._timer = self._node.create_timer(.2, self._poll)
        self._observationTimer = self._node.create_timer(.005, self._drainPendingObservations)
        self._observationTimer.cancel()
        self._resources = [r for r in [self._rawSub, self._syncSub, self._timer, self._observationTimer, self._attitudePub, self._pub, self._positionWatch, self._armClient, self._commandClient] if r is not None]
        self._conn = self._node  # 缓存层的连接标志，不是 MAVLink socket。
        self._closed = False
        return True

    def close(self):
        self._closed = True
        for future, _ in list(self._pending.values()): future.cancel()
        self._pending.clear()
        self._pendingObservations.clear()
        for resource in self._resources:
            if resource is self._timer or resource is self._observationTimer: self._node.destroy_timer(resource)
            elif resource is self._rawSub or resource is self._syncSub or resource is self._positionWatch: self._node.destroy_subscription(resource)
            elif resource is self._pub or resource is self._attitudePub: self._node.destroy_publisher(resource)
            else: self._node.destroy_client(resource)
        self._resources.clear()
        self._conn = None
        self._endpointVerified = False

    def _poll(self):
        if self.control_mode == "companion_attitude": self._checkAttitudeScaling()
        now = self._clock()
        for key, (future, sent) in list(self._pending.items()):
            if now - sent > 3.:
                future.cancel()
                self._pending.pop(key, None)
                self._refuse("mavros_service_timeout", str(key), "command")
        if now - self._lastEndpointQuery < 1. or not self._parameters.services_are_ready():
            return
        self._lastEndpointQuery = now
        self._parameters.get_parameters(["fcu_url", "tgt_system", "tgt_component"]).add_done_callback(self._endpointResult)

    def _checkAttitudeScaling(self):
        if self._clock()-self._lastScalingQuery < 1.: return
        if self._scalingFuture is not None or not self._scalingParameters.services_are_ready(): return
        self._lastScalingQuery = self._clock()
        self._scalingFuture = self._scalingParameters.get_parameters(["thrust_scaling"])
        def checked(f):
            self._scalingFuture = None
            if self._closed: return
            try:
                value = f.result().values[0].double_value
                self._attitudeScalingReady = math.isfinite(value) and abs(value - 1.) < 1e-9
                if self._attitudeScalingReady: return
                from rclpy.parameter import Parameter
                self._scalingFuture = self._scalingParameters.set_parameters([Parameter("thrust_scaling", value=1.0)])
                self._scalingFuture.add_done_callback(lambda _: setattr(self, "_scalingFuture", None))
            except Exception as exc:
                self._attitudeScalingReady = False
                self._refuse("attitude_scaling_unverified", str(exc), "send_attitude_setpoint")
        self._scalingFuture.add_done_callback(checked)

    def _endpointResult(self, future):
        if self._closed: return
        try:
            values = future.result().values
            valid = (values[0].string_value == self._fcuUrl and
                     values[1].integer_value == self.target_system and
                     values[2].integer_value == self.target_component)
        except Exception:
            valid = False
        if self._endpointVerified and not valid:
            self._markRestart("mavros_endpoint_changed", self._clock(), self._bootMs or 0, 0)
        self._endpointVerified = valid
        if not valid: self._refuse("mavros_endpoint_mismatch", self._fcuUrl, "connect")

    def _onSync(self, msg):
        previous = self._timeGate.restarts
        self._timeGate.onSync(msg, self._clock(), self._node.get_clock().now().nanoseconds*1e-9)
        if self._timeGate.restarts != previous:
            self._attitudeHistory.clear()
            self._pendingObservations.clear()

    def attitudeAt(self, observedMono, toleranceS):
        gate = self._timeGate
        if self._closed or not self._attitudeHistory: return None
        if not self._sihReceiptTime and (gate.lastSync is None or gate.count < gate.converge_samples
                or not 0 <= self._clock()-gate.lastSync <= gate.sync_timeout_s):
            return None
        stamp, angles = min(self._attitudeHistory, key=lambda item:abs(item[0]-observedMono))
        self._attitudePairDelta = stamp-observedMono
        return angles if abs(stamp-observedMono) <= toleranceS else None

    def _drainPendingObservations(self):
        # 不提前使用、不改采样时间；映射失效或等待超过 20 ms 后重新检查。
        gate = self._timeGate
        if (self._closed or gate.lastSync is None or gate.count < gate.converge_samples
                or not 0 <= self._clock() - gate.lastSync <= gate.sync_timeout_s):
            self._pendingObservations.clear()
            if timer := getattr(self, "_observationTimer", None): timer.cancel()
            return
        while self._pendingObservations:
            msg, sampleRos, queuedAt = self._pendingObservations[0]
            nowRos = self._node.get_clock().now().nanoseconds * 1e-9
            if nowRos < sampleRos and self._clock() - queuedAt < .02: return
            self._pendingObservations.popleft()
            self._onRaw(msg, _deferred=True)
        if timer := getattr(self, "_observationTimer", None): timer.cancel()

    def _onRaw(self, rosMsg, *, _deferred=False):
        if not self._endpointVerified: return
        now = self._clock()
        rosNow = self._node.get_clock().now().nanoseconds * 1e-9
        offset = rosNow - now
        if self._rosOffset is not None and abs(offset - self._rosOffset) > .05:
            self._markRestart("ros_clock_changed", now, self._bootMs or 0, 0)
        self._rosOffset = offset
        stamp = rosMsg.header.stamp.sec + rosMsg.header.stamp.nanosec * 1e-9
        age = rosNow - stamp
        if not math.isfinite(age) or age < -.01 or age > self._sourceAgeLimit:
            self._bump("stale_mavros_observations")
            return
        try:
            msg = decodeObservation(rosMsg)
            if msg is None: return
            if not self._acceptSource(msg.get_srcSystem(), msg.get_srcComponent()):
                self._bump("rejected_source")
                return
            observed = now if self._sihReceiptTime else now - max(age, 0.)
            mid = msg.get_msgId()
            gate = self._timeGate
            mapped = (not self._sihReceiptTime and mid in (30,32) and gate.lastSync is not None
                and gate.count >= gate.converge_samples and 0 <= now-gate.lastSync <= gate.sync_timeout_s)
            if mapped:
                sampleRos = (int(msg.time_boot_ms)*1000000 + gate.offsetNs)*1e-9
                sampleAge = rosNow-sampleRos
                if not _deferred and (-.02 <= sampleAge < 0 or (self._pendingObservations and 0 <= sampleAge <= self._sourceAgeLimit)):
                    if len(self._pendingObservations) >= 200:
                        self._bump("observation_pending_overflow")
                        return
                    self._pendingObservations.append((rosMsg, sampleRos, now))
                    timer = getattr(self, "_observationTimer", None)
                    if timer is not None and timer.isCanceled(): timer.reset()
                    self._bump("observations_future_buffered")
                    return
                if not 0 <= sampleAge <= self._sourceAgeLimit:
                    self._bump("observation_sample_age_rejected")
                    return
                observed = now-sampleAge
            if mid == 30:
                self._handleAttitude(msg, observed)
                if mapped or self._sihReceiptTime: self._attitudeHistory.append((observed,(float(msg.roll),float(msg.pitch),float(msg.yaw))))
            elif mid == 32: self._handleLocalPosition(msg, observed)
            elif mid == 0: self._handleHeartbeat(msg, observed)
            elif mid == 77: self._handleCommandAck(msg, observed)
            else:
                self._handleMessage(msg)
                if mid == 436: self._currentModeMono = observed
                elif mid == 331: self._odomResetMono = observed
                elif mid == 245: self._landedMono = observed
            self._peerSeen = True
        except (ValueError, TypeError, AttributeError) as exc:
            self._bump("parse_errors")
            self.last_error = f"mavros_observation_invalid: {exc}"

    def isConnected(self):
        return self._endpointVerified and super().isConnected()

    def _txPathReady(self):
        if self._closed or not self._endpointVerified:
            return False, "MAVROS FCU URL / target 未核实"
        if not self.isConnected(): return False, "PX4 心跳过期"
        return True, ""

    def _sendCommandLong(self, name, command, *, isArming=False, commanded_armed=None,
                           commanded_mode_name=None, **params):
        if isArming and not self._armingAllowed()[0]:
            self._refuse("arming_not_allowed", self._armingAllowed()[1], name)
            return False
        if self.dry_run:
            self._refuse("dry_run_command_suppressed", name, name)
            return False
        ready, why = self._txPathReady()
        if not ready:
            self._refuse("tx_path_not_ready", why, name)
            return False
        if command in self._pending:
            self._refuse("mavros_command_pending", str(command), name)
            return False
        client = self._armClient if command == MAV_CMD_COMPONENT_ARM_DISARM else self._commandClient
        if not client.serviceIsReady():
            self._refuse("mavros_service_unavailable", name, name)
            return False
        if command == MAV_CMD_COMPONENT_ARM_DISARM:
            request = self._armType.Request(value=bool(commanded_armed))
        else:
            request = self._commandType.Request()
            request.broadcast, request.command, request.confirmation = False, int(command), 0
            for i in range(1, 8): setattr(request, f"param{i}", float(params.get(f"param{i}", 0.)))
        epoch = self._restartEpoch
        try:
            future = client.callAsync(request)
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
            if self._closed or epoch != self._restartEpoch or pending is None or pending[0] is not result: return
            self._pending.pop(command, None)
            try:
                response = result.result()
                ack = int(response.result)
                if not response.success and ack == 0: ack = 4
                self._handleCommandAck(SimpleNamespace(command=command, result=ack), self._clock())
            except Exception as exc:
                self._refuse("mavros_service_failed", str(exc), name)
        future.add_done_callback(done)
        self._bump("commands_sent")
        return True

    def requestObservationStreams(self):
        # MAVROS 只有 common 消息插件；CURRENT_MODE 通过 router 的原始观测读取。
        if self._observationStreamsRequested or not self.isConnected(): return
        if self._clock() - self._lastStreamRequest < 5.: return
        self._lastStreamRequest = self._clock()
        intervals = ({30:50000., 32:500000., 245:1000000., 436:1000000., 331:1000000.}
                     if self.control_mode == "companion_attitude" else
                     {30:20000., 32:20000., 245:1000000., 436:100000., 331:100000.})
        ids = iter(intervals)
        def nextStream(previous=None):
            if self._closed: return
            if previous is not None:
                try:
                    if not previous.result().success: return
                except Exception: return
            mid = next(ids, None)
            if mid is None:
                self._observationStreamsRequested = True
                return
            if not self._commandClient.serviceIsReady(): return
            req = self._commandType.Request(command=511, param1=float(mid),
                param2=intervals[mid])
            self._commandClient.callAsync(req).add_done_callback(nextStream)
        nextStream()

    def activatePlatformVelocity(self, token):
        if not getattr(self, "allow_platform_handoff", False) or not token:
            return False
        if getattr(self, "_platformToken", None):
            return self._platformToken == token
        if self._foreignPosition:
            return False
        if not self.dry_run and (
                self._node.count_publishers(self._namespace + "/setpoint_raw/local") != (1 if self._pub else 0)
                or self._node.count_publishers(self._namespace + "/setpoint_raw/attitude") != (1 if self._attitudePub else 0)):
            return False
        # 先销毁姿态出口，再建立速度出口；token 在本次后端生命周期内不允许换绑。
        for name, destroy in (("_attitudePub", self._node.destroy_publisher),
                              ("_positionWatch", self._node.destroy_subscription)):
            resource = getattr(self, name, None)
            if resource is not None:
                destroy(resource)
                self._resources.remove(resource)
                setattr(self, name, None)
        if self._pub is None:
            from rclpy.qos import qos_profile_sensor_data
            self._pub = self._node.create_publisher(self._positionType,
                self._namespace + "/setpoint_raw/local", qos_profile_sensor_data)
            self._resources.append(self._pub)
        self.control_mode = "px4_position"
        self._platformToken = token
        return True

    def sendSetpoint(self, setpoint, type_mask=None):
        if self.control_mode != "px4_position":
            self._refuse("control_output_conflict", self.control_mode, "send_setpoint")
            return False
        if getattr(self, "_platformToken", None):
            from .platform_landing import VELOCITY_YAW_RATE_MASK
            if type_mask != VELOCITY_YAW_RATE_MASK:
                self._refuse("platform_requires_velocity_only", str(type_mask), "send_setpoint")
                return False
        if self.sih_setpoint_inhibited: return False
        values, mask, reason, detail = self._extractSetpoint(setpoint, type_mask)
        if reason:
            self._refuse(reason, detail, "send_setpoint")
            return False
        self._recordLastSetpoint(values, mask, self._currentBootTimeMs())
        self._bump("setpoints_built")
        if self.dry_run:
            self._bump("setpoints_suppressed_dry_run")
            return True
        ready, why = self._txPathReady()
        if not ready or self._pub.get_subscription_count() == 0:
            self._refuse("tx_path_not_ready", why or "MAVROS setpoint 订阅者缺失", "send_setpoint")
            return False
        msg = self._positionType()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        # FRAME_LOCAL_NED 指 MAVLink 输出系；MAVROS 的 ROS 输入字段仍须使用 ENU。
        # 上游已转成 NED，这里还原 ENU 交给插件；插件再转 NED，mask 原样转发。
        msg.coordinate_frame = 1
        msg.type_mask = mask
        for attr, names in (("position", ("x", "y", "z")), ("velocity", ("vx", "vy", "vz")),
                            ("acceleration_or_force", ("afx", "afy", "afz"))):
            target = getattr(msg, attr)
            target.x, target.y, target.z = nedToEnu(tuple(values[k] for k in names))
        msg.yaw = math.atan2(math.sin(math.pi / 2 - values["yaw"]), math.cos(math.pi / 2 - values["yaw"]))
        msg.yaw_rate = -values["yaw_rate"]
        self._pub.publish(msg)
        self._bump("setpoints_sent")
        return True

    def sendAttitudeSetpoint(self, setpoint):
        from .attitude_control import rotation
        if self.control_mode != "companion_attitude" or self.sih_setpoint_inhibited:
            self._refuse("control_output_conflict", self.control_mode, "send_attitude_setpoint")
            return False
        try:
            rotation(setpoint.orientation_xyzw)
            thrust = float(setpoint.thrust)
            if not math.isfinite(thrust) or not 0 <= thrust <= 1: raise ValueError("thrust_bounds")
        except (ValueError, TypeError, AttributeError) as exc:
            self._refuse("attitude_setpoint_invalid", str(exc), "send_attitude_setpoint")
            return False
        self._bump("setpoints_built")
        self._lastAttitudeSetpoint = dict(orientation_xyzw=list(setpoint.orientation_xyzw), thrust=thrust)
        if self.dry_run:
            self._bump("setpoints_suppressed_dry_run")
            return True
        ready, why = self._txPathReady()
        if not ready or not self._attitudeScalingReady or self._attitudePub.get_subscription_count() == 0:
            self._refuse("attitude_tx_not_ready", why or "thrust_scaling/attitude subscriber not verified", "send_attitude_setpoint")
            return False
        msg = self._attitudeType()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        # 忽略三个机体角速度字段，启用 ENU/FLU 姿态和归一化推力；轴换算由 MAVROS 完成。
        msg.type_mask = 7
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = setpoint.orientation_xyzw
        msg.thrust = thrust
        if (self._node.count_publishers(self._namespace + "/setpoint_raw/attitude") != 1
                or self._foreignPosition):
            self._refuse("conflicting_setpoint_publisher", "MAVROS raw outputs", "send_attitude_setpoint")
            return False
        self._attitudePub.publish(msg)
        self._bump("setpoints_sent")
        return True

    def readVehicleState(self):
        return replace(super().readVehicleState(), source="mavros")

    def streamDiagnostics(self):
        out = super().streamDiagnostics()
        out.update(backend="mavros", endpoint_verified=self._endpointVerified,
            sample_time_source="TEST_ONLY_SIH_router_receipt" if self._sihReceiptTime else "PX4_boot_timesync",
            attitude_pair_delta_s=getattr(self,"_attitudePairDelta",None),
            timesync_reason=self._timeGate.reason, timesync_count=self._timeGate.count,
                   mavros_namespace=self._namespace, live_transport="ROS 2 MAVROS",
                   control_mode=self.control_mode, attitude_scaling_ready=self._attitudeScalingReady,
                   last_attitude_setpoint=getattr(self, "_lastAttitudeSetpoint", None))
        return out
