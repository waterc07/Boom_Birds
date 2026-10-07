"""ROS 任务编排；所有模式/解锁请求只经过 PX4 接口服务。

接线要点
========

* 本节点**只**有三条出口：``/boom_birds/control/command``（高层 setpoint / 取消）、
  ``/boom_birds/planner/request``（规划使能）与 ``VehicleAction`` 服务（解锁/起飞/
  模式请求）。它不持 MAVLink、不发 PWM/DShot，也不做失效的最终判断。
* 观测只来自 ``ExecutionStatus``（``px4_interface`` 的 ``px4_failsafe`` 判定结果）：
  ``sending`` / ``sensors_ready`` / ``offboard_confirmed`` / ``mode_detail``。
  **命令成功不等于模式回读成功**——Land/Return 的识别按 ``mode_detail``
  （``auto:land`` / ``auto:rtl``）而不是 ``mode``（两者 main mode 都是 ``auto``）。
* 自动恢复缺省关闭（``RuntimeConfig.recovery_enabled=False``，真实设备默认禁用）。
  只有 SIH 任务入口显式把 ROS 参数 ``recovery_enabled`` 置 true 才会打开。
* 旧轨迹不得续播：进入恢复与恢复确认都调 ``_retire_trajectory``，把当前
  ``trajectory_id`` 记入 ``ControlIngress.retired``，同 id 命令随后一律被拒绝。
"""
import copy
from dataclasses import replace
import json
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from boom_birds_interfaces.msg import ControlCommand, ExecutionStatus, PlannerRequest, PlannerStatus
from boom_birds_interfaces.srv import VehicleAction, Mission
from boom_birds_control.control_protocol import Command, ControlIngress
from boom_birds_bringup.lifecycle import Lifecycle, Observation, State, offboard_observed
from boom_birds_control.runtime_config import DEFAULTS, SCENES
from boom_birds_control.px4_frames import LocalFrameAlignment

#: 动作名 → ``VehicleAction`` 请求常量。动作由 ``lifecycle`` 输出，本节点执行；
#: 只有这里能发模式/解锁请求，编排器自己绝不做这件事。
VEHICLE_ACTIONS = {"arm": 1, "takeoff": 2, "hold": 3, "offboard": 4, "land": 5, "return": 6, "disarm": 8}


class LifecycleNode(Node):
    def __init__(self, *, context=None, parameter_overrides=None):
        super().__init__("boom_birds_lifecycle", context=context, parameter_overrides=parameter_overrides)
        self.declare_parameter("scene", "local")
        self.config = SCENES[str(self.get_parameter("scene").value)]
        # 自动恢复的任务级开关：缺省跟随 RuntimeConfig（false），只有 SIH 入口显式打开。
        self.declare_parameter("recovery_enabled", bool(self.config.recovery_enabled))
        if bool(self.get_parameter("recovery_enabled").value) != self.config.recovery_enabled:
            self.config = self.config.with_overrides(
                recovery_enabled=bool(self.get_parameter("recovery_enabled").value))
        self.declare_parameter("control_mode", "px4_position")
        self.declare_parameter("attitude_config_file", "")
        self.control_mode = str(self.get_parameter("control_mode").value)
        if self.control_mode == "companion_attitude":
            import yaml
            from .attitude_lifecycle import AttitudeLifecycle, AttitudeMissionConfig
            with open(str(self.get_parameter("attitude_config_file").value), encoding="utf-8") as f:
                profile = yaml.safe_load(f)
            self.fsm = AttitudeLifecycle(self.config, AttitudeMissionConfig(**profile.get("mission", {})))
        elif self.control_mode == "px4_position":
            self.fsm = Lifecycle(self.config)
        else: raise ValueError("unknown control_mode")
        self.alignment = LocalFrameAlignment(translation_m=self.config.origin)
        self.alignment_observed = self.alignment_data_valid = False
        self._hold_yaw = None
        self.control = None
        self.planner = None
        self.planner_session = self.executor_session = None
        self.executor_received = -math.inf
        self.control_received = self.planner_received = -math.inf
        #: 最近一次观测到 sending=True 的单调时刻，用于 setpoint 断流的**去抖**。
        self._sensor_invalid_since = None
        self._sending_seen_at = -math.inf
        #: 最近一次成功转发规划指令的时刻，用于判断"规划流是否已停止"。
        self._planner_cmd_seen_at = -math.inf
        #: 最近一次重发使能请求的时刻（用于限频，避免把 EGO 的 FSM 打乱）。
        self._enable_request_at = -math.inf
        #: 规划流开始静默的时刻（用于"规划器暂时没有可行轨迹"的有界等待）。
        self._planner_quiet_since = None
        self.start_future = None
        self.start_deadline = 0.
        self.pending_goal = None
        self.action_futures = []
        self.request_seq = self.command_seq = self.trajectory_id = 0
        self.planner_key = None
        self.playing = False
        self.ingress = ControlIngress(self.config.command_timeout_s, self.config.command_future_tolerance_s)
        self.pub_command = self.create_publisher(ControlCommand, DEFAULTS.command_topic, 50)
        self.pub_request = self.create_publisher(PlannerRequest, DEFAULTS.planner_request_topic, 50)
        self.pub_state = self.create_publisher(String, DEFAULTS.mission_status_topic, 10)
        self.client = self.create_client(VehicleAction, DEFAULTS.action_service)
        self.create_subscription(ExecutionStatus, DEFAULTS.status_topic, self._control_status, 50)
        self.create_subscription(PlannerStatus, DEFAULTS.planner_status_topic, self._planner_status, 10)
        self.create_subscription(PlannerStatus, DEFAULTS.executor_status_topic, self._executor_status, 10)
        self.create_subscription(ControlCommand, DEFAULTS.planner_command_topic, self._planner_command, 50)
        self.create_service(Mission, DEFAULTS.mission_service, self._mission)
        from .platform_mission import PlatformMissionGate
        self.platform_gate = PlatformMissionGate(self)
        self.create_timer(.02, self._tick)

    def ros_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _control_status(self, msg):
        age = self.ros_now() - self.stamp(msg)
        if not -self.config.command_future_tolerance_s <= age <= self.config.command_timeout_s: return
        if self.control is not None and self.stamp(msg) < self.stamp(self.control): return
        self.alignment_data_valid = False
        if msg.alignment_valid:
            t = msg.alignment_translation_m
            origin = (t.x, t.y, t.z)
            yaw = msg.alignment_yaw_offset_rad
            if all(math.isfinite(v) for v in (*origin, yaw)):
                changed = (self.alignment_observed and
                    (math.dist(origin, self.alignment.translation_m) > 1e-9 or
                     abs(math.atan2(math.sin(yaw - self.alignment.yaw_offset_rad),
                                    math.cos(yaw - self.alignment.yaw_offset_rad))) > 1e-9))
                if changed and self.fsm.state not in (State.IDLE, State.PRECHECK, State.COMPLETE):
                    if not self.fsm.recovery_revoked:
                        self._actions(self.fsm.latch(time.monotonic(), "frame_reset", detail="alignment_transform_changed"))
                else:
                    self.alignment = LocalFrameAlignment(yaw_offset_rad=yaw, translation_m=origin)
                    self.alignment_observed = self.alignment_data_valid = True
        self.control, self.control_received = msg, time.monotonic()

    def _planner_status(self, msg):
        age = self.ros_now() - self.stamp(msg)
        if msg.session_id != self.fsm.session or not -self.config.command_future_tolerance_s <= age <= self.config.command_timeout_s: return
        if not self._producer_status(msg, "planner_session"): return
        self.planner, self.planner_received = msg, time.monotonic()

    def _producer_status(self, msg, field):
        if not msg.producer_session_id: return False
        known = getattr(self, field)
        if known is not None and known != msg.producer_session_id:
            if self.fsm.recovery_revoked: return False
            if self.fsm.state not in (State.IDLE, State.COMPLETE, State.FAULT_LATCHED):
                self._actions(self.fsm.latch(time.monotonic(), "session_changed", detail=field + "_changed"))
            return False
        setattr(self, field, msg.producer_session_id)
        return True

    def _executor_status(self, msg):
        age = self.ros_now() - self.stamp(msg)
        if (msg.session_id != self.fsm.session or not
                -self.config.command_future_tolerance_s <= age <= self.config.command_timeout_s): return
        if self._producer_status(msg, "executor_session"):
            self.executor_received = time.monotonic()

    def observation(self):
        msg = self.control
        if msg is None: return Observation()
        now = time.monotonic()
        c = self.config
        # `status_age` 描述的是"这份控制接口观测有多旧"，只用报文自身的新鲜度。
        # 曾经把 `msg.heartbeat_age_s` 也取 max 进来，而 PX4 心跳本来就是 ~1 Hz，
        # 于是 status_age 恒为 ≈1.0 s，任何 `<= status_timeout_s (1.0)` 的判定都会
        # 间歇失败（实测 window_reset_by:status_age=1.02），恢复的"连续 1 s 有效"
        # 窗口永远攒不满；接管闸门同样会被误拒。
        # 飞控是否还在线由 `connected` / `sending` / `reasons` 单独表达，
        # 并由 px4_failsafe 用它自己的心跳窗口（heartbeat_timeout_s=2.5）判定，
        # 不需要在这里二次计入。
        age = max(0., now - self.control_received, self.ros_now() - self.stamp(msg))
        position = velocity = (math.nan,) * 3
        companion = self.control_mode == "companion_attitude"
        if companion and msg.control_mode != self.control_mode:
            return Observation(session=msg.session_id, fault="frame_reset")
        if companion and math.isfinite(msg.control_pose_age_s):
            p, v = msg.control_position_world, msg.control_velocity_world
            position, velocity = (p.x,p.y,p.z), (v.x,v.y,v.z)
        elif not companion and msg.position_known:
            p, v = msg.position_ned, msg.velocity_ned
            if all(math.isfinite(x) for x in (p.x, p.y, p.z, v.x, v.y, v.z)):
                position = self.alignment.position_ned_to_ros((p.x, p.y, p.z))
                velocity = self.alignment.rotation().T @ (v.x, v.y, v.z)
        planner_fresh = self.planner is not None and now - self.planner_received <= c.planning_timeout_s
        if msg.sending:
            self._sending_seen_at = now
        executing = self.fsm.state == State.EXECUTING
        recovering = self.fsm.state == State.RECOVERING
        sensors_ready = getattr(msg, "sensors_ready", False)
        if sensors_ready or not executing:
            self._sensor_invalid_since = None
        elif self._sensor_invalid_since is None:
            self._sensor_invalid_since = now
        sensor_fault_confirmed = (self._sensor_invalid_since is not None
                                  and now - self._sensor_invalid_since >= c.sensor_fault_confirm_s)
        fault = ""
        if (executing or recovering) and not (msg.alignment_valid and self.alignment_data_valid):
            # 坐标系对齐证据失效 = 坐标系重置：闭锁并永久撤销恢复资格，不尝试恢复。
            fault = "frame_reset"
        elif planner_fresh and self.planner.geometry_fault:
            fault = "geometry_changed"
        elif executing:
            if (sensor_fault_confirmed or (not sensors_ready
                    and msg.mode_detail in ("auto:land", "auto:rtl"))):
                fault = "sensor_link"
            elif self.playing and (not planner_fresh or not self.planner.map_ready):
                fault = "planning_link"
            elif (not msg.sending
                  and now - self._sending_seen_at > c.setpoint_interrupt_timeout_s):
                # 等待重规划和到点收尾也必须持续发送 HOLD。
                fault = "setpoint_link"
        elif recovering and not getattr(msg, "sensors_ready", False):
            # 恢复期：定位/IMU/深度断流就是恢复条件不满足，如实报告，不当作“还在等”。
            fault = "sensor_link"
        return Observation(session=msg.session_id, connected=msg.connected, armed=msg.armed,
            landed=msg.landed_state if msg.landed_known else None, status_age=age,
            pose_age=(msg.control_pose_age_s if companion else msg.position_age_s) + age, position=tuple(position), velocity=tuple(velocity),
            offboard=msg.offboard_confirmed, mode=msg.mode, mode_detail=msg.mode_detail, boot_epoch=msg.restart_epoch,
            frame_reset_known=msg.frame_reset_known,
            frame_reset_age_s=msg.frame_reset_age_s + age,
            current_mode_detail=msg.current_mode_detail, intended_mode_detail=msg.intended_mode_detail,
            current_mode_age_s=msg.current_mode_age_s + age,
            px4_safety_mode=msg.px4_safety_mode, px4_failsafe_cause=msg.px4_failsafe_cause,
            px4_safety_age_s=msg.px4_safety_age_s + age,
            alignment=msg.alignment_valid and self.alignment_data_valid, map_ready=(planner_fresh and self.planner.map_ready and bool(self.executor_session)
                       and now - self.executor_received <= c.planning_timeout_s),
            sensors_ready=getattr(msg, "sensors_ready", False), sending=msg.sending, fault=fault)

    def _mission(self, request, response):
        response.accepted = False
        if request.action == request.START:
            goal = (request.goal.x, request.goal.y, request.goal.z)
            if self.fsm.state in (State.IDLE, State.COMPLETE) and self.start_future is None and all(math.isfinite(v) for v in goal) and self.client.service_is_ready():
                self.pending_goal = goal
                self.start_future = self.client.call_async(VehicleAction.Request(action=VehicleAction.Request.OPEN_SESSION))
                self.start_deadline = time.monotonic() + self.config.mode_timeout_s
                response.accepted = True
                response.reason = "opening_session"
        elif request.action == request.CANCEL:
            if getattr(self, "platform_gate", None) is not None: self.platform_gate.cancel()
            self.pending_goal = None
            self.start_future = None
            self._actions(self.fsm.cancel(time.monotonic()))
            response.accepted = True
        elif request.action == request.LAND and self.fsm.session:
            if getattr(self, "platform_gate", None) is not None and self.platform_gate.enabled:
                response.accepted = self.platform_gate.request()
                response.reason = "platform_capture_requested"
                response.session_id, response.state = self.fsm.session, self.fsm.state.value
                return response
            if self.control_mode == "companion_attitude":
                self.fsm.hold_here(self.observation())
            self._actions(self.fsm.land(time.monotonic()))
            response.accepted = True
        response.session_id, response.state = self.fsm.session, self.fsm.state.value
        return response

    def _request(self, enabled):
        if not self.fsm.session: return
        msg = PlannerRequest()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.config.world_frame
        msg.session_id = self.fsm.session
        self.request_seq += 1
        msg.sequence = self.request_seq
        msg.enabled = enabled
        if self.fsm.goal is not None:
            msg.goal.x, msg.goal.y, msg.goal.z = self.fsm.goal
        self.pub_request.publish(msg)

    def _publish_command(self, msg):
        self.command_seq += 1
        msg.sequence = self.command_seq
        msg.session_id = self.fsm.session
        msg.trajectory_id = self.trajectory_id
        self.pub_command.publish(msg)

    def _hold(self):
        # 恢复期按加速度上限给出制动段，其余状态维持保持点。
        # 实际发送仍由既有执行许可决定。
        if getattr(self, "platform_gate", None) is not None and self.platform_gate.active:
            point, velocity, acceleration = self.fsm.hold, (0.,)*3, (0.,)*3
        else:
            point, velocity, acceleration = self.fsm.hold_setpoint(time.monotonic())
        if point is None: return
        status = self.control
        heading_age = (float("inf") if status is None else
                       status.attitude_age_s + max(0., time.monotonic() - self.control_received,
                                                  self.ros_now() - self.stamp(status)))
        if status is None: return
        if self.control_mode == "px4_position" and (not status.attitude_known
                or not math.isfinite(status.yaw_ned_rad) or not 0 <= heading_age <= self.config.pose_timeout_s):
            return
        if self.trajectory_id == 0: self.trajectory_id = 1
        msg = ControlCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.config.world_frame
        msg.valid_for.nanosec = int(self.config.command_timeout_s * 1e9)
        msg.command_type = msg.HOLD
        msg.landing = (self.control_mode == "companion_attitude" and self.fsm.state == State.LANDING
                       and not (getattr(self, "platform_gate", None) is not None and self.platform_gate.active))
        msg.ground_z_world_m = float(self.fsm.ground_z or 0.)
        if self.control_mode == "companion_attitude":
            if not math.isfinite(status.control_yaw_world_rad): return
            # HOLD 目标可持续更新命令有效期；执行端仍独立检查实时 VIO 年龄。
            if self._hold_yaw is None: self._hold_yaw = status.control_yaw_world_rad
            msg.yaw = self._hold_yaw
        else:
            msg.yaw = math.atan2(math.sin(-status.yaw_ned_rad - self.alignment.yaw_offset_rad),
                             math.cos(-status.yaw_ned_rad - self.alignment.yaw_offset_rad))
        msg.position.x, msg.position.y, msg.position.z = point
        msg.velocity.x, msg.velocity.y, msg.velocity.z = velocity
        msg.acceleration.x, msg.acceleration.y, msg.acceleration.z = acceleration
        self._publish_command(msg)

    def _retire_trajectory(self, detail):
        """旧轨迹立即退役：停止转发、清缓存；同一 trajectory_id 不再可能续播。

        进入恢复与恢复确认共用这一条：``ControlIngress.cancel()`` 把当前轨迹记入
        ``retired``，之后同 id 的命令一律被 ``retired_trajectory`` 拒绝（见
        ``control_protocol``）。重新规划只认**新的** trajectory_id。
        """
        self.playing = False
        self.planner_key = None
        self.ingress.cancel(detail)
        self.trajectory_id = max(1, self.trajectory_id) + 1

    def _actions(self, actions):
        for action in actions:
            if action == "hold_setpoint": self._hold()
            elif action == "enable_planner": self._request(True)
            elif action == "disable_planner": self._request(False)
            elif action in ("recover", "replan"): self._retire_trajectory(action)
            elif action == "cancel":
                self.playing = False
                self.planner_key = None
                msg = ControlCommand()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.command_type = msg.CANCEL
                self.trajectory_id = max(1, self.trajectory_id)
                self._publish_command(msg)
                self.trajectory_id += 1
                self.ingress.cancel()
            else:
                if not self.client.service_is_ready():
                    self._actions(self.fsm.latch(time.monotonic(), "control_service_missing"))
                    return
                action_id = VEHICLE_ACTIONS[action]
                future = self.client.call_async(VehicleAction.Request(session_id=self.fsm.session, action=action_id))
                self.action_futures.append((future, time.monotonic() + self.config.mode_timeout_s, action))

    def _planner_command(self, msg):
        if getattr(self, "platform_gate", None) is not None and self.platform_gate.active:
            return
        if self.fsm.state != State.EXECUTING: return
        if msg.session_id != self.fsm.session: return
        age = self.ros_now() - self.stamp(msg)
        if not -self.config.command_future_tolerance_s <= age <= self.config.command_timeout_s: return
        if (not self.planner_session or not self.executor_session or
                msg.planner_session_id != self.planner_session or
                msg.executor_session_id != self.executor_session):
            self._actions(self.fsm.latch(time.monotonic(), "session_changed", detail="unregistered_command_producer"))
            return
        o = self.observation()
        # 会话一致 + OFFBOARD 已被模式回读确认：恢复期/未确认时不转发任何规划指令。
        if not offboard_observed(o) or o.session != self.fsm.session: return
        if self.control_mode == "companion_attitude" and not 0 <= o.pose_age <= self.config.pose_timeout_s:
            return
        vector = lambda v: (v.x, v.y, v.z)
        cmd = Command(msg.session_id, msg.trajectory_id, msg.sequence, self.stamp(msg),
            msg.valid_for.sec + msg.valid_for.nanosec * 1e-9, msg.command_type, msg.header.frame_id,
            vector(msg.position), vector(msg.velocity), vector(msg.acceleration), msg.yaw, msg.yaw_rate)
        accepted, _ = self.ingress.receive(cmd, self.ros_now(), time.monotonic())
        if not accepted: return
        if msg.command_type == msg.CANCEL:
            # 规划器作废当前轨迹。这**不等于**任务故障：EGO 在任一次重规划失败时
            # 都会发失效（实测同一任务里反复出现）。早先这里立刻判定
            # planning_link → 恢复，而恢复的前置条件（地图/输入连续有效）在规划器
            # 刚失效时必然不成立，于是两次预算被白耗并闭锁降落。
            # 正确做法：退役旧轨迹、转入"规划流静默"，由 EXECUTING 的有界等待
            # （保持 + 限频重发使能，上限 planner_activate_timeout_s）处理；
            # 只有静默超过该窗口才闭锁（planner_timeout）。
            self._actions(["cancel"])  # 同一回调发布取消屏障，立即清除控制出口缓存。
            if self._planner_quiet_since is None:
                self._planner_quiet_since = time.monotonic()
            return
        if msg.header.frame_id != self.config.world_frame:
            self._actions(self.fsm.latch(time.monotonic(), "frame_reset"))
            return
        key = (msg.session_id, msg.trajectory_id)
        if key != self.planner_key:
            c = self.config
            # 接管闸门：会话已一致，这里再查时间新鲜度 + 位置距离 + **速度连续性**。
            # 位置接得上而速度跳变同样意味着“错过轨迹前段”，必须一并拒绝。
            finite = all(math.isfinite(v) for v in (*o.position, *o.velocity))
            dist_pos = math.dist(cmd.position, o.position) if finite else float("inf")
            dist_vel = math.dist(cmd.velocity, o.velocity) if finite else float("inf")
            reasons = []
            if not finite:
                reasons.append("position_or_velocity_not_finite")
            if not 0 <= o.pose_age <= c.pose_timeout_s:
                reasons.append(f"pose_age={o.pose_age:.3f}>{c.pose_timeout_s}")
            if o.status_age > c.status_timeout_s:
                reasons.append(f"status_age={o.status_age:.3f}>{c.status_timeout_s}")
            if dist_pos > c.handoff_max_distance_m:
                reasons.append(f"dist_pos={dist_pos:.3f}>{c.handoff_max_distance_m}")
            if dist_vel > c.handoff_max_speed_m_s:
                reasons.append(f"dist_vel={dist_vel:.3f}>{c.handoff_max_speed_m_s}")
            if reasons:
                self.get_logger().warning(
                    f"handoff_rejected trajectory={msg.trajectory_id} reasons={','.join(reasons)}")
                now = time.monotonic()
                # 新鲜状态确认的有限、过期位姿属于传感器链路故障。
                # 退役旧轨迹并等待观测恢复；过期期间不发布保持点。
                if (finite and math.isfinite(o.pose_age)
                        and o.pose_age > c.pose_timeout_s
                        and 0 <= o.status_age <= c.status_timeout_s):
                    self._actions(self.fsm.inflight_fault_actions(
                        now, replace(o, fault="sensor_link", sensors_ready=False), "sensor_link"))
                elif (not finite or not 0 <= o.pose_age <= c.pose_timeout_s
                        or not 0 <= o.status_age <= c.status_timeout_s
                        or not self.fsm.hold_here(o)):
                    self._actions(self.fsm.latch(
                        now, "handoff_discontinuous",
                        detail="handoff_discontinuous:" + ",".join(reasons)))
                else:
                    if self._planner_quiet_since is None:
                        self._planner_quiet_since = now
                    self._actions(["cancel", "disable_planner", "hold_setpoint"])
                return
            self.planner_key = key
            self.trajectory_id += 1
        self.playing = True
        self._planner_cmd_seen_at = time.monotonic()
        self._planner_quiet_since = None
        self._publish_command(copy.deepcopy(msg))  # 保留原始时间戳，不给旧命令续期。

    def _tick(self):
        now = time.monotonic()
        if getattr(self, "platform_gate", None) is not None and self.platform_gate.tick(now):
            return
        if self.start_future is not None:
            if self.start_future.done():
                result = self.start_future.result()
                self.start_future = None
                if result.accepted and self.pending_goal is not None:
                    self.alignment_observed = self.alignment_data_valid = False
                    self.planner_session = self.executor_session = None
                    self.planner = None
                    self.executor_received = self.planner_received = -math.inf
                    self._hold_yaw = None
                    self.fsm.start(result.session_id, self.pending_goal, now)
                    self.ingress = ControlIngress(self.config.command_timeout_s, self.config.command_future_tolerance_s)
                    self.ingress.session = result.session_id
                    self.command_seq = self.request_seq = self.trajectory_id = 0
                    self.planner_key = None
                    self.playing = False
                else:
                    self._actions(self.fsm.latch(now, "session_open_rejected"))
            elif now >= self.start_deadline:
                self.start_future = None
                self._actions(self.fsm.latch(now, "session_open_timeout"))
        for future, deadline, action in list(self.action_futures):
            if future.done() or now >= deadline:
                self.action_futures.remove((future, deadline, action))
                if not future.done() or not future.result().accepted:
                    self._actions(self.fsm.latch(now, action + "_request_rejected"))
        # 等待第一条新会话回读；OPEN_SESSION 服务先完成不代表状态话题已到达。
        observed = None
        if self.fsm.state == State.PRECHECK and (self.control is None or self.control.session_id != self.fsm.session) and now - self.fsm.entered < self.config.mode_timeout_s:
            actions = []
        else:
            observed = self.observation()
            actions = self.fsm.tick(now, observed)
        if self.fsm.state == State.LANDING and getattr(self, "platform_gate", None) is not None and self.platform_gate.enabled:
            self.platform_gate.request()
            self.platform_gate.tick(now)
            return
        if self.fsm.state == State.EXECUTING:
            o = self.observation()
            # (a) 规划流已停止（轨迹走完 / 规划器收工）：编排器自己维持一段受约束的
            #     保持/制动，保持在**当前观测位置**。不这样做，setpoint 会断流并先
            #     触发 setpoint_link，把已经到点的任务闭锁降落。
            # 给控制话题和失效检测留一个周期以上的接管余量。
            hold_lead_s = min(self.config.command_timeout_s, self.config.setpoint_timeout_s) / 2
            if now - self._planner_cmd_seen_at > hold_lead_s:
                if self.playing and now - self._planner_cmd_seen_at > self.config.command_timeout_s:
                    self._actions(["cancel", "disable_planner"])
                    self._planner_quiet_since = now
                # 规划流停止后的保持点怎么选：
                #   * 离目标已在接管距离（handoff_max_distance_m，同"近距离接管点"语义）
                #     以内 ⇒ 保持点放到**目标**，让飞控把最后一段收敛掉，
                #     之后 FSM 的到点判定（goal_tolerance_m + 稳定窗）即可成立；
                #   * 否则保持点在**当前位置**，不在没有规划的情况下继续飞。
                goal = self.fsm.goal
                pos = o.position
                close_to_goal = (goal is not None and all(math.isfinite(v) for v in pos)
                                 and math.dist(tuple(pos), tuple(goal))
                                 <= self.config.handoff_max_distance_m)
                if self.fsm.hold_at(goal) if close_to_goal else self.fsm.hold_here(o):
                    if "hold_setpoint" not in actions:
                        actions.append("hold_setpoint")
            # (b) 规划器尚未接管：同一窗口内**重发**使能请求。EGO 的 projectRequest
            #     在 offboard_confirmed / odom / 地图未就绪时直接 return（无日志、无回执），
            #     只发一次会让任务永久停在 WAIT_TARGET 并被判成链路故障。
            if not self.playing:
                if self._planner_quiet_since is None:
                    self._planner_quiet_since = now
                if now - self._planner_quiet_since <= self.config.planner_activate_timeout_s:
                    if "hold_setpoint" not in actions:
                        actions.append("hold_setpoint")
                    # 重发**限频**：EGO 每次 projectRequest 都会重新触发规划，按控制周期
                    # 重发会让它反复重启 FSM 并自行取消目标。
                    if (now - self._enable_request_at >= self.config.planner_enable_retry_s
                            and "enable_planner" not in actions):
                        self._enable_request_at = now
                        actions.append("enable_planner")
                else:
                    actions = self.fsm.latch(now, "planner_timeout")
        self._actions(actions)
        # HOLD_READY → OFFBOARD_PENDING 的三项闸门逐项上报：只报一个
        # map_or_stream_not_ready 无法区分是执行许可、传感器还是地图没就绪
        # （SIH 实测：森林场景五次全部卡在这一步，而深度链本身在跑）。
        gate = None
        if observed is not None:
            gate = dict(
                sending=bool(observed.sending),
                sensors_ready=bool(observed.sensors_ready),
                map_ready=bool(observed.map_ready),
                planner_seen=self.planner is not None,
                planner_map_ready=(None if self.planner is None else bool(self.planner.map_ready)),
                planner_age_s=(None if self.planner is None else round(now - self.planner_received, 3)),
                geometry_fault=(None if self.planner is None else bool(self.planner.geometry_fault)),
                pose_age_s=round(observed.pose_age, 3),
                status_age_s=round(observed.status_age, 3),
            )
        msg = String()
        msg.data = json.dumps(dict(state=self.fsm.state.value, session=self.fsm.session, reason=self.fsm.reason,
            hold_ready_gate=gate, goal_reached=bool(self.fsm.goal_reached),
            planner_session_id=self.planner_session, executor_session_id=self.executor_session,
            history=[dict(state=e["state"], t=time.time() - now + e["at"], reason=e["reason"], detail=e["detail"])
                     for e in self.fsm.events],
            trajectory=self.trajectory_id, sequence=self.command_seq,
            mode=(self.control.mode if self.control is not None else "unknown"),
            mode_detail=(self.control.mode_detail if self.control is not None else "unknown"),
            recovery=dict(enabled=bool(self.config.recovery_enabled), fault=self.fsm.recovery_fault,
                source=self.fsm.recovery_source, attempts=self.fsm.recovery_attempts,
                total_attempts=self.fsm.recovery_total_attempts,
                budget=int(self.config.recovery_attempts), revoked=self.fsm.recovery_revoked,
                detail=self.fsm.recovery_detail)))
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
