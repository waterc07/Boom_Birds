"""VIO 位置闭环的起降；不调用 PX4 AUTO Takeoff/Loiter，故障后停止外部控制。"""
from dataclasses import dataclass
import math
from .lifecycle import Lifecycle, State, offboard_observed


@dataclass(frozen=True)
class AttitudeMissionConfig:
    prestream_s: float = 1.2
    takeoff_speed: float = .3
    landing_speed: float = .25
    flight_budget_s: float = 30.
    landing_reserve_s: float = 14.
    landed_confirm_s: float = 1.

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in self.__dict__.values()): raise ValueError("mission_positive_values")
        if self.landing_reserve_s >= self.flight_budget_s: raise ValueError("landing_reserve")


class AttitudeLifecycle(Lifecycle):
    def __init__(self, config, mission=AttitudeMissionConfig()):
        super().__init__(config)
        if config.recovery_enabled: raise ValueError("attitude recovery is not accepted")
        self.mission = mission
        self.armed_seen = False
        self.airborne_hold = False
        self.flight_started = None
        self.takeoff_origin = None
        self.landing_origin = None
        self.land_since = None
        self.disarm_requested = False
        self._sending_missing_since = self._sensors_missing_since = self._pose_missing_since = None
        self.mode_requested = False
        self.arm_requested = False

    def start(self, session, goal, now):
        accepted = super().start(session, goal, now)
        if accepted:
            self.airborne_hold = self.mode_requested = self.arm_requested = self.disarm_requested = False
            self.armed_seen = False
            self.flight_started = self.land_since = None
            self._sending_missing_since = self._sensors_missing_since = self._pose_missing_since = None
        return accepted

    def latch(self, now, reason, detail=None):
        if self.state == State.FAULT_LATCHED: return []
        actions = super().latch(now, reason, detail)
        # 人工切模式与飞控重启后不争抢模式；其他故障请求后备 Land。
        fallback = self.flight_started is not None and reason not in ("manual_mode","flight_controller_restart")
        return actions + (["land"] if fallback else [])

    def cancel(self, now):
        self.cancelled = True
        return self.latch(now, "manual_cancel")

    def land(self, now):
        if self.state == State.FAULT_LATCHED:
            return ["land"] if self.flight_started is not None else []
        if self.hold is None: return self.latch(now, "landing_pose_missing")
        self.landing_origin = tuple(self.hold)
        self.land_since = None
        self.disarm_requested = False
        self._revoke("landing")
        self.transition(State.LANDING, now)
        return ["disable_planner", "cancel", "hold_setpoint"]

    def hold_setpoint(self, now):
        if self.hold is None: return None, (0.,)*3, (0.,)*3
        if self.state == State.TAKEOFF and self.flight_started is not None:
            t = max(0., now-self.flight_started)
            distance = self.config.takeoff_altitude_agl_m
            # 平滑启动/停止的五次曲线；峰值速度不超过 takeoff_speed。
            duration = 1.875*distance/self.mission.takeoff_speed
            u = min(1., t/duration)
            z = distance*(10*u**3-15*u**4+6*u**5)
            vz = distance*(30*u**2-60*u**3+30*u**4)/duration if u < 1 else 0.
            az = distance*(60*u-180*u**2+120*u**3)/duration**2 if u < 1 else 0.
            return (self.takeoff_origin[0], self.takeoff_origin[1], self.ground_z+z), (0.,0.,vz), (0.,0.,az)
        if self.state == State.LANDING:
            t = max(0., now-self.entered)
            z = max(self.ground_z-.2, self.landing_origin[2]-self.mission.landing_speed*t)
            vz = -self.mission.landing_speed if z > self.ground_z-.2 else 0.
            return (*self.landing_origin[:2], z), (0.,0.,vz), (0.,)*3
        return super().hold_setpoint(now)

    def tick(self, now, o):
        c, m = self.config, self.mission
        if self.state in (State.IDLE, State.COMPLETE, State.FAULT_LATCHED): return []
        if not math.isfinite(now) or now < self.entered: return self.latch(now,"clock_reset")
        if o.session != self.session: return self.latch(now,"session_changed")
        if not 0 <= o.status_age <= c.mode_timeout_s: return self.latch(now,"status_missing")
        if self.boot_epoch is not None and o.boot_epoch != self.boot_epoch: return self.latch(now,"flight_controller_restart")
        if self.state == State.LANDING and not o.armed and o.landed == 1:
            self.transition(State.COMPLETE,now)
            return ["cancel"]
        if self.flight_started is not None and not offboard_observed(o):
            return self.latch(now,"manual_mode")
        if o.armed and not self.arm_requested:
            return self.latch(now,"unexpected_arming")
        if o.armed: self.armed_seen = True
        fresh = (0 <= o.pose_age <= c.pose_timeout_s and o.alignment
                 and all(math.isfinite(x) for x in (*o.position,*o.velocity)))
        if self.armed_seen:
            if not o.alignment: return self.latch(now,"vio_or_attitude_invalid")
            if fresh: self._pose_missing_since = None
            elif self._pose_missing_since is None: self._pose_missing_since = now
            if self._pose_missing_since is not None and now-self._pose_missing_since >= c.sensor_fault_confirm_s:
                return self.latch(now,"vio_or_attitude_invalid")
        if self.armed_seen:
            if o.sending: self._sending_missing_since = None
            elif self._sending_missing_since is None: self._sending_missing_since = now
            if o.sensors_ready: self._sensors_missing_since = None
            elif self._sensors_missing_since is None: self._sensors_missing_since = now
            if self._sending_missing_since is not None and now-self._sending_missing_since >= c.setpoint_interrupt_timeout_s:
                return self.latch(now,"setpoint_link")
            if self._sensors_missing_since is not None and now-self._sensors_missing_since >= c.sensor_fault_confirm_s:
                return self.latch(now,"sensor_link")
        if self.state == State.PRECHECK:
            if o.connected and not o.armed and o.landed == 1 and fresh and o.sensors_ready:
                self.ground_z, self.boot_epoch = o.position[2], o.boot_epoch
                self.hold = self.takeoff_origin = tuple(o.position)
                self.transition(State.HOLD_READY,now)
                return ["hold_setpoint"]
            if now-self.entered > c.precheck_timeout_s: return self.latch(now,"precheck_timeout")
            return []
        if self.state == State.LANDING:
            if o.fault: return self.latch(now,o.fault)
            if not o.armed and o.landed == 1:
                self.transition(State.COMPLETE,now)
                return ["cancel"]
            if not offboard_observed(o): return self.latch(now,"manual_mode")
            if o.landed == 1:
                if self.land_since is None: self.land_since = now
                if now-self.land_since >= m.landed_confirm_s and not self.disarm_requested:
                    self.disarm_requested = True
                    return ["hold_setpoint","disarm"]
            else: self.land_since = None
            if now-self.entered > m.landing_reserve_s: return self.latch(now,"landing_timeout")
            return ["hold_setpoint"]
        if self.flight_started is not None:
            if not o.armed and self.state != State.TAKEOFF: return self.latch(now,"landed_or_disarmed")
            if now-self.flight_started >= m.flight_budget_s-m.landing_reserve_s:
                self.hold_here(o)
                self.reason = "mission_time_budget"
                actions = self.land(now)
                self.reason = "mission_time_budget"
                return actions
            if not offboard_observed(o): return self.latch(now,"manual_mode")
            if o.fault: return self.latch(now,o.fault)

        if self.state == State.HOLD_READY:
            if self.airborne_hold:
                if o.map_ready and o.sending and o.sensors_ready:
                    self.transition(State.EXECUTING,now)
                    return ["hold_setpoint","enable_planner"]
                if now-self.entered > c.hold_ready_timeout_s: return self.latch(now,"map_or_stream_not_ready")
            elif now-self.entered >= m.prestream_s and o.sending and o.sensors_ready and fresh:
                self.transition(State.OFFBOARD_PENDING,now)
                return ["hold_setpoint","offboard"]
            elif now-self.entered > c.hold_ready_timeout_s: return self.latch(now,"ground_stream_timeout")
            return ["hold_setpoint"]
        if self.state == State.OFFBOARD_PENDING:
            if offboard_observed(o):
                self.transition(State.TAKEOFF,now)
                self.arm_requested = True
                self.flight_started = now
                return ["hold_setpoint","arm"]
            if now-self.entered >= c.mode_timeout_s: return self.latch(now,"offboard_not_confirmed")
            return ["hold_setpoint"]
        if self.state == State.TAKEOFF:
            if not o.armed and now-self.entered >= c.mode_timeout_s: return self.latch(now,"arming_not_confirmed")
            if o.armed and o.landed == 2 and self.stable.update(now,o,self.ground_z):
                self.hold = tuple(o.position)
                self.airborne_hold = True
                self.transition(State.HOLD_READY,now)
            return ["hold_setpoint"]
        if self.state == State.EXECUTING:
            if math.dist(o.position,self.goal) <= c.goal_tolerance_m and math.hypot(*o.velocity) <= c.stable_speed_m_s:
                if self.goal_since is None: self.goal_since = now
                if now-self.goal_since >= c.stable_duration_s:
                    self.goal_reached = True
                    self.hold_here(o)
                    return self.land(now)
            else: self.goal_since = None
            return []
        return []
