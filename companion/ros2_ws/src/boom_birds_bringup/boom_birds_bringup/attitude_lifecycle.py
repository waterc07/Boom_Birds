"""VIO 位置闭环的起降；不调用 PX4 AUTO Takeoff/Loiter，故障后停止外部控制。"""
from dataclasses import dataclass
import math
from .lifecycle import Lifecycle, State, offboardObserved


@dataclass(frozen=True)
class AttitudeMissionConfig:
    prestream_s: float = 1.2
    takeoff_speed: float = .3
    landing_speed: float = .25
    flight_budget_s: float = 30.
    landing_reserve_s: float = 14.
    landed_confirm_s: float = 1.

    def __post_init__(self):
        if any(not math.isfinite(value) or value <= 0 for value in self.__dict__.values()):
            raise ValueError("mission_positive_values")
        if self.landing_reserve_s >= self.flight_budget_s:
            raise ValueError("landing_reserve")


class AttitudeLifecycle(Lifecycle):
    def __init__(self, config, mission=AttitudeMissionConfig()):
        super().__init__(config)
        if config.recovery_enabled:
            raise ValueError("attitude recovery is not accepted")
        self.mission = mission
        self.armed_seen = False
        self.airborneHold = False
        self.flightStarted = None
        self.takeoffOrigin = None
        self.landingOrigin = None
        self.landSince = None
        self.disarmRequested = False
        self._sendingMissingSince = self._sensorsMissingSince = self._poseMissingSince = None
        self.modeRequested = False
        self.armRequested = False

    def start(self, session, goal, now):
        accepted = super().start(session, goal, now)
        if accepted:
            self.airborneHold = self.modeRequested = self.armRequested = self.disarmRequested = False
            self.armed_seen = False
            self.flightStarted = self.landSince = None
            self._sendingMissingSince = self._sensorsMissingSince = self._poseMissingSince = None
        return accepted

    def latch(self, now, reason, detail=None):
        if self.state == State.FAULT_LATCHED:
            return []
        actions = super().latch(now, reason, detail)
        # 人工切模式与飞控重启后不争抢模式；其他故障请求后备 Land。
        fallback = (self.flightStarted is not None
                    and reason not in ("manual_mode", "flight_controller_restart"))
        return actions + (["land"] if fallback else [])

    def cancel(self, now):
        self.cancelled = True
        return self.latch(now, "manual_cancel")

    def land(self, now):
        if self.state == State.FAULT_LATCHED:
            return ["land"] if self.flightStarted is not None else []
        if self.hold is None:
            return self.latch(now, "landing_pose_missing")
        self.landingOrigin = tuple(self.hold)
        self.landSince = None
        self.disarmRequested = False
        self._revoke("landing")
        self.transition(State.LANDING, now)
        return ["disable_planner", "cancel", "hold_setpoint"]

    def holdSetpoint(self, now):
        if self.hold is None:
            return None, (0., )*3, (0., )*3
        if self.state == State.TAKEOFF and self.flightStarted is not None:
            t = max(0., now-self.flightStarted)
            distance = self.config.takeoff_altitude_agl_m
            # 平滑启动/停止的五次曲线；峰值速度不超过 takeoff_speed。
            duration = 1.875*distance/self.mission.takeoff_speed
            u = min(1., t/duration)
            z = distance*(10*u**3-15*u**4+6*u**5)
            vz = distance*(30*u**2-60*u**3+30*u**4)/duration if u < 1 else 0.
            az = distance*(60*u-180*u**2+120*u**3)/duration**2 if u < 1 else 0.
            return (self.takeoffOrigin[0], self.takeoffOrigin[1], self.groundZ+z), (0., 0., vz), (0., 0., az)
        if self.state == State.LANDING:
            t = max(0., now - self.entered)
            z = max(self.groundZ-.2, self.landingOrigin[2]-self.mission.landing_speed*t)
            vz = -self.mission.landing_speed if z > self.groundZ-.2 else 0.
            return (*self.landingOrigin[:2], z), (0., 0., vz), (0., )*3
        return super().holdSetpoint(now)

    def tick(self, now, o):
        observation = o
        config, mission = self.config, self.mission
        if self.state in (State.IDLE, State.COMPLETE, State.FAULT_LATCHED):
            return []
        if not math.isfinite(now) or now < self.entered:
            return self.latch(now, "clock_reset")
        if observation.session != self.session:
            return self.latch(now, "session_changed")
        if not 0 <= observation.status_age <= config.mode_timeout_s:
            return self.latch(now, "status_missing")
        if self.boot_epoch is not None and observation.boot_epoch != self.boot_epoch:
            return self.latch(now, "flight_controller_restart")
        if self.state == State.LANDING and not observation.armed and observation.landed == 1:
            self.transition(State.COMPLETE, now)
            return ["cancel"]
        if self.flightStarted is not None and not offboardObserved(observation):
            return self.latch(now, "manual_mode")
        if observation.armed and not self.armRequested:
            return self.latch(now, "unexpected_arming")
        if observation.armed:
            self.armed_seen = True
        poseFresh = (
            0 <= observation.pose_age <= config.pose_timeout_s
            and observation.alignment
            and all(math.isfinite(x) for x in (*observation.position, *observation.velocity))
        )
        if self.armed_seen:
            fault = self._armedInputFault(now, observation, poseFresh)
            if fault:
                return self.latch(now, fault)
        if self.state == State.PRECHECK:
            return self._precheckActions(now, observation, poseFresh)
        if self.state == State.LANDING:
            return self._landingActions(now, observation)
        if self.flightStarted is not None:
            if not observation.armed and self.state != State.TAKEOFF:
                return self.latch(now, "landed_or_disarmed")
            # 预算从解锁请求计时，并为下降留出时间；不代表已完成赛事起止计时验收。
            if now - self.flightStarted >= mission.flight_budget_s - mission.landing_reserve_s:
                self.holdHere(observation)
                self.reason = "mission_time_budget"
                actions = self.land(now)
                self.reason = "mission_time_budget"
                return actions
            if not offboardObserved(observation):
                return self.latch(now, "manual_mode")
            if observation.fault:
                return self.latch(now, observation.fault)

        if self.state == State.HOLD_READY:
            return self._holdReadyActions(now, observation, poseFresh)
        if self.state == State.OFFBOARD_PENDING:
            return self._offboardPendingActions(now, observation)
        if self.state == State.TAKEOFF:
            return self._takeoffActions(now, observation)
        if self.state == State.EXECUTING:
            return self._executingActions(now, observation)
        return []

    def _armedInputFault(self, now, observation, poseFresh):
        config = self.config
        if not observation.alignment:
            return "vio_or_attitude_invalid"
        if poseFresh:
            self._poseMissingSince = None
        elif self._poseMissingSince is None:
            self._poseMissingSince = now
        if (self._poseMissingSince is not None
                and now - self._poseMissingSince >= config.sensor_fault_confirm_s):
            return "vio_or_attitude_invalid"
        if observation.sending:
            self._sendingMissingSince = None
        elif self._sendingMissingSince is None:
            self._sendingMissingSince = now
        if observation.sensors_ready:
            self._sensorsMissingSince = None
        elif self._sensorsMissingSince is None:
            self._sensorsMissingSince = now
        if (self._sendingMissingSince is not None
                and now - self._sendingMissingSince >= config.setpoint_interrupt_timeout_s):
            return "setpoint_link"
        if (self._sensorsMissingSince is not None
                and now - self._sensorsMissingSince >= config.sensor_fault_confirm_s):
            return "sensor_link"
        return None

    def _precheckActions(self, now, observation, poseFresh):
        config = self.config
        if (observation.connected and not observation.armed and observation.landed == 1
                and poseFresh and observation.sensors_ready):
            self.groundZ, self.boot_epoch = observation.position[2], observation.boot_epoch
            self.hold = self.takeoffOrigin = tuple(observation.position)
            self.transition(State.HOLD_READY, now)
            return ["hold_setpoint"]
        if now - self.entered > config.precheck_timeout_s:
            return self.latch(now, "precheck_timeout")
        return []

    def _landingActions(self, now, observation):
        mission = self.mission
        if observation.fault:
            return self.latch(now, observation.fault)
        if not observation.armed and observation.landed == 1:
            self.transition(State.COMPLETE, now)
            return ["cancel"]
        if not offboardObserved(observation):
            return self.latch(now, "manual_mode")
        if observation.landed == 1:
            if self.landSince is None:
                self.landSince = now
            if now - self.landSince >= mission.landed_confirm_s and not self.disarmRequested:
                self.disarmRequested = True
                return ["hold_setpoint", "disarm"]
        else:
            self.landSince = None
        if now - self.entered > mission.landing_reserve_s:
            return self.latch(now, "landing_timeout")
        return ["hold_setpoint"]

    def _holdReadyActions(self, now, observation, poseFresh):
        config = self.config
        mission = self.mission
        if self.airborneHold:
            if observation.map_ready and observation.sending and observation.sensors_ready:
                self.transition(State.EXECUTING, now)
                return ["hold_setpoint", "enable_planner"]
            if now - self.entered > config.hold_ready_timeout_s:
                return self.latch(now, "map_or_stream_not_ready")
        elif (now - self.entered >= mission.prestream_s and observation.sending
                and observation.sensors_ready and poseFresh):
            self.transition(State.OFFBOARD_PENDING, now)
            return ["hold_setpoint", "offboard"]
        elif now - self.entered > config.hold_ready_timeout_s:
            return self.latch(now, "ground_stream_timeout")
        return ["hold_setpoint"]

    def _offboardPendingActions(self, now, observation):
        config = self.config
        if offboardObserved(observation):
            self.transition(State.TAKEOFF, now)
            self.armRequested = True
            self.flightStarted = now
            return ["hold_setpoint", "arm"]
        if now - self.entered >= config.mode_timeout_s:
            return self.latch(now, "offboard_not_confirmed")
        return ["hold_setpoint"]

    def _takeoffActions(self, now, observation):
        config = self.config
        if not observation.armed and now - self.entered >= config.mode_timeout_s:
            return self.latch(now, "arming_not_confirmed")
        if (observation.armed and observation.landed == 2
                and self.stable.update(now, observation, self.groundZ)):
            self.hold = tuple(observation.position)
            self.airborneHold = True
            self.transition(State.HOLD_READY, now)
        return ["hold_setpoint"]

    def _executingActions(self, now, observation):
        config = self.config
        if (math.dist(observation.position, self.goal) <= config.goal_tolerance_m
                and math.hypot(*observation.velocity) <= config.stable_speed_m_s):
            if self.goalSince is None:
                self.goalSince = now
            if now-self.goalSince >= config.stable_duration_s:
                self.goal_reached = True
                self.holdHere(observation)
                return self.land(now)
        else:
            self.goalSince = None
        return []
