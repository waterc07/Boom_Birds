"""Fixed flight tasks in the takeoff frame; no planner or output-mode handoff."""
from dataclasses import dataclass
import math
from pathlib import Path
import yaml
import numpy as np
from .attitude_lifecycle import AttitudeLifecycle
from .lifecycle import State
from boom_birds_control.attitude_control import wrap


@dataclass(frozen=True)
class LandingFix:
    stamp: float
    target: tuple
    height: float
    source: str = "tag"
    identity_stamp: float = 0.
    yaw_valid: bool = True
    tag_ids: tuple = ()


def loadTask(path):
    task = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(task, dict) or task.get("schema") != 1:
        raise ValueError("flight_task_schema")
    names = ("height_m", "max_speed_m_s", "max_accel_m_s2", "max_yaw_rate_rad_s",
             "max_yaw_accel_rad_s2", "position_tolerance_m", "yaw_tolerance_rad",
             "stable_speed_m_s", "settle_s", "stage_timeout_s", "tag_timeout_s",
             "tag_acquire_timeout_s", "tag_loss_timeout_s", "tag_align_m", "tag_align_s", "max_tag_correction_m",
             "max_tag_ground_error_m", "max_radius_m", "ceiling_margin_m",
             "landing_speed_m_s", "max_landing_xy_m_s", "flare_height_m", "flare_speed_m_s",
             "final_land_height_m", "final_land_confirm_s", "final_land_timeout_s")
    if any(type(task.get(k)) not in (int, float) or not math.isfinite(task[k]) or task[k] <= 0 for k in names):
        raise ValueError("flight_task_positive_values")
    if (task["flare_height_m"] >= task["height_m"]
            or task["flare_speed_m_s"] > task["landing_speed_m_s"]
            or task["tag_align_m"] >= task["max_tag_correction_m"]
            or task["yaw_tolerance_rad"] >= math.pi
            or task["final_land_height_m"] >= task["flare_height_m"]):
        raise ValueError("flight_task_bounds")
    steps = task.get("steps")
    if not isinstance(steps, list) or not steps or steps[-1] != {"action": "land"}:
        raise ValueError("flight_task_final_land")
    if not isinstance(task.get("name"), str) or not task["name"]:
        raise ValueError("flight_task_name")
    for step in steps[:-1]:
        if not isinstance(step, dict) or step.get("action") not in ("hold", "move", "turn"):
            raise ValueError("flight_task_step")
        allowed = {"hold": {"action", "duration_s", "require_tag"},
                   "move": {"action", "forward_m"},
                   "turn": {"action", "yaw_deg"}}[step["action"]]
        if set(step) != allowed:
            raise ValueError("flight_task_step_fields")
        if step["action"] == "hold":
            if type(step["require_tag"]) is not bool or type(step["duration_s"]) not in (int, float) or not math.isfinite(step["duration_s"]) or step["duration_s"] < 0:
                raise ValueError("flight_task_hold")
        else:
            key = "forward_m" if step["action"] == "move" else "yaw_deg"
            if type(step[key]) not in (int, float) or not math.isfinite(step[key]):
                raise ValueError("flight_task_step_value")
            if key == "forward_m" and abs(step[key]) >= task["max_radius_m"]:
                raise ValueError("flight_task_radius")
    return task


def handoverSourceAllowed(source, handover):
    """平台交接只接受完整身份解码；关闭交接时保留旧的解码/特征两条末段路径。"""
    if handover:
        return source == "tag"
    return source in ("tag", "resolved_tag")


def smoothSegment(elapsed, duration):
    u = min(1., max(0., elapsed / duration))
    return (10*u**3-15*u**4+6*u**5,
            (30*u**2-60*u**3+30*u**4)/duration,
            (60*u-180*u**2+120*u**3)/duration**2)


class TagFlightTask(AttitudeLifecycle):
    requiresPlanner = False

    def __init__(self, config, mission, task, obstacleConfig=None, handover=False):
        super().__init__(config.withOverrides(takeoff_altitude_agl_m=task["height_m"],
                                              hold_lock_min_altitude_agl_m=task["height_m"]), mission)
        self.task = task
        # 交接开启时，末段必须由完整身份解码进入平台速度接管；只靠局部特征的
        # 定位置信不能在 VIO 退出后继续使用，因此不作为交接入口。
        self.handover = bool(handover)
        self.landingInputsReady = True
        from .task_depth_guard import ObstacleConfig, ObstacleReport
        self.obstacleConfig = obstacleConfig or ObstacleConfig()
        self.obstacle = ObstacleReport(-math.inf,"UNKNOWN","depth_missing")
        self.validateBudget()
        self.currentYaw = None
        self.fix = None
        self.resetTask()


    def validateBudget(self):
        elapsed = 1.875*self.task["height_m"]/self.mission.takeoff_speed+self.config.stable_duration_s
        forward = yaw = 0.
        for step in self.task["steps"]:
            if step["action"] == "land":
                elapsed += self.task["tag_acquire_timeout_s"]
                break
            distance = angle = 0.
            if step["action"] == "move":
                distance = abs(step["forward_m"]-forward)
                forward = step["forward_m"]
            elif step["action"] == "turn":
                nextYaw = math.radians(step["yaw_deg"])
                angle, yaw = abs(nextYaw-yaw), nextYaw
            duration = max(.02, 1.875*distance/self.task["max_speed_m_s"],
                math.sqrt(5.774*distance/self.task["max_accel_m_s2"]),
                1.875*angle/self.task["max_yaw_rate_rad_s"],
                math.sqrt(5.774*angle/self.task["max_yaw_accel_rad_s2"]))
            elapsed += duration+max(self.task["settle_s"],step.get("duration_s",0.))
        if elapsed >= self.mission.flight_budget_s-self.mission.landing_reserve_s:
            raise ValueError("tag_task_nominal_duration_exceeds_flight_budget")

    def resetTask(self):
        from .task_avoidance import TaskAvoidance
        self.avoidance = TaskAvoidance(self.task,self.obstacleConfig)
        self.obstacleMissionFailed = self.reachedHome = False
        self.savedPhase = None
        self.stageIndex = -1
        self.phase = "IDLE"
        self.yawSetpoint = None
        self.initialYaw = None
        self.stageSince = self.settleSince = self.alignSince = None
        self.segmentStart = self.segmentGoal = None
        self.segmentYawStart = self.segmentYawGoal = None
        self.segmentDuration = 0.02
        self.landingSetpoint = None
        self.landingVelocity = (0.,)*3
        self.lastLandingTick = None
        self.landingSince = None
        self.tagCaptured = False
        self.tagSamples = 0
        self.lastTagStamp = None
        self.lastConfirmedFix = None
        self.nativeLandStarted = None
        self.nativeLandConfirmed = False
        self.nearGroundSince = None
        self.finalCaptureZ = None
        self.fix = None
        # 交接证据：身份、来源、连续样本数、采样时刻与偏航有效性；单帧不成立。
        self.capture = dict(ready=False, reason="not_captured")

    def start(self, session, goal, now):
        accepted = super().start(session, goal, now)
        if accepted:
            self.resetTask()
        return accepted

    def freshFix(self, now):
        fix = self.fix
        return (fix is not None and math.isfinite(fix.stamp)
                and 0 <= now-fix.stamp <= self.task["tag_timeout_s"]
                and len(fix.target) == 3 and all(math.isfinite(v) for v in (*fix.target, fix.height))
                and fix.height > 0 and self.groundZ is not None
                and abs(fix.target[2]-self.groundZ) <= self.task["max_tag_ground_error_m"])

    def _precheckActions(self, now, observation, poseFresh):
        if not self.landingInputsReady:
            if now-self.entered>self.config.precheck_timeout_s:
                return self.latch(now,"landing_inputs_precheck_unavailable")
            return []
        if not self.obstacleClear(now):
            if now-self.entered > self.config.precheck_timeout_s:
                return self.latch(now,"obstacle_precheck_unavailable")
            return []
        if self.currentYaw is None or not math.isfinite(self.currentYaw):
            if now-self.entered > self.config.precheck_timeout_s:
                return self.latch(now, "task_heading_missing")
            return []
        actions = super()._precheckActions(now, observation, poseFresh)
        if self.state == State.HOLD_READY:
            self.initialYaw = self.yawSetpoint = self.currentYaw
            self.phase = "TAKEOFF"
        return actions

    def _holdReadyActions(self, now, observation, poseFresh):
        if not self.airborneHold:
            if not self.obstacleClear(now):
                if now-self.entered > self.config.hold_ready_timeout_s:
                    return self.latch(now,"obstacle_prearm_unavailable")
                return ["hold_setpoint"]
            return super()._holdReadyActions(now, observation, poseFresh)
        self.hold = (*self.takeoffOrigin[:2], self.groundZ+self.task["height_m"])
        self.transition(State.EXECUTING, now)
        self.beginStage(now)
        return ["hold_setpoint"]


    def _takeoffActions(self, now, observation):
        if not observation.armed and now-self.entered >= self.config.mode_timeout_s:
            return self.latch(now,"arming_not_confirmed")
        duration = 1.875*self.task["height_m"]/self.mission.takeoff_speed
        target = (*self.takeoffOrigin[:2],self.groundZ+self.task["height_m"])
        ready = (observation.armed and observation.landed == 2
            and now-self.flightStarted >= duration
            and 0 <= observation.pose_age <= self.config.pose_timeout_s
            and math.dist(observation.position,target) <= self.task["position_tolerance_m"]
            and math.hypot(*observation.velocity) <= self.task["stable_speed_m_s"])
        self.settleSince = (now if self.settleSince is None else self.settleSince) if ready else None
        if self.settleSince is not None and now-self.settleSince >= self.config.stable_duration_s:
            self.hold = tuple(target)
            self.airborneHold = True
            self.settleSince = None
            self.transition(State.HOLD_READY,now)
        return ["hold_setpoint"]

    def beginStage(self, now):
        self.stageIndex += 1
        self.stageSince, self.settleSince = now, None
        step = self.task["steps"][self.stageIndex]
        self.phase = step["action"].upper()
        self.segmentStart = tuple(self.hold)
        self.segmentGoal = tuple(self.hold)
        self.segmentYawStart = self.yawSetpoint
        self.segmentYawGoal = self.yawSetpoint
        if step["action"] == "move":
            distance = step["forward_m"]
            self.segmentGoal = (self.takeoffOrigin[0]+distance*math.cos(self.initialYaw),
                                self.takeoffOrigin[1]+distance*math.sin(self.initialYaw),
                                self.groundZ+self.task["height_m"])
        elif step["action"] == "turn":
            # Unwrapped +180 degrees specifies a positive rotation even across +/-pi.
            self.segmentYawGoal = self.initialYaw+math.radians(step["yaw_deg"])
        distance = math.dist(self.segmentStart, self.segmentGoal)
        angle = abs(self.segmentYawGoal-self.segmentYawStart)
        self.segmentDuration = max(.02, 1.875*distance/self.task["max_speed_m_s"],
            math.sqrt(5.774*distance/self.task["max_accel_m_s2"]),
            1.875*angle/self.task["max_yaw_rate_rad_s"],
            math.sqrt(5.774*angle/self.task["max_yaw_accel_rad_s2"]))
        self.hold = self.segmentGoal
        if step["action"] == "land":
            self.phase = "TAG_ACQUIRE"
            self.alignSince = None
            self.landingCaptureSamples = 0
            self.landingCaptureStamp = None
            self.landingSetpoint = list(self.segmentGoal)
            self.lastLandingTick = now

    def tick(self, now, observation):
        if self.state == State.LANDING and self.nativeLandStarted is not None:
            return self.nativeFinalTick(now,observation)
        if self.state not in (State.IDLE, State.PRECHECK, State.COMPLETE, State.FAULT_LATCHED):
            p = observation.position
            if all(math.isfinite(v) for v in p) and self.takeoffOrigin is not None:
                if (math.dist(p[:2], self.takeoffOrigin[:2]) > self.task["max_radius_m"]
                        or p[2] > self.groundZ+self.task["height_m"]+self.task["ceiling_margin_m"]):
                    return self.latch(now, "task_flight_boundary")
        actions = super().tick(now, observation)
        if (observation.armed and self.state in (State.TAKEOFF,State.HOLD_READY,State.LANDING)
                and not self.landingInputsReady):
            return self.latch(now,"landing_sensor_link")
        if (observation.armed and self.state in (State.TAKEOFF,State.HOLD_READY,State.LANDING)
                and not self.obstacleClear(now)):
            return self.latch(now,"obstacle_during_vertical_flight")
        return actions

    def _offboardPendingActions(self, now, observation):
        if not self.landingInputsReady:
            return self.latch(now,"landing_inputs_missing_before_arm")
        if not self.obstacleClear(now):
            return self.latch(now,"obstacle_before_arm")
        if (self.currentYaw is None or not 0 <= observation.pose_age <= self.config.pose_timeout_s
                or not observation.alignment or not observation.sensors_ready or not observation.sending):
            if now-self.entered >= self.config.mode_timeout_s:
                return self.latch(now,"inputs_not_ready_before_arm")
            return ["hold_setpoint"]
        return super()._offboardPendingActions(now,observation)

    def holdSetpoint(self, now):
        if self.state == State.EXECUTING and self.avoidance.active:
            p,v,a,yaw = self.avoidance.sample(now)
            self.yawSetpoint = yaw
            return p,v,a
        if self.state == State.EXECUTING and self.stageIndex >= 0:
            if self.phase == "TAG_ACQUIRE":
                return tuple(self.landingSetpoint), (0.,)*3, (0.,)*3
            s, ds, dds = smoothSegment(now-self.stageSince, self.segmentDuration)
            delta = np.asarray(self.segmentGoal)-self.segmentStart
            self.yawSetpoint = self.segmentYawStart+s*(self.segmentYawGoal-self.segmentYawStart)
            return tuple(np.asarray(self.segmentStart)+s*delta), tuple(ds*delta), tuple(dds*delta)
        if self.state == State.LANDING and self.landingSetpoint is not None:
            return tuple(self.landingSetpoint), self.landingVelocity, (0.,)*3
        return super().holdSetpoint(now)

    def _executingActions(self, now, observation):
        if (self.currentYaw is None or not math.isfinite(self.currentYaw)
                or not 0 <= observation.pose_age <= self.config.pose_timeout_s):
            return ["hold_setpoint"]
        a=self.avoidance
        if (not self.landingInputsReady and a.active and a.purpose!="RETURN"
                and a.intent not in ("RETURN","LAND")):
            a.brake(now,observation,self.currentYaw,"RETURN")
        avoidance = self.avoidanceActions(now,observation)
        if avoidance is not None:
            return avoidance
        if not self.landingInputsReady:
            if self.reachedHome:return self.latch(now,"home_landing_inputs_unavailable")
            self.savedPhase=self.phase
            a.begin(now,observation,self.currentYaw,goal=self.segmentGoal,goalYaw=self.segmentYawGoal,
                allowDetour=False,deadline=self.flightStarted+self.mission.flight_budget_s-self.mission.landing_reserve_s)
            self.phase="LANDING_INPUT_BRAKE"
            return ["cancel","disable_planner","hold_setpoint"]
        if self.phase == "TAG_ACQUIRE":
            return self.landingActions(now, observation, acquiring=True)
        step = self.task["steps"][self.stageIndex]
        self.holdSetpoint(now)
        arrived = (now-self.stageSince >= self.segmentDuration
                   and math.dist(observation.position, self.segmentGoal) <= self.task["position_tolerance_m"]
                   and math.hypot(*observation.velocity) <= self.task["stable_speed_m_s"]
                   and abs(wrap(self.currentYaw-self.segmentYawGoal)) <= self.task["yaw_tolerance_rad"])
        requiresTag = step["action"] == "hold" and step["require_tag"]
        validTag = self.freshFix(now) and self.fix.source in ("tag","resolved_tag") and math.dist(self.fix.target[:2],self.takeoffOrigin[:2]) <= self.task["max_tag_correction_m"]
        if requiresTag and validTag and self.fix.stamp != self.lastTagStamp:
            self.tagSamples = (self.tagSamples+1 if self.lastTagStamp is not None
                and self.fix.stamp-self.lastTagStamp <= self.task["tag_timeout_s"] else 1)
            self.lastTagStamp = self.fix.stamp
        self.tagCaptured = requiresTag and validTag and self.tagSamples >= 4
        if arrived:
            if self.settleSince is None:
                self.settleSince = now
            dwell = max(self.task["settle_s"], step.get("duration_s", 0.))
            if now-self.settleSince >= dwell and (not requiresTag or self.tagCaptured):
                self.yawSetpoint = self.segmentYawGoal
                self.beginStage(now)
        else:
            self.settleSince = None
        if now-self.stageSince > self.segmentDuration+self.task["stage_timeout_s"]:
            return self.latch(now, "task_stage_timeout")
        return ["hold_setpoint"]


    def handoverEvidence(self):
        """平台交接前必须成立的 Tag 证据；供任务闸门读取，不接受单帧检测。"""
        return dict(self.capture)

    def handoverYawOffset(self):
        """相对板 +x 的末段航向偏置：任务 2 保持起飞航向 +180°。"""
        if self.initialYaw is None or self.segmentYawGoal is None:
            return 0.
        return wrap(self.segmentYawGoal-self.initialYaw)

    def obstacleClear(self, now):
        r = self.obstacle
        return r.state == "CLEAR" and 0 <= now-r.stamp <= self.obstacleConfig.depth_timeout_s

    def obstacleReport(self, now):
        from .task_depth_guard import ObstacleReport
        if not 0 <= now-self.obstacle.stamp <= self.obstacleConfig.depth_timeout_s:
            return ObstacleReport(self.obstacle.stamp,"UNKNOWN","depth_stale")
        return self.obstacle

    def avoidanceActions(self, now, observation):
        from .task_depth_guard import ObstacleReport
        a = self.avoidance
        report = self.obstacleReport(now)
        if a.home is None:
            a.home = np.array([*self.takeoffOrigin[:2],self.groundZ+self.task["height_m"]])
            a.breadcrumbs.append(a.home.copy())
        if not a.active:
            if not a.record(observation.position):
                report = ObstacleReport(now,"UNKNOWN","breadcrumb_capacity")
            if report.state == "CLEAR":
                return None
            if self.obstacleMissionFailed and self.phase == "TAG_ACQUIRE":
                return self.latch(now,"return_landing_depth_unavailable")
            self.savedPhase = self.phase
            allowDetour = self.landingInputsReady and self.phase == "MOVE" and report.state == "BLOCKED" and not report.emergency
            a.begin(now,observation,self.currentYaw,goal=self.segmentGoal,goalYaw=self.segmentYawGoal,
                    allowDetour=allowDetour,
                    deadline=self.flightStarted+self.mission.flight_budget_s-self.mission.landing_reserve_s)
            self.phase = "OBSTACLE_BRAKE"
            return ["cancel","disable_planner","hold_setpoint"]
        if (a.purpose == "DETOUR" and a.state != "BRAKE"
                and a.routeCost(observation.position,self.currentYaw,a.returnPoints(observation.position))
                    +self.obstacleConfig.brake_timeout_s >= a.deadline-now):
            a.brake(now,observation,self.currentYaw,"RETURN")
        result = a.tick(now,observation,self.currentYaw,report)
        self.phase = (a.purpose or a.intent)+"_"+a.state
        if result == "LAND":
            self.obstacleMissionFailed = True
            return self.latch(now,a.reason or "avoidance_failed")
        if result == "HOME":
            self.obstacleMissionFailed = self.reachedHome = True
            self.hold = tuple(a.home)
            self.yawSetpoint = self.currentYaw
            if not any(step.get("require_tag",False) for step in self.task["steps"]):
                self.phase = "RETURN_NATIVE_LAND"
                return self.latch(now,"returned_home_native_land")
            self.phase = "TAG_ACQUIRE"
            self.stageSince = self.lastLandingTick = now
            self.alignSince = None
            self.segmentGoal = tuple(a.home)
            self.landingSetpoint = list(a.home)
            return ["hold_setpoint"]
        if result == "RESUME":
            self.phase = self.savedPhase
            self.stageSince,self.settleSince = now,None
            self.segmentStart = tuple(observation.position)
            self.segmentYawStart = self.yawSetpoint = self.currentYaw
            self.segmentDuration = a.segmentDuration(math.dist(self.segmentStart,self.segmentGoal),
                                                       self.segmentYawGoal-self.segmentYawStart)
        return ["hold_setpoint"]

    def land(self, now):
        # An operator LAND or exhausted budget uses native Land; it must not invent a Tag fix.
        return self.latch(now, "task_land_requested")


    def nativeFinalTick(self, now, observation):
        if not math.isfinite(now) or now < self.nativeLandStarted:
            return self.latch(now,"clock_reset")
        if not observation.connected or observation.session != self.session or not 0 <= observation.status_age <= self.config.mode_timeout_s:
            return self.latch(now,"final_land_status_missing")
        if self.boot_epoch is not None and observation.boot_epoch != self.boot_epoch:
            return self.latch(now,"flight_controller_restart")
        if (now-self.nativeLandStarted >= self.task["final_land_timeout_s"]
                or now-self.flightStarted >= self.mission.flight_budget_s):
            return self.latch(now,"final_land_timeout")
        if self.nativeLandConfirmed and observation.landed == 1 and not observation.armed:
            self.transition(State.COMPLETE,now,"landed_disarmed")
            return ["cancel"]
        if observation.mode_detail == "auto:land":
            self.nativeLandConfirmed = True
        elif observation.mode_detail != "offboard" or self.nativeLandConfirmed:
            return self.latch(now,"manual_mode")
        if not self.nativeLandConfirmed and now-self.nativeLandStarted >= self.config.mode_timeout_s:
            return self.latch(now,"final_land_mode_not_confirmed")
        if observation.landed == 1 and self.nativeLandConfirmed:
            if self.landSince is None:
                self.landSince = now
            if not observation.armed:
                self.transition(State.COMPLETE,now,"landed_disarmed")
                return ["cancel"]
            if now-self.landSince >= self.mission.landed_confirm_s and not self.disarmRequested:
                self.disarmRequested = True
                return ["disarm"]
        else:
            self.landSince = None
        return []

    def _landingActions(self, now, observation):
        if self.currentYaw is None or not 0 <= observation.pose_age <= self.config.pose_timeout_s:
            self.landingVelocity = (0.,)*3
            self.alignSince = None
            if self.landingSetpoint is not None and math.isfinite(observation.position[2]):
                self.landingSetpoint[2] = observation.position[2]
            return ["hold_setpoint"]
        return self.landingActions(now, observation, acquiring=False)

    def landingActions(self, now, observation, *, acquiring):
        self.landingVelocity = (0.,)*3
        if observation.fault:
            return self.latch(now, observation.fault)
        if not acquiring and observation.landed == 1:
            return super()._landingActions(now, observation)
        dt = min(.05, max(0., now-self.lastLandingTick))
        self.lastLandingTick = now
        valid = self.freshFix(now) and self.fix.yaw_valid
        if valid and math.dist(self.fix.target[:2], self.segmentGoal[:2]) > self.task["max_tag_correction_m"]:
            return self.latch(now, "task_tag_outside_landing_area")
        if not valid:
            self.nearGroundSince = None
            previous = self.lastConfirmedFix
            maintained = (previous is not None and 0 <= now-previous.stamp <= self.task["tag_timeout_s"]
                and math.dist(observation.position[:2],previous.target[:2]) <= self.task["tag_align_m"]
                and math.hypot(*observation.velocity[:2]) <= self.task["stable_speed_m_s"]
                and abs(wrap(self.currentYaw-self.yawSetpoint)) <= self.task["yaw_tolerance_rad"])
            if not maintained:
                self.alignSince = None
            if not acquiring:
                # Freeze the current height immediately; fresh pose remains required by the parent FSM.
                self.landingSetpoint[2] = observation.position[2]
                if self.landingSince is None:
                    self.landingSince = now
                if now-self.landingSince >= self.task["tag_loss_timeout_s"]:
                    return self.latch(now, "task_tag_lost")
            elif now-self.stageSince >= self.task["tag_acquire_timeout_s"]:
                return self.latch(now, "task_tag_acquire_timeout")
            return ["hold_setpoint"]
        self.landingSince = None
        self.lastConfirmedFix = self.fix
        delta = np.asarray(self.fix.target[:2])-self.landingSetpoint[:2]
        length = float(np.linalg.norm(delta))
        limit = self.task["max_landing_xy_m_s"]*dt
        self.landingSetpoint[:2] = np.asarray(self.landingSetpoint[:2])+delta*min(1., limit/max(length, 1e-9))
        aligned = (math.dist(observation.position[:2], self.fix.target[:2]) <= self.task["tag_align_m"]
                   and math.hypot(*observation.velocity[:2]) <= self.task["stable_speed_m_s"]
                   and abs(wrap(self.currentYaw-self.yawSetpoint)) <= self.task["yaw_tolerance_rad"])
        if not aligned:
            self.alignSince = None
            if not acquiring:
                self.landingSetpoint[2] = observation.position[2]
        elif self.alignSince is None:
            self.alignSince = now
        if not acquiring:
            heightAgrees = abs(self.fix.height-(observation.position[2]-self.groundZ
                +self.obstacleConfig.ground_clearance_m)) <= self.task["max_tag_ground_error_m"]
            if aligned and heightAgrees and self.fix.height <= self.task["final_land_height_m"]:
                if self.finalCaptureZ is None:
                    self.finalCaptureZ = observation.position[2]
            if self.finalCaptureZ is not None:
                self.landingSetpoint[2] = self.finalCaptureZ
            near = (aligned and heightAgrees and self.finalCaptureZ is not None
                and self.fix.height <= self.task["final_land_height_m"]+.02
                and math.hypot(*observation.velocity) <= self.task["stable_speed_m_s"])
            self.nearGroundSince = (now if self.nearGroundSince is None else self.nearGroundSince) if near else None
            if self.nearGroundSince is not None and now-self.nearGroundSince >= self.task["final_land_confirm_s"]:
                self.nativeLandStarted = now
                self.nativeLandConfirmed = False
                self.landSince = None
                self.phase = "PX4_FINAL_LAND"
                return ["cancel","disable_planner","land"]
        if acquiring:
            if self.fix.source in ("tag","resolved_tag") and self.fix.stamp!=getattr(self,"landingCaptureStamp",None):
                previous=getattr(self,"landingCaptureStamp",None)
                self.landingCaptureSamples=(getattr(self,"landingCaptureSamples",0)+1
                    if previous is not None and self.fix.stamp-previous<=self.task["tag_timeout_s"] else 1)
                self.landingCaptureStamp=self.fix.stamp
            # 交接模式只接受完整身份解码；单帧、低质量或仅特征定位都不足以
            # 进入平台速度接管，VIO 必须一直保留到确认成功。
            sourceAllowed = handoverSourceAllowed(self.fix.source, self.handover)
            if (sourceAllowed and aligned and self.fix.yaw_valid
                    and now-self.alignSince >= self.task["tag_align_s"]
                    and getattr(self,"landingCaptureSamples",0)>=4):
                self.capture = dict(ready=True, at=now, tag_stamp=self.fix.stamp,
                    age_s=max(0., now-self.fix.stamp), source=self.fix.source,
                    ids=list(self.fix.tag_ids), samples=int(self.landingCaptureSamples),
                    identity_stamp=self.fix.identity_stamp, yaw_valid=bool(self.fix.yaw_valid),
                    height_m=self.fix.height)
                actions = super().land(now)
                if self.handover:
                    # 交接模式不要在 LANDING 过渡拍发下降 hold：下一拍由任务闸门
                    # 以固定交接高度接管，避免确认前先掉高度。
                    actions = [action for action in actions if action != "hold_setpoint"]
                self.goal_reached = True
                self.phase = "TAG_DESCEND"
                self.lastLandingTick = now
                return actions
            if now-self.stageSince >= self.task["tag_acquire_timeout_s"]:
                return self.latch(now, "task_tag_alignment_timeout")
        elif self.finalCaptureZ is None and aligned and now-self.alignSince >= self.task["tag_align_s"]:
            speed = (self.task["flare_speed_m_s"] if self.fix.height <= self.task["flare_height_m"]
                     else self.task["landing_speed_m_s"])
            self.landingSetpoint[2] = max(self.groundZ-.2, self.landingSetpoint[2]-speed*dt)
            self.landingVelocity = (0.,0.,-speed) if self.landingSetpoint[2] > self.groundZ-.2 else (0.,)*3
        if not acquiring and now-self.entered > self.mission.landing_reserve_s:
            return self.latch(now, "landing_timeout")
        return ["hold_setpoint"]
