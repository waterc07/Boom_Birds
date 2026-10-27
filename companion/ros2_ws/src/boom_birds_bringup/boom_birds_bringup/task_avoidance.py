"""One bounded detour, then retrace observed positions; no map or planner."""
import math
import numpy as np
from boom_birds_control.attitude_control import wrap


class TaskAvoidance:
    def __init__(self, task, config):
        self.task,self.config = task,config
        self.state = "IDLE"
        self.purpose = self.intent = ""
        self.reason = ""
        self.used = False
        self.aborted = False
        self.breadcrumbs = []
        self.points = []
        self.index = 0
        self.settledSince = self.clearSince = None
        self.home = None
        self.started = 0.
        self.deadline = math.inf
        self.p0 = self.p1 = None
        self.yaw0 = self.yaw1 = 0.
        self.duration = .02
        self.brakeVelocity = np.zeros(3)
        self.goal = None
        self.resumeYaw = None
        self.finishTurn = False

    @property
    def active(self):
        return self.state != "IDLE"

    def record(self, position):
        if self.purpose == "RETURN":
            return True
        p = np.asarray(position,float)
        if p.shape != (3,) or not np.isfinite(p).all():
            return False
        if self.breadcrumbs and np.linalg.norm(p-self.breadcrumbs[-1]) < self.config.breadcrumb_spacing_m:
            return True
        if len(self.breadcrumbs) >= 2:
            a,b = self.breadcrumbs[-2:]
            d = p-a
            norm = float(d@d)
            projection = 0. if norm == 0 else float((b-a)@d/norm)
            if 0 <= projection <= 1 and np.linalg.norm(b-(a+projection*d)) <= self.config.breadcrumb_tolerance_m:
                self.breadcrumbs[-1] = p.copy()
                return True
        if len(self.breadcrumbs) >= self.config.breadcrumb_capacity:
            self.reason = "breadcrumb_capacity"
            return False
        self.breadcrumbs.append(p.copy())
        return True

    def begin(self, now, observation, yaw, *, goal, goalYaw, allowDetour, deadline):
        self.goal = np.asarray(goal,float)
        self.resumeYaw = goalYaw
        self.deadline = deadline
        self.intent = "DETOUR" if allowDetour and not self.used else "RETURN"
        self.reason = "obstacle_brake"
        self.brake(now,observation,yaw,self.intent)

    def brake(self, now, observation, yaw, intent):
        self.state,self.intent,self.started = "BRAKE",intent,now
        self.p0 = np.asarray(observation.position,float)
        self.brakeVelocity = np.asarray(observation.velocity,float)
        self.duration = max(.02,float(np.linalg.norm(self.brakeVelocity))/self.config.braking_accel_m_s2)
        self.p1 = self.p0+self.brakeVelocity*self.duration/2
        self.yaw0 = self.yaw1 = yaw
        self.settledSince = None

    def sample(self, now):
        from .tag_flight_task import smoothSegment
        if self.state == "BRAKE":
            t = min(self.duration,max(0.,now-self.started))
            p = self.p0+self.brakeVelocity*(t-t*t/(2*self.duration))
            v = self.brakeVelocity*(1-t/self.duration)
            a = -self.brakeVelocity/self.duration if t < self.duration else np.zeros(3)
            return tuple(p),tuple(v),tuple(a),self.yaw0
        s,ds,dds = smoothSegment(now-self.started,self.duration)
        delta = self.p1-self.p0
        return tuple(self.p0+s*delta),tuple(ds*delta),tuple(dds*delta),self.yaw0+s*(self.yaw1-self.yaw0)

    def segmentDuration(self, distance, angle=0.):
        return max(.02,1.875*distance/self.task["max_speed_m_s"],
                   math.sqrt(5.774*distance/self.task["max_accel_m_s2"]),
                   1.875*abs(angle)/self.task["max_yaw_rate_rad_s"],
                   math.sqrt(5.774*abs(angle)/self.task["max_yaw_accel_rad_s2"]))

    def routeCost(self, start, yaw, points, finalYaw=None):
        total = 0.
        p = np.asarray(start)
        for goal in points:
            d = goal-p
            distance = float(np.linalg.norm(d))
            if distance <= self.task["position_tolerance_m"]:
                continue
            heading = math.atan2(d[1],d[0])
            total += self.segmentDuration(0.,wrap(heading-yaw))+self.config.settle_s+self.config.observation_s
            total += self.segmentDuration(distance)+self.config.settle_s
            p,yaw = goal,heading
        if finalYaw is not None:
            total += self.segmentDuration(0.,wrap(finalYaw-yaw))+self.config.settle_s
        return total

    def returnPoints(self, position):
        points = [p.copy() for p in reversed(self.breadcrumbs)]
        while points and np.linalg.norm(points[0]-position) <= self.task["position_tolerance_m"]:
            points.pop(0)
        return points

    def beginReturn(self, now, observation, yaw):
        self.aborted = True
        self.record(observation.position)
        points = self.returnPoints(observation.position)
        if not points and (self.home is None or math.dist(observation.position,self.home) > self.task["position_tolerance_m"]):
            self.reason = "return_trace_missing"
            return "LAND"
        if self.routeCost(observation.position,yaw,points) >= self.deadline-now:
            self.reason = "return_budget_insufficient"
            return "LAND"
        self.purpose,self.points,self.index = "RETURN",points,0
        self.reason = "retrace_home"
        return self.nextLeg(now,observation,yaw)

    def detour(self, now, observation, yaw, report):
        self.used = True
        p = np.asarray(observation.position,float)
        delta = self.goal[:2]-p[:2]
        distance = float(np.linalg.norm(delta))
        if distance <= self.task["position_tolerance_m"]:
            return self.beginReturn(now,observation,yaw)
        d = np.r_[delta/distance,0.]
        side = np.array([-d[1],d[0],0.])*self.config.detour_side_m
        advance = max(distance,(report.nearest_m or self.config.stop_distance_m)+self.config.pass_margin_m)
        first = min(self.config.detour_forward_m,advance/2)
        points = [p+d*first+side,p+d*advance+side,self.goal.copy()]
        inside = all(math.dist(point[:2],self.home[:2])+self.config.half_width_m < self.task["max_radius_m"] for point in points)
        if not inside or self.routeCost(p,yaw,points,self.resumeYaw) >= self.deadline-now:
            self.reason = "detour_boundary_or_budget"
            return self.beginReturn(now,observation,yaw)
        self.purpose,self.points,self.index = "DETOUR",points,0
        self.reason = "detour_once"
        return self.nextLeg(now,observation,yaw)

    def turn(self, now, observation, yaw, targetYaw):
        self.state,self.started = "TURN",now
        self.p0 = self.p1 = np.asarray(observation.position,float)
        self.yaw0,self.yaw1 = yaw,yaw+wrap(targetYaw-yaw)
        self.duration = self.segmentDuration(0.,self.yaw1-yaw)
        self.settledSince = None

    def nextLeg(self, now, observation, yaw):
        while self.index < len(self.points) and math.dist(observation.position,self.points[self.index]) <= self.task["position_tolerance_m"]:
            self.index += 1
        if self.index >= len(self.points):
            if self.purpose == "DETOUR" and abs(wrap(yaw-self.resumeYaw)) > self.task["yaw_tolerance_rad"]:
                self.finishTurn = True
                self.turn(now,observation,yaw,self.resumeYaw)
                return "RUNNING"
            purpose = self.purpose
            self.state = "IDLE"
            return "HOME" if purpose == "RETURN" else "RESUME"
        point = self.points[self.index]
        d = point-np.asarray(observation.position)
        self.finishTurn = False
        self.turn(now,observation,yaw,math.atan2(d[1],d[0]))
        return "RUNNING"

    def tick(self, now, observation, yaw, report):
        if now >= self.deadline:
            self.reason = "avoidance_navigation_budget"
            return "LAND"
        if self.purpose != "RETURN" and not self.record(observation.position):
            self.brake(now,observation,yaw,"RETURN")
        if self.state == "BRAKE":
            if now-self.started > self.config.brake_timeout_s:
                self.reason = "brake_not_confirmed"
                return "LAND"
            stable = (now-self.started >= self.duration and math.hypot(*observation.velocity) <= self.task["stable_speed_m_s"]
                      and math.dist(observation.position,self.p1) <= self.task["position_tolerance_m"])
            if not stable:
                self.settledSince = None
                return "RUNNING"
            if self.settledSince is None:
                self.settledSince = now
            if now-self.settledSince < self.config.settle_s:
                return "RUNNING"
            if self.intent == "LAND" or report.emergency:
                self.reason = "return_obstructed_or_emergency"
                return "LAND"
            if self.intent == "DETOUR" and report.state == "BLOCKED":
                return self.detour(now,observation,yaw,report)
            return self.beginReturn(now,observation,yaw)
        if report.emergency:
            self.brake(now,observation,yaw,"LAND")
            return "RUNNING"
        if self.state == "TURN":
            if now-self.started > self.duration+self.config.stage_timeout_s:
                self.reason = "avoidance_turn_timeout"
                return self.beginReturn(now,observation,yaw) if self.purpose == "DETOUR" else "LAND"
            arrived = (now-self.started >= self.duration
                       and abs(wrap(yaw-self.yaw1)) <= self.task["yaw_tolerance_rad"]
                       and math.dist(observation.position,self.p1) <= self.task["position_tolerance_m"]
                       and math.hypot(*observation.velocity) <= self.task["stable_speed_m_s"])
            self.settledSince = (now if self.settledSince is None else self.settledSince) if arrived else None
            if self.settledSince is not None and now-self.settledSince >= self.config.settle_s:
                if self.finishTurn:
                    self.state = "IDLE"
                    return "RESUME"
                self.state,self.started = "OBSERVE",now
                self.p0 = self.p1 = np.asarray(observation.position,float)
                self.yaw0 = self.yaw1 = yaw
                self.duration = .02
                self.clearSince = None
            return "RUNNING"
        if self.state == "OBSERVE":
            clear = report.state == "CLEAR" and report.stamp >= self.started
            self.clearSince = (now if self.clearSince is None else self.clearSince) if clear else None
            if self.clearSince is not None and now-self.clearSince >= self.config.observation_s:
                self.state,self.started = "MOVE",now
                self.p0,self.p1 = np.asarray(observation.position,float),self.points[self.index]
                self.duration = self.segmentDuration(float(np.linalg.norm(self.p1-self.p0)))
                self.settledSince = None
            elif now-self.started >= self.config.observe_timeout_s:
                self.reason = "detour_or_return_corridor_unavailable"
                if self.purpose == "DETOUR":
                    return self.beginReturn(now,observation,yaw)
                return "LAND"
            return "RUNNING"
        if self.state == "MOVE":
            if report.state != "CLEAR":
                self.brake(now,observation,yaw,"RETURN" if self.purpose == "DETOUR" else "LAND")
                return "RUNNING"
            if now-self.started > self.duration+self.config.stage_timeout_s:
                self.brake(now,observation,yaw,"RETURN" if self.purpose == "DETOUR" else "LAND")
                return "RUNNING"
            arrived = (now-self.started >= self.duration
                       and math.dist(observation.position,self.p1) <= self.task["position_tolerance_m"]/2
                       and math.hypot(*observation.velocity) <= self.task["stable_speed_m_s"])
            self.settledSince = (now if self.settledSince is None else self.settledSince) if arrived else None
            if self.settledSince is not None and now-self.settledSince >= self.config.settle_s:
                self.index += 1
                return self.nextLeg(now,observation,yaw)
        return "RUNNING"
