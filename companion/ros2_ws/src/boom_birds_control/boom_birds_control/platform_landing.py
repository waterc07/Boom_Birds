"""Platform-relative guidance and exclusive output handoff; no I/O or arming."""
from dataclasses import dataclass
import math
import uuid
import numpy as np
from .platform_model import validate_profile, transform
from .px4_frames import Px4LocalSetpoint

# Position/acceleration/yaw ignored, velocity and yaw-rate active.
VELOCITY_YAW_RATE_MASK = 1 | 2 | 4 | 64 | 128 | 256 | 1024


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def ned_from_flu(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    r = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                  [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                  [-sp, cp*sr, cp*cr]])
    # 飞控姿态描述 FRD→NED；先把观测使用的 FLU 转成 FRD。
    return r @ np.diag([1., -1., -1.])


@dataclass(frozen=True)
class LandingConfig:
    kp_xy: float
    kp_yaw: float
    max_xy_m_s: float
    max_up_m_s: float
    max_down_m_s: float
    max_yaw_rate_rad_s: float
    max_accel_m_s2: float
    max_yaw_accel_rad_s2: float
    observation_timeout_s: float
    telemetry_timeout_s: float
    range_timeout_s: float
    future_tolerance_s: float
    max_pose_jump_m: float
    max_rotation_jump_rad: float
    max_range_jump_m: float
    range_min_m: float
    range_max_m: float
    max_tilt_rad: float
    range_pose_tolerance_m: float
    acquire_samples: int
    align_samples: int
    align_xy_m: float
    align_yaw_rad: float
    flare_height_m: float
    flare_down_m_s: float
    touchdown_height_m: float
    touchdown_speed_m_s: float
    touchdown_window_s: float
    touchdown_confirmation_timeout_s: float
    acquire_timeout_s: float
    loss_land_timeout_s: float
    handoff_timeout_s: float
    handoff_samples: int
    handoff_min_stream_s: float
    confirmation_timeout_s: float
    takeoff_height_m: float
    takeoff_tolerance_m: float

    def __post_init__(self):
        for name, value in self.__dict__.items():
            if name.endswith("samples"):
                if type(value) is not int or value < 1:
                    raise ValueError(name)
            elif type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(name)
        if not self.range_min_m < self.touchdown_height_m < self.flare_height_m < self.range_max_m:
            raise ValueError("height_order")
        if self.flare_down_m_s > self.max_down_m_s or self.takeoff_height_m >= self.range_max_m:
            raise ValueError("speed_or_takeoff_bounds")
        if self.max_tilt_rad >= math.pi/2 or self.future_tolerance_s >= self.observation_timeout_s:
            raise ValueError("tilt_or_clock_bounds")


@dataclass(frozen=True)
class RangeSample:
    stamp: float
    distance_m: float
    valid: bool = True


@dataclass(frozen=True)
class FlightSample:
    stamp: float
    roll: float
    pitch: float
    yaw: float
    velocity_ned: tuple
    connected: bool
    offboard: bool
    velocity_estimator_valid: bool
    landed: bool | None = None
    armed: bool = True
    epoch: int = 0


@dataclass(frozen=True)
class HandoffFeedback:
    stamp: float
    token: str
    sequence: int
    epoch: int
    velocity_active: bool
    attitude_active: bool
    echoed_velocity_ned: tuple
    source: str  # PX4 OCM + setpoint echo; local send success is insufficient.


@dataclass(frozen=True)
class LandingOutput:
    state: str
    reason: str
    velocity_ned: tuple | None
    yaw_rate: float
    navigation_allowed: bool
    vio_required: bool
    release_compute: bool
    request_native_land: bool
    request_disarm: bool
    descent_permitted: bool
    token: str
    sequence: int

    def setpoint(self):
        if self.velocity_ned is None:
            raise ValueError("no_velocity_output")
        return Px4LocalSetpoint((0., 0., 0.), self.velocity_ned, (0., 0., 0.), 0., self.yaw_rate)


class PlatformLanding:
    def __init__(self, profile, *, test_only=False):
        validate_profile(profile, test_only=test_only)
        self.profile = profile
        self.cfg = LandingConfig(**profile["guidance"])
        self.range_mount = transform(profile["range"]["T_body_sensor"], "range_extrinsics")
        if not np.allclose(self.range_mount[:3, 2], [0, 0, -1], atol=1e-6):
            raise ValueError("range_beam_must_point_body_down")
        self.state, self.reason = "NAVIGATION", "navigation"
        self.intent = "land"
        self.token = ""
        self.sequence = 0
        self.confirmed = False
        self.released = False
        self.pending_since = None
        self.request_since = None
        self.touchdown_pending_since = None
        self.last_now = None
        self.last_pose = None
        self.last_range = None
        self.last_observation_stamp = None
        self.last_range_stamp = None
        self.good = self.aligned = self.acks = 0
        self.last_ack_sequence = 0
        self.last_ack_stamp = None
        self.epoch = None
        self.loss_since = self.landed_since = None
        self.last_velocity = np.zeros(3)
        self.last_yaw_rate = 0.
        self.last_sent_sequence = 0
        self.last_sent_at = None
        self.sent_history = {}

    def fresh(self, stamp, now, timeout):
        return math.isfinite(stamp) and -self.cfg.future_tolerance_s <= now-stamp <= timeout

    def request(self, intent="land"):
        if intent not in ("land", "takeoff"):
            raise ValueError("intent")
        if self.state != "NAVIGATION":
            raise ValueError("handoff_already_started")
        self.intent = intent
        self.state = "ACQUIRE"
        self.reason = "acquiring"

    def cancel(self, reason="cancelled"):
        # No automatic return to VIO after compute release.
        self.state, self.reason = "NATIVE_LAND", reason
        self.last_velocity[:] = 0.
        self.last_yaw_rate = 0.

    def note_sent(self, output, now, accepted):
        if output.sequence != self.sequence or output.token != self.token:
            raise ValueError("send_receipt_mismatch")
        if not accepted:
            self.cancel("velocity_send_failed")
            return
        self.last_sent_sequence, self.last_sent_at = output.sequence, now
        self.sent_history[output.sequence] = (now, output.velocity_ned)
        if len(self.sent_history) > 256:
            del self.sent_history[min(self.sent_history)]

    def _inputs(self, now, observation, range_sample, flight):
        config = self.cfg
        if flight is None or not self.fresh(flight.stamp, now, config.telemetry_timeout_s):
            return None, "telemetry_stale"
        numbers = (flight.roll, flight.pitch, flight.yaw, *flight.velocity_ned)
        if len(flight.velocity_ned) != 3 or not all(math.isfinite(x) for x in numbers):
            return None, "telemetry_invalid"
        if not flight.connected or not flight.velocity_estimator_valid:
            return None, "px4_velocity_estimator_unavailable"
        if abs(flight.roll) > config.max_tilt_rad or abs(flight.pitch) > config.max_tilt_rad:
            return None, "tilt_limit"
        if self.epoch is not None and self.epoch != flight.epoch:
            self.cancel("fcu_epoch_changed")
            return None, self.reason
        if (observation is None or not observation.valid
                or observation.board != self.profile["board"]["name"]):
            return None, "board_not_visible"
        known_ids = {tag["id"] for tag in self.profile["board"]["tags"]}
        quality = self.profile["quality"]
        if (len(observation.tag_ids) < quality["min_tags"]
                or len(set(observation.tag_ids)) != len(observation.tag_ids)
                or any(tag not in known_ids for tag in observation.tag_ids)):
            return None, "observation_id"
        if (not math.isfinite(observation.reprojection_px)
                or not 0 <= observation.reprojection_px <= quality["max_reprojection_px"]
                or not math.isfinite(observation.min_edge_px) or observation.min_edge_px < quality["min_edge_px"]
                or not math.isfinite(observation.ambiguity_ratio)
                or (observation.quality.get("planar_distinct", True)
                    and observation.ambiguity_ratio < quality["min_ambiguity_ratio"])):
            return None, "observation_quality"
        if (not self.fresh(observation.stamp, now, config.observation_timeout_s)
                or self.last_observation_stamp is not None and observation.stamp < self.last_observation_stamp):
            return None, "observation_stale_or_reordered"
        try:
            pose = observation.pose()
        except ValueError:
            return None, "observation_invalid"
        # 控制周期可复用同一观测；只有采样时间变化才参与跳变检查和连续样本计数。
        new_pose = observation.stamp != self.last_observation_stamp
        if new_pose and self.last_pose is not None:
            previous_pose = self.last_pose
            rotation_jump = math.acos(float(np.clip(
                (np.trace(previous_pose[:3, :3].T @ pose[:3, :3]) - 1) / 2, -1, 1)))
            if (np.linalg.norm(pose[:3, 3] - previous_pose[:3, 3]) > config.max_pose_jump_m
                    or rotation_jump > config.max_rotation_jump_rad):
                return None, "pose_jump"
        if range_sample is None or not range_sample.valid:
            return None, "range_missing"
        if (not self.fresh(range_sample.stamp, now, config.range_timeout_s)
                or self.last_range_stamp is not None and range_sample.stamp < self.last_range_stamp):
            return None, "range_stale_or_reordered"
        distance_m = range_sample.distance_m
        if not math.isfinite(distance_m) or not config.range_min_m <= distance_m <= config.range_max_m:
            return None, "range_blind_or_invalid"
        new_range = range_sample.stamp != self.last_range_stamp
        if (new_range and self.last_range is not None
                and abs(distance_m - self.last_range) > config.max_range_jump_m):
            return None, "range_jump"
        body_to_ned = ned_from_flu(flight.roll, flight.pitch, flight.yaw)
        # distance_m 是传感器斜距；安装平移与倾角换算后取 NED +z 的垂直高度。
        height = float((body_to_ned @ (
            self.range_mount[:3, 3] + self.range_mount[:3, 2] * distance_m))[2])
        visual_height = float((body_to_ned @ pose[:3,3])[2])
        if height <= 0 or abs(height-visual_height) > config.range_pose_tolerance_m:
            return None, "range_pose_disagreement"
        normal = body_to_ned @ pose[:3,2]
        if normal[2] > -math.cos(config.max_tilt_rad):
            return None, "platform_tilt"
        landing_point = np.array([*self.profile["board"]["landing_point_m"], 1.])
        xy_error = (body_to_ned @ (pose @ landing_point)[:3])[:2]
        target_yaw = self.profile["board"]["target_yaw_rad"]
        heading = body_to_ned @ pose[:3,:3] @ np.array([math.cos(target_yaw), math.sin(target_yaw), 0])
        yaw_error = wrap(math.atan2(heading[1], heading[0])-flight.yaw)
        if new_pose:
            self.last_pose, self.last_observation_stamp = pose.copy(), observation.stamp
        if new_range:
            self.last_range, self.last_range_stamp = distance_m, range_sample.stamp
        # 板和测距都更新才累加一次合格样本；重复回读不能扩大捕获/对准证据。
        return (xy_error, yaw_error, height, new_pose and new_range), "ok"

    def _feedback(self, feedback, now, flight):
        if feedback is None:
            return False
        # 回读必须关联已发送的目标；OFFBOARD 或本地发送成功不能替代确认。
        sent = self.sent_history.get(feedback.sequence)
        if (sent is None or not self.fresh(feedback.stamp, now, self.cfg.confirmation_timeout_s)
                or feedback.token != self.token or feedback.epoch != self.epoch
                or feedback.sequence <= self.last_ack_sequence
                or feedback.stamp < sent[0] or not feedback.velocity_active or feedback.attitude_active
                or feedback.source not in ("PX4_OCM_AND_SETPOINT_ECHO", "TEST_ONLY_FAKE_PX4")
                or not flight.offboard or not flight.velocity_estimator_valid
                or len(feedback.echoed_velocity_ned) != 3
                or not np.isfinite(feedback.echoed_velocity_ned).all()
                or not np.allclose(feedback.echoed_velocity_ned, sent[1], atol=.01)):
            return False
        self.last_ack_sequence, self.last_ack_stamp = feedback.sequence, feedback.stamp
        return True

    def step(self, now, observation=None, range_sample=None, flight=None, feedback=None,
             *, navigation_revoked=False, navigation_ready=False, release_ack=False):
        # now 与输入 stamp 均为同一主机单调时钟秒；桥接层保留 ROS 消息年龄。
        # 本层返回动作意图；发送、资源退出和飞控回读由执行器确认。
        if not math.isfinite(now):
            raise ValueError("now")
        config = self.cfg
        dt = .02 if self.last_now is None else max(0., min(now - self.last_now, .1))
        if self.last_now is not None and now < self.last_now:
            self.cancel("clock_reset")
        self.last_now = now

        if self.state in ("NAVIGATION", "COMPLETE"):
            return self._output(None)
        if self.state == "NATIVE_LAND":
            return self._native_land_output(now, flight)
        if self.request_since is None:
            self.request_since = now
        if self.state == "ACQUIRE" and now - self.request_since >= config.acquire_timeout_s:
            self.cancel("acquire_timeout")
            return self._output(None)
        if not self.confirmed and not navigation_ready:
            self.good = self.aligned = self.acks = 0
            self.reason = "navigation_dependency_unavailable"
            if self.state == "PREPARE":
                self.cancel(self.reason)
            return self._output(None)

        data, reason = self._inputs(now, observation, range_sample, flight)
        if self.state == "NATIVE_LAND":
            return self._output(None)
        if data is None:
            return self._input_loss_output(now, reason)
        self.loss_since = None
        xy_error, yaw_error, height, new_sample = data

        if self.state == "ACQUIRE":
            if not self._acquire(now, new_sample, navigation_revoked, flight):
                return self._output(None)
            # 捕获达标的同一周期开始预发零速度，避免额外延迟一个周期。
        if self.state == "PREPARE":
            return self._prepare_output(now, navigation_revoked, feedback, flight)

        if not navigation_revoked:
            self.cancel("navigation_output_conflict")
            return self._output(None)
        if not flight.offboard:
            self.cancel("offboard_lost")
            return self._output(None)
        self._feedback(feedback, now, flight)
        if (self.last_ack_stamp is None
                or now - self.last_ack_stamp > config.confirmation_timeout_s):
            self.cancel("velocity_confirmation_lost")
            return self._output(None)
        # 起飞保留 VIO/双目；返回导航需要另行反向交接。
        if self.confirmed and self.intent == "land" and release_ack:
            self.released = True
        return self._guidance_output(now, dt, xy_error, yaw_error, height, new_sample, flight)

    def _native_land_output(self, now, flight):
        # Land 请求成功不等于完成，仍需新鲜的接地和已锁定回读。
        if (flight is not None and flight.connected and flight.landed is True
                and not flight.armed
                and self.fresh(flight.stamp, now, self.cfg.telemetry_timeout_s)):
            self.state, self.reason = "COMPLETE", "native_land_landed_disarmed"
        return self._output(None)

    def _input_loss_output(self, now, reason):
        config = self.cfg
        self.good = self.aligned = self.acks = 0
        self.landed_since = None
        self.reason = reason
        if self.loss_since is None:
            self.loss_since = now
        if self.state in ("ACQUIRE", "PREPARE"):
            if self.state == "PREPARE" and now - self.pending_since >= config.handoff_timeout_s:
                self.cancel("handoff_timeout")
        elif now - self.loss_since >= config.loss_land_timeout_s:
            self.cancel(reason)
        # 输入失效立即停下降，不通过变化率限制逐渐减速。
        self.last_velocity[:] = 0.
        self.last_yaw_rate = 0.
        velocity = None if self.state in ("ACQUIRE", "NATIVE_LAND") else (0., 0., 0.)
        return self._output(velocity)

    def _acquire(self, now, new_sample, navigation_revoked, flight):
        if new_sample:
            self.good += 1
        self.reason = "acquiring"
        if self.good < self.cfg.acquire_samples or not navigation_revoked:
            return False
        self.state, self.reason = "PREPARE", "velocity_prestream"
        self.token = str(uuid.uuid4())
        self.pending_since = now
        self.epoch = flight.epoch
        return True

    def _prepare_output(self, now, navigation_revoked, feedback, flight):
        config = self.cfg
        if not navigation_revoked:
            self.cancel("navigation_output_not_revoked")
            return self._output(None)
        if now - self.pending_since >= config.handoff_timeout_s:
            self.cancel("handoff_timeout")
            return self._output(None)

        if self._feedback(feedback, now, flight):
            self.acks += 1
        # 重复回读既不累加，也不清零确认窗；错误回读才清零。
        elif feedback is not None and not (
                feedback.token == self.token and feedback.sequence == self.last_ack_sequence
                and feedback.stamp == self.last_ack_stamp):
            self.acks = 0
        if (self.acks >= config.handoff_samples
                and now - self.pending_since >= config.handoff_min_stream_s):
            self.confirmed = True
            self.state = "TAKEOFF" if self.intent == "takeoff" else "ALIGN"
            self.reason = "handoff_confirmed"
        return self._output((0., 0., 0.))

    def _guidance_output(self, now, dt, xy_error, yaw_error, height, new_sample, flight):
        config = self.cfg
        aligned = (np.linalg.norm(xy_error) <= config.align_xy_m
                   and abs(yaw_error) <= config.align_yaw_rad)
        if new_sample:
            self.aligned = self.aligned + 1 if aligned else 0

        velocity = np.zeros(3)
        velocity[:2] = xy_error * config.kp_xy
        xy_speed = np.linalg.norm(velocity[:2])
        if xy_speed > config.max_xy_m_s:
            velocity[:2] *= config.max_xy_m_s / xy_speed
        yaw_rate = float(np.clip(yaw_error * config.kp_yaw,
                                -config.max_yaw_rate_rad_s, config.max_yaw_rate_rad_s))
        descent = False
        if self.state == "TAKEOFF":
            velocity[2] = -min(config.max_up_m_s, max(0., config.takeoff_height_m - height))
            self.reason = "takeoff_vio_required"
            if height >= config.takeoff_height_m - config.takeoff_tolerance_m:
                self.state, self.reason = "TAKEOFF_HOLD", "reverse_handoff_required"
                velocity[2] = 0.
        elif self.state == "TAKEOFF_HOLD":
            self.reason = "reverse_handoff_required"
        else:
            if self.aligned >= config.align_samples:
                self.state = "FLARE" if height <= config.flare_height_m else "DESCEND"
                descent = height > config.touchdown_height_m
                if descent:
                    velocity[2] = (config.flare_down_m_s if self.state == "FLARE"
                                   else config.max_down_m_s)
            else:
                self.state = "ALIGN"
            self.reason = "aligned_descent" if descent else "alignment_or_touchdown_hold"
            if not self._update_touchdown(now, height, flight):
                return self._output(None)
            if self.state == "TOUCHDOWN" or self.state == "COMPLETE":
                velocity[:] = 0.
                yaw_rate = 0.
                descent = False

        velocity, yaw_rate = self._limit_velocity(velocity, yaw_rate, dt, descent)
        self.last_velocity, self.last_yaw_rate = velocity.copy(), yaw_rate
        output_velocity = None if self.state == "COMPLETE" else tuple(float(x) for x in velocity)
        return self._output(output_velocity, yaw_rate, descent)

    def _update_touchdown(self, now, height, flight):
        config = self.cfg
        if height <= config.touchdown_height_m:
            if self.touchdown_pending_since is None:
                self.touchdown_pending_since = now
            if (now - self.touchdown_pending_since >= config.touchdown_confirmation_timeout_s
                    and flight.landed is not True):
                self.cancel("touchdown_confirmation_timeout")
                return False
        else:
            self.touchdown_pending_since = None

        # PX4 接地回读、独立低高度和低速度须同时满足连续窗口。
        ground = (flight.landed is True and height <= config.touchdown_height_m
                  and np.linalg.norm(flight.velocity_ned) <= config.touchdown_speed_m_s)
        if not ground:
            self.landed_since = None
            return True
        self.state = "TOUCHDOWN"
        if self.landed_since is None:
            self.landed_since = now
        if now - self.landed_since >= config.touchdown_window_s:
            if not flight.armed:
                self.state, self.reason = "COMPLETE", "landed_disarmed"
            else:
                self.reason = "landed_disarm_permitted"
        return True

    def _limit_velocity(self, velocity, yaw_rate, dt, descent):
        config = self.cfg
        delta = velocity - self.last_velocity
        speed_change = np.linalg.norm(delta)
        if speed_change > config.max_accel_m_s2 * dt:
            velocity = self.last_velocity + delta * (config.max_accel_m_s2 * dt / speed_change)
        # NED +z 为下降；许可撤销时立即清零，不等变化率限制收敛。
        if not descent and velocity[2] > 0:
            velocity[2] = 0.
        if self.state == "TOUCHDOWN":
            velocity[:] = 0.
        yaw_rate = float(np.clip(yaw_rate,
                                self.last_yaw_rate - config.max_yaw_accel_rad_s2 * dt,
                                self.last_yaw_rate + config.max_yaw_accel_rad_s2 * dt))
        return velocity, yaw_rate

    def _output(self, velocity, yaw_rate=0., descent=False):
        if velocity is not None:
            self.sequence += 1
        return LandingOutput(
            state=self.state,
            reason=self.reason,
            velocity_ned=velocity,
            yaw_rate=yaw_rate,
            navigation_allowed=self.state in ("NAVIGATION", "ACQUIRE"),
            vio_required=not self.released,
            release_compute=self.confirmed and self.intent == "land" and not self.released,
            request_native_land=self.state == "NATIVE_LAND",
            request_disarm=self.reason == "landed_disarm_permitted",
            descent_permitted=descent,
            token=self.token,
            sequence=self.sequence,
        )
