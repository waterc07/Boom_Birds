"""Platform-relative guidance and exclusive output handoff; no I/O or arming."""
from dataclasses import dataclass
import math
import uuid
import numpy as np
from .platform_model import validateProfile, transform
from .px4_frames import Px4LocalSetpoint

# Position/acceleration/yaw ignored, velocity and yaw-rate active.
VELOCITY_YAW_RATE_MASK = 1 | 2 | 4 | 64 | 128 | 256 | 1024


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def nedFromFlu(roll, pitch, yaw):
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
    yaw_offset_rad: float = 0.
    failure_reason: str = ""

    def setpoint(self):
        if self.velocity_ned is None:
            raise ValueError("no_velocity_output")
        return Px4LocalSetpoint((0., 0., 0.), self.velocity_ned, (0., 0., 0.), 0., self.yaw_rate)


class PlatformLanding:
    def __init__(self, profile, *, test_only=False):
        validateProfile(profile, test_only=test_only)
        self.profile = profile
        self.cfg = LandingConfig(**profile["guidance"])
        # 捕获连续性：允许短暂单帧丢失，但相邻合格样本间隔不得超过该上限。
        self.maxGoodGap = max(.2, 3*self.cfg.observation_timeout_s)
        self.lastGood = None
        self.rangeMount = transform(profile["range"]["T_body_sensor"], "range_extrinsics")
        if not np.allclose(self.rangeMount[:3, 2], [0, 0, -1], atol=1e-6):
            raise ValueError("range_beam_must_point_body_down")
        self.state, self.reason = "NAVIGATION", "navigation"
        self.failureReason = ""
        self.intent = "land"
        self.token = ""
        self.sequence = 0
        self.confirmed = False
        self.released = False
        self.pendingSince = None
        self.requestSince = None
        self.touchdownPendingSince = None
        self.lastNow = None
        self.lastPose = None
        self.lastRange = None
        self.lastObservationStamp = None
        self.lastRangeStamp = None
        self.good = self.aligned = self.acks = 0
        self.lastAckSequence = 0
        self.lastAckStamp = None
        self.epoch = None
        self.lossSince = self.landedSince = None
        self.lastVelocity = np.zeros(3)
        self.lastYawRate = 0.
        self.lastSentSequence = 0
        self.lastSentAt = None
        self.sent_history = {}
        # 任务侧可在交接请求里指定相对板 +x 的降落航向偏置；交接后只依赖
        # Tag 位姿与飞控姿态，不需要 VIO 世界系航向。
        self.yawOffset = 0.

    def fresh(self, stamp, now, timeout):
        return math.isfinite(stamp) and -self.cfg.future_tolerance_s <= now-stamp <= timeout

    def request(self, intent="land", yawOffset=0.):
        if intent not in ("land", "takeoff"):
            raise ValueError("intent")
        if (type(yawOffset) not in (int, float) or not math.isfinite(yawOffset)
                or abs(yawOffset) > math.pi):
            raise ValueError("yaw_offset")
        if self.state != "NAVIGATION":
            raise ValueError("handoff_already_started")
        self.intent = intent
        self.yawOffset = float(yawOffset)
        self.state = "ACQUIRE"
        self.reason = "acquiring"

    def cancel(self, reason="cancelled"):
        # No automatic return to VIO after compute release.
        if not self.failureReason:
            self.failureReason = reason
        self.state, self.reason = "NATIVE_LAND", reason
        self.lastVelocity[:] = 0.
        self.lastYawRate = 0.

    def noteSent(self, output, now, accepted):
        if output.sequence != self.sequence or output.token != self.token:
            raise ValueError("send_receipt_mismatch")
        if not accepted:
            self.cancel("velocity_send_failed")
            return
        self.lastSentSequence, self.lastSentAt = output.sequence, now
        self.sent_history[output.sequence] = (now, output.velocity_ned)
        if len(self.sent_history) > 256:
            del self.sent_history[min(self.sent_history)]

    def _inputs(self, now, observation, rangeSample, flight):
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
        knownIds = {tag["id"] for tag in self.profile["board"]["tags"]}
        quality = self.profile["quality"]
        if (len(observation.tag_ids) < quality["min_tags"]
                or len(set(observation.tag_ids)) != len(observation.tag_ids)
                or any(tag not in knownIds for tag in observation.tag_ids)):
            return None, "observation_id"
        if (not math.isfinite(observation.reprojection_px)
                or not 0 <= observation.reprojection_px <= quality["max_reprojection_px"]
                or not math.isfinite(observation.min_edge_px) or observation.min_edge_px < quality["min_edge_px"]
                or not math.isfinite(observation.ambiguity_ratio)
                or (observation.quality.get("planar_distinct", True)
                    and observation.ambiguity_ratio < quality["min_ambiguity_ratio"])):
            return None, "observation_quality"
        if (not self.fresh(observation.stamp, now, config.observation_timeout_s)
                or self.lastObservationStamp is not None and observation.stamp < self.lastObservationStamp):
            return None, "observation_stale_or_reordered"
        try:
            pose = observation.pose()
        except ValueError:
            return None, "observation_invalid"
        # 控制周期可复用同一观测；只有采样时间变化才参与跳变检查和连续样本计数。
        newPose = observation.stamp != self.lastObservationStamp
        if newPose and self.lastPose is not None:
            previousPose = self.lastPose
            rotationJump = math.acos(float(np.clip(
                (np.trace(previousPose[:3, :3].T @ pose[:3, :3]) - 1) / 2, -1, 1)))
            if (np.linalg.norm(pose[:3, 3] - previousPose[:3, 3]) > config.max_pose_jump_m
                    or rotationJump > config.max_rotation_jump_rad):
                return None, "pose_jump"
        if rangeSample is None or not rangeSample.valid:
            return None, "range_missing"
        if (not self.fresh(rangeSample.stamp, now, config.range_timeout_s)
                or self.lastRangeStamp is not None and rangeSample.stamp < self.lastRangeStamp):
            return None, "range_stale_or_reordered"
        distanceM = rangeSample.distance_m
        if not math.isfinite(distanceM) or not config.range_min_m <= distanceM <= config.range_max_m:
            return None, "range_blind_or_invalid"
        newRange = rangeSample.stamp != self.lastRangeStamp
        if (newRange and self.lastRange is not None
                and abs(distanceM - self.lastRange) > config.max_range_jump_m):
            return None, "range_jump"
        bodyToNed = nedFromFlu(flight.roll, flight.pitch, flight.yaw)
        # distance_m 是传感器斜距；安装平移与倾角换算后取 NED +z 的垂直高度。
        height = float((bodyToNed @ (
            self.rangeMount[:3, 3] + self.rangeMount[:3, 2] * distanceM))[2])
        visualHeight = float((bodyToNed @ pose[:3,3])[2])
        if height <= 0 or abs(height-visualHeight) > config.range_pose_tolerance_m:
            return None, "range_pose_disagreement"
        normal = bodyToNed @ pose[:3,2]
        if normal[2] > -math.cos(config.max_tilt_rad):
            return None, "platform_tilt"
        landingPoint = np.array([*self.profile["board"]["landing_point_m"], 1.])
        xyError = (bodyToNed @ (pose @ landingPoint)[:3])[:2]
        targetYaw = self.profile["board"]["target_yaw_rad"] + self.yawOffset
        heading = bodyToNed @ pose[:3,:3] @ np.array([math.cos(targetYaw), math.sin(targetYaw), 0])
        yawError = wrap(math.atan2(heading[1], heading[0])-flight.yaw)
        if newPose:
            self.lastPose, self.lastObservationStamp = pose.copy(), observation.stamp
        if newRange:
            self.lastRange, self.lastRangeStamp = distanceM, rangeSample.stamp
        # 合格样本按"新的板观测 + 当拍有效新鲜测距"计数。图像与测距各自独立更新，
        # 若要求两者在同一控制周期同时变化，真实异步传感器会永远攒不满捕获样本。
        # 测距的新鲜度、跳变与板/测距一致性仍逐拍检查，重复回读也不扩大证据。
        return (xyError, yawError, height, newPose), "ok"

    def _feedback(self, feedback, now, flight):
        if feedback is None:
            return False
        # 回读必须关联已发送的目标；OFFBOARD 或本地发送成功不能替代确认。
        sent = self.sent_history.get(feedback.sequence)
        if (sent is None or not self.fresh(feedback.stamp, now, self.cfg.confirmation_timeout_s)
                or feedback.token != self.token or feedback.epoch != self.epoch
                or feedback.sequence <= self.lastAckSequence
                or feedback.stamp < sent[0] or not feedback.velocity_active or feedback.attitude_active
                or feedback.source not in ("PX4_OCM_AND_SETPOINT_ECHO", "TEST_ONLY_FAKE_PX4", "PX4_MAVLINK_VELOCITY_TARGET")
                or not flight.offboard or not flight.velocity_estimator_valid
                or len(feedback.echoed_velocity_ned) != 3
                or not np.isfinite(feedback.echoed_velocity_ned).all()
                or not np.allclose(feedback.echoed_velocity_ned, sent[1], atol=.01)):
            return False
        self.lastAckSequence, self.lastAckStamp = feedback.sequence, feedback.stamp
        return True

    def step(self, now, observation=None, rangeSample=None, flight=None, feedback=None,
             *, navigationRevoked=False, navigationReady=False, releaseAck=False):
        # now 与输入 stamp 均为同一主机单调时钟秒；桥接层保留 ROS 消息年龄。
        # 本层返回动作意图；发送、资源退出和飞控回读由执行器确认。
        if not math.isfinite(now):
            raise ValueError("now")
        config = self.cfg
        dt = .02 if self.lastNow is None else max(0., min(now - self.lastNow, .1))
        if self.lastNow is not None and now < self.lastNow:
            self.cancel("clock_reset")
        self.lastNow = now

        if self.state in ("NAVIGATION", "COMPLETE"):
            return self._output(None)
        if self.state == "NATIVE_LAND":
            return self._nativeLandOutput(now, flight)
        if self.requestSince is None:
            self.requestSince = now
        if self.state == "ACQUIRE" and now - self.requestSince >= config.acquire_timeout_s:
            self.cancel("acquire_timeout")
            return self._output(None)
        if not self.confirmed and not navigationRevoked and not navigationReady:
            # 导航依赖只约束"撤销导航输出前"的捕获保持；撤销后平台速度闭环
            # 不再依赖 VIO，不能在 PREPARE 因一次 VIO 抖动而放弃已开始的交接。
            self.good = self.aligned = self.acks = 0
            self.reason = "navigation_dependency_unavailable"
            return self._output(None)

        data, reason = self._inputs(now, observation, rangeSample, flight)
        if self.state == "NATIVE_LAND":
            return self._output(None)
        if data is None:
            if self.lastGood is not None and now-self.lastGood > self.maxGoodGap:
                self.good = 0
            return self._inputLossOutput(now, reason)
        self.lossSince = None
        self.lastGood = now
        xyError, yawError, height, newSample = data

        if self.state == "ACQUIRE":
            if not self._acquire(now, newSample, navigationRevoked, flight):
                return self._output(None)
            # 捕获达标的同一周期开始预发零速度，避免额外延迟一个周期。
        if self.state == "PREPARE":
            return self._prepareOutput(now, navigationRevoked, feedback, flight)

        if not navigationRevoked:
            self.cancel("navigation_output_conflict")
            return self._output(None)
        if not flight.offboard:
            self.cancel("offboard_lost")
            return self._output(None)
        self._feedback(feedback, now, flight)
        if (self.lastAckStamp is None
                or now - self.lastAckStamp > config.confirmation_timeout_s):
            self.cancel("velocity_confirmation_lost")
            return self._output(None)
        # 起飞保留 VIO/双目；返回导航需要另行反向交接。
        if self.confirmed and self.intent == "land" and releaseAck:
            self.released = True
        return self._guidanceOutput(now, dt, xyError, yawError, height, newSample, flight)

    def _nativeLandOutput(self, now, flight):
        # Land 请求成功不等于完成，仍需新鲜的接地和已锁定回读。
        if (flight is not None and flight.connected and flight.landed is True
                and not flight.armed
                and self.fresh(flight.stamp, now, self.cfg.telemetry_timeout_s)):
            self.state, self.reason = "COMPLETE", "native_land_landed_disarmed"
        return self._output(None)

    def _inputLossOutput(self, now, reason):
        config = self.cfg
        # 单次瞬时丢帧不清零捕获计数；超过 maxGoodGap 的间隔在 step 中清零。
        self.aligned = self.acks = 0
        self.landedSince = None
        self.reason = reason
        if self.lossSince is None:
            self.lossSince = now
        if self.state in ("ACQUIRE", "PREPARE"):
            if self.state == "PREPARE" and now - self.pendingSince >= config.handoff_timeout_s:
                self.cancel("handoff_timeout")
        elif now - self.lossSince >= config.loss_land_timeout_s:
            self.cancel(reason)
        # 输入失效立即停下降，不通过变化率限制逐渐减速。
        self.lastVelocity[:] = 0.
        self.lastYawRate = 0.
        velocity = None if self.state in ("ACQUIRE", "NATIVE_LAND") else (0., 0., 0.)
        return self._output(velocity)

    def _acquire(self, now, newSample, navigationRevoked, flight):
        if newSample:
            self.good += 1
        self.reason = "acquiring"
        if self.good < self.cfg.acquire_samples or not navigationRevoked:
            return False
        self.state, self.reason = "PREPARE", "velocity_prestream"
        self.token = str(uuid.uuid4())
        self.pendingSince = now
        self.epoch = flight.epoch
        return True

    def _prepareOutput(self, now, navigationRevoked, feedback, flight):
        config = self.cfg
        if not navigationRevoked:
            self.cancel("navigation_output_not_revoked")
            return self._output(None)
        if now - self.pendingSince >= config.handoff_timeout_s:
            self.cancel("handoff_timeout")
            return self._output(None)

        if self._feedback(feedback, now, flight):
            self.acks += 1
        # 重复回读既不累加，也不清零确认窗；错误回读才清零。
        elif feedback is not None and not (
                feedback.token == self.token and feedback.sequence == self.lastAckSequence
                and feedback.stamp == self.lastAckStamp):
            self.acks = 0
        if (self.acks >= config.handoff_samples
                and now - self.pendingSince >= config.handoff_min_stream_s):
            self.confirmed = True
            self.state = "TAKEOFF" if self.intent == "takeoff" else "ALIGN"
            self.reason = "handoff_confirmed"
        return self._output((0., 0., 0.))

    def _guidanceOutput(self, now, dt, xyError, yaw_error, height, newSample, flight):
        config = self.cfg
        aligned = (np.linalg.norm(xyError) <= config.align_xy_m
                   and abs(yaw_error) <= config.align_yaw_rad)
        if newSample:
            self.aligned = self.aligned + 1 if aligned else 0

        velocity = np.zeros(3)
        velocity[:2] = xyError * config.kp_xy
        xySpeed = np.linalg.norm(velocity[:2])
        if xySpeed > config.max_xy_m_s:
            velocity[:2] *= config.max_xy_m_s / xySpeed
        yawRate = float(np.clip(yaw_error * config.kp_yaw,
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
            if not self._updateTouchdown(now, height, flight,
                    alignedWindow=self.aligned >= config.align_samples):
                return self._output(None)
            if self.state == "TOUCHDOWN" or self.state == "COMPLETE":
                velocity[:] = 0.
                yawRate = 0.
                descent = False

        velocity, yawRate = self._limitVelocity(velocity, yawRate, dt, descent)
        self.lastVelocity, self.lastYawRate = velocity.copy(), yawRate
        outputVelocity = None if self.state == "COMPLETE" else tuple(float(x) for x in velocity)
        return self._output(outputVelocity, yawRate, descent)

    def _updateTouchdown(self, now, height, flight, alignedWindow=False):
        config = self.cfg
        # 近地设计交接：已连续对准并到达 touchdown 高度时，主动请求 PX4 原生 Land。
        # 视觉/测距高度只用于发起交接，不用于判定已接地；完成仍要求 landed+disarmed 回读。
        # 不在本判据里用视觉速度替代 PX4 速度估计，也不把"看不见 Tag"当作接地。
        if (alignedWindow and height <= config.touchdown_height_m and flight.landed is not True
                and self.state in ("ALIGN", "DESCEND", "FLARE")):
            self.state, self.reason = "NATIVE_LAND", "near_ground_native_land"
            return True
        if height <= config.touchdown_height_m:
            if self.touchdownPendingSince is None:
                self.touchdownPendingSince = now
            if (now - self.touchdownPendingSince >= config.touchdown_confirmation_timeout_s
                    and flight.landed is not True):
                self.cancel("touchdown_confirmation_timeout")
                return False
        else:
            self.touchdownPendingSince = None

        # PX4 接地回读、独立低高度和低速度须同时满足连续窗口。
        ground = (flight.landed is True and height <= config.touchdown_height_m
                  and np.linalg.norm(flight.velocity_ned) <= config.touchdown_speed_m_s)
        if not ground:
            self.landedSince = None
            return True
        self.state = "TOUCHDOWN"
        if self.landedSince is None:
            self.landedSince = now
        if now - self.landedSince >= config.touchdown_window_s:
            if not flight.armed:
                self.state, self.reason = "COMPLETE", "landed_disarmed"
            else:
                self.reason = "landed_disarm_permitted"
        return True

    def _limitVelocity(self, velocity, yaw_rate, dt, descent):
        config = self.cfg
        delta = velocity - self.lastVelocity
        speedChange = np.linalg.norm(delta)
        if speedChange > config.max_accel_m_s2 * dt:
            velocity = self.lastVelocity + delta * (config.max_accel_m_s2 * dt / speedChange)
        # NED +z 为下降；许可撤销时立即清零，不等变化率限制收敛。
        if not descent and velocity[2] > 0:
            velocity[2] = 0.
        if self.state == "TOUCHDOWN":
            velocity[:] = 0.
        yaw_rate = float(np.clip(yaw_rate,
                                self.lastYawRate - config.max_yaw_accel_rad_s2 * dt,
                                self.lastYawRate + config.max_yaw_accel_rad_s2 * dt))
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
            yaw_offset_rad=self.yawOffset,
            failure_reason=self.failureReason,
        )
