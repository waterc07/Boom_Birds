"""Test-only visual-to-PX4-EKF landing reference; no sensor or control I/O."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class EkfReferenceSample:
    stamp: float
    position_ned_xy: tuple[float, float]
    horizontal_position_valid: bool
    constant_position_mode: bool
    position_accuracy_m: float
    epoch: int
    reset_counter: int
    position_ned_z: float | None = None
    velocity_ned_z: float | None = None
    vertical_position_valid: bool = False
    vertical_accuracy_m: float | None = None


@dataclass(frozen=True)
class VisualReference:
    stamp: float
    error_ned_xy: tuple[float, float]


class LandingEkfReference:
    """Pair a validated visual error with EKF XY and propagate it after visual loss.

    All input clocks must share a monotonic time base. The caller must verify the
    FCU source, estimator validity and reset counter independently. The
    configurable limits are experimental; no values here establish flight safety.
    """
    def __init__(self, *, max_age_s, max_pair_skew_s, max_accuracy_m,
                 max_duration_s, max_displacement_m, max_speed_m_s):
        values = (max_age_s, max_pair_skew_s, max_accuracy_m, max_duration_s,
                  max_displacement_m, max_speed_m_s)
        if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in values):
            raise ValueError("reference_limits")
        self.maxAge, self.maxSkew, self.maxAccuracy, self.maxDuration, self.maxDisplacement, self.maxSpeed = values
        self.reset()

    def reset(self):
        self.anchor = None
        self.previous = None
        self.reason = "unseeded"

    def check(self, now, sample):
        if (type(now) not in (int, float) or not math.isfinite(now)
                or not isinstance(sample, EkfReferenceSample)):
            return "sample_missing"
        if (type(sample.horizontal_position_valid) is not bool or not sample.horizontal_position_valid
                or type(sample.constant_position_mode) is not bool or sample.constant_position_mode
                or type(sample.epoch) is not int or type(sample.reset_counter) is not int
                or sample.epoch < 0 or sample.reset_counter < 0):
            return "estimator_unavailable"
        try:
            position = tuple(sample.position_ned_xy)
            numbers = (sample.stamp, sample.position_accuracy_m, *position)
            if len(position) != 2 or not all(type(v) in (int, float) and math.isfinite(v) for v in numbers):
                return "sample_invalid"
        except TypeError:
            return "sample_invalid"
        if not 0 <= now-sample.stamp <= self.maxAge:
            return "position_stale"
        if not 0 < sample.position_accuracy_m <= self.maxAccuracy:
            return "position_accuracy"
        return "ok"

    def seed(self, now, visual, sample):
        self.reset()
        reason = self.check(now, sample)
        if reason != "ok":
            self.reason = reason
            return False
        try:
            if not isinstance(visual, VisualReference):
                raise ValueError
            error = tuple(visual.error_ned_xy)
            if (len(error) != 2 or not all(type(v) in (int, float) and math.isfinite(v) for v in error)
                    or type(visual.stamp) not in (int, float) or not math.isfinite(visual.stamp)
                    or not 0 <= now-visual.stamp <= self.maxAge
                    or abs(visual.stamp-sample.stamp) > self.maxSkew):
                raise ValueError
        except (TypeError, ValueError):
            self.reason = "visual_pair_invalid"
            return False
        p = tuple(sample.position_ned_xy)
        self.anchor = (sample.stamp, p, error, sample.epoch, sample.reset_counter)
        self.previous = sample
        self.reason = "anchored"
        return True

    def update(self, now, sample):
        if self.anchor is None:
            return None
        reason = self.check(now, sample)
        if reason == "ok":
            stamp, origin, error, epoch, counter = self.anchor
            if sample.epoch != epoch or sample.reset_counter != counter:
                reason = "estimator_reset"
            elif sample.stamp < self.previous.stamp:
                reason = "sample_reordered"
            elif now-stamp > self.maxDuration:
                reason = "reference_expired"
            else:
                p = tuple(sample.position_ned_xy)
                dt = sample.stamp-self.previous.stamp
                movement = math.dist(p, self.previous.position_ned_xy)
                if (math.dist(p, origin) > self.maxDisplacement or (dt == 0 and movement > 0)
                        or (dt > 0 and movement/dt > self.maxSpeed)):
                    reason = "position_jump"
                else:
                    self.previous = sample
                    self.reason = "ekf"
                    return tuple(error[i]-(p[i]-origin[i]) for i in range(2))
        self.reset()
        self.reason = reason
        return None


class LandingReferenceSelector:
    """Use visual fixes until loss, then latch EKF for the remainder of the landing.

    A visual input must already have passed its detector, geometry and identity
    checks; None means invalid or missing. No height is used to select a source.
    A failed EKF reference remains failed until a new landing calls reset().
    """
    def __init__(self, reference):
        self.reference = reference
        self.reset()

    def reset(self):
        self.reference.reset()
        self.mode = "visual"
        self.reason = "waiting_for_visual"

    def step(self, now, visual, sample):
        if self.mode == "failed":
            return None
        if self.mode == "visual" and visual is not None:
            # Visual remains usable even if an EKF pair cannot be formed, but
            # do not retain an older anchor when the newest pairing fails.
            try:
                error = tuple(visual.error_ned_xy)
                valid = (isinstance(visual, VisualReference) and len(error) == 2
                    and all(type(v) in (int, float) and math.isfinite(v) for v in error)
                    and type(visual.stamp) in (int, float) and math.isfinite(visual.stamp)
                    and 0 <= now-visual.stamp <= self.reference.maxAge)
            except (AttributeError, TypeError):
                valid = False
            if valid:
                self.reference.seed(now, visual, sample)
                self.reason = "visual"
                return error
            visual = None
        if self.mode == "visual":
            self.mode = "ekf"
        result = self.reference.update(now, sample)
        self.reason = self.reference.reason
        if result is None:
            self.mode = "failed"
        return result
