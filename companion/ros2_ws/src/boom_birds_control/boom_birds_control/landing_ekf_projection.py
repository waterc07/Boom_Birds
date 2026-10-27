"""TEST-ONLY feature/EKF XY projection for an attitude-controller candidate."""
from dataclasses import dataclass
import math
import numbers
from .landing_ekf_reference import LandingReferenceSelector, VisualReference


@dataclass(frozen=True)
class FeatureWorldSample:
    stamp: float
    vehicle_stamp: float
    target_world_xy: tuple[float, float]
    vehicle_world_xy: tuple[float, float]
    # Frozen AttitudeController offset: VIO world -> PX4 ENU.
    world_to_enu_yaw_rad: float


@dataclass(frozen=True)
class LandingWorldFeedback:
    stamp: float
    position_world_xy: tuple[float, float]
    velocity_world_xy: tuple[float, float]
    target_world_xy: tuple[float, float]
    source: str


def vector2(value):
    try:
        value = tuple(value)
    except TypeError:
        raise ValueError("invalid_vector") from None
    if len(value) != 2 or not all(isinstance(x, numbers.Real) and not isinstance(x, bool) and math.isfinite(x) for x in value):
        raise ValueError("invalid_vector")
    return tuple(float(x) for x in value)


def rotate(matrix, vector):
    return tuple(sum(matrix[i][j]*vector[j] for j in range(2)) for i in range(2))


class EkfLandingProjection:
    """Project PX4 NED XY into the frozen VIO world frame without matching origins.

    Samples are supplied by the caller; this class has no ROS or control output.
    The VIO pose and feature fix are only used to anchor while vision is valid.
    After vision loss position and velocity are derived from EKF alone.
    """
    def __init__(self, reference):
        self.selector = LandingReferenceSelector(reference)
        self.anchor = None
        self.reason = "unseeded"

    def reset(self):
        self.selector.reset()
        self.anchor = None
        self.reason = "unseeded"

    def step(self, now, feature, ekf, velocity_ned_xy, *, anchor_ekf=None):
        try:
            velocity = vector2(velocity_ned_xy)
            if math.hypot(*velocity) > self.selector.reference.maxSpeed:
                raise ValueError("ekf_velocity_limit")
            if self.selector.mode == "visual" and feature is not None:
                if not isinstance(feature, FeatureWorldSample):
                    raise ValueError("feature_schema")
                target = vector2(feature.target_world_xy)
                vehicle = vector2(feature.vehicle_world_xy)
                if (not all(type(v) in (int, float) and math.isfinite(v)
                            for v in (now, feature.stamp, feature.vehicle_stamp, feature.world_to_enu_yaw_rad))
                        or not 0 <= now-feature.stamp <= self.selector.reference.maxAge
                        or not 0 <= now-feature.vehicle_stamp <= self.selector.reference.maxAge
                        or abs(feature.stamp-feature.vehicle_stamp) > self.selector.reference.maxSkew):
                    raise ValueError("feature_pose_pair_invalid")
                angle = feature.world_to_enu_yaw_rad
                c, s = math.cos(angle), math.sin(angle)
                # NED -> ENU swap, then ENU -> VIO world rotate by -offset.
                nedToWorld = ((s, c), (c, -s))
                worldError = (target[0]-vehicle[0], target[1]-vehicle[1])
                nedError = rotate(((nedToWorld[0][0], nedToWorld[1][0]),
                                   (nedToWorld[0][1], nedToWorld[1][1])), worldError)
                visual = VisualReference(feature.stamp, nedError)
                paired = ekf if anchor_ekf is None else anchor_ekf
                self.selector.step(now, visual, paired)
                if self.selector.reference.anchor is None:
                    raise ValueError(self.selector.reference.reason)
                self.anchor = (vehicle, tuple(paired.position_ned_xy), nedToWorld)
                error = self.selector.reference.update(now, ekf)
                if error is None:
                    raise ValueError(self.selector.reference.reason)
            else:
                error = self.selector.step(now, None, ekf)
                if error is None or self.anchor is None:
                    raise ValueError(self.selector.reason)
            originWorld, originNed, matrix = self.anchor
            position = rotate(matrix, (ekf.position_ned_xy[0]-originNed[0],
                                       ekf.position_ned_xy[1]-originNed[1]))
            position = (originWorld[0]+position[0], originWorld[1]+position[1])
            delta = rotate(matrix, error)
            target = (position[0]+delta[0], position[1]+delta[1])
            worldVelocity = rotate(matrix, velocity)
            self.reason = self.selector.mode
            return LandingWorldFeedback(ekf.stamp, position, worldVelocity, target, self.reason)
        except (ValueError, TypeError, AttributeError, IndexError, OverflowError) as exc:
            self.selector.mode = "failed"
            self.selector.reference.reset()
            self.anchor = None
            self.reason = str(exc) or "projection_invalid"
            return None
