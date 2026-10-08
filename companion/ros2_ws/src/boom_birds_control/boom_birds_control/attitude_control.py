"""机载位置闭环；输入世界系位置/速度，输出 MAVROS ENU/FLU 姿态与归一化推力。"""
from dataclasses import dataclass
import math
import numpy as np
from .frames import quat_to_rot, rot_to_quat, is_rotation
from .runtime_config import DEFAULTS

NED_TO_ENU = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
FRD_TO_FLU = np.diag([1., -1., -1.])


def finite_vector(value, size):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise ValueError("non_finite_or_wrong_shape")
    return a


def rotation(q):
    q = finite_vector(q, 4)
    if abs(np.linalg.norm(q) - 1.) > .01:
        raise ValueError("quaternion_not_unit")
    return quat_to_rot(q)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_rotation(a):
    return np.array([[math.cos(a), -math.sin(a), 0.],
                     [math.sin(a), math.cos(a), 0.], [0., 0., 1.]])


def px4_attitude_enu_flu(roll, pitch, yaw):
    finite_vector((roll, pitch, yaw), 3)
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    r = yaw_rotation(yaw) @ np.array([[cp, 0., sp], [0., 1., 0.], [-sp, 0., cp]]) @ np.array([[1., 0., 0.], [0., cr, -sr], [0., sr, cr]])
    return NED_TO_ENU @ r @ FRD_TO_FLU


@dataclass(frozen=True)
class AttitudeConfig:
    kp: tuple = (1.5, 1.5, 1.5)
    kv: tuple = (2.0, 2.0, 2.0)
    gravity: float = 9.81
    hover_thrust: float = .5
    min_thrust: float = .08
    max_thrust: float = .7
    max_tilt_rad: float = .3490658504
    max_acceleration: float = 3.0
    max_position_error: float = 1.0
    max_velocity: float = 2.0
    alignment_samples: int = 10
    alignment_tolerance_rad: float = .0872664626
    alignment_tilt_rad: float = .1745329252
    reset_position_margin: float = .2

    def __post_init__(self):
        for gain in (self.kp, self.kv):
            if np.any(finite_vector(gain, 3) <= 0): raise ValueError("gain_must_be_positive")
        for name in ('gravity', 'max_tilt_rad', 'max_acceleration', 'max_position_error',
                     'max_velocity', 'alignment_tolerance_rad', 'alignment_tilt_rad', 'reset_position_margin'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0: raise ValueError(name)
        if not 0 < self.min_thrust < self.hover_thrust < self.max_thrust <= 1: raise ValueError("thrust_bounds")
        if self.max_acceleration >= self.gravity: raise ValueError('acceleration_exceeds_gravity')
        if not self.max_tilt_rad < math.pi/2 or not self.alignment_tilt_rad < math.pi/2: raise ValueError("tilt_bounds")
        if not isinstance(self.alignment_samples, int) or self.alignment_samples < 2: raise ValueError("alignment_samples")


@dataclass(frozen=True)
class AttitudeSetpoint:
    orientation_xyzw: tuple
    thrust: float
    acceleration_world: tuple
    saturated: bool = False


class AttitudeController:
    def __init__(self, config=AttitudeConfig()):
        self.config = config
        self.offset = None
        self.candidate = None
        self.samples = 0
        self.latched = ''
        self.last_stamp = None
        self.last_position = None

    def invalidate(self, reason):
        self.latched = reason
        self.offset = None
        self.samples = 0

    def observe(self, position, velocity, q_vio, px4_rotation, stamp, armed):
        c = self.config
        p, v = finite_vector(position, 3), finite_vector(velocity, 3)
        rv = rotation(q_vio)
        if self.latched: raise ValueError(self.latched)
        if not math.isfinite(stamp): raise ValueError('vio_stamp_invalid')
        if self.last_stamp is not None:
            dt = stamp - self.last_stamp
            if self.offset is not None and dt > 2*DEFAULTS.pose_timeout_s:
                self.invalidate("vio_gap_reference_invalid")
                raise ValueError(self.latched)
            if dt <= 0:
                self.invalidate('vio_clock_reset')
                raise ValueError(self.latched)
            if np.linalg.norm(p-self.last_position) > c.max_velocity*dt + c.reset_position_margin:
                self.invalidate('vio_position_reset')
                raise ValueError(self.latched)
        self.last_stamp, self.last_position = stamp, p.copy()
        if np.linalg.norm(v) > c.max_velocity:
            raise ValueError('vio_velocity_limit')
        rp = np.asarray(px4_rotation, dtype=float)
        if rp.shape != (3,3) or not np.isfinite(rp).all() or not is_rotation(rp):
            raise ValueError("fcu_rotation_invalid")
        # 两个姿态均为 body FLU→各自世界系；相乘得到 VIO 世界系→PX4 ENU。
        # 这里只冻结水平旋转；位置误差仍在 VIO 世界系计算，无需两套位置原点相同。
        relative = rp @ rv.T
        tilt = math.acos(float(np.clip(relative[2, 2], -1, 1)))
        if tilt > c.alignment_tilt_rad:
            self.invalidate('body_axes_or_gravity_mismatch')
            raise ValueError(self.latched)
        angle = math.atan2(relative[1,0], relative[0,0])
        if self.offset is not None:
            if abs(wrap(angle-self.offset)) > c.alignment_tolerance_rad:
                self.invalidate('attitude_reference_reset')
                raise ValueError(self.latched)
            return
        if armed: raise ValueError('alignment_requires_disarmed_samples')
        if np.linalg.norm(v) > DEFAULTS.stable_speed_m_s:
            self.samples = 0
            raise ValueError('alignment_requires_stationary_samples')
        if self.candidate is None or abs(wrap(angle-self.candidate)) > c.alignment_tolerance_rad:
            self.candidate, self.samples = angle, 1
        else:
            self.samples += 1
        if self.samples >= c.alignment_samples:
            self.offset = self.candidate

    @property
    def ready(self):
        return self.offset is not None and not self.latched

    def calculate(self, desired, position, velocity, q_vio):
        if not self.ready: raise ValueError(self.latched or 'attitude_alignment_pending')
        c = self.config
        p, v = finite_vector(position, 3), finite_vector(velocity, 3)
        pd = finite_vector(desired.position_m, 3)
        vd = finite_vector(desired.velocity_m_s, 3)
        ad = finite_vector(desired.acceleration_m_s2, 3)
        if not math.isfinite(desired.yaw_rad): raise ValueError('yaw_not_finite')
        if np.linalg.norm(pd-p) > c.max_position_error: raise ValueError('position_error_limit')
        if np.linalg.norm(vd) > c.max_velocity: raise ValueError('command_velocity_limit')
        if np.linalg.norm(ad) > c.max_acceleration: raise ValueError('command_acceleration_limit')
        rv = rotation(q_vio)
        if rv[2,2] < .5: raise ValueError('vehicle_tilt_limit')
        a = ad + np.asarray(c.kp)*(pd-p) + np.asarray(c.kv)*(vd-v)
        limited = a*min(1., c.max_acceleration/max(float(np.linalg.norm(a)),1e-9))
        saturated = not np.allclose(a, limited)
        force = limited + np.array([0., 0., c.gravity])
        horizontal = np.linalg.norm(force[:2])
        max_horizontal = force[2]*math.tan(c.max_tilt_rad)
        if horizontal > max_horizontal:
            force[:2] *= max_horizontal/horizontal
            saturated = True
        z = force/np.linalg.norm(force)
        heading = np.array([math.cos(desired.yaw_rad), math.sin(desired.yaw_rad), 0.])
        y = np.cross(z, heading); y /= np.linalg.norm(y)
        rd = np.column_stack((np.cross(y,z), y, z))
        # force 为世界系所需比力（m/s²）；沿当前机体 +z 投影并用悬停推力归一化。
        thrust_unlimited = c.hover_thrust*float(force @ rv[:,2])/c.gravity
        thrust = float(np.clip(thrust_unlimited, c.min_thrust, c.max_thrust))
        saturated |= abs(thrust-thrust_unlimited) > 1e-9
        q = rot_to_quat(yaw_rotation(self.offset) @ rd)
        return AttitudeSetpoint(tuple(q), thrust, tuple(force-np.array([0.,0.,c.gravity])), saturated)


class TouchdownRamp:
    """连续接地候选后降低归一化推力；实际 disarm 仍由 PX4 落地回读决定。"""
    def __init__(self, min_thrust):
        self.min_thrust = min_thrust
        self.since = self.last = None

    def reset(self):
        self.since = self.last = None

    def apply(self, setpoint, *, position, velocity, ground_z, now, landing):
        from dataclasses import replace
        if not landing:
            self.reset()
            return setpoint
        p, v = finite_vector(position,3), finite_vector(velocity,3)
        if not math.isfinite(ground_z) or not math.isfinite(now): raise ValueError('touchdown_nonfinite')
        if self.last is not None and (now < self.last or now-self.last > DEFAULTS.pose_timeout_s):
            self.reset()
        self.last = now
        near = abs(p[2]-ground_z) <= DEFAULTS.stable_radius_m and np.linalg.norm(v) <= DEFAULTS.stable_speed_m_s
        if not near:
            self.reset()
            return setpoint
        if self.since is None: self.since = now
        fraction = min(1., max(0., (now-self.since-DEFAULTS.stable_duration_s)/DEFAULTS.stable_duration_s))
        return replace(setpoint, thrust=setpoint.thrust*(1-fraction)+self.min_thrust*fraction)
