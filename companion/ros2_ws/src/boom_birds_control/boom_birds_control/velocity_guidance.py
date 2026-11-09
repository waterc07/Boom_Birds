"""局部位置误差转有界速度；不计算姿态、推力或悬停推力。"""
from dataclasses import dataclass
import math
import numpy as np
from .attitude_control import AttitudeReference, finiteVector


@dataclass(frozen=True)
class VelocityConfig:
    kp: tuple = (2.0, 2.0, 2.0)
    kv: tuple = (1.0, 1.0, 1.0)
    max_velocity: float = 2.0
    max_command_velocity: float = 0.3
    max_acceleration: float = 0.3
    max_position_error: float = 1.0
    max_feedback_velocity_error: float = 0.3
    max_tilt_rad: float = 0.3490658504
    alignment_samples: int = 10
    alignment_tolerance_rad: float = 0.0872664626
    alignment_tilt_rad: float = 0.1745329252
    reset_position_margin: float = 0.2
    acceleration_feedforward: bool = False

    def __post_init__(self):
        if np.any(finiteVector(self.kp, 3) <= 0) or np.any(finiteVector(self.kv,3) < 0):
            raise ValueError('velocity_guidance_kp')
        for name in ('max_velocity', 'max_command_velocity', 'max_acceleration',
                     'max_position_error', 'max_feedback_velocity_error', 'max_tilt_rad', 'alignment_tolerance_rad',
                     'alignment_tilt_rad', 'reset_position_margin'):
            if type(getattr(self, name)) not in (int,float) or not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(name)
        if (self.max_command_velocity > self.max_velocity or self.max_tilt_rad >= math.pi/2
                or self.alignment_tilt_rad >= math.pi/2 or type(self.alignment_samples) is not int
                or self.alignment_samples < 2 or type(self.acceleration_feedforward) is not bool):
            raise ValueError('velocity_guidance_bounds')


@dataclass(frozen=True)
class VelocitySetpoint:
    velocity_m_s: tuple
    acceleration_m_s2: tuple
    yaw_rad: float


class VelocityGuidance(AttitudeReference):
    def __init__(self, config=VelocityConfig(), **kwargs):
        super().__init__(config, **kwargs)
        self.resetOutput()

    def resetOutput(self):
        self.outputVelocity = np.zeros(3)
        self.outputAt = None

    def invalidate(self, reason):
        super().invalidate(reason)
        self.resetOutput()

    def calculate(self, desired, position, velocity, qVio, *, now, px4Local=False):
        if not (px4Local or self.ready) or self.latched:
            raise ValueError(self.latched or 'velocity_reference_pending')
        if not math.isfinite(now):
            raise ValueError('velocity_command_clock')
        c = self.config
        p, v = finiteVector(position, 3), finiteVector(velocity, 3)
        pd = finiteVector(desired.position_m, 3)
        vd = finiteVector(desired.velocity_m_s, 3)
        ad = finiteVector(desired.acceleration_m_s2, 3)
        if (np.linalg.norm(pd-p) > c.max_position_error or np.linalg.norm(v) > c.max_velocity
                or np.linalg.norm(vd) > c.max_command_velocity+1e-9
                or np.linalg.norm(ad) > c.max_acceleration+1e-9 or not math.isfinite(desired.yaw_rad)):
            raise ValueError('velocity_command_limits')
        from .attitude_control import rotation
        if rotation(qVio)[2,2] < math.cos(c.max_tilt_rad):
            raise ValueError('velocity_vehicle_tilt')
        target = vd + np.asarray(c.kp)*(pd-p) + np.asarray(c.kv)*(vd-v)
        target *= min(1., c.max_command_velocity/max(float(np.linalg.norm(target)), 1e-9))
        dt = 0.02 if self.outputAt is None else now-self.outputAt
        if not 0 < dt <= .5:
            self.resetOutput()
            raise ValueError('velocity_output_gap_or_reset')
        delta = target-self.outputVelocity
        # 窗口内只按实际间隔限速；长停发不得积累一次跳变额度。
        delta *= min(1., c.max_acceleration*dt/max(float(np.linalg.norm(delta)), 1e-9))
        self.outputVelocity = self.outputVelocity+delta
        self.outputAt = now
        acceleration = tuple(ad) if c.acceleration_feedforward else (0.,0.,0.)
        return VelocitySetpoint(tuple(self.outputVelocity), acceleration, desired.yaw_rad)


def velocityToNed(setpoint, yawOffset):
    """冻结参考是 VIO→PX4 ENU；不要套用旧 NWU 局部位置对齐约定。"""
    from .attitude_control import NED_TO_ENU, yawRotation, wrap
    from .px4_frames import Px4LocalSetpoint
    if not math.isfinite(yawOffset):
        raise ValueError('velocity_yaw_reference')
    transform = NED_TO_ENU.T @ yawRotation(yawOffset)
    velocity = transform @ finiteVector(setpoint.velocity_m_s, 3)
    acceleration = transform @ finiteVector(setpoint.acceleration_m_s2, 3)
    return Px4LocalSetpoint(position_m=(0.,0.,0.), velocity_m_s=tuple(velocity),
        acceleration_m_s2=tuple(acceleration),
        yaw_rad=wrap(math.pi/2-setpoint.yaw_rad-yawOffset), yaw_rate_rad_s=0.)
