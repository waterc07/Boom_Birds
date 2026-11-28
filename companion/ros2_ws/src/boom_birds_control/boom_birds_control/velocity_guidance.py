# ******************************************************************************
# @file           : velocity_guidance.py
# @brief          : 局部位置误差转有界速度。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

"""位置误差 → 有界速度参考（PX4 速度环的外环）。

本模块只把 VIO 世界系中的期望位置/速度转换为有界速度指令，交给 PX4 速度闭环；
不计算姿态、推力或悬停推力，也不直接驱动电机。坐标系与单位见各函数说明。
"""
from dataclasses import dataclass
import math

import numpy as np

from .attitude_control import (
    AttitudeReference,
    NED_TO_ENU,
    finiteVector,
    rotation,
    wrap,
    yawRotation,
)
from .px4_frames import Px4LocalSetpoint

# 比较浮点限值时留出的余量，避免恰好在门限上的合法指令被误拒。
LIMIT_EPSILON = 1e-9
# 超过该间隔没有调用 calculate()，视为停发：清空斜率状态，而不是补发积累的额度。
MAX_STEP_GAP_S = 0.5
# 首帧没有上一帧时间戳，按一个典型控制周期估算 dt。
INITIAL_STEP_S = 0.02

# 这些配置项必须是有限正数；集中列出便于统一校验。
POSITIVE_LIMITS = (
    "max_velocity",
    "max_command_velocity",
    "max_acceleration",
    "max_position_error",
    "max_feedback_velocity_error",
    "max_tilt_rad",
    "alignment_tolerance_rad",
    "alignment_tilt_rad",
    "reset_position_margin",
)


@dataclass(frozen=True)
class VelocityConfig:
    """速度导引参数。

    字段名属于配置契约（velocity_hardware.yaml 与测试），保持 snake_case。
    位置环为 PD 律：``v_target = v_d + kp·(p_d − p) + kv·(v_d − v)``。
    """

    kp: tuple = (2.0, 2.0, 2.0)
    kv: tuple = (1.0, 1.0, 1.0)
    max_velocity: float = 2.0  # 允许的机体反馈速度上限（m/s）
    max_command_velocity: float = 0.3  # 下发给 PX4 的速度上限（m/s）
    max_acceleration: float = 0.3  # 速度指令的加速度包络（m/s²）
    max_position_error: float = 1.0  # 位置误差超过此值拒绝控制（m）
    max_feedback_velocity_error: float = 0.3  # VIO 与 PX4 速度一致性门限（m/s）
    max_tilt_rad: float = 0.3490658504  # 20°，超过则拒绝继续导引
    alignment_samples: int = 10  # 冻结 yaw 参考所需连续样本数
    alignment_tolerance_rad: float = 0.0872664626  # 5°
    alignment_tilt_rad: float = 0.1745329252  # 10°
    reset_position_margin: float = 0.2  # 位置跳变检测的额外余量（m）
    acceleration_feedforward: bool = False  # 是否把期望加速度透传给 PX4

    def __post_init__(self):
        # 位置增益必须为正；速度增益允许为零（纯位置比例环）。
        kp = finiteVector(self.kp, 3)
        kv = finiteVector(self.kv, 3)
        if np.any(kp <= 0) or np.any(kv < 0):
            raise ValueError("velocity_guidance_kp")

        for name in POSITIVE_LIMITS:
            value = getattr(self, name)
            # 故意用 type() 而不是 isinstance()：拒绝 bool 与 NumPy 标量混入配置。
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(name)

        if (
            self.max_command_velocity > self.max_velocity
            or self.max_tilt_rad >= math.pi / 2
            or self.alignment_tilt_rad >= math.pi / 2
            or type(self.alignment_samples) is not int
            or self.alignment_samples < 2
            or type(self.acceleration_feedforward) is not bool
        ):
            raise ValueError("velocity_guidance_bounds")


@dataclass(frozen=True)
class VelocitySetpoint:
    """速度导引输出，位于 VIO 世界系（尚未转换到 PX4 的 NED）。

    velocity_m_s、acceleration_m_s2 的单位分别为 m/s、m/s²；yaw_rad 是该世界系的偏航。
    没有 thrust 字段：姿态与推力分配由 PX4 完成。
    """

    velocity_m_s: tuple
    acceleration_m_s2: tuple
    yaw_rad: float


class VelocityGuidance(AttitudeReference):
    """位置误差 → 速度参考，复用 AttitudeReference 的对齐与断流检查。

    与 AttitudeController 的分工：这里只产生速度（可选加速度前馈），姿态和推力由
    PX4 速度环内部计算。outputVelocity/outputAt 保存跨调用的斜率状态，停发或失效时
    由 resetOutput() 清空，防止在旧速度上继续累加。
    """

    def __init__(self, config=VelocityConfig(), **kwargs):
        super().__init__(config, **kwargs)
        self.resetOutput()

    def resetOutput(self) -> None:
        """把速度斜率状态复位到静止；下一步从零重新加速。"""
        self.outputVelocity = np.zeros(3)
        self.outputAt = None

    def invalidate(self, reason):
        super().invalidate(reason)
        self.resetOutput()

    def calculate(self, desired, position, velocity, qVio, *, now, px4Local=False):
        """计算一个控制周期的有界速度参考。

        Args:
            desired: 期望轨迹快照，含 position_m、velocity_m_s、acceleration_m_s2、yaw_rad。
            position: 当前世界系位置（m）。
            velocity: 当前世界系速度（m/s）。
            qVio: VIO 世界系到机体的单位四元数 (x, y, z, w)，用于倾角保护。
            now: 单调时钟秒；用实际调用间隔限制加速度，不按名义周期。
            px4Local: 反馈来自 PX4 本地估计时为 True，可跳过 VIO 对齐参考。

        Returns:
            VelocitySetpoint，仍位于 VIO 世界系（ENU），由 velocityToNed() 转到 PX4。

        Raises:
            ValueError: 参考未就绪/已失效、输入越界、机体倾斜过大或停发时间过长。
        """
        if self.latched:
            raise ValueError(self.latched)
        if not (px4Local or self.ready):
            raise ValueError("velocity_reference_pending")
        if not math.isfinite(now):
            raise ValueError("velocity_command_clock")

        cfg = self.config
        currentPosition = finiteVector(position, 3)
        currentVelocity = finiteVector(velocity, 3)
        targetPosition = finiteVector(desired.position_m, 3)
        targetVelocity = finiteVector(desired.velocity_m_s, 3)
        feedforwardAcceleration = finiteVector(desired.acceleration_m_s2, 3)

        self._checkCommandLimits(
            desired,
            cfg,
            currentPosition,
            currentVelocity,
            targetPosition,
            targetVelocity,
            feedforwardAcceleration,
        )
        # 机体倾斜过大时宁可报错，也不发出可能让 PX4 误解释的速度指令。
        if rotation(qVio)[2, 2] < math.cos(cfg.max_tilt_rad):
            raise ValueError("velocity_vehicle_tilt")

        # 位置 PD：位置误差给出趋近速度，速度误差提供阻尼。
        commandVelocity = (
            targetVelocity
            + np.asarray(cfg.kp) * (targetPosition - currentPosition)
            + np.asarray(cfg.kv) * (targetVelocity - currentVelocity)
        )
        commandVelocity = _clampMagnitude(commandVelocity, cfg.max_command_velocity)

        # 用实际调用间隔限制加速度；首帧按 INITIAL_STEP_S 估算。
        dt = INITIAL_STEP_S if self.outputAt is None else now - self.outputAt
        if not 0 < dt <= MAX_STEP_GAP_S:
            self.resetOutput()
            raise ValueError("velocity_output_gap_or_reset")

        velocityStep = commandVelocity - self.outputVelocity
        # 窗口内只按实际间隔限速；长停发不得积累一次跳变额度。
        velocityStep = _clampMagnitude(velocityStep, cfg.max_acceleration * dt)
        self.outputVelocity = self.outputVelocity + velocityStep
        self.outputAt = now

        # 加速度前馈默认关闭：开启后 PX4 使用 velocity_acceleration 的 type_mask。
        acceleration = (
            tuple(feedforwardAcceleration) if cfg.acceleration_feedforward else (0.0, 0.0, 0.0)
        )
        return VelocitySetpoint(tuple(self.outputVelocity), acceleration, desired.yaw_rad)

    @staticmethod
    def _checkCommandLimits(
        desired, cfg, position, velocity, targetPosition, targetVelocity, feedforwardAcceleration
    ) -> None:
        """拒绝越界指令；各项上限保证下发给 PX4 的速度可解释且可追踪。"""
        if (
            np.linalg.norm(targetPosition - position) > cfg.max_position_error
            or np.linalg.norm(velocity) > cfg.max_velocity
            or np.linalg.norm(targetVelocity) > cfg.max_command_velocity + LIMIT_EPSILON
            or np.linalg.norm(feedforwardAcceleration) > cfg.max_acceleration + LIMIT_EPSILON
            or not math.isfinite(desired.yaw_rad)
        ):
            raise ValueError("velocity_command_limits")


def _clampMagnitude(vector, maxMagnitude):
    """按欧氏范数等比缩放向量；范数未超限或为零时原样返回。"""
    vector = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm <= maxMagnitude or norm == 0.0:
        return vector
    return vector * (maxMagnitude / norm)


def velocityToNed(setpoint, yawOffset) -> Px4LocalSetpoint:
    """把 VIO 世界系（ENU）速度参考转换到 PX4 局部 NED。

    冻结的 yaw 参考 ``yawOffset`` 表示 VIO 世界系相对 PX4 ENU 的水平旋转；
    这里只做旋转，不改变位置原点（位置字段固定为 0，type_mask 会忽略位置）。
    """
    if not math.isfinite(yawOffset):
        raise ValueError("velocity_yaw_reference")

    # 先按 offset 把 VIO 世界系转到 PX4 ENU，再从 ENU 转到 NED。
    enuToNed = NED_TO_ENU.T
    vioToPx4Enu = yawRotation(yawOffset)
    transform = enuToNed @ vioToPx4Enu

    velocity = transform @ finiteVector(setpoint.velocity_m_s, 3)
    acceleration = transform @ finiteVector(setpoint.acceleration_m_s2, 3)
    # ENU 偏航角 θ 与 NED 偏航角 ψ 的关系：ψ = π/2 − θ。
    yaw = wrap(math.pi / 2 - setpoint.yaw_rad - yawOffset)
    return Px4LocalSetpoint(
        position_m=(0.0, 0.0, 0.0),
        velocity_m_s=tuple(velocity),
        acceleration_m_s2=tuple(acceleration),
        yaw_rad=yaw,
        yaw_rate_rad_s=0.0,
    )
