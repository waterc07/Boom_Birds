"""本轮修复 1 的针对性回归：**完整** setpoint 换算（非零 yaw_offset 下各字段方向一致）。

旧实现只对 position 施加了航向旋转，velocity/acceleration/yaw/yaw_rate 仍走零偏移轴映射，
因此下面的用例在旧实现上必然失败：
- `test_velocity_and_acceleration_follow_the_same_rotation_as_position`
- `test_yaw_includes_the_declared_offset`
- `test_heading_from_transformed_velocity_matches_transformed_yaw`（自洽性总检，
  但旧实现的两项错误可相互抵消，单独使用没有判别力）

自洽性检验的物理含义：ROS 里以 yaw=ψ 飞行的速度，转成 NED 后其朝向必须等于由
ψ 换算出的 NED yaw。显式期望值断言分别锁定旋转矩阵和偏航偏移；
仅靠两条路径互相比较会漏掉两者同时错位的旧实现。
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from boom_birds_nav.px4_frames import (
    LocalFrameAlignment,
    RosLocalSetpoint,
    ned_to_ros_local_setpoint,
    normalize_angle_pi,
    ros_local_to_ned_setpoint,
    ros_local_to_px4_ned,
)


def _sp(x=1.0, y=2.0, z=3.0, vx=1.0, vy=0.0, vz=0.5, ax=0.0, ay=0.0, az=0.0,
        yaw=0.0, yaw_dot=0.0):
    return RosLocalSetpoint(
        position_m=(x, y, z),
        velocity_m_s=(vx, vy, vz),
        acceleration_m_s2=(ax, ay, az),
        yaw_rad=yaw,
        yaw_dot_rad_s=yaw_dot,
    )


# ============================================================ 零偏移：必须与旧路径一致
def test_zero_offset_matches_legacy_conversion_field_by_field():
    """yaw_offset=0、translation=0 时，新函数与旧 `ros_local_to_px4_ned` 逐字段相同。"""
    sp = _sp(yaw=0.3, yaw_dot=0.7)
    legacy = ros_local_to_px4_ned(sp)
    aligned = ros_local_to_ned_setpoint(sp, alignment=LocalFrameAlignment())
    assert aligned.position_m == pytest.approx(legacy.position_m)
    assert aligned.velocity_m_s == pytest.approx(legacy.velocity_m_s)
    assert aligned.acceleration_m_s2 == pytest.approx(legacy.acceleration_m_s2)
    assert aligned.yaw_rad == pytest.approx(legacy.yaw_rad)
    assert aligned.yaw_rate_rad_s == pytest.approx(legacy.yaw_rate_rad_s)


# ============================================================ 非零偏移：四个分量
@pytest.mark.parametrize("offset_deg", [30.0, -30.0, 90.0, 5.0, -175.0])
def test_velocity_and_acceleration_follow_the_same_rotation_as_position(offset_deg):
    """速度/加速度必须与位置走**同一个** R(φ)：旧实现只转位置，这里必然不一致。

    取位置与速度相同方向：左目/世界系 v=(1,0,0)、p=(1,0,0)，则
    p_ned 与 v_ned 必须平行且同向（平移不影响方向，这里 translation=0）。
    """
    offset = math.radians(offset_deg)
    align = LocalFrameAlignment(yaw_offset_rad=offset)
    sp = _sp(x=1.0, y=0.0, z=0.0, vx=1.0, vy=0.0, vz=0.0,
             ax=1.0, ay=0.0, az=0.0, yaw=0.0)
    out = ros_local_to_ned_setpoint(sp, alignment=align)

    r = align.rotation()
    expected = r @ np.array([1.0, 0.0, 0.0])
    assert out.velocity_m_s == pytest.approx(expected, abs=1e-12), "速度未按 R(φ) 旋转"
    assert out.acceleration_m_s2 == pytest.approx(expected, abs=1e-12), "加速度未按 R(φ) 旋转"
    assert out.position_m == pytest.approx(expected, abs=1e-12)

    # 旧实现的错误特征：速度仍等于零偏移轴映射
    stale = np.array([1.0, -0.0, -0.0])
    if abs(offset_deg) > 1e-9:
        assert not np.allclose(out.velocity_m_s, stale, atol=1e-9), (
            "速度看起来还走的是零偏移轴映射（旧实现的 bug）"
        )


def test_zero_offset_fingerprints_are_absent_when_offset_is_nonzero():
    """直接排除旧实现的指纹：速度等于零偏移轴映射、或偏航等于 −yaw_ros。

    旧实现下这两条必然命中，因此这是**有判别力**的回归（自洽性检查做不到）。
    """
    offset = math.radians(40.0)
    yaw = math.radians(15.0)
    align = LocalFrameAlignment(yaw_offset_rad=offset)
    out = ros_local_to_ned_setpoint(
        _sp(x=1.0, y=0.0, z=0.0, vx=1.0, vy=0.0, vz=0.0, ax=1.0, ay=0.0, az=0.0, yaw=yaw),
        alignment=align,
    )
    axis_only = np.array([1.0, -0.0, -0.0])
    assert not np.allclose(out.velocity_m_s, axis_only, atol=1e-9), (
        "速度仍等于零偏移轴映射 diag(1,-1,-1)·v —— 旧实现的指纹"
    )
    assert not np.allclose(out.acceleration_m_s2, axis_only, atol=1e-9), (
        "加速度仍等于零偏移轴映射 —— 旧实现的指纹"
    )
    assert abs(out.yaw_rad - (-yaw)) > 1e-9, (
        "偏航等于 −yaw_ros（漏掉 yaw_offset）—— 旧实现的指纹"
    )
    assert out.yaw_rad == pytest.approx(-(yaw + offset), abs=1e-12)


def test_yaw_includes_the_declared_offset():
    """目标偏航必须是 −(yaw_ros + φ)；旧实现是 −yaw_ros（漏掉 φ）。"""
    offset = math.radians(30.0)
    align = LocalFrameAlignment(yaw_offset_rad=offset)
    out = ros_local_to_ned_setpoint(_sp(yaw=math.radians(10.0)), alignment=align)
    assert out.yaw_rad == pytest.approx(-math.radians(40.0), abs=1e-12)


def test_yaw_rate_sign_is_flipped_and_has_no_offset():
    """yaw_rate 是角速度（赝矢量）：只翻符号，不加偏移、不减平移。"""
    align = LocalFrameAlignment(yaw_offset_rad=math.radians(30.0),
                               translation_m=(5.0, 6.0, 7.0))
    out = ros_local_to_ned_setpoint(_sp(yaw_dot=1.5), alignment=align)
    assert out.yaw_rate_rad_s == pytest.approx(-1.5, abs=1e-12)


def test_translation_only_affects_position():
    """平移**只**作用于位置：速度/加速度/偏航在有无平移时必须完全相同。"""
    sp = _sp(yaw=0.2, yaw_dot=0.3)
    base = ros_local_to_ned_setpoint(sp, alignment=LocalFrameAlignment())
    moved = ros_local_to_ned_setpoint(
        sp, alignment=LocalFrameAlignment(translation_m=(10.0, -20.0, 30.0))
    )
    assert moved.position_m != pytest.approx(base.position_m), "位置应受平移影响"
    assert moved.velocity_m_s == pytest.approx(base.velocity_m_s)
    assert moved.acceleration_m_s2 == pytest.approx(base.acceleration_m_s2)
    assert moved.yaw_rad == pytest.approx(base.yaw_rad)
    assert moved.yaw_rate_rad_s == pytest.approx(base.yaw_rate_rad_s)


def test_translation_is_subtracted_before_rotation():
    """平移定义在 ROS 系：先减平移再旋转。"""
    align = LocalFrameAlignment(translation_m=(1.0, 0.0, 0.0))
    out = ros_local_to_ned_setpoint(
        _sp(x=3.0, y=0.0, z=0.0, vx=0.0, vy=0.0, vz=0.0), alignment=align
    )
    assert out.position_m == pytest.approx((2.0, 0.0, 0.0), abs=1e-12)


# ============================================================ 自洽性总检
@pytest.mark.parametrize("offset_deg", [0.0, 30.0, -45.0, 90.0, 180.0, -170.0])
@pytest.mark.parametrize("yaw_deg", [0.0, 30.0, 90.0, 179.0, -179.0])
def test_heading_from_transformed_velocity_matches_transformed_yaw(offset_deg, yaw_deg):
    """核心自洽性：速度旋转后的**实际朝向**必须等于偏航换算的结果。

    两条路径独立推导：
      * 路径 A：把"以 yaw=ψ 飞行的速度"按 R(φ) 旋转，再看它在 NED 里的朝向；
      * 路径 B：直接算 yaw_ned = −(ψ + φ)。
    A 与 B 必须一致——否则同一个 setpoint 里"速度指向"和"目标朝向"互相矛盾。

    **判别力说明（实测）**：这条自洽性检查对旧实现（只转位置、其余走零偏移轴映射）
    **没有判别力**。旧实现是"轴映射 + −ψ"一对同时错位，两边恰好抵消，
    所以恒等式在旧代码上也成立（25 组 offset/yaw 组合全部"通过"）。
    真正能抓住旧实现的是显式期望值断言：
    `test_velocity_and_acceleration_follow_the_same_rotation_as_position` 与
    `test_yaw_includes_the_declared_offset`。保留本条是为了锁住"两条独立路径不矛盾"，
    不可用它替代前者。
    """
    offset = math.radians(offset_deg)
    yaw = math.radians(yaw_deg)
    align = LocalFrameAlignment(yaw_offset_rad=offset)
    # ROS 中 yaw=ψ 的机头方向即速度方向
    sp = _sp(vx=math.cos(yaw), vy=math.sin(yaw), vz=0.0,
             ax=0.0, ay=0.0, az=0.0, yaw=yaw)
    out = ros_local_to_ned_setpoint(sp, alignment=align)

    vx, vy = out.velocity_m_s[0], out.velocity_m_s[1]
    path_a = math.atan2(vy, vx)                      # NED 中朝向（NED yaw 定义）
    path_b = normalize_angle_pi(out.yaw_rad)
    # ±180° 是同一个方向，必须按圆周差比较，不能直接比数值
    delta = normalize_angle_pi(path_a - path_b)
    assert abs(delta) < 1e-9, (
        f"速度朝向 {math.degrees(path_a):.6f} 与偏航换算 "
        f"{math.degrees(path_b):.6f} 不一致（offset={offset_deg}, yaw={yaw_deg}）"
    )


# ============================================================ 反向转换
@pytest.mark.parametrize("offset_deg", [0.0, 30.0, -90.0, 120.0])
def test_round_trip_is_exact_including_yaw(offset_deg):
    """正反变换必须互逆（含偏航偏移）；旧的反向路径不带 offset，会在这里失败。"""
    align = LocalFrameAlignment(yaw_offset_rad=math.radians(offset_deg),
                               translation_m=(0.5, -1.5, 2.0))
    sp = _sp(x=1.0, y=2.0, z=3.0, vx=0.1, vy=0.2, vz=0.3,
             ax=0.01, ay=0.02, az=0.03, yaw=math.radians(20.0), yaw_dot=0.4)
    ned = ros_local_to_ned_setpoint(sp, alignment=align)
    back = ned_to_ros_local_setpoint(ned, alignment=align)
    assert back.position_m == pytest.approx(sp.position_m, abs=1e-12)
    assert back.velocity_m_s == pytest.approx(sp.velocity_m_s, abs=1e-12)
    assert back.acceleration_m_s2 == pytest.approx(sp.acceleration_m_s2, abs=1e-12)
    assert back.yaw_rad == pytest.approx(sp.yaw_rad, abs=1e-12)
    assert back.yaw_dot_rad_s == pytest.approx(sp.yaw_dot_rad_s, abs=1e-12)


def test_nonzero_offset_rejects_unwrapable_yaw_when_strict():
    """|yaw + offset| > π 且禁止归一化时必须报错（不能静默绕圈）。"""
    from boom_birds_nav.px4_frames import FrameValidationError

    align = LocalFrameAlignment(yaw_offset_rad=math.radians(120.0))
    with pytest.raises(FrameValidationError):
        ros_local_to_ned_setpoint(_sp(yaw=math.radians(120.0)), alignment=align,
                                  normalize_yaw=False)
    # 允许归一化时正常返回，且落在 (−π, π]
    out = ros_local_to_ned_setpoint(_sp(yaw=math.radians(120.0)), alignment=align,
                                    normalize_yaw=True)
    assert -math.pi <= out.yaw_rad <= math.pi
