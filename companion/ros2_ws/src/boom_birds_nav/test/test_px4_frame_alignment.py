"""P1-2 回归：局部系对齐的数学 + 位置闸门。

覆盖四件事：
1. `identity` 对齐下位置换算与原来的纯轴变换**逐位一致**（不能悄悄改掉既有语义）。
2. `declared` 带非零 yaw 偏移/平移时，换算确实是完整刚体变换，且可逆往返。
3. `YawAlignmentResidual` 的核实语义：同向→通过；航向差超容差→不通过；倾斜/角速率大→不采样。
4. 闸门：对齐未核实 ⇒ 位置与速度都不发（避免"只发速度"换个形式继续误导），
   且原因码为 `local_frame_not_aligned`、状态为 STOPPED。
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from boom_birds_nav.px4_frames import (
    LocalFrameAlignment,
    RosLocalSetpoint,
    YawAlignmentResidual,
    ros_local_to_ned_setpoint,
    ros_local_to_px4_ned,
    vector_ros_local_to_ned,
)


def _sp(x=1.0, y=2.0, z=3.0, vx=0.1, vy=0.2, vz=0.3, ax=0.0, ay=0.0, az=0.0,
        yaw=0.0, yaw_dot=0.0):
    return RosLocalSetpoint(
        position_m=(x, y, z),
        velocity_m_s=(vx, vy, vz),
        acceleration_m_s2=(ax, ay, az),
        yaw_rad=yaw,
        yaw_dot_rad_s=yaw_dot,
    )


# ------------------------------------------------------------------ 1) identity
def test_identity_alignment_position_equals_plain_axis_conversion():
    """identity 对齐必须与历史行为完全一致（否则等于悄悄改语义）。"""
    sp = _sp()
    identity = LocalFrameAlignment()
    via_alignment = identity.position_ros_to_ned(sp.position_m)
    via_axis = vector_ros_local_to_ned(sp.position_m)
    assert np.array_equal(via_alignment, via_axis)
    assert list(via_alignment) == pytest.approx([1.0, -2.0, -3.0])


def test_identity_alignment_is_flagged_as_identity_transform():
    assert LocalFrameAlignment().is_identity_transform() is True
    assert LocalFrameAlignment(yaw_offset_rad=0.1).is_identity_transform() is False
    assert LocalFrameAlignment(translation_m=(0.0, 0.0, 1.0)).is_identity_transform() is False


def _feed(residual, count: int) -> None:
    """喂 count 个"两系同向、水平、不转"的合格样本。"""
    for _ in range(count):
        residual.observe(ros_yaw_rad=0.0, px4_yaw_rad=0.0, roll_rad=0.0,
                         pitch_rad=0.0, yaw_rate_rad_s=0.0)


# ------------------------------------------------------------------ 2) 刚体变换
def test_declared_yaw_offset_rotates_the_target_position():
    """yaw 偏移是绕"下"轴的旋转：ROS +x（前）在偏移 +90° 后指向 NED 的 -E。"""
    align = LocalFrameAlignment(yaw_offset_rad=math.pi / 2)
    out = align.position_ros_to_ned((1.0, 0.0, 0.0))
    assert out == pytest.approx([0.0, -1.0, 0.0], abs=1e-12)
    assert align.position_ros_to_ned((0.0, 0.0, 5.0)) == pytest.approx(
        [0.0, 0.0, -5.0], abs=1e-12
    )


def test_declared_translation_is_subtracted_before_rotation():
    """平移定义在 ROS 系：先减平移再旋转。"""
    align = LocalFrameAlignment(translation_m=(1.0, 0.0, 0.0))
    out = align.position_ros_to_ned((3.0, 0.0, 0.0))
    assert out == pytest.approx([2.0, 0.0, 0.0], abs=1e-12)
    assert align.has_translation() is True


def test_alignment_round_trip_is_lossless():
    align = LocalFrameAlignment(yaw_offset_rad=-0.7, translation_m=(0.5, -1.25, 2.0))
    for p in ((0.0, 0.0, 0.0), (1.0, 2.0, 3.0), (-4.5, 0.25, -6.0)):
        ned = align.position_ros_to_ned(p)
        back = align.position_ned_to_ros(ned)
        assert back == pytest.approx(list(p), abs=1e-12)


def test_alignment_rejects_nonfinite_input():
    from boom_birds_nav.px4_frames import FrameValidationError

    align = LocalFrameAlignment()
    with pytest.raises(FrameValidationError):
        align.position_ros_to_ned((float("nan"), 0.0, 0.0))
    with pytest.raises(FrameValidationError):
        align.position_ned_to_ros((float("inf"), 0.0, 0.0))


def test_rotation_is_proper_and_norm_preserving():
    """任意 yaw 偏移下仍是正交、det=+1、保范数的真旋转（不是镜像）。"""
    for yaw in (0.0, 0.3, math.pi / 2, -2.1, math.pi):
        align = LocalFrameAlignment(yaw_offset_rad=yaw)
        r = align.rotation()
        assert np.allclose(r @ r.T, np.eye(3), atol=1e-12)
        assert np.linalg.det(r) == pytest.approx(1.0, abs=1e-12)
        v = np.array([1.3, -2.7, 0.9])
        assert np.linalg.norm(r @ v) == pytest.approx(np.linalg.norm(v), rel=1e-12)


def test_ros_local_to_px4_ned_still_uses_pure_axis_flip():
    """`ros_local_to_px4_ned` 保持纯轴变换语义；对齐由调用方单独施加（职责分离）。"""
    out = ros_local_to_px4_ned(_sp())
    assert out.position_m == pytest.approx((1.0, -2.0, -3.0))
    assert out.yaw_rad == pytest.approx(0.0)


# ------------------------------------------------------------------ 3) 核实语义
def test_residual_verifies_when_frames_agree():
    r = YawAlignmentResidual(tolerance_rad=math.radians(5), required_samples=5)
    for _ in range(5):
        assert r.observe(ros_yaw_rad=0.3, px4_yaw_rad=-0.3, roll_rad=0.0, pitch_rad=0.0,
                             yaw_rate_rad_s=0.0, declared_yaw_offset_rad=0.0) is True
    assert r.samples == 5
    assert r.yaw_verified is True
    assert r.mean_residual_rad == pytest.approx(0.0, abs=1e-12)


def test_residual_rejects_when_headings_differ_beyond_tolerance():
    """两系航向差 30° 远超 5° 容差 ⇒ 不得核实通过（这正是"位置会落错地方"的情形）。"""
    r = YawAlignmentResidual(tolerance_rad=math.radians(5), required_samples=5)
    for _ in range(5):
        r.observe(ros_yaw_rad=math.radians(30), px4_yaw_rad=0.0, roll_rad=0.0,
                             pitch_rad=0.0, yaw_rate_rad_s=0.0,
                             declared_yaw_offset_rad=0.0)
    assert r.samples == 5
    assert r.yaw_verified is False
    assert abs(r.mean_residual_rad) == pytest.approx(math.radians(30), rel=1e-6)


def test_residual_accepts_when_declared_offset_matches_the_difference():
    """声明的偏移量正确时残差为 0：declared 模式的正确用法。

    约定：declared_offset 是 ROS 局部系 +x 相对 PX4 +x 的航向差。
    一致时的关系为 px4_yaw = -(ros_yaw + declared_offset)。
    取 (ros_yaw, px4_yaw) = (0, -offset) 时的残差应为 0。
    """
    r = YawAlignmentResidual(tolerance_rad=math.radians(5), required_samples=5)
    for _ in range(5):
        assert r.observe(ros_yaw_rad=0.0, px4_yaw_rad=math.radians(-30), roll_rad=0.0,
                             pitch_rad=0.0, yaw_rate_rad_s=0.0,
                             declared_yaw_offset_rad=math.radians(30)) is True
    assert r.mean_residual_rad == pytest.approx(0.0, abs=1e-9)
    assert r.yaw_verified is True


@pytest.mark.parametrize(
    ("ros_deg", "offset_deg"),
    [(0.0, 30.0), (30.0, 0.0), (30.0, 20.0), (179.0, 15.0), (-179.0, -15.0)],
)
def test_residual_accepts_sent_heading_and_rejects_opposite_sign(ros_deg, offset_deg):
    """真实 PX4 yaw 必须按发送方向核实；旧残差公式会把符号反的航向判为通过。"""
    ros_yaw = math.radians(ros_deg)
    offset = math.radians(offset_deg)
    sent = ros_local_to_ned_setpoint(
        _sp(yaw=ros_yaw), LocalFrameAlignment(yaw_offset_rad=offset)
    ).yaw_rad
    residual = YawAlignmentResidual(tolerance_rad=math.radians(5), required_samples=1)
    assert residual.observe(
        ros_yaw_rad=ros_yaw, px4_yaw_rad=sent,
        declared_yaw_offset_rad=offset, roll_rad=0.0, pitch_rad=0.0,
        yaw_rate_rad_s=0.0,
    )
    assert residual.yaw_verified
    if abs(math.sin(sent)) > 0.1:
        residual.reset()
        assert residual.observe(
            ros_yaw_rad=ros_yaw, px4_yaw_rad=-sent,
            declared_yaw_offset_rad=offset, roll_rad=0.0, pitch_rad=0.0,
            yaw_rate_rad_s=0.0,
        )
        assert not residual.yaw_verified


def test_residual_rejects_when_declared_offset_does_not_match():
    """声明错了偏移量（差 20°）必须核实失败——否则等于把闸门交给配置。"""
    r = YawAlignmentResidual(tolerance_rad=math.radians(5), required_samples=5)
    for _ in range(5):
        r.observe(ros_yaw_rad=0.0, px4_yaw_rad=math.radians(-50), roll_rad=0.0,
                             pitch_rad=0.0, yaw_rate_rad_s=0.0,
                             declared_yaw_offset_rad=math.radians(30))
    assert r.yaw_verified is False
    assert abs(r.mean_residual_rad) == pytest.approx(math.radians(20), rel=1e-6)


def test_residual_needs_enough_samples():
    r = YawAlignmentResidual(tolerance_rad=math.radians(5), required_samples=5)
    _feed(r, 4)
    assert r.samples == 4
    assert r.yaw_verified is False, "样本不足不得核实通过"
    _feed(r, 1)
    assert r.samples == 5
    assert r.yaw_verified is True


def test_residual_skips_samples_while_tilted_or_turning():
    """倾斜或角速率大时偏航残差不可信，必须不采样。"""
    r = YawAlignmentResidual(tolerance_rad=math.radians(5), required_samples=1)
    assert r.observe(ros_yaw_rad=0.0, px4_yaw_rad=0.0, roll_rad=0.6,
                     pitch_rad=0.0, yaw_rate_rad_s=0.0) is False
    assert r.observe(ros_yaw_rad=0.0, px4_yaw_rad=0.0, roll_rad=0.0,
                     pitch_rad=-0.6, yaw_rate_rad_s=0.0) is False
    assert r.observe(ros_yaw_rad=0.0, px4_yaw_rad=0.0, roll_rad=0.0,
                     pitch_rad=0.0, yaw_rate_rad_s=1.0) is False
    assert r.samples == 0
    assert r.yaw_verified is False


def test_residual_rejects_nonfinite_and_resets():
    r = YawAlignmentResidual(tolerance_rad=math.radians(5), required_samples=1)
    assert r.observe(ros_yaw_rad=float("nan"), px4_yaw_rad=0.0, roll_rad=0.0,
                     pitch_rad=0.0, yaw_rate_rad_s=0.0) is False
    assert r.observe(ros_yaw_rad=0.0, px4_yaw_rad=float("inf"), roll_rad=0.0,
                     pitch_rad=0.0, yaw_rate_rad_s=0.0) is False
    r.observe(ros_yaw_rad=0.1, px4_yaw_rad=0.1, roll_rad=0.0, pitch_rad=0.0,
              yaw_rate_rad_s=0.0)
    assert r.samples == 1
    r.reset()
    assert r.samples == 0 and r.yaw_verified is False


def test_residual_handles_yaw_wrap_around():
    """正负 pi 交界处绕圈不能把残差算成 2pi（否则永远核实不过）。"""
    r = YawAlignmentResidual(tolerance_rad=math.radians(5), required_samples=3)
    for _ in range(3):
        r.observe(ros_yaw_rad=-math.pi + 0.01, px4_yaw_rad=math.pi - 0.01, roll_rad=0.0,
                  pitch_rad=0.0, yaw_rate_rad_s=0.0)
    assert r.yaw_verified is True
    assert abs(r.mean_residual_rad) < math.radians(5)


def test_residual_spread_rejects_moving_measurements():
    r = YawAlignmentResidual(tolerance_rad=math.radians(5), required_samples=3,
                             max_sample_spread_rad=math.radians(1))
    _feed(r, 1)
    r.observe(ros_yaw_rad=math.radians(3), px4_yaw_rad=0.0, roll_rad=0.0,
              pitch_rad=0.0, yaw_rate_rad_s=0.0)
    r.observe(ros_yaw_rad=math.radians(-3), px4_yaw_rad=0.0, roll_rad=0.0,
              pitch_rad=0.0, yaw_rate_rad_s=0.0)
    assert r.spread_rad == pytest.approx(math.radians(6), rel=1e-6)
    assert r.yaw_verified is False, "样本不集中时不得核实通过"


def test_residual_dict_is_json_safe():
    import json

    r = YawAlignmentResidual(tolerance_rad=0.1, required_samples=2)
    r.observe(ros_yaw_rad=0.0, px4_yaw_rad=0.0, roll_rad=0.0, pitch_rad=0.0,
              yaw_rate_rad_s=0.0)
    json.dumps(r.to_dict())


def test_alignment_dict_is_json_safe():
    import json

    json.dumps(LocalFrameAlignment(yaw_offset_rad=0.2,
                                   translation_m=(1.0, 2.0, 3.0)).to_dict())


# ------------------------------------------------------------------ 4) 闸门
def _core_with_gate(allow: bool, reason: str):
    """用最小替身构造 Core，只为验证闸门语义（不依赖 ROS）。"""
    from boom_birds_nav.px4_failsafe import FailsafeConfig, SignalId
    from boom_birds_nav.px4_interface_node import Px4InterfaceCore

    class _Backend:
        def __init__(self):
            self.sent = []

        def connect(self):
            return True

        def close(self):
            return None

        def is_connected(self):
            return True

        def link_state(self):
            return {"connected": True, "heartbeat_age_s": 0.0,
                    "heartbeat_timeout_s": 1.0, "counters": {}, "restart_epoch": 0}

        def read_vehicle_state(self):
            return None

        def stream_diagnostics(self):
            return {"connected": True, "heartbeat_age_s": 0.0,
                    "heartbeat_timeout_s": 1.0, "counters": {}}

        def send_setpoint(self, setpoint, type_mask=None):
            self.sent.append((setpoint, type_mask))
            return True

    backend = _Backend()
    timeouts = {
        SignalId.SETPOINT: 0.15,
        SignalId.VIO_POSE: 0.15,
        SignalId.IMU: 0.5,
        SignalId.CAMERA: 1.0,
        SignalId.MAVLINK_LINK: 1.0,
        SignalId.PX4_HEARTBEAT: 1.0,
        SignalId.ODOM_EGO: 0.15,
    }
    config = FailsafeConfig(timeouts=timeouts, recovery_fresh_samples=1,
                            require_px4_identity=False)
    core = Px4InterfaceCore(
        backend, config,
        position_gate=lambda: (allow, reason, [] if allow else [reason]),
    )
    return core, backend


def test_gate_blocks_position_and_velocity_when_alignment_unverified():
    """对齐未核实 ⇒ 一个 setpoint 都不发（含速度），原因码明确为 local_frame_not_aligned。"""
    core, backend = _core_with_gate(False, "local_frame_not_aligned")
    cmd = type("C", (), {
        "position": type("P", (), {"x": 1.0, "y": 2.0, "z": 3.0})(),
        "velocity": type("V", (), {"x": 0.1, "y": 0.2, "z": 0.3})(),
        "acceleration": type("A", (), {"x": 0.0, "y": 0.0, "z": 0.0})(),
        "yaw": 0.0, "yaw_dot": 0.0, "trajectory_id": 9, "trajectory_flag": 1,
        "header": type("H", (), {"frame_id": "world"})(),
    })()
    now = 1000.0
    for _ in range(8):
        core.monitor.note_vio_pose(now)
        core.monitor.note_odom(now)
        core.monitor.note_imu(now)
        core.monitor.note_camera(now)
        core.monitor.note_mavlink_link(now)
        core.on_position_cmd(cmd, now)
        now += 0.02
    out = core.step(cmd, now, frame_id_ok=True)
    assert out.allow_setpoint is False, "对齐未核实必须拦住"
    assert backend.sent == [], f"不得发出任何 setpoint，实际 {backend.sent}"
    codes = [r.get("code") for r in out.reasons]
    assert "local_frame_not_aligned" in codes, f"原因码缺失：{codes}"
    assert out.detail.get("alignment") == "local_frame_not_aligned"
    assert core.counters["position_blocked_by_alignment"] >= 1
    assert core.counters["last_alignment_reason"] == "local_frame_not_aligned"


def test_gate_allows_setpoint_when_alignment_verified():
    """核实通过后链路必须照常工作（闸门不能把正常路径也堵死）。"""
    core, backend = _core_with_gate(True, "frame_alignment_verified")
    cmd = type("C", (), {
        "position": type("P", (), {"x": 1.0, "y": 2.0, "z": 3.0})(),
        "velocity": type("V", (), {"x": 0.0, "y": 0.0, "z": 0.0})(),
        "acceleration": type("A", (), {"x": 0.0, "y": 0.0, "z": 0.0})(),
        "yaw": 0.0, "yaw_dot": 0.0, "trajectory_id": 9, "trajectory_flag": 1,
        "header": type("H", (), {"frame_id": "world"})(),
    })()
    now = 1000.0
    for _ in range(8):
        core.monitor.note_vio_pose(now)
        core.monitor.note_odom(now)
        core.monitor.note_imu(now)
        core.monitor.note_camera(now)
        core.monitor.note_mavlink_link(now)
        core.on_position_cmd(cmd, now)
        now += 0.02
    out = core.step(cmd, now, frame_id_ok=True)
    assert out.allow_setpoint is True, f"核实通过后应放行：{out.reasons}"
    assert out.sent is True
    assert len(backend.sent) == 1
    sp, mask = backend.sent[0]
    assert sp.position_m == pytest.approx((1.0, -2.0, -3.0))
    assert mask is not None
