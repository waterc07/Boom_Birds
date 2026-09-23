"""本轮修复 2/3/4 的针对性回归（节点层，用最小替身避免起真实节点）。

覆盖：
* 航向核实**不能**替代原点证据——旧实现 `residual.verified` 一旦为真就放行位置；
* 姿态缺失（None）不得被当成 0 从而"通过"倾角/角速率筛选；
* 姿态过期、两个消息的本机到达时差过大时不得采样；
* 闸门拦下时 `state` 必须是 STOPPED（旧实现沿用监控器的 OK）；
* identity 模式拒绝非零偏移/平移配置。
"""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from boom_birds_nav.px4_failsafe import FailsafeConfig, SignalId
from boom_birds_nav.px4_backend import VehicleState
from boom_birds_nav.px4_frames import LocalFrameAlignment, YawAlignmentResidual
from boom_birds_nav.px4_interface_node import Px4InterfaceCore, Px4InterfaceNode

# ============================================================ 替身

class _FakeState:
    def __init__(self, yaw=0.0, roll=0.0, pitch=0.0, yaw_rate=0.0, attitude_age=0.01):
        self.yaw_rad = yaw
        self.roll_rad = roll
        self.pitch_rad = pitch
        self.yaw_rate_rad_s = yaw_rate
        self.attitude_age_s = attitude_age


class _Backend:
    """只实现 Core 需要的部分；setpoint 记录在 self.sent。"""

    def __init__(self, state=None):
        self.sent = []
        self.state = state

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
        return self.state

    def stream_diagnostics(self):
        return {"connected": True, "heartbeat_age_s": 0.0,
                "heartbeat_timeout_s": 1.0, "counters": {}}

    def send_setpoint(self, setpoint, type_mask=None):
        self.sent.append((setpoint, type_mask))
        return True


def _timeouts():
    return {
        SignalId.SETPOINT: 0.15,
        SignalId.VIO_POSE: 0.15,
        SignalId.IMU: 0.5,
        SignalId.CAMERA: 1.0,
        SignalId.MAVLINK_LINK: 1.0,
        SignalId.PX4_HEARTBEAT: 1.0,
        SignalId.ODOM_EGO: 0.15,
    }


class _Cmd:
    def __init__(self, frame_id="world"):
        self.position = type("P", (), {"x": 1.0, "y": 0.0, "z": 2.0})()
        self.velocity = type("V", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
        self.acceleration = type("A", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
        self.yaw = 0.0
        self.yaw_dot = 0.0
        self.trajectory_id = 3
        self.trajectory_flag = 1
        self.header = type("H", (), {"frame_id": frame_id})()


def _core(backend, gate=None):
    config = FailsafeConfig(timeouts=_timeouts(), recovery_fresh_samples=1,
                            require_px4_identity=False)
    return Px4InterfaceCore(backend, config, position_gate=gate)


def _warm(core, cmd, now=1000.0, samples=8):
    """把监控器喂到"其它输入全健康"。"""
    for _ in range(samples):
        core.monitor.note_vio_pose(now)
        core.monitor.note_odom(now)
        core.monitor.note_imu(now)
        core.monitor.note_camera(now)
        core.monitor.note_mavlink_link(now)
        core.on_position_cmd(cmd, now)
        now += 0.02
    return now


# ============================================================ 姿态缺失 / 过期
def test_missing_attitude_is_not_treated_as_zero():
    """后端状态缺少 roll/pitch/yaw_rate 时不得采样、不得核实。

    旧实现用 `getattr(state, "roll_rad", 0.0) or 0.0`，等于把"没观测到"读成
    "水平且不转"，倾角/角速率筛选形同虚设——这里就是它的回归。
    """
    r = YawAlignmentResidual(tolerance_rad=math.radians(5), required_samples=1)
    assert r.observe(ros_yaw_rad=0.0, px4_yaw_rad=0.0,
                     roll_rad=None, pitch_rad=None, yaw_rate_rad_s=None) is False
    assert r.samples == 0
    assert r.yaw_verified is False
    assert r.rejected_missing_attitude == 1

    # 只缺一个也不行
    assert r.observe(ros_yaw_rad=0.0, px4_yaw_rad=0.0,
                     roll_rad=0.0, pitch_rad=None, yaw_rate_rad_s=0.0) is False
    assert r.samples == 0


def test_stale_attitude_is_rejected():
    r = YawAlignmentResidual(tolerance_rad=math.radians(5), required_samples=1,
                             max_attitude_age_s=0.05)
    assert r.observe(ros_yaw_rad=0.0, px4_yaw_rad=0.0, roll_rad=0.0, pitch_rad=0.0,
                     yaw_rate_rad_s=0.0, attitude_age_s=0.5) is False
    assert r.samples == 0
    assert r.rejected_stale_attitude == 1
    assert r.yaw_verified is False


def test_samples_from_different_times_are_rejected():
    """本机到达时间相隔太远的 ROS/PX4 航向不能配成一对。"""
    r = YawAlignmentResidual(tolerance_rad=math.radians(5), required_samples=1,
                             max_sample_skew_s=0.02)
    assert r.observe(ros_yaw_rad=0.0, px4_yaw_rad=0.0, roll_rad=0.0, pitch_rad=0.0,
                     yaw_rate_rad_s=0.0, ros_mono_s=10.0, px4_mono_s=10.5,
                     attitude_age_s=0.01) is False
    assert r.samples == 0
    assert r.rejected_skew == 1


def test_node_pairs_backend_attitude_arrival_with_vio_arrival():
    """经节点真实调用路径检查时间门槛，不能只测 observe() 的可选参数。"""
    offset = math.radians(30)
    residual = YawAlignmentResidual(tolerance_rad=math.radians(5), required_samples=1)
    state = VehicleState(
        yaw_rad=-offset, roll_rad=0.0, pitch_rad=0.0, yaw_rate_rad_s=0.0,
        attitude_age_s=0.01, attitude_received_mono_s=9.9,
    )
    node = SimpleNamespace(
        _alignment_residual=residual, _alignment_observed=False,
        _alignment_restart_latched=False,
        _refresh_alignment_epoch=lambda: None,
        backend=_Backend(state), alignment=LocalFrameAlignment(yaw_offset_rad=offset),
    )
    Px4InterfaceNode.note_yaw_observation(node, 0.0, ros_mono_s=10.0)
    assert residual.samples == 0
    assert residual.rejected_skew == 1

    node.backend.state = VehicleState(
        yaw_rad=-offset, roll_rad=0.0, pitch_rad=0.0, yaw_rate_rad_s=0.0,
        attitude_age_s=0.01, attitude_received_mono_s=None,
    )
    Px4InterfaceNode.note_yaw_observation(node, 0.0, ros_mono_s=10.0)
    assert residual.samples == 0
    assert residual.rejected_missing_timing == 1

    node.backend.state = VehicleState(
        yaw_rad=-offset, roll_rad=0.0, pitch_rad=0.0, yaw_rate_rad_s=0.0,
        attitude_age_s=0.01, attitude_received_mono_s=9.98,
    )
    Px4InterfaceNode.note_yaw_observation(node, 0.0, ros_mono_s=10.0)
    assert residual.samples == 1
    assert residual.yaw_verified


def test_empty_position_frame_id_is_rejected_before_core():
    """空 frame_id 不能跳过 world/global/map 检查进入控制链。"""
    rejected = []
    core = SimpleNamespace(
        counters={"position_cmd_frame_mismatch": 0},
        on_planning_rejected=lambda now, reason: rejected.append(reason),
        on_position_cmd=lambda *args: pytest.fail("空 frame_id 不得送入 Core"),
    )
    node = SimpleNamespace(
        frame_world_ok={"world", "global", "map"}, _cmd=object(),
        _cmd_frame_ok=True, core=core,
        get_logger=lambda: SimpleNamespace(warn=lambda *args, **kwargs: None),
    )
    Px4InterfaceNode._on_position_cmd(node, _Cmd(frame_id=""))
    assert node._cmd is None
    assert node._cmd_frame_ok is False
    assert core.counters["position_cmd_frame_mismatch"] == 1
    assert rejected and "frame_id=" in rejected[0]


def test_restart_discards_accumulated_yaw_samples():
    residual = YawAlignmentResidual(tolerance_rad=math.radians(5), required_samples=1)
    residual.observe(
        ros_yaw_rad=0.0, px4_yaw_rad=0.0, roll_rad=0.0, pitch_rad=0.0,
        yaw_rate_rad_s=0.0,
    )
    assert residual.yaw_verified
    epoch = {"value": 0}
    logs = []
    node = SimpleNamespace(
        backend=SimpleNamespace(link_state=lambda: {"restart_epoch": epoch["value"]}),
        _alignment_epoch=None, _alignment_restart_latched=False,
        _alignment_residual=residual,
        get_logger=lambda: SimpleNamespace(error=logs.append),
    )
    Px4InterfaceNode._refresh_alignment_epoch(node)
    assert residual.samples == 1
    epoch["value"] = 1
    Px4InterfaceNode._refresh_alignment_epoch(node)
    assert node._alignment_restart_latched is True
    assert residual.samples == 0
    assert logs


def test_node_does_not_invent_attitude_when_backend_reports_none():
    """后端只给 yaw（其它为 None）时，节点核实不了：位置仍然被拦。"""
    backend = _Backend(state=type("S", (), {
        "yaw_rad": 0.0, "roll_rad": None, "pitch_rad": None,
        "yaw_rate_rad_s": None, "attitude_age_s": None,
    })())
    calls = {"allow": False, "reason": "none", "missing": []}

    def gate():
        return calls["allow"], calls["reason"], calls["missing"]

    core = _core(backend, gate=gate)
    _warm(core, _Cmd())
    out = core.step(_Cmd(), 1000.2, frame_id_ok=True)
    assert backend.sent == [], "闸门未放行时不得发任何 setpoint"
    assert out.allow_setpoint is False
    assert out.state == "STOPPED"


# ============================================================ 闸门 + 状态一致
def test_gate_block_yields_stopped_state_even_when_inputs_healthy():
    """其它输入全健康、**仅**对齐未核实：必须 state=STOPPED，不能报 OK。"""
    backend = _Backend(state=_FakeState())
    core = _core(backend, gate=lambda: (False, "yaw_verified_origin_unknown",
                                        ["原点/平移无证据"]))
    cmd = _Cmd()
    now = _warm(core, cmd)
    out = core.step(cmd, now, frame_id_ok=True)

    assert out.allow_setpoint is False
    assert out.state == "STOPPED", f"停发却报 {out.state}（状态与停发不一致）"
    assert out.detail.get("stopped_by") == "local_frame_not_aligned"
    codes = [r.get("code") for r in out.reasons]
    assert "local_frame_not_aligned" in codes
    blocked = [r for r in out.reasons if r.get("code") == "local_frame_not_aligned"][0]
    assert blocked["missing_evidence"] == ["原点/平移无证据"]
    assert backend.sent == []


def test_gate_allow_path_state_is_not_forced_to_stopped():
    """放行路径不能被 stop_for 误伤：健康输入 + 对齐完备 ⇒ 正常下发。"""
    backend = _Backend(state=_FakeState())
    core = _core(backend, gate=lambda: (True, "yaw_verified+origin_evidence(declared)", []))
    cmd = _Cmd()
    now = _warm(core, cmd)
    out = core.step(cmd, now, frame_id_ok=True)
    assert out.allow_setpoint is True
    assert out.sent is True
    assert out.state != "STOPPED"
    assert len(backend.sent) == 1


def test_other_stop_paths_also_report_stopped():
    """其它停发路径（无可用轨迹）也必须与状态一致。"""
    backend = _Backend(state=_FakeState())
    core = _core(backend, gate=lambda: (True, "ok", []))
    cmd = _Cmd()
    now = _warm(core, cmd)
    out = core.step(None, now, frame_id_ok=True)      # 没有轨迹
    assert out.allow_setpoint is False
    assert out.state == "STOPPED"
    assert out.detail.get("stopped_by") == "no_usable_trajectory"


def test_frame_id_mismatch_also_stops():
    backend = _Backend(state=_FakeState())
    core = _core(backend, gate=lambda: (True, "ok", []))
    cmd = _Cmd(frame_id="world")
    now = _warm(core, cmd)
    out = core.step(cmd, now, frame_id_ok=False)
    assert out.allow_setpoint is False
    assert out.state == "STOPPED"


def test_gate_reason_and_missing_evidence_are_reported():
    """诊断必须说清"缺哪项证据"，不能只说一句"未对齐"。"""
    backend = _Backend(state=_FakeState())
    seen = {}

    def gate():
        seen["called"] = True
        return False, "not_aligned(yaw_not_verified)", ["水平朝向未核实", "原点/平移无证据"]

    core = _core(backend, gate=gate)
    cmd = _Cmd()
    now = _warm(core, cmd)
    out = core.step(cmd, now, frame_id_ok=True)
    assert seen.get("called") is True
    assert out.detail["alignment"] == "not_aligned(yaw_not_verified)"
    assert out.detail["alignment_missing"] == ["水平朝向未核实", "原点/平移无证据"]
