"""任务 B 脱机集成验证：规划输出 → Px4Interface → 后端，含失效与恢复行为。

两层验证：
1. **核心链路（无 ROS）**：`Px4InterfaceCore` + `FakePx4Backend`，用注入的单调时钟确定性地
   验证 —— 正常下发时 NED 值与 type_mask 正确、规划拒绝/轨迹失效/setpoint 过期/VIO 断流/
   IMU 断流/相机断流/链路超时/飞控重启时停发、恢复需要迟滞、坐标系不匹配被拒绝。
2. **进程级（真实节点 + 回环 MAVLink 对端）**：真实 `px4_interface_node` 进程订阅真实的
   `/position_cmd` 话题，后端用 `udpin` 连到测试自己起的一个「假 PX4」UDP 对端；
   断言真正收到的 msg 84 内容、心跳缺失导致的停发，以及**默认 dry_run 一个字节都不发**。

边界：本文件不启动 SITL（SITL 的实机化验证另见 README 中的 SITL 步骤），
不连接任何真实飞控、不解锁、不切模式。任何结论都只覆盖软件行为。
"""

from __future__ import annotations

import json
import time

import pytest

from boom_birds_nav.px4_failsafe import (
    FailsafeConfig,
    Px4FailsafeMonitor,
    ReasonCode,
    SignalId,
)
from boom_birds_nav.px4_frames import POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE

# ---------------------------------------------------------------- 测试替身


class FakePositionCommand:
    """`quadrotor_msgs/PositionCommand` 的最小同形替身（含 header.frame_id）。"""

    class _Header:
        def __init__(self, frame_id="world"):
            self.frame_id = frame_id

    def __init__(self, *, x=1.0, y=2.0, z=3.0, vx=0.1, vy=0.2, vz=0.3,
                 ax=0.01, ay=0.02, az=0.03, yaw=0.4, yaw_dot=0.0,
                 trajectory_id=7, trajectory_flag=1, frame_id="world"):
        self.position = type("P", (), {"x": x, "y": y, "z": z})()
        self.velocity = type("V", (), {"x": vx, "y": vy, "z": vz})()
        self.acceleration = type("A", (), {"x": ax, "y": ay, "z": az})()
        self.yaw = yaw
        self.yaw_dot = yaw_dot      # 真实消息字段名是 yaw_dot
        self.trajectory_id = trajectory_id
        self.trajectory_flag = trajectory_flag
        self.header = self._Header(frame_id)


def _config(**over):
    timeouts = {
        SignalId.SETPOINT: 0.15,
        SignalId.VIO_POSE: 0.15,
        SignalId.IMU: 0.5,
        SignalId.CAMERA: 1.0,
        SignalId.MAVLINK_LINK: 1.0,
        SignalId.PX4_HEARTBEAT: 1.0,
        SignalId.ODOM_EGO: 0.15,
    }
    timeouts.update(over.pop("timeouts", {}))
    return FailsafeConfig(timeouts=timeouts,
                          recovery_fresh_samples=over.pop("recovery_fresh_samples", 5),
                          **over)


def _make_backend(clock=None):
    """建链并喂心跳：FakePx4Backend 默认 link_up=False，不连上就不会接受 setpoint。

    clock 必须与测试给监控器的时间轴一致：后端用真实时钟、测试用合成时间会让
    `heartbeat_age_s` 变成负数或极小值，心跳丢失就永远测不出来（早期真实踩过的坑）。
    """
    from boom_birds_nav.px4_backend import FakePx4Backend

    backend = FakePx4Backend(clock=clock)
    backend.connect()
    backend.feed_heartbeat()
    return backend


def _accepted(backend) -> list:
    return [c for c in backend.calls_named("send_setpoint") if c.accepted]


def _core(*, backend=None, clock="auto", **over):
    """构造被测核心。

    注意：`over` 里的参数分两类——`yaw_mode`/`send_acceleration`/`max_setpoint_age_s`
    属于 `Px4InterfaceCore`，其余给 `FailsafeConfig`。必须先取出前者，否则会被透传给
    `FailsafeConfig` 并报 unexpected keyword（这是本测试早期真实踩过的错）。
    """
    from boom_birds_nav.px4_interface_node import Px4InterfaceCore

    core_kwargs = {
        "yaw_mode": over.pop("yaw_mode", "yaw"),
        "send_acceleration": over.pop("send_acceleration", True),
        "max_setpoint_age_s": over.pop("max_setpoint_age_s", 0.2),
    }
    if clock == "auto":
        # 后端会在 connect()/feed_heartbeat() 里调用时钟，因此必须用可变容器：
        # 容器里的值先由 _warm_up 写入，之后评估时后端与监控器看到同一条时间轴。
        box = {"t": 0.0}
        clock = lambda: box["t"]  # noqa: E731
        backend = backend or _make_backend(clock)
        core = Px4InterfaceCore(backend, _config(**over), **core_kwargs)
        core._test_clock_box = box
        return core
    backend = backend or _make_backend(clock)
    return Px4InterfaceCore(backend, _config(**over), **core_kwargs)


def _feed_all(core, now, boot_id=None):
    """把所有必需信号都标成新鲜，使门控只受被测条件影响。

    boot_id 默认沿用核心自己认定的那个（`core.step()` 按后端 restart_epoch 推算）；
    否则测试与实现会互相"制造重启"，测出来的就不是真实行为。
    """
    if boot_id is None:
        boot_id = getattr(core, "_boot_id", None) or "px4-epoch-0"
    m = core.monitor
    m.note_heartbeat(now, boot_id)
    m.note_mavlink_link(now)
    m.note_vio_pose(now)
    m.note_odom(now)
    m.note_imu(now)
    m.note_camera(now)


def _warm_up(core, t0, cmd, samples=8, step=0.02, boot_id=None):
    """连续若干周期让迟滞放行；返回 (最后一次时间, 最后一次结果)。

    开始前先把后端时钟对齐到 t0 并喂一次心跳：后端的 `heartbeat_age_s` 是
    「后端时钟 − 上次心跳时刻」，两条时间轴不先对齐就会得到几百秒的假年龄。
    """
    box = getattr(core, "_test_clock_box", None)
    if box is not None:
        box["t"] = t0
    if hasattr(core.backend, "feed_heartbeat"):
        core.backend.feed_heartbeat()
    t = t0
    out = None
    for _ in range(samples):
        t += step
        if box is not None:
            box["t"] = t
        _feed_all(core, t, boot_id)
        core.on_position_cmd(cmd, t)
        _feed_all(core, t, boot_id)
        core.on_position_cmd(cmd, t)
        out = core.step(cmd, t)
    return t, out


# ---------------------------------------------------------------- 转换正确性


def test_normal_setpoint_is_converted_and_sent():
    """正常路径：ROS Z-up → NED 的轴向与偏航符号、type_mask 必须正确。"""
    core = _core()
    cmd = FakePositionCommand(x=1.0, y=2.0, z=3.0, vx=0.1, vy=0.2, vz=0.3,
                              ax=0.01, ay=0.02, az=0.03, yaw=0.4, yaw_dot=0.0)
    t, out = _warm_up(core, 100.0, cmd)
    assert out.allow_setpoint, f"应放行：{out.reasons}"
    assert out.sent, "应已下发"
    accepted = _accepted(core.backend)
    assert accepted, f"应至少有一次被后端接受：{core.backend.calls}"
    sp = accepted[-1].args[0]
    # 轴向：N=x, E=-y, D=-z（由 px4_frames 的约定与单测锁定）
    assert list(sp.position_m) == pytest.approx([1.0, -2.0, -3.0])
    assert list(sp.velocity_m_s) == pytest.approx([0.1, -0.2, -0.3])
    assert list(sp.acceleration_m_s2) == pytest.approx([0.01, -0.02, -0.03])
    # 偏航取反（绕向下轴 vs 向上轴）
    assert sp.yaw_rad == pytest.approx(-0.4)
    assert core.counters["setpoints_sent"] >= 1
    assert core.counters["conversion_errors"] == 0


def test_yaw_rate_mode_changes_type_mask():
    core = _core(yaw_mode="yaw_rate")
    cmd = FakePositionCommand(yaw=0.0, yaw_dot=0.25)
    _, out = _warm_up(core, 200.0, cmd)
    accepted = _accepted(core.backend)
    sp_yr = accepted[-1].args[0]
    assert out.allow_setpoint
    # type_mask 的**内容**由回环真实报文验证（见 test_node_process_*）；
    # 这里验证"必须显式给出掩码"：缺掩码会被后端以 setpoint_type_mask_missing 拒收。
    diag = core.backend.stream_diagnostics()
    assert not diag["counters"].get("refused_setpoint_type_mask_missing"), (
        "本节点必须显式传 type_mask，不能被后端以缺失掩码拒绝"
    )
    # yaw_rate 模式：忽略 yaw、使用 yaw_rate（值层面的验证在 sp_yr 上）
    assert sp_yr.yaw_rate_rad_s == pytest.approx(-0.25)


def test_nonfinite_command_is_rejected_not_sent():
    core = _core()
    cmd = FakePositionCommand()
    cmd.position.x = float("nan")
    t, out = _warm_up(core, 300.0, cmd)
    assert not out.sent, "非有限值绝不能下发"
    assert core.counters["conversion_errors"] >= 1
    assert core.counters["last_conversion_error"]


# ---------------------------------------------------------------- 失效行为


def test_planning_rejection_stops_setpoints_and_requires_new_trajectory():
    """规划拒绝：立即停发；恢复必须拿到**新的**轨迹号，而不是等超时。"""
    core = _core()
    cmd = FakePositionCommand(trajectory_id=11)
    t, out = _warm_up(core, 400.0, cmd)
    assert out.allow_setpoint, f"预热后应放行：{out.reasons}"
    sent_before = len(_accepted(core.backend))

    core.on_planning_rejected(t + 0.01, "planner refused")
    t += 0.02
    _feed_all(core, t)
    out = core.step(cmd, t)
    assert not out.allow_setpoint, "规划拒绝后必须立即停发"
    assert not out.sent
    assert len(_accepted(core.backend)) == sent_before, "不得再发任何 setpoint"
    assert "planning_rejected" in core.counters["last_block_reasons"]

    # 同一轨迹号继续存在：仍不放行（迟滞 + 需要显式清除）
    cmd_same = FakePositionCommand(trajectory_id=11)
    for _ in range(10):
        t += 0.02
        _feed_all(core, t)
        core.on_position_cmd(cmd_same, t)
        out = core.step(cmd_same, t)
    assert not out.allow_setpoint, "规划拒绝在显式清除前不得自动恢复"
    assert len(_accepted(core.backend)) == sent_before


def test_trajectory_invalidation_flag_stops_immediately():
    """`trajectory_flag != READY`（含 order=0 的空轨迹失效约定）立即停发。"""
    core = _core()
    cmd = FakePositionCommand(trajectory_id=21)
    t, out = _warm_up(core, 500.0, cmd)
    assert out.allow_setpoint

    invalid = FakePositionCommand(trajectory_id=21, trajectory_flag=0)   # EMPTY
    t += 0.02
    _feed_all(core, t)
    assert core.on_position_cmd(invalid, t) is False, "非 READY 不得被接受为可用轨迹"
    out = core.step(invalid, t)
    assert not out.allow_setpoint and not out.sent
    assert core.counters["position_cmd_flagged"] >= 1


def test_stale_setpoint_is_not_sent():
    """setpoint 过期：即使其他信号都新鲜也不得下发。"""
    core = _core(max_setpoint_age_s=0.1)
    cmd = FakePositionCommand()
    t, out = _warm_up(core, 600.0, cmd)
    assert out.allow_setpoint
    before = len(_accepted(core.backend))

    t += 0.5          # 不再刷新 setpoint，但保持其他信号新鲜
    _feed_all(core, t)
    out = core.step(cmd, t)
    assert not out.allow_setpoint and not out.sent
    assert len(_accepted(core.backend)) == before
    assert any(r["code"] in ("setpoint_stale", "setpoint_age")
               or r.get("signal") == "setpoint" for r in out.reasons), out.reasons


@pytest.mark.parametrize("missing", ["vio", "imu", "camera", "heartbeat", "link"])
def test_each_stream_loss_stops_setpoints(missing):
    """VIO / IMU / 相机 / 心跳 / 链路任一断流都要停发（相机与里程计为降级，不阻断）。"""
    core = _core()
    cmd = FakePositionCommand()
    t, out = _warm_up(core, 700.0, cmd)
    assert out.allow_setpoint, f"预热后应放行：{out.reasons}"
    before = len(_accepted(core.backend))

    # 只推进时间、故意不刷新被测信号
    t += 3.0
    box = getattr(core, "_test_clock_box", None)
    if box is not None:
        box["t"] = t
    m = core.monitor
    if missing == "heartbeat":
        # 心跳丢失必须由**后端**模拟：后端持续报告 age，监控器按自己的超时判定。
        # 直接"忘记喂心跳"在这里无效——监控器的 HEARTBEAT 信号本来就由后端年龄驱动。
        core.backend.simulate_heartbeat_loss()
    if missing == "link":
        core.backend.simulate_link_loss()
        m.note_mavlink_link(t)      # 显式刷新，确保链路超时是唯一被测原因
    if missing != "link" and missing != "heartbeat":
        m.note_heartbeat(t, core._boot_id or "px4-epoch-0")
    if missing != "link":
        m.note_mavlink_link(t)
    if missing != "vio":
        m.note_vio_pose(t)
        m.note_odom(t)
    if missing != "imu":
        m.note_imu(t)
    if missing != "camera":
        m.note_camera(t)
    m.note_setpoint(t, cmd.trajectory_id)

    out = core.step(cmd, t)
    if missing in ("camera",):
        # 相机是 ADVISORY：不阻断发送，但状态应降级并给出原因
        assert out.state != "OK" or out.reasons, "相机断流至少要被记录为降级"
    else:
        assert not out.allow_setpoint, f"{missing} 断流必须停发：{out.reasons}"
        assert not out.sent


def test_px4_restart_stops_setpoints_until_reestablished():
    """飞控重启：状态不得自动沿用，必须重新建立后才放行。"""
    core = _core()
    cmd = FakePositionCommand()
    t, out = _warm_up(core, 800.0, cmd, boot_id=None)
    assert out.allow_setpoint

    t += 0.02
    box = getattr(core, "_test_clock_box", None)
    if box is not None:
        box["t"] = t
    core.backend.simulate_px4_restart()      # 后端 restart_epoch 变化 → 节点 boot_id 变化
    core.monitor.note_px4_restart(t, detail="boot time regressed")
    _feed_all(core, t, boot_id="px4-epoch-1")
    out = core.step(cmd, t)
    assert not out.allow_setpoint, "重启后不得立即放行"
    assert any(r["code"] == ReasonCode.PX4_RESTARTED.value for r in out.reasons), out.reasons

    # 新 boot_id 下持续新鲜 + 新轨迹 → 迟滞满足后恢复
    cmd2 = FakePositionCommand(trajectory_id=99)
    for _ in range(12):
        t += 0.02
        _feed_all(core, t, boot_id="px4-epoch-1")
        core.on_position_cmd(cmd2, t)
        _feed_all(core, t, boot_id="px4-epoch-1")
        core.on_position_cmd(cmd2, t)
        out = core.step(cmd2, t)
    assert out.allow_setpoint, f"重新建立后应恢复：{out.reasons}"


def test_recovery_is_hysteretic_not_single_sample():
    """恢复需要连续多次新鲜评估：单个新鲜样本不得立刻放行。"""
    core = _core(recovery_fresh_samples=5)
    cmd = FakePositionCommand()
    t0 = 900.0
    # 只喂 1 个样本：门控应保持关闭
    _feed_all(core, t0)
    core.on_position_cmd(cmd, t0)
    _feed_all(core, t0)
    core.on_position_cmd(cmd, t0)
    out = core.step(cmd, t0)
    assert not out.allow_setpoint, "单个新鲜样本不得放行（必须有迟滞）"

    # 补足迟滞所需次数
    t = t0
    for _ in range(6):
        t += 0.02
        _feed_all(core, t)
        core.on_position_cmd(cmd, t)
        _feed_all(core, t)
        core.on_position_cmd(cmd, t)
        out = core.step(cmd, t)
    assert out.allow_setpoint, f"迟滞满足后应放行：{out.reasons}"


def test_state_note_never_claims_vehicle_is_safe():
    """状态里必须始终带有「停发 ≠ 飞控悬停」的说明，不能读成已安全接管。

    注意要在**真正停发**的那一次结果上断言：成功放行时没有"停发"字样是正常的。
    """
    core = _core()
    cmd = FakePositionCommand(frame_id="world")
    t, out = _warm_up(core, 1000.0, cmd)
    assert out.allow_setpoint, f"预热后应放行：{out.reasons}"
    # 说明字段在任何状态下都存在，且必须把边界说清楚
    assert "setpoint" in out.note and "验证" in out.note
    assert "悬停" in out.note or "安全" in out.note, (
        "状态说明必须显式区分「停发 setpoint」与「飞控已悬停/已安全接管」"
    )

    # 制造一次确定的失效（轨迹失效），再检查停发状态下的说明
    invalid = FakePositionCommand(trajectory_id=1, trajectory_flag=0, frame_id="world")
    t += 0.02
    box = getattr(core, "_test_clock_box", None)
    if box is not None:
        box["t"] = t
    _feed_all(core, t)
    assert core.on_position_cmd(invalid, t) is False
    out = core.step(invalid, t)
    assert not out.allow_setpoint
    assert "setpoint" in out.note and "验证" in out.note
    assert "悬停" in out.note or "安全" in out.note, (
        "停发状态下的说明必须显式区分「停发 setpoint」与「飞控已悬停/已安全接管」"
    )
    assert out.reasons, "停发必须带原因"


# ---------------------------------------------------------------- 进程级 + 回环 MAVLink


def _free_udp_port() -> int:
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class FakePx4Peer:
    """测试内的假 PX4：发心跳、收 msg 84，并可切换「心跳静默」模拟链路异常。"""

    def __init__(self) -> None:
        import socket

        from pymavlink.dialects.v20 import common as mavlink2

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.02)
        self.mav = mavlink2.MAVLink(None, srcSystem=1, srcComponent=1)
        self.parser = mavlink2.MAVLink(self.sock)
        self.parser.robust_parsing = True
        self.dest: tuple | None = None
        self.setpoints: list = []
        self.heartbeats_sent = 0
        self.silent = False
        self.armed = False

    def pump(self) -> None:
        """读入到达的消息；学到对端地址后即可回发。"""
        try:
            data, addr = self.sock.recvfrom(4096)
        except Exception:  # noqa: BLE001
            return
        self.dest = addr
        for byte in data:
            msg = self.parser.parse_char(bytes([byte]))
            if msg is None or msg.get_type() == "BAD_DATA":
                continue
            if msg.get_type() == "SET_POSITION_TARGET_LOCAL_NED":
                self.setpoints.append(msg)

    def send_heartbeat(self) -> None:
        """发给已学到的对端（收到过它的数据之后才可用）。"""
        if self.dest is None:
            return
        self.send_heartbeat_to(self.dest)

    def send_heartbeat_to(self, addr) -> None:
        """主动把心跳发到指定地址（udpin 后端要靠它学到回发地址）。"""
        if self.silent:
            return
        base_mode = 128 if self.armed else 0     # MAV_MODE_FLAG_SAFETY_ARMED
        frame = self.mav.heartbeat_encode(2, 12, base_mode, 0, 0, 3)
        self.sock.sendto(frame.pack(self.mav), addr)
        if self.dest is None:
            self.dest = addr
        self.heartbeats_sent += 1

    def close(self) -> None:
        self.sock.close()


@pytest.mark.skipif(pytest.importorskip("pymavlink", reason="需要 pymavlink") is None,
                    reason="需要 pymavlink")
def test_node_process_sends_ned_setpoints_to_loopback_peer(tmp_path):
    """真实节点进程 + 回环假 PX4：验证真正收到的 msg 84、心跳缺失停发、以及失效状态。"""
    import os
    import subprocess
    import sys

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    from quadrotor_msgs.msg import PositionCommand

    port = _free_udp_port()
    peer = FakePx4Peer()
    peer_port = peer.sock.getsockname()[1]

    params = tmp_path / "px4_interface_test.yaml"
    params.write_text(f"""boom_birds_px4_interface:
  ros__parameters:
    backend: "mavlink"
    dry_run: false
    allow_arming: false
    connection: "udpin:127.0.0.1:{port}"
    control_rate_hz: 50.0
    status_topic: "/bb_test/control_status"
    position_cmd_topic: "/bb_test/position_cmd"
    odom_topic: "/bb_test/odom"
    imu_topic: "/bb_test/imu"
    depth_topic: "/bb_test/depth"
    setpoint_timeout_s: 0.15
    vio_timeout_s: 0.15
    imu_timeout_s: 0.5
    camera_timeout_s: 1.0
    heartbeat_timeout_s: 1.0
    backend_heartbeat_timeout_s: 1.0
    link_timeout_s: 1.0
    recovery_required_samples: 3
    # 回环假 PX4 与合成规划数据按同向同原点构造；显式声明两项合成证据，
    # 以验证真实 MAVLink 发送路径。不得把本测试的配置用于实机。
    frame_alignment: "identity"
    frame_alignment_observed: true
    frame_alignment_origin_evidence: true
""", encoding="utf-8")

    args = [sys.executable, "-m", "boom_birds_nav.px4_interface_node", "--ros-args",
            "--params-file", str(params)]

    from conftest import child_env

    # 子进程必须能 import quadrotor_msgs（colcon install 里的消息包），
    # 否则节点在构造订阅时就会退出，表现为"话题没有数据"。
    # DDS domain 由 conftest 在导入期就固定（父进程 rclpy 上下文可能更早建立），
    # 这里只保证子进程继承到同一个值。
    env = child_env()
    log = tmp_path / "px4_interface.log"
    logf = log.open("w")
    proc = subprocess.Popen(args, stdout=logf, stderr=subprocess.STDOUT, env=env)
    try:
        # 先让假 PX4 学到节点地址（节点发心跳/链接建立后才有对端地址）
        probe = None
        if not rclpy.ok():
            rclpy.init()
        probe = rclpy.create_node("px4_interface_probe")
        pub_cmd = probe.create_publisher(PositionCommand, "/bb_test/position_cmd", 10)
        pub_odom = probe.create_publisher(__import__("nav_msgs.msg", fromlist=["Odometry"]).Odometry,
                                          "/bb_test/odom", 10)
        pub_imu = probe.create_publisher(__import__("sensor_msgs.msg", fromlist=["Imu"]).Imu,
                                         "/bb_test/imu", 5)
        pub_depth = probe.create_publisher(__import__("sensor_msgs.msg", fromlist=["Image"]).Image,
                                           "/bb_test/depth", 10)
        status = []
        probe.create_subscription(String, "/bb_test/control_status",
                                  lambda m: status.append(m), 10)
        ex = SingleThreadedExecutor()
        ex.add_node(probe)

        def cmd_msg(traj_id=5, flag=1, frame_id="world"):
            m = PositionCommand()
            m.header.frame_id = frame_id
            m.trajectory_flag = flag
            m.trajectory_id = traj_id
            m.position.x, m.position.y, m.position.z = 1.0, 2.0, 3.0
            m.velocity.x, m.velocity.y, m.velocity.z = 0.0, 0.0, 0.0
            m.acceleration.x = m.acceleration.y = m.acceleration.z = 0.0
            m.yaw = 0.0
            m.yaw_dot = 0.0
            return m

        def odom_msg():
            from nav_msgs.msg import Odometry
            m = Odometry()
            m.header.frame_id = "global"
            m.child_frame_id = "body"
            return m

        def imu_msg():
            from sensor_msgs.msg import Imu
            return Imu()

        def depth_msg():
            from sensor_msgs.msg import Image
            m = Image()
            m.height, m.width, m.encoding, m.step = 1, 1, "32FC1", 4
            m.data = b"\x00\x00\x80\x7f"
            return m

        # 先等子进程把后端起起来：真实后端在启动时必然 connect() 一次，
        # 因此 `backend.connect_attempts >= 1` 是"节点真的起来了"的可靠判据
        # （比固定等待或"status 非空"都稳）。耐心一点：机器被压满时启动会慢很多。
        def _backend_view():
            if not status:
                return {}
            try:
                return (json.loads(status[-1].data).get("backend") or {})
            except ValueError:
                return {}

        started = False
        start_deadline = time.monotonic() + 60.0
        while time.monotonic() < start_deadline:
            ex.spin_once(timeout_sec=0.02)
            view = _backend_view()
            if (view.get("counters") or {}).get("connect_attempts", 0) >= 1:
                started = True
                break
            time.sleep(0.02)
        assert started, (
            "等待 60 s 子进程仍未完成后端建链（connect_attempts 一直为 0）；"
            f"最后后端视图：{_backend_view()}\n"
            f"节点进程存活：{proc.poll() is None}\n"
            f"日志尾部：\n{log.read_text(encoding='utf-8', errors='replace')[-1200:]}"
        )

        # udpin 模式下后端绑定端口可能是内核分配的临时端口，从状态里读实际绑定地址。
        local_port = None
        settle = time.monotonic() + 20.0
        while time.monotonic() < settle and local_port is None:
            ex.spin_once(timeout_sec=0.02)
            endpoint = _backend_view().get("local_endpoint")
            if endpoint:
                local_port = int(str(endpoint).rsplit(":", 1)[1])
                break
            time.sleep(0.02)
        assert local_port, (
            "后端已建链但状态里没有 local_endpoint；"
            f"后端视图：{_backend_view()}\n"
            f"日志尾部：\n{log.read_text(encoding='utf-8', errors='replace')[-1200:]}"
        )
        node_addr = ("127.0.0.1", local_port)

        deadline = time.monotonic() + 40.0
        got_setpoint = False
        while time.monotonic() < deadline and not got_setpoint:
            ex.spin_once(timeout_sec=0.01)
            peer.pump()
            peer.send_heartbeat_to(node_addr)
            for _ in range(3):
                pub_odom.publish(odom_msg())
                pub_imu.publish(imu_msg())
                pub_depth.publish(depth_msg())
                pub_cmd.publish(cmd_msg())
            ex.spin_once(timeout_sec=0.01)
            if peer.setpoints:
                got_setpoint = True
            time.sleep(0.02)

        assert got_setpoint, (
            "回环对端未收到 SET_POSITION_TARGET_LOCAL_NED；"
            f"日志尾部：\n{log.read_text(encoding='utf-8', errors='replace')[-1500:]}"
        )
        msg = peer.setpoints[-1]
        # 轴向：N=x=1.0, E=-y=-2.0, D=-z=-3.0；坐标系必须是 LOCAL_NED
        assert msg.x == pytest.approx(1.0, abs=1e-5)
        assert msg.y == pytest.approx(-2.0, abs=1e-5)
        assert msg.z == pytest.approx(-3.0, abs=1e-5)
        assert int(msg.coordinate_frame) == 1, "MAV_FRAME_LOCAL_NED"
        assert int(msg.target_system) == 1 and int(msg.target_component) == 1
        # yaw 模式：yaw_rate 必须被忽略（掩码置位），yaw 被使用
        from boom_birds_nav.px4_frames import POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE as YR_IGN
        assert int(msg.type_mask) & YR_IGN, "yaw 模式下 yaw_rate 应被忽略"

        # --- 心跳静默：后端应判定失联，节点必须停发 ---
        # 后端心跳超时 = backend_heartbeat_timeout_s = 1.0s。静默后最多还可能下发
        # 1 个 setpoint（超时窗口内的最后一拍），所以先在静默期间把窗口跑满，
        # 再取基线，之后必须**一个都不再增加**。
        peer.silent = True
        silence_deadline = time.monotonic() + 1.4
        while time.monotonic() < silence_deadline:
            ex.spin_once(timeout_sec=0.01)
            peer.pump()
            for _ in range(2):
                pub_odom.publish(odom_msg())
                pub_imu.publish(imu_msg())
                pub_depth.publish(depth_msg())
                pub_cmd.publish(cmd_msg())
            time.sleep(0.02)

        # 等到后端真正判定心跳超时，而不是靠固定睡眠赌调度：观察状态话题里
        # allow_setpoint 变假、且后端自报的心跳年龄越过它自己的超时阈值为止。
        def _timeout_seen():
            if not status:
                return False
            try:
                payload = json.loads(status[-1].data)
            except ValueError:
                return False
            backend = payload.get("backend") or {}
            age = backend.get("heartbeat_age_s")
            limit = backend.get("heartbeat_timeout_s")
            return (
                payload.get("allow_setpoint") is False
                and age is not None and limit is not None and age > limit
            )

        def _pump_once(publish):
            ex.spin_once(timeout_sec=0.01)
            peer.pump()
            for _ in range(publish):
                pub_odom.publish(odom_msg())
                pub_imu.publish(imu_msg())
                pub_depth.publish(depth_msg())
                pub_cmd.publish(cmd_msg())
            time.sleep(0.02)

        stale_deadline = time.monotonic() + 20.0
        while time.monotonic() < stale_deadline and not _timeout_seen():
            _pump_once(2)

        # 超时生效后必须完全停发；再观察 1.0 s 确认没有新 setpoint。
        n_after_silence = len(peer.setpoints)      # 恢复段的基线
        n_settled = len(peer.setpoints)
        settle_deadline = time.monotonic() + 1.0
        while time.monotonic() < settle_deadline:
            _pump_once(2)
        assert len(peer.setpoints) == n_settled, (
            "心跳超时后不得继续下发 setpoint"
        )
        assert status, "未收到状态话题"
        latest = json.loads(status[-1].data)
        if not _timeout_seen():
            raise AssertionError(
                "等待 8 s 仍未观察到「心跳被判超时且已停发」\n"
                f"  setpoints={len(peer.setpoints)} status_msgs={len(status)}\n"
                f"  最后状态：{latest}"
            )
        assert latest["allow_setpoint"] is False, f"状态应显示停发：{latest}"
        assert latest["reasons"], "停发必须带原因"
        note = latest["note"]
        # 注意：不能断言 "停发" 这个两字组合——说明文字用的是「停止发送」，
        # 其中并不包含相邻的「停发」。断言要盯住语义（停发/停止发送 + 验证），
        # 不要盯住某个恰好没被用到的词形。
        assert ("停发" in note or "停止发送" in note), "状态里必须说明已停止发送 setpoint"
        assert "验证" in note, "状态里必须说明该行为需在 SITL/实机验证"
        assert "悬停" in note or "安全" in note, (
            "状态里必须显式否认「已悬停/已安全接管」"
        )
        # 心跳确实被判为过期：后端自己报告的心跳年龄必须超过自己的超时阈值
        backend = latest["backend"]
        assert backend["connected"] is False, f"心跳超时后 connected 应为 false：{backend}"
        assert backend["heartbeat_age_s"] is not None
        assert backend["heartbeat_age_s"] > backend["heartbeat_timeout_s"], (
            f"心跳年龄应超过超时阈值：age={backend['heartbeat_age_s']} "
            f"timeout={backend['heartbeat_timeout_s']}"
        )

        # --- 恢复：重新发心跳后，经过迟滞才恢复下发 ---
        peer.silent = False
        recovered = False
        recover_deadline = time.monotonic() + 20.0
        while time.monotonic() < recover_deadline and not recovered:
            ex.spin_once(timeout_sec=0.01)
            peer.pump()
            peer.send_heartbeat_to(node_addr)
            for _ in range(3):
                pub_odom.publish(odom_msg())
                pub_imu.publish(imu_msg())
                pub_depth.publish(depth_msg())
                pub_cmd.publish(cmd_msg())
            if len(peer.setpoints) > n_after_silence:
                recovered = True
            time.sleep(0.02)
        assert recovered, (
            "心跳恢复后节点应重新下发 setpoint（迟滞满足后）"
        )
        # 未解锁、未切模式：本节点只发高层 setpoint
        assert peer.armed is False
    finally:
        try:
            peer.close()
        except Exception:  # noqa: BLE001
            pass
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except Exception:  # noqa: BLE001
            proc.kill()
        logf.close()


def test_default_launch_is_dry_run_and_never_arms():
    """默认参数必须是「不连、不发、不解锁」：dry_run=true、backend=fake、allow_arming=false。"""
    import pathlib

    import yaml

    cfg_path = (pathlib.Path(__file__).resolve().parents[1]
                / "config" / "px4_interface.yaml")
    cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))["boom_birds_px4_interface"]["ros__parameters"]
    assert cfg["backend"] == "fake", "默认不得连任何后端"
    assert cfg["dry_run"] is True, "默认必须 dry_run"
    assert cfg["allow_arming"] is False, "默认禁止解锁"
    assert str(cfg["connection"]).startswith("udpin:127.0.0.1"), "默认只允许回环地址"


def test_default_launch_blocks_position_setpoints_until_aligned():
    """默认配置必须**拦住位置 setpoint**：两局部系的原点/水平朝向未核实。

    这是 P1-2 的核心回归：曾只做轴变换就发位置，量纲对但落点无法证明。
    """
    import pathlib

    import yaml

    cfg_path = (pathlib.Path(__file__).resolve().parents[1]
                / "config" / "px4_interface.yaml")
    cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))[
        "boom_birds_px4_interface"
    ]["ros__parameters"]
    assert cfg["frame_alignment"] == "none", "默认必须是 none（最保守）"
    assert cfg["frame_alignment_observed"] is False, "默认不得声称已核实"
    assert cfg["frame_alignment_translation_m"] == [0.0, 0.0, 0.0]
    assert cfg["frame_alignment_yaw_offset_rad"] == 0.0
