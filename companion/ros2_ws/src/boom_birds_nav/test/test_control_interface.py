import time
import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from boom_birds_interfaces.msg import ControlCommand
from boom_birds_interfaces.srv import VehicleAction
from boom_birds_control.px4_interface_node import Px4InterfaceNode

@pytest.fixture
def node():
    context = Context()
    rclpy.init(context=context)
    node = Px4InterfaceNode(context=context, parameter_overrides=[Parameter("require_session", value=True)])
    try:
        yield node
    finally:
        node.shutdown()
        node.destroy_node()
        context.shutdown()


def open_session(node):
    req = VehicleAction.Request()
    req.action = req.OPEN_SESSION
    result = node._vehicle_action(req, VehicleAction.Response())
    assert result.accepted
    return result.session_id


def command(node, session, seq=1, kind=1, traj=1):
    msg = ControlCommand()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "global"
    msg.session_id = session
    msg.sequence = seq
    msg.trajectory_id = traj
    msg.command_type = kind
    msg.valid_for.nanosec = 200000000
    return msg


def test_cancel_clears_cached_setpoint_and_blocks_late_execute(node):
    session = open_session(node)
    node._on_control_command(command(node, session))
    assert node._cmd is not None
    node._on_control_command(command(node, session, 2, ControlCommand.CANCEL))
    assert node._cmd is None
    node._on_control_command(command(node, session, 3))
    assert node._cmd is None
    node._tick()
    assert not node.backend.setpoints


def test_new_session_cannot_reuse_old_monitor_trajectory(node):
    session = open_session(node)
    node._on_control_command(command(node, session))
    old_id = node._cmd.trajectory_id
    new_session = open_session(node)
    node._on_control_command(command(node, new_session))
    assert node._cmd.trajectory_id > old_id
    node._on_control_command(command(node, session, 100, ControlCommand.CANCEL))
    assert node._cmd is not None


def test_session_does_not_bypass_failsafe_or_confirm_mode(node):
    session = open_session(node)
    node._on_control_command(command(node, session))
    node._tick()
    assert not node.backend.setpoints
    req = VehicleAction.Request(session_id=session, action=VehicleAction.Request.OFFBOARD)
    result = node._vehicle_action(req, VehicleAction.Response())
    assert not result.accepted
    assert result.reason == "setpoint_not_streaming"


def test_expired_cached_command_stops_at_next_tick(node):
    session = open_session(node)
    node._on_control_command(command(node, session))
    node.ingress.deadline = time.monotonic() - 1
    node._tick()
    assert node._cmd is None
    assert not node.backend.setpoints


def test_manual_cancel_closes_session(node):
    session = open_session(node)
    req = VehicleAction.Request(session_id=session, action=VehicleAction.Request.CANCEL)
    assert node._vehicle_action(req, VehicleAction.Response()).accepted
    node._on_control_command(command(node, session))
    assert node._cmd is None
    assert node.ingress.session == ""


def test_stale_pose_receipt_does_not_refresh_measurement(node):
    from nav_msgs.msg import Odometry
    from boom_birds_control.px4_failsafe import SignalId
    pose = Odometry()
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.header.stamp.sec -= 2
    node._on_odom_impl(pose)
    outcome = node.core.step(None, time.monotonic())
    assert outcome.detail["signal_ages_s"][SignalId.ODOM_EGO.value] > 1.9
    assert not outcome.detail["sensors_ready"]


def test_execution_status_reports_observed_auto_sub_mode(node):
    """只报主模式区分不了 AUTO Land 与 AUTO Return：必须发实际观测到的子模式。"""
    published = []
    node.pub_execution.publish = lambda msg: published.append(msg)
    assert node.backend.set_mode("auto:land")
    node._tick()
    land = published[-1]
    assert land.mode == "auto"
    assert land.mode_detail == "auto:land"
    assert land.custom_sub_mode == 6 and land.custom_main_mode == 4
    assert node.backend.set_mode("auto:rtl")
    node._tick()
    rtl = published[-1]
    assert rtl.mode == "auto"
    assert rtl.mode_detail == "auto:rtl"
    assert rtl.custom_sub_mode == 5
    # 两者在 mode 上相同，只有 mode_detail 能区分；这正是本项要补的观测
    assert land.mode == rtl.mode
    assert land.mode_detail != rtl.mode_detail


def test_offboard_confirmation_uses_observed_main_mode(node):
    published = []
    node.pub_execution.publish = lambda msg: published.append(msg)
    assert node.backend.set_mode("offboard")
    node._tick()
    assert published[-1].mode_detail == "offboard"
    assert published[-1].mode == "offboard"


def test_mode_detail_unknown_is_not_invented(node):
    """未映射的 custom_mode 必须如实报 unknown，不得编一个名字。"""
    from boom_birds_control.px4_backend import px4_custom_mode

    published = []
    node.pub_execution.publish = lambda msg: published.append(msg)
    node.backend.custom_mode = px4_custom_mode(200, 0)     # 保留值，不在 PX4 名表里
    node._tick()
    assert published[-1].mode == "unknown"
    assert published[-1].mode_detail == "unknown"


def test_missing_custom_mode_is_not_converted_to_a_mode(node):
    """后端没有模式观测时，状态里不得出现被编造的模式名。"""
    published = []
    node.pub_execution.publish = lambda msg: published.append(msg)
    node.backend.read_vehicle_state = lambda: type(
        "S", (), dict(connected=True, is_offboard=False, heartbeat_age_s=0.01,
                      armed=False, landed_state=1, landed_age_s=0.01,
                      mode_name=None, mode_detail=None, custom_main_mode=None,
                      custom_sub_mode=None, restart_epoch=0, position_age_s=0.01,
                      current_mode_detail=None, intended_mode_detail=None,
                      current_mode_age_s=None, frame_reset_epoch=0, frame_reset_age_s=None,
                      position_ned_m=None, velocity_ned_m_s=None,
                      yaw_rad=None, attitude_age_s=None)
    )()
    node._tick()
    assert published[-1].mode == "unknown"
    assert published[-1].mode_detail == "unknown"
    assert published[-1].custom_main_mode == 0


def test_restart_between_session_open_and_first_tick_is_detected(node):
    """开会话时就绑定启动周期：开完会话后立刻重启也必须作废，不能等第一次 tick。"""
    session = open_session(node)
    node._on_control_command(command(node, session))
    node.backend.restart_epoch += 1          # 开会话之后、首次 tick 之前重启
    node._tick()
    assert node.ingress.session == ""
    assert node._cmd is None


def test_flight_controller_restart_invalidates_session_and_cache(node):
    """飞控重启：旧会话、旧缓存 setpoint、旧轨迹号一律作废。"""
    session = open_session(node)
    node._on_control_command(command(node, session))
    node._tick()
    assert node.ingress.session == session
    node.backend.restart_epoch += 1          # 观测到飞控重启
    node._tick()
    assert node.ingress.session == ""        # 旧会话被关闭
    assert node._cmd is None                 # 旧 setpoint 不沿用
    assert not node.backend.setpoints
    node._on_control_command(command(node, session, 5))
    assert node._cmd is None                 # 旧会话的迟到命令不得复活
    node._tick()
    assert not node.backend.setpoints


def test_restart_requires_a_new_session_before_control_resumes(node):
    session = open_session(node)
    node._on_control_command(command(node, session))
    node.backend.restart_epoch += 1
    node._tick()
    fresh = open_session(node)               # 重新开会话
    node._on_control_command(command(node, fresh))
    assert node._cmd is not None
    assert node.ingress.session == fresh


def feed_fresh_inputs(node, pose_age_s=0.0):
    """喂一轮新鲜输入：VIO/里程计、IMU、相机。"""
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, Image

    stamp = node.get_clock().now().to_msg()
    stamp.nanosec += int(pose_age_s * 1e9)
    if stamp.nanosec >= 1_000_000_000:
        stamp.sec += 1
        stamp.nanosec -= 1_000_000_000
    odom = Odometry()
    odom.header.stamp = stamp
    node._on_odom_impl(odom)
    imu = Imu()
    imu.header.stamp = node.get_clock().now().to_msg()
    node._on_imu(imu)
    image = Image()
    image.header.stamp = node.get_clock().now().to_msg()
    node._on_depth(image)


@pytest.fixture
def session_node():
    """require_session + 跳过坐标系闸门（仅脱机 fake 后端）的节点。"""
    context = Context()
    rclpy.init(context=context)
    node = Px4InterfaceNode(context=context, parameter_overrides=[
        Parameter("require_session", value=True),
        Parameter("frame_alignment", value="unverified_test_only"),
    ])
    try:
        yield node
    finally:
        node.shutdown()
        node.destroy_node()
        context.shutdown()


def test_session_path_can_actually_open_the_transmit_gate(session_node):
    """端到端闸门必须真的能打开。

    这一条是 SIH 实测暴露的回归：`OPEN_SESSION` 曾经调用 `on_planning_rejected`，
    而该原因只能由 `clear_planning_rejected()` 清除、没人调用它 —— 于是整条会话链路
    永久闭锁，`setpoints_sent` 恒为 0，任务停在 HOLD_READY 直到超时。
    """
    node = session_node
    session = open_session(node)
    sent_before = len(node.backend.setpoints)
    seq = 1
    for _ in range(12):
        feed_fresh_inputs(node)
        seq += 1
        node._on_control_command(command(node, session, seq=seq))
        node._tick()
    assert len(node.backend.setpoints) > sent_before, node._last_outcome
    assert node._last_outcome.sent
    assert "planning_rejected" not in [r["code"] for r in node._last_outcome.reasons]


def test_open_session_does_not_latch_planning_rejected(session_node):
    """开会话本身不得产生"规划拒绝"闭锁。"""
    node = session_node
    open_session(node)
    node._tick()
    assert not node.core.monitor._planning_rejected_active
    assert "planning_rejected" not in [
        r["code"] for r in (node._last_outcome.reasons if node._last_outcome else [])
    ]


def test_planning_cancel_latches_and_new_trajectory_clears(session_node):
    """取消闭锁 -> 新轨迹到达后必须能重新武装（否则同样是永久闭锁）。"""
    node = session_node
    session = open_session(node)
    seq = 1
    for _ in range(12):
        feed_fresh_inputs(node)
        seq += 1
        node._on_control_command(command(node, session, seq=seq))
        node._tick()
    assert node._last_outcome.sent
    seq += 1
    node._on_control_command(command(node, session, seq=seq, kind=ControlCommand.CANCEL))
    assert node.core.monitor._planning_rejected_active
    sent_after_cancel = len(node.backend.setpoints)
    # 旧 trajectory_id 必须继续被拒（禁止续播旧轨迹）
    seq += 1
    node._on_control_command(command(node, session, seq=seq, traj=1))
    assert node._cmd is None
    # 只有**新**轨迹号才允许重新武装并真的发出 setpoint
    for _ in range(12):
        feed_fresh_inputs(node)
        seq += 1
        node._on_control_command(command(node, session, seq=seq, traj=2))
        node._tick()
    assert len(node.backend.setpoints) > sent_after_cancel


def test_mode_confirmation_uses_the_same_heartbeat_window_as_execution_gate(node):
    from types import SimpleNamespace
    from boom_birds_control.px4_backend import VehicleState
    from boom_birds_control.runtime_config import DEFAULTS
    messages = []
    node.pub_execution.publish = messages.append
    for age in (1.1, DEFAULTS.heartbeat_timeout_s + .01):
        node.backend.read_vehicle_state = lambda: VehicleState(
            connected=True, custom_main_mode=6, custom_mode=6 << 16,
            heartbeat_age_s=age)
        node._publish_execution(SimpleNamespace(sent=False, detail={}, reasons=[]))
    assert messages[0].offboard_confirmed
    assert not messages[1].offboard_confirmed
