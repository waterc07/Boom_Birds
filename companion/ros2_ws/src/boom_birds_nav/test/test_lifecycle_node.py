import json
import math
import time
from types import SimpleNamespace
from unittest.mock import patch

import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter

from boom_birds_bringup.lifecycle import State
from boom_birds_bringup.lifecycle_node import LifecycleNode
from boom_birds_interfaces.msg import ControlCommand, ExecutionStatus, PlannerStatus
from boom_birds_interfaces.srv import Mission


class FakeTime:
    """可推进的单调时钟：只替换 ``lifecycle_node`` 看到的 ``time`` 模块。"""

    def __init__(self, start=1000.0):
        self.now = float(start)

    def monotonic(self):
        return self.now

    def time(self):
        return self.now + 1700000000.

    def advance(self, dt):
        self.now += float(dt)
        return self.now


class AcceptedFuture:
    """``VehicleAction`` 的假 future：请求被接口接受。"""

    def done(self):
        return True

    def result(self):
        return SimpleNamespace(accepted=True)


def make_node(**parameters):
    """建一个已完成起飞、停在 EXECUTING 的节点（不启动 ROS 执行器）。"""
    context = Context()
    rclpy.init(context=context)
    overrides = [Parameter(name, value=value) for name, value in parameters.items()]
    node = LifecycleNode(context=context, parameter_overrides=overrides or None)
    node.fsm.start("session", (2., 0., 1.5), time.monotonic())
    node.fsm.ground_z = 0.
    node.fsm.hold = (0., 0., 1.5)
    node.fsm.transition(State.EXECUTING, time.monotonic())
    node.ingress.session = "session"
    node.planner_session, node.executor_session = "planner-instance", "executor-instance"
    node.executor_received = time.monotonic()
    status = ExecutionStatus(session_id="session", connected=True, armed=True, landed_known=True, landed_state=2,
        offboard_confirmed=True, alignment_valid=True, position_known=True, sensors_ready=True, sending=True,
        mode="offboard", mode_detail="offboard", attitude_known=True, attitude_age_s=0.,
        frame_reset_known=True, frame_reset_age_s=0.)
    status.position_ned.z = -1.5
    status.header.stamp = node.get_clock().now().to_msg()
    node._control_status(status)
    return node, context


@pytest.fixture
def node():
    node, context = make_node()
    try:
        yield node
    finally:
        node.destroy_node()
        context.shutdown()


@pytest.fixture
def recovery_node():
    """SIH 入口的等价物：显式把 recovery_enabled 打开。"""
    node, context = make_node(recovery_enabled=True)
    try:
        yield node
    finally:
        node.destroy_node()
        context.shutdown()


def publish_status(node, **fields):
    status = node.control
    fields.setdefault("current_mode_detail", fields.get("mode_detail", status.mode_detail))
    fields.setdefault("intended_mode_detail", "offboard")
    fields.setdefault("current_mode_age_s", 0.)
    fields.setdefault("px4_safety_mode", fields.get("mode_detail", status.mode_detail))
    fields.setdefault("px4_failsafe_cause", "offboard_link")
    fields.setdefault("px4_safety_age_s", 0.)
    status.alignment_yaw_offset_rad = node.alignment.yaw_offset_rad
    status.alignment_translation_m.x, status.alignment_translation_m.y, status.alignment_translation_m.z = node.alignment.translation_m
    for name, value in fields.items():
        setattr(status, name, value)
    status.header.stamp = node.get_clock().now().to_msg()
    node._control_status(status)
    return status


def publish_planner(node, *, map_ready=True, geometry_fault=False, goal_active=True):
    status = PlannerStatus(session_id="session", producer_session_id="planner-instance", map_ready=map_ready, geometry_fault=geometry_fault, goal_active=goal_active)
    status.header.stamp = node.get_clock().now().to_msg()
    node._planner_status(status)
    executor = PlannerStatus(session_id="session", producer_session_id="executor-instance")
    executor.header.stamp = node.get_clock().now().to_msg()
    node._executor_status(executor)
    return status


def command(node, *, trajectory_id=1, sequence=1):
    msg = ControlCommand(session_id="session", planner_session_id="planner-instance", executor_session_id="executor-instance", sequence=sequence, trajectory_id=trajectory_id,
                         command_type=ControlCommand.EXECUTE)
    msg.header.frame_id = "global"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.valid_for.nanosec = 200000000
    msg.position.z = 1.5
    return msg


def test_only_current_session_can_forward_and_cancel(node):
    msg = command(node)
    msg.session_id = "old"
    node._planner_command(msg)
    assert node.command_seq == 0
    msg.session_id = "session"
    node._planner_command(msg)
    assert node.command_seq == 1
    req = Mission.Request(action=Mission.Request.CANCEL)
    assert node._mission(req, Mission.Response()).accepted
    assert node.command_seq == 2
    msg.sequence = 2
    node._planner_command(msg)
    assert node.command_seq == 2
    assert node.fsm.state == State.FAULT_LATCHED


@pytest.mark.parametrize("case", ["distance", "velocity", "pose_age", "no_position", "old_stamp"])
def test_discontinuous_or_stale_handoff_is_rejected(node, case):
    msg = command(node)
    if case == "distance": msg.position.x = .501
    elif case == "velocity": msg.velocity.x = .301
    elif case == "pose_age": node.control.position_age_s = .151
    elif case == "no_position": node.control.position_known = False
    else: msg.header.stamp.sec -= 1
    node._planner_command(msg)
    assert not node.playing
    if case in ("distance", "velocity"):
        assert node.fsm.state == State.EXECUTING
        assert node._planner_quiet_since is not None
        assert node.ingress.retired >= msg.trajectory_id
        assert node.command_seq == 2  # CANCEL followed by HOLD, never EXECUTE
    elif case == "old_stamp":
        assert node.command_seq == 0
    elif case == "pose_age":
        assert node.fsm.reason == "sensor_link"
    else:
        assert node.fsm.reason == "handoff_discontinuous"


def test_handoff_thresholds_are_the_frozen_values_and_come_from_one_source(node):
    # 任务冻结值：距离 0.5 m、速度连续性 0.3 m/s；两者都取自 RuntimeConfig。
    assert node.config.handoff_max_distance_m == 0.5
    assert node.config.handoff_max_speed_m_s == 0.3
    at_limit = command(node)
    at_limit.velocity.x = node.config.handoff_max_speed_m_s          # 边界值：允许
    node._planner_command(at_limit)
    assert node.playing and node.command_seq == 1
    over = command(node, trajectory_id=2, sequence=2)
    over.velocity.x = node.config.handoff_max_speed_m_s + 1e-3       # 超过连续速度 → 拒绝
    node._planner_command(over)
    assert not node.playing
    assert node.fsm.state == State.EXECUTING
    assert node.ingress.retired >= 2
    assert node.command_seq == 3  # accepted EXECUTE, then CANCEL and HOLD


def test_planner_command_requires_confirmed_offboard_mode_readback(node):
    # 命令侧没有报错（offboard_confirmed=True），但模式回读是 AUTO Land：
    # 不能再把规划指令当作 OFFBOARD 有效。
    publish_status(node, mode_detail="auto:land")
    node._planner_command(command(node))
    assert node.command_seq == 0
    assert not node.playing


def test_cancel_clears_cache_and_next_cycle_sends_no_setpoint(node):
    published = []
    node.pub_command.publish = published.append
    node._planner_command(command(node))
    assert [msg.command_type for msg in published] == [ControlCommand.EXECUTE]
    assert node.playing
    request = Mission.Request(action=Mission.Request.CANCEL)
    assert node._mission(request, Mission.Response()).accepted
    assert published[-1].command_type == ControlCommand.CANCEL
    assert not node.playing
    sent = len(published)
    for _ in range(3):
        node._tick()
    assert len(published) == sent              # 取消后下一控制周期不再发任何 setpoint
    node._planner_command(command(node, sequence=2))
    assert len(published) == sent


def test_recovery_switch_defaults_off_and_sih_entry_can_enable_it(node, recovery_node):
    assert node.config.recovery_enabled is False
    assert node.fsm.config.recovery_enabled is False
    assert recovery_node.config.recovery_enabled is True
    assert recovery_node.fsm.config.recovery_enabled is True


def test_node_recovery_is_off_by_default(node):
    publish_status(node, offboard_confirmed=False, mode="auto", mode_detail="auto:land")
    node._tick()
    assert node.fsm.state == State.FAULT_LATCHED
    assert node.fsm.reason == "manual_mode"
    assert node.fsm.recovery_attempts == 0


@pytest.mark.parametrize("detail,fault", [("auto:land", "px4_auto_land"), ("auto:rtl", "px4_auto_return")])
def test_node_identifies_auto_land_and_return_from_mode_detail_not_main_mode(recovery_node, detail, fault):
    node = recovery_node
    node.playing = True
    publish_status(node, offboard_confirmed=False, mode="auto", mode_detail=detail, sensors_ready=False)
    node._tick()
    assert node.fsm.state == State.RECOVERING
    assert node.fsm.recovery_fault == fault
    assert node.fsm.recovery_source == f"status.mode_detail={detail}"
    assert node.fsm.recovery_attempts == 1


def test_node_latches_manual_mode_intervention_without_racing_it(recovery_node):
    node = recovery_node
    publish_status(node, offboard_confirmed=False, mode="position", mode_detail="position")
    node._tick()
    assert node.fsm.state == State.FAULT_LATCHED
    assert node.fsm.reason == "manual_mode"
    assert node.fsm.recovery_revoked is True
    assert node.fsm.recovery_attempts == 0


def test_mission_status_reports_mode_detail_and_recovery_diagnostics(recovery_node):
    node = recovery_node
    payloads = []
    node.pub_state.publish = payloads.append
    node.playing = True
    publish_status(node, offboard_confirmed=False, mode="auto", mode_detail="auto:rtl", sensors_ready=False)
    node._tick()
    payload = json.loads(payloads[-1].data)
    assert payload["mode_detail"] == "auto:rtl"
    assert payload["mode"] == "auto"
    assert payload["state"] == "RECOVERING"
    assert payload["recovery"]["enabled"] is True
    assert payload["recovery"]["fault"] == "px4_auto_return"
    assert payload["recovery"]["source"] == "status.mode_detail=auto:rtl"
    assert payload["recovery"]["attempts"] == 1
    assert payload["recovery"]["budget"] == 2
    assert payload["recovery"]["revoked"] is False


def test_node_recovery_retires_old_trajectory_and_replans_after_readback(recovery_node):
    node = recovery_node
    node.client.service_is_ready = lambda: True
    node.client.call_async = lambda request: AcceptedFuture()
    published = []
    node.pub_command.publish = published.append
    clock = FakeTime(time.monotonic() + 10.)
    with patch("boom_birds_bringup.lifecycle_node.time", clock):
        publish_status(node)                                   # control_received 改用假时钟
        publish_planner(node)
        node._planner_command(command(node))                   # 正常执行：轨迹 1 被接受
        assert node.playing and node.command_seq == 1
        # PX4 自主进入 AUTO Land：命令侧没有变化，只有模式回读变了。
        publish_status(node, offboard_confirmed=False, mode="auto", mode_detail="auto:land", sensors_ready=False)
        publish_planner(node)
        node._tick()
        assert node.fsm.state == State.RECOVERING
        assert node.fsm.recovery_fault == "px4_auto_land"
        assert not node.playing
        assert node.ingress.retired >= 1                       # 旧轨迹立即退役
        before = node.command_seq
        node._planner_command(command(node, sequence=2))       # 同一 trajectory_id 不得续播
        assert node.command_seq == before
        # 状态流每周期到达，直到向 PX4 接口请求 OFFBOARD（经既有执行许可）
        for _ in range(40):
            clock.advance(.1)
            publish_status(node, offboard_confirmed=False, mode="auto", mode_detail="auto:land", sensors_ready=True)
            publish_planner(node)
            node._tick()
            if node.fsm.recovery_request_at is not None:
                break
        assert node.fsm.recovery_request_at is not None
        assert any(msg.command_type == ControlCommand.HOLD for msg in published)
        # PX4 回读确认（mode_detail 变成 offboard）→ 重新规划剩余目标
        clock.advance(.1)
        publish_status(node, offboard_confirmed=True, mode="offboard", mode_detail="offboard")
        publish_planner(node)
        node._tick()
        assert node.fsm.state == State.EXECUTING
        assert node.fsm.recovery_detail == "confirmed"
        # 恢复确认后旧轨迹仍然不能续播，只有新的 trajectory_id 才能接管
        before = node.command_seq
        node._planner_command(command(node, sequence=2))
        assert node.command_seq == before and not node.playing
        node._planner_command(command(node, trajectory_id=2, sequence=3))
        assert node.playing
        assert node.command_seq == before + 1
        assert node.planner_key == ("session", 2)


def test_node_publishes_brake_velocity_and_acceleration(recovery_node):
    node = recovery_node
    published = []
    node.pub_command.publish = published.append
    node.playing = True
    node.control.velocity_ned.x = .8
    publish_status(node, sensors_ready=False)
    node._sensor_invalid_since = time.monotonic() - node.config.sensor_fault_confirm_s - .01
    node._tick()
    assert node.fsm.state == State.RECOVERING
    hold = [msg for msg in published if msg.command_type == ControlCommand.HOLD][-1]
    assert hold.velocity.x == pytest.approx(.8, abs=.02)
    assert hold.acceleration.x == pytest.approx(-node.config.recovery_brake_accel_m_s2)


def test_discontinuous_trajectory_is_retired_until_a_fresh_id_arrives(node):
    published = []
    node.pub_command.publish = published.append
    jumped = command(node)
    jumped.position.x = 2.0
    node._planner_command(jumped)
    assert [m.command_type for m in published] == [ControlCommand.CANCEL, ControlCommand.HOLD]
    assert node.fsm.state == State.EXECUTING
    old = command(node, sequence=2)
    node._planner_command(old)
    assert len(published) == 2
    fresh = command(node, trajectory_id=2, sequence=3)
    node._planner_command(fresh)
    assert node.playing
    assert published[-1].command_type == ControlCommand.EXECUTE


def test_rejected_trajectories_have_a_bounded_wait(node):
    jumped = command(node)
    jumped.position.x = 2.0
    node._planner_command(jumped)
    assert node.fsm.state == State.EXECUTING
    node._planner_quiet_since = time.monotonic() - node.config.planner_activate_timeout_s - .1
    publish_status(node)
    node._tick()
    assert node.fsm.state == State.FAULT_LATCHED
    assert node.fsm.reason == "planner_timeout"


def test_hold_precedes_setpoint_expiration(node):
    published = []
    node.pub_command.publish = published.append
    node.playing = True
    node._planner_cmd_seen_at = time.monotonic() - 0.15
    publish_status(node)
    publish_planner(node)
    node._tick()
    assert node.fsm.state == State.EXECUTING, node.fsm.reason
    assert [msg.command_type for msg in published] == [ControlCommand.HOLD]
    assert node.playing
    node._planner_cmd_seen_at = time.monotonic() - 0.25
    publish_status(node)
    node._tick()
    assert [msg.command_type for msg in published][-2:] == [ControlCommand.CANCEL, ControlCommand.HOLD]
    assert not node.playing
    assert node._planner_quiet_since is not None


@pytest.mark.parametrize("at_goal", [False, True])
def test_hold_wait_does_not_hide_setpoint_loss(recovery_node, at_goal):
    node = recovery_node
    node.playing = False
    publish_planner(node)
    if at_goal:
        node.control.position_ned.x = 2.0
    publish_status(node, sending=False)
    node._sending_seen_at = time.monotonic() - node.config.setpoint_interrupt_timeout_s - .1
    node._tick()
    assert node.fsm.state == State.RECOVERING
    assert node.fsm.recovery_fault == "setpoint_link"


def test_hold_wait_does_not_hide_sensor_loss(recovery_node):
    node = recovery_node
    node.playing = False
    publish_planner(node)
    publish_status(node, sensors_ready=False)
    node._sensor_invalid_since = time.monotonic() - node.config.sensor_fault_confirm_s - .01
    node._tick()
    assert node.fsm.state == State.RECOVERING
    assert node.fsm.recovery_fault == "sensor_link"


def test_discontinuous_handoff_explicitly_stops_the_planner_before_retry(node):
    requests = []
    node.pub_request.publish = requests.append
    jumped = command(node)
    jumped.position.x = 2.0
    node._planner_command(jumped)
    assert [m.enabled for m in requests] == [False]
    publish_status(node)
    publish_planner(node)
    node._tick()
    assert [m.enabled for m in requests] == [False, True]
    assert requests[0].sequence < requests[1].sequence


@pytest.mark.parametrize("intention", ["auto:land", "auto:rtl", "unknown"])
def test_node_manual_mode_intention_overrides_simultaneous_sensor_fault(recovery_node, intention):
    node = recovery_node
    publish_status(node, offboard_confirmed=False, mode="auto", mode_detail="auto:land",
                   sensors_ready=False, intended_mode_detail=intention)
    node._tick()
    assert node.fsm.state == State.FAULT_LATCHED
    assert node.fsm.reason == "manual_mode"
    assert node.fsm.recovery_revoked


def test_short_sensor_rejection_does_not_spend_recovery_budget(recovery_node):
    node = recovery_node
    clock = FakeTime(time.monotonic() + 10.)
    with patch("boom_birds_bringup.lifecycle_node.time", clock):
        publish_planner(node)
        publish_status(node, sensors_ready=False)
        node._tick()
        assert node.fsm.state == State.EXECUTING
        clock.advance(node.config.sensor_fault_confirm_s / 2)
        publish_status(node, sensors_ready=True)
        node._tick()
        assert node.fsm.recovery_attempts == 0
        publish_status(node, sensors_ready=False)
        node._tick()
        clock.advance(node.config.sensor_fault_confirm_s + .01)
        publish_status(node, sensors_ready=False)
        node._tick()
        assert node.fsm.state == State.RECOVERING
        assert node.fsm.recovery_fault == "sensor_link"


def test_hold_preserves_observed_heading_and_declared_frame_rotation(node):
    from boom_birds_control.px4_frames import LocalFrameAlignment
    node.alignment = LocalFrameAlignment(yaw_offset_rad=.4, translation_m=(3., 2., 1.))
    publish_status(node, yaw_ned_rad=.7, attitude_known=True, attitude_age_s=0.)
    node.fsm.hold = (3., 2., 2.)
    published = []
    node.pub_command.publish = published.append
    node._hold()
    assert published[-1].yaw == pytest.approx(-1.1)
    assert published[-1].position.z == 2.


@pytest.mark.parametrize("fields", [dict(attitude_known=False),
                                   dict(attitude_age_s=1.), dict(yaw_ned_rad=float("nan"))])
def test_hold_refuses_missing_stale_or_invalid_heading(node, fields):
    publish_status(node, **fields)
    node.fsm.hold = (0., 0., 1.5)
    published = []
    node.pub_command.publish = published.append
    node._hold()
    assert published == []


def test_stale_handoff_waits_for_sensor_recovery_without_setpoints(recovery_node):
    node = recovery_node
    published = []
    node.pub_command.publish = published.append
    node._planner_command(command(node))
    assert node.playing
    publish_planner(node)
    publish_status(node, position_age_s=.365)
    node._planner_command(command(node, trajectory_id=2, sequence=2))
    assert node.fsm.state == State.RECOVERING
    assert node.fsm.recovery_total_attempts == 1
    assert node.fsm.recovery_brake_started is None
    assert not node.playing
    assert node.ingress.retired >= 2
    assert all(msg.command_type == ControlCommand.CANCEL for msg in published[1:])


@pytest.mark.parametrize("age", [-.1, float("inf"), float("nan")])
def test_invalid_handoff_pose_age_cannot_enable_recovery(recovery_node, age):
    node = recovery_node
    node.control.position_age_s = age
    node._planner_command(command(node))
    assert node.fsm.state == State.FAULT_LATCHED
    assert node.fsm.recovery_total_attempts == 0
    assert not node.playing


@pytest.mark.parametrize("actor", ["planner", "executor"])
def test_producer_restart_latches_task(node, actor):
    status = PlannerStatus(session_id="session", producer_session_id="restarted-instance")
    status.header.stamp = node.get_clock().now().to_msg()
    getattr(node, "_" + actor + "_status")(status)
    assert node.fsm.state == State.FAULT_LATCHED
    assert node.fsm.recovery_revoked
    msg = command(node, trajectory_id=1000)
    before = node.command_seq
    node._planner_command(msg)
    assert node.command_seq == before


@pytest.mark.parametrize("actor", ["planner", "executor"])
@pytest.mark.parametrize("producer", ["", "old-instance"])
def test_missing_or_old_command_producer_cannot_execute(node, actor, producer):
    msg = command(node)
    setattr(msg, actor + "_session_id", producer)
    node._planner_command(msg)
    assert node.fsm.state == State.FAULT_LATCHED
    assert not node.playing


@pytest.mark.parametrize("actor", ["planner", "executor"])
def test_repeated_restart_status_does_not_interrupt_cleanup_land(node, actor):
    status = PlannerStatus(session_id="session", producer_session_id="restarted-instance")
    status.header.stamp = node.get_clock().now().to_msg()
    getattr(node, "_" + actor + "_status")(status)
    assert node.fsm.state == State.FAULT_LATCHED
    node.fsm.land(time.monotonic())
    assert node.fsm.state == State.LANDING
    getattr(node, "_" + actor + "_status")(status)
    assert node.fsm.state == State.LANDING


def test_expired_foreign_producer_message_is_rejected_without_new_fault(node):
    msg = command(node)
    msg.executor_session_id = "old-instance"
    msg.header.stamp.sec -= 1
    node._planner_command(msg)
    assert node.fsm.state == State.EXECUTING
    assert not node.playing


def test_px4_observation_rotates_velocity_and_translates_only_position(node):
    node.fsm.state = State.PRECHECK
    status = publish_status(node, alignment_yaw_offset_rad=math.pi / 2)
    status.alignment_translation_m.x, status.alignment_translation_m.y, status.alignment_translation_m.z = 10., 20., 30.
    status.position_ned.x, status.position_ned.y, status.position_ned.z = 2., 3., -4.
    status.velocity_ned.x, status.velocity_ned.y, status.velocity_ned.z = .2, .3, -.4
    node._control_status(status)
    o = node.observation()
    assert o.position == pytest.approx((7., 18., 34.))
    assert o.velocity == pytest.approx((-.3, -.2, .4))
    node.fsm.state = State.EXECUTING
    msg = command(node)
    msg.position.x, msg.position.y, msg.position.z = o.position
    msg.velocity.x, msg.velocity.y, msg.velocity.z = o.velocity
    node._planner_command(msg)
    assert node.playing


def test_alignment_transform_change_revokes_running_task(node):
    status = publish_status(node, alignment_yaw_offset_rad=.1)
    assert node.fsm.state == State.FAULT_LATCHED
    assert node.fsm.reason == "frame_reset"
    assert node.fsm.recovery_revoked
    assert not node.observation().alignment
    node.fsm.land(time.monotonic())
    node._control_status(status)
    assert node.fsm.state == State.LANDING


def test_nonfinite_position_observation_does_not_crash(node):
    status = publish_status(node)
    status.position_ned.x = float("nan")
    node._control_status(status)
    assert all(math.isnan(v) for v in node.observation().position)


def test_planner_cancel_publishes_control_barrier_before_next_tick(node):
    published = []
    node.pub_command.publish = published.append
    node._planner_command(command(node))
    active_id = published[-1].trajectory_id
    cancel = command(node, sequence=2)
    cancel.command_type = ControlCommand.CANCEL
    node._planner_command(cancel)
    assert [message.command_type for message in published] == [
        ControlCommand.EXECUTE, ControlCommand.CANCEL]
    assert published[-1].trajectory_id == active_id
    assert published[-1].sequence > published[0].sequence
    assert not node.playing and node.ingress.current is None
    stale = command(node, sequence=3)
    node._planner_command(stale)
    assert len(published) == 2  # 较高序号不能续播已退役轨迹。
    node._tick()
    assert published[-1].command_type == ControlCommand.HOLD
    assert published[-1].trajectory_id > active_id

@pytest.mark.parametrize("transport_age", [.02, .2])
def test_delayed_execution_status_keeps_observation_age(node, transport_age):
    status = node.control
    node.ros_now = lambda: 1000.
    status.header.stamp.sec = 999
    status.header.stamp.nanosec = int((1. - transport_age) * 1e9)
    node.control_received = time.monotonic()
    status.position_age_s = status.frame_reset_age_s = .01
    status.current_mode_age_s = status.px4_safety_age_s = .01
    status.attitude_age_s = .01
    observation = node.observation()
    for observed_age in (observation.pose_age, observation.frame_reset_age_s,
                         observation.current_mode_age_s, observation.px4_safety_age_s):
        assert observed_age == pytest.approx(.01 + transport_age, abs=.002)
    fresh = transport_age < node.config.pose_timeout_s
    assert node.fsm.recovery_preconditions(observation) == (
        (True, "") if fresh else (False, "pose_stale"))
    published = []
    node.pub_command.publish = published.append
    node._hold()
    assert bool(published) == fresh
