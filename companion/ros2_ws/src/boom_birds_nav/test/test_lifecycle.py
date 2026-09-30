import math
from dataclasses import replace

import pytest

from boom_birds_bringup.lifecycle import (
    Lifecycle,
    Observation,
    State,
    StableWindow,
    offboard_observed,
)
from boom_birds_control.runtime_config import DEFAULTS

#: 只把自动恢复开关打开（其余阈值仍取唯一来源 RuntimeConfig）。
RECOVERY = DEFAULTS.with_overrides(recovery_enabled=True)


def observed(**kw):
    base = Observation(frame_reset_known=True, frame_reset_age_s=0., session="s", connected=True, landed=1, status_age=0., pose_age=0., position=(0., 0., 0.), velocity=(0., 0., 0.), alignment=True, map_ready=True, sensors_ready=True)
    return replace(base, **kw)


def airborne(**kw):
    """执行期的合格观测：解锁、明确在空中、OFFBOARD 已回读、链路新鲜。"""
    base = observed(armed=True, landed=2, position=(2., 0., 1.5), velocity=(0., 0., 0.),
                    offboard=True, mode="offboard", mode_detail="offboard", sending=True,
                    intended_mode_detail="offboard", current_mode_age_s=0.)
    return replace(base, **kw)


#: PX4 自主进入 AUTO Land，同时 OFFBOARD 已丢失的观测。
AUTO_LAND = replace(airborne(), offboard=False, mode_detail="auto:land", current_mode_detail="auto:land", px4_safety_mode="auto:land",
                    px4_failsafe_cause="offboard_link", px4_safety_age_s=0., fault="sensor_link")


def running(config=DEFAULTS):
    """跑到 HOLD_READY 之前（起飞完成，锁点已定）。"""
    f = Lifecycle(config)
    assert f.start("s", (2., 0., 1.5), 0.)
    assert f.tick(0., observed()) == ["arm"]
    air = observed(armed=True, landed=2, position=(0., 0., 1.5))
    assert f.tick(.1, air) == ["takeoff"]
    for i in range(10):
        assert f.tick(.2 + i * .1, air) == []
    assert f.tick(1.2, air) == ["hold_setpoint"]
    assert f.state == State.HOLD_READY
    assert f.ground_z == 0.
    return f, air


def offboard_pending(config=DEFAULTS):
    f, air = running(config)
    assert f.tick(2.3, replace(air, sending=True)) == ["hold_setpoint", "offboard"]
    assert f.state == State.OFFBOARD_PENDING
    return f, air


def executing(config=DEFAULTS):
    """跑到 EXECUTING（每一步都是真实状态迁移）。"""
    f, air = offboard_pending(config)
    assert f.tick(2.4, replace(air, sending=True, offboard=True, mode_detail="offboard")) == ["hold_setpoint", "enable_planner"]
    assert f.state == State.EXECUTING
    return f


def recovering(config=RECOVERY):
    """注入 PX4 自主 AUTO Land → RECOVERING（尝试 1；3.2 起累积连续有效期窗口）。"""
    f = executing(config)
    assert f.tick(3., airborne()) == []
    assert f.tick(3.1, AUTO_LAND) == ["disable_planner", "recover", "hold_setpoint"]
    assert f.state == State.RECOVERING
    assert f.tick(3.2, AUTO_LAND) == ["hold_setpoint"]
    return f


# ---------------------------------------------------------------------------
# 锁点（HOLD_READY 之前的 hold 定位）
# ---------------------------------------------------------------------------
def test_stable_lock_requires_full_second_and_resets_on_motion_or_staleness():
    w = StableWindow()
    o = observed(armed=True, landed=2, position=(0., 0., 1.5))
    for i in range(10):
        assert not w.update(i * .1, o, 0.)
    assert w.update(1., o, 0.)
    assert not w.update(1.1, replace(o, velocity=(.16, 0., 0.)), 0.)
    for i in range(10):
        assert not w.update(1.2 + i * .1, o, 0.)
    assert not w.update(2.2, replace(o, pose_age=.151), 0.)
    assert not w.update(2.3, o, 0.)
    assert not w.update(2.4, replace(o, position=(.101, 0., 1.5)), 0.)


def test_lock_uses_prcheck_ground_z_as_agl_reference_and_never_the_first_crossing_sample():
    w = StableWindow()
    below = observed(armed=True, landed=2, position=(0., 0., 11.4))
    for i in range(30):
        assert not w.update(i * .1, below, 10.)      # AGL 1.4 < takeoff_altitude_agl_m
    assert not w.samples
    at = observed(armed=True, landed=2, position=(0., 0., 11.5))
    assert not w.update(3.0, at, 10.)                # 首次越过高度的样本不算
    for i in range(1, 10):
        assert not w.update(3.0 + i * .1, at, 10.)
    assert w.update(4.0, at, 10.)                    # 连续 1 s 才锁定
    assert not w.update(4.1, replace(at, position=(0., 0., 11.3)), 10.)   # 掉回高度以下 → 清零
    assert not w.samples


def test_lock_altitude_gate_is_the_stricter_of_takeoff_and_hold_lock():
    strict = DEFAULTS.with_overrides(hold_lock_min_altitude_agl_m=1.8)
    w = StableWindow(strict)
    o = observed(armed=True, landed=2, position=(0., 0., 1.5))
    for i in range(20):
        assert not w.update(i * .1, o, 0.)           # 起飞高度够、锁定下限不够


def test_lock_window_restarts_after_dropping_below_takeoff_altitude():
    w = StableWindow()
    o = observed(armed=True, landed=2, position=(0., 0., 1.5))
    for i in range(10):
        w.update(i * .1, o, 0.)
    assert not w.update(1.0, replace(o, position=(0., 0., 1.2)), 0.)
    assert not w.update(1.1, o, 0.)
    for i in range(1, 10):
        assert not w.update(1.1 + i * .1, o, 0.)
    assert w.update(2.1, o, 0.)


# ---------------------------------------------------------------------------
# 全状态机主链
# ---------------------------------------------------------------------------
def test_takeoff_hold_offboard_planning_and_confirmed_disarm():
    f = Lifecycle()
    assert f.start("s", (2., 0., 1.5), 0.)
    assert f.tick(0., observed()) == ["arm"]
    assert f.state == State.TAKEOFF
    air = observed(armed=True, landed=2, position=(0., 0., 1.5))
    assert f.tick(.1, air) == ["takeoff"]
    for i in range(10):
        assert f.tick(.2 + i * .1, air) == []
    assert f.tick(1.2, air) == ["hold_setpoint"]
    assert f.state == State.HOLD_READY
    assert f.tick(2.3, air) == ["hold_setpoint"]  # no observed setpoint stream
    assert f.tick(2.4, replace(air, sending=True)) == ["hold_setpoint", "offboard"]
    assert f.state == State.OFFBOARD_PENDING
    assert f.tick(2.5, air) == ["hold_setpoint"]  # sent mode command is not confirmation
    assert f.tick(2.6, replace(air, offboard=True, mode_detail="offboard")) == ["hold_setpoint", "enable_planner"]
    goal = replace(air, offboard=True, mode_detail="offboard", position=(2., 0., 1.5))
    assert f.tick(3., goal) == []
    assert f.tick(4.1, goal) == ["disable_planner", "cancel", "land"]
    f.tick(5., replace(goal, landed=1))
    assert f.state == State.LANDING  # still armed
    f.tick(5.1, replace(goal, landed=1, armed=False))
    assert f.state == State.COMPLETE


def test_offboard_confirmation_requires_mode_readback_not_the_command_result():
    f, air = offboard_pending()
    # 命令被接受（offboard=True），但模式回读还是 auto:loiter → 不算确认
    assert f.tick(2.4, replace(air, offboard=True, mode_detail="auto:loiter")) == ["hold_setpoint"]
    assert f.state == State.OFFBOARD_PENDING
    # 完整模式名已经回读 offboard；布尔值可能因心跳年龄短暂变假。
    assert f.tick(2.5, replace(air, offboard=False, mode_detail="offboard")) == ["hold_setpoint", "enable_planner"]
    assert f.state == State.EXECUTING


def test_offboard_not_confirmed_within_mode_timeout_latches():
    f, air = offboard_pending()
    assert f.tick(2.4, replace(air, offboard=False, mode_detail="auto:loiter")) == ["hold_setpoint"]
    assert f.tick(5.4, replace(air, offboard=False, mode_detail="auto:loiter")) == ["disable_planner", "cancel"]
    assert f.state == State.FAULT_LATCHED
    assert f.reason == "offboard_not_confirmed"


def test_goal_tolerance_and_settle_time_come_from_runtime_config():
    f = executing()
    near = replace(airborne(), position=(2.25, 0., 1.5))
    assert f.tick(3., near) == []
    assert f.tick(4.1, near) == ["disable_planner", "cancel", "land"]


def test_offboard_observed_prefers_a_known_mode_detail():
    assert offboard_observed(airborne(offboard=True, mode_detail="offboard"))
    assert not offboard_observed(airborne(offboard=True, mode_detail="auto:land"))
    assert offboard_observed(airborne(offboard=False, mode_detail="offboard"))
    assert not offboard_observed(airborne(offboard=True, mode_detail="unknown"))
    assert not offboard_observed(airborne(offboard=False, mode_detail=""))


@pytest.mark.parametrize("changed,reason", [
    (dict(boot_epoch=1), "flight_controller_restart"),
    (dict(session="old"), "session_changed"),
    (dict(fault="manual_mode"), "manual_mode"),
    (dict(fault="unknown"), "unknown"),
    (dict(fault="frame_reset"), "frame_reset"),
    (dict(fault="geometry_changed"), "geometry_changed"),
    (dict(status_age=4.), "status_missing"),
])
def test_faults_latch_and_do_not_restart(changed, reason):
    f = Lifecycle()
    f.start("s", (2., 0., 1.5), 0.)
    f.tick(0., observed())
    assert f.tick(.1, observed(**changed)) == ["disable_planner", "cancel"]
    assert f.state == State.FAULT_LATCHED
    assert f.reason == reason
    assert f.tick(1., observed()) == []
    assert not f.start("new", (0., 0., 1.), 1.)


@pytest.mark.parametrize("state", [State.PRECHECK, State.TAKEOFF, State.HOLD_READY,
                                   State.OFFBOARD_PENDING, State.EXECUTING, State.RECOVERING])
def test_any_active_state_can_enter_fault_latched(state):
    f = Lifecycle(RECOVERY)
    assert f.start("s", (2., 0., 1.5), 0.)
    f.transition(state, 1.)
    f.ground_z, f.boot_epoch = 0., 0.
    o = observed() if state == State.PRECHECK else airborne()
    assert f.tick(2., replace(o, fault="unknown")) == ["disable_planner", "cancel"]
    assert f.state == State.FAULT_LATCHED
    assert f.reason == "unknown"
    assert f.recovery_revoked is True
    assert not f.start("s2", (1., 0., 1.), 3.)


def test_mode_confirmation_timeout_and_manual_cancel():
    f = Lifecycle()
    f.start("s", (2., 0., 1.5), 0.)
    f.transition(State.OFFBOARD_PENDING, 0.)
    assert f.tick(3., observed(armed=True, landed=2)) == ["disable_planner", "cancel"]
    assert f.reason == "offboard_not_confirmed"
    g = Lifecycle()
    g.start("s", (2., 0., 1.5), 0.)
    assert g.cancel(.1) == ["disable_planner", "cancel"]
    assert g.cancelled
    assert g.tick(.2, observed()) == []


# ---------------------------------------------------------------------------
# 自动恢复（B）
# ---------------------------------------------------------------------------
def test_auto_recovery_is_disabled_by_default():
    assert DEFAULTS.recovery_enabled is False
    assert RECOVERY.recovery_enabled is True
    f = executing(DEFAULTS)
    assert f.tick(3., airborne()) == []
    assert f.tick(3.1, AUTO_LAND) == ["disable_planner", "cancel"]
    assert f.state == State.FAULT_LATCHED
    assert f.reason == "px4_auto_land"
    assert f.recovery_attempts == 0
    assert f.recovery_revoked is True
    assert not f.recovery_available("px4_auto_land")


def test_recovering_is_reachable_and_requires_gate_valid_window_and_readback():
    f = recovering()
    assert f.recovery_fault == "px4_auto_land"
    assert f.recovery_source == "status.mode_detail=auto:land"
    assert f.recovery_hold == (2., 0., 1.5)
    assert f.recovery_attempts == 1
    # 既有执行许可（px4_failsafe 放行）没打开：只保持，不请求 OFFBOARD
    assert f.tick(3.3, replace(AUTO_LAND, sending=False)) == ["hold_setpoint"]
    assert f.recovery_request_at is None
    assert "execution_gate_closed" in f.recovery_detail
    # 许可打开后还要连续 recovery_valid_duration_s 有效（窗口自 3.2 起）
    for t in (3.4, 4.0, 4.1):
        assert f.tick(t, AUTO_LAND) == ["hold_setpoint"]
    assert f.recovery_request_at is None
    assert f.tick(4.2, AUTO_LAND) == ["hold_setpoint", "offboard"]
    assert f.recovery_request_at == 4.2
    assert f.recovery_attempts == 1
    # 命令成功 ≠ 模式回读成功：offboard=True 但模式名仍是 auto:land → 不算确认
    assert f.tick(4.3, replace(AUTO_LAND, offboard=True)) == ["hold_setpoint"]
    assert f.state == State.RECOVERING
    assert f.recovery_detail == "awaiting_mode_readback"
    # 回读确认 → 重新规划剩余目标（禁止续播旧轨迹）
    actions = f.tick(4.4, replace(AUTO_LAND, offboard=True, mode_detail="offboard"))
    assert f.state == State.EXECUTING
    assert actions == ["hold_setpoint", "replan", "enable_planner"]
    assert f.recovery_detail == "confirmed"
    assert f.hold == (2., 0., 1.5)      # 恢复后的 hold 停在当前位置，不跳回起飞锁点
    assert f.recovery_attempts == 1


def test_recovery_never_reenables_the_old_trajectory_before_confirmation():
    f = recovering()
    collected = []
    for t in (3.3, 3.4, 4.2, 4.3):
        collected.extend(f.tick(t, AUTO_LAND))
    assert f.state == State.RECOVERING
    assert "enable_planner" not in collected
    assert collected.count("offboard") == 1          # 一次尝试只请求一次
    actions = f.tick(4.4, replace(AUTO_LAND, offboard=True, mode_detail="offboard"))
    assert "replan" in actions and "enable_planner" in actions


def test_recovery_mode_timeout_exhausts_attempts_then_latches_without_self_unlock():
    f = recovering()
    assert f.tick(4.2, AUTO_LAND) == ["hold_setpoint", "offboard"]
    assert f.recovery_attempts == 1
    # 首次请求 3 s 内没有模式回读 → 第 2 次尝试
    assert f.tick(7.2, AUTO_LAND) == ["hold_setpoint", "offboard"]
    assert f.recovery_attempts == 2
    assert f.recovery_detail == "offboard_requested"
    # 第 2 次仍无回读 → 闭锁（次数耗尽，不循环争抢）
    assert f.tick(10.3, AUTO_LAND) == ["disable_planner", "cancel"]
    assert f.state == State.FAULT_LATCHED
    assert f.reason == "px4_auto_land"
    assert f.recovery_detail == "exhausted"
    assert f.recovery_attempts == f.config.recovery_attempts == 2
    assert f.recovery_exhausted
    assert not f.recovery_available("px4_auto_land")
    # 不自动重新解锁：完美观测也保持闭锁，且不能开新任务
    assert f.tick(11., replace(AUTO_LAND, offboard=True, mode_detail="offboard")) == []
    assert f.state == State.FAULT_LATCHED
    assert not f.start("s2", (1., 0., 1.5), 12.)


def test_recovery_budget_is_not_reset_by_a_new_fault_episode():
    f = recovering()
    assert f.tick(4.2, AUTO_LAND) == ["hold_setpoint", "offboard"]
    assert f.recovery_attempts == 1
    assert f.tick(4.3, replace(AUTO_LAND, offboard=True, mode_detail="offboard")) == ["hold_setpoint", "replan", "enable_planner"]
    assert f.state == State.EXECUTING
    # 新故障（AUTO Return）：预算不重置，只剩 1 次
    returned = replace(airborne(), offboard=False, mode_detail="auto:rtl", current_mode_detail="auto:rtl", px4_safety_mode="auto:rtl",
                       px4_failsafe_cause="offboard_link", px4_safety_age_s=0.)
    assert f.tick(5.0, replace(returned, fault="sensor_link")) == ["disable_planner", "recover", "hold_setpoint"]
    assert f.recovery_fault == "px4_auto_return"
    assert f.recovery_attempts == 2
    assert f.tick(5.1, returned) == ["hold_setpoint"]        # 新恢复段的连续有效期重新累积
    assert f.tick(6.1, returned) == ["hold_setpoint", "offboard"]
    assert f.tick(9.2, returned) == ["disable_planner", "cancel"]
    assert f.state == State.FAULT_LATCHED
    assert f.recovery_detail == "exhausted"


@pytest.mark.parametrize("change,reason", [
    (dict(mode_detail="position"), "manual_mode"),
    (dict(mode_detail="manual"), "manual_mode"),
    (dict(mode_detail="auto:mission"), "manual_mode"),
    (dict(mode_detail="auto:loiter"), "manual_mode"),
    (dict(fault="unknown"), "unknown"),
    (dict(fault="frame_reset"), "frame_reset"),
    (dict(fault="geometry_changed"), "geometry_changed"),
    (dict(armed=False, landed=1), "landed_or_disarmed"),
    (dict(boot_epoch=1), "flight_controller_restart"),
    (dict(session="other"), "session_changed"),
])
def test_recovery_eligibility_is_permanently_revoked(change, reason):
    f = recovering()
    assert f.tick(3.5, replace(AUTO_LAND, **change)) == ["disable_planner", "cancel"]
    assert f.state == State.FAULT_LATCHED
    assert f.reason == reason
    assert f.recovery_revoked is True
    assert f.recovery_detail.startswith("revoked:")
    assert not f.recovery_available("px4_auto_land")
    # 撤销是永久的：随后观测完美也不再恢复
    assert f.tick(4.5, replace(AUTO_LAND, offboard=True, mode_detail="offboard")) == []
    assert f.state == State.FAULT_LATCHED


def test_manual_cancel_during_recovery_revokes_eligibility_immediately():
    f = recovering()
    assert f.cancel(3.5) == ["disable_planner", "cancel"]
    assert f.state == State.FAULT_LATCHED
    assert f.reason == "manual_cancel"
    assert f.recovery_revoked is True
    assert not f.recovery_available("px4_auto_land")
    assert f.tick(4.5, AUTO_LAND) == []


def test_manual_land_during_recovery_revokes_eligibility():
    f = recovering()
    assert f.land(3.5) == ["disable_planner", "cancel", "land"]
    assert f.state == State.LANDING
    assert f.recovery_revoked is True
    assert not f.recovery_available("px4_auto_land")
    assert f.tick(4.5, AUTO_LAND) == []
    assert f.state == State.LANDING


@pytest.mark.parametrize("change,why", [
    (dict(landed=None), "not_airborne"),
    (dict(landed=0), "not_airborne"),
    (dict(status_age=2.0), "status_stale"),      # 超过 status_timeout_s，尚未超过 mode_timeout_s
    (dict(pose_age=.2), "pose_stale"),
    (dict(position=(math.nan, 0., 1.5)), "position_unknown"),
])
def test_recovery_is_rejected_when_observation_is_missing_or_stale(change, why):
    f = executing(RECOVERY)
    assert f.tick(3., airborne()) == []
    o = replace(AUTO_LAND, **change)
    assert f.tick(3.1, o) == ["disable_planner", "cancel"]
    assert f.state == State.FAULT_LATCHED
    assert f.reason == "px4_auto_land"
    assert f.recovery_detail == f"rejected:{why}"
    assert f.recovery_attempts == 0
    assert not f.recovery_available("px4_auto_land")


def test_disarmed_or_landed_observation_latches_and_revokes_before_recovery():
    f = executing(RECOVERY)
    assert f.tick(3., airborne()) == []
    assert f.tick(3.1, replace(airborne(), armed=False, landed=1)) == ["disable_planner", "cancel"]
    assert f.reason == "landed_or_disarmed"
    assert f.recovery_revoked is True


def test_recovery_valid_window_restarts_after_any_input_break():
    f = recovering()
    assert f.tick(3.5, replace(AUTO_LAND, map_ready=False)) == ["hold_setpoint"]
    assert f.recovery_valid_since is None
    for t in (3.6, 4.0, 4.4, 4.5):
        assert f.tick(t, AUTO_LAND) == ["hold_setpoint"]
    assert f.recovery_request_at is None
    assert "valid_duration_not_met" in f.recovery_detail
    assert f.tick(4.6, AUTO_LAND) == ["hold_setpoint", "offboard"]


def test_recovery_below_min_altitude_never_requests_offboard_and_exhausts_budget():
    f = recovering()
    low = replace(AUTO_LAND, position=(2., 0., .5))
    seen, details = [], []
    t = 3.3
    while f.state == State.RECOVERING and t < 20.:
        details.append(f.recovery_detail)
        seen.extend(f.tick(t, low))
        t += .1
    assert f.state == State.FAULT_LATCHED
    assert f.recovery_detail == "exhausted"
    assert any("below_min_altitude" in d for d in details)
    assert f.recovery_request_at is None
    assert "offboard" not in seen


def test_recovery_above_max_speed_never_requests_offboard():
    f = recovering()
    fast = replace(AUTO_LAND, velocity=(1.2, 0., 0.))
    assert f.tick(4.2, fast) == ["hold_setpoint"]
    assert "above_max_speed" in f.recovery_detail
    assert f.recovery_request_at is None


@pytest.mark.parametrize("detail,fault", [("auto:land", "px4_auto_land"), ("auto:rtl", "px4_auto_return")])
def test_auto_land_and_return_are_interruptible_only_with_recovery_enabled(detail, fault):
    f = executing(RECOVERY)
    assert f.tick(3., airborne()) == []
    assert f.tick(3.1, replace(airborne(), offboard=False, mode_detail=detail, current_mode_detail=detail, px4_safety_mode=detail,
                    px4_failsafe_cause="offboard_link", px4_safety_age_s=0., fault="sensor_link")) == ["disable_planner", "recover", "hold_setpoint"]
    assert f.state == State.RECOVERING
    assert f.recovery_fault == fault
    assert f.recovery_source == f"status.mode_detail={detail}"
    g = executing(DEFAULTS)
    assert g.tick(3., airborne()) == []
    assert g.tick(3.1, replace(airborne(), offboard=False, mode_detail=detail, current_mode_detail=detail, px4_safety_mode=detail,
                    px4_failsafe_cause="offboard_link", px4_safety_age_s=0., fault="sensor_link")) == ["disable_planner", "cancel"]
    assert g.state == State.FAULT_LATCHED
    assert g.reason == fault


@pytest.mark.parametrize("detail", ["position", "manual", "altitude", "auto:mission", "auto:loiter", "auto:follow_target"])
def test_observed_modes_other_than_auto_land_return_are_never_raced(detail):
    f = executing(RECOVERY)
    assert f.tick(3., airborne()) == []
    assert f.tick(3.1, replace(airborne(), offboard=False, mode_detail=detail)) == ["disable_planner", "cancel"]
    assert f.state == State.FAULT_LATCHED
    assert f.reason == "manual_mode"
    assert f.recovery_revoked is True
    assert f.recovery_attempts == 0


def test_executing_alignment_loss_latches_as_frame_reset_without_recovery():
    f = executing(RECOVERY)
    assert f.tick(3., airborne()) == []
    assert f.tick(3.1, replace(airborne(), alignment=False)) == ["disable_planner", "cancel"]
    assert f.reason == "frame_reset"
    assert f.recovery_revoked is True
    assert not f.recovery_available("offboard_lost")


def test_unknown_mode_readback_latches():
    f = executing(RECOVERY)
    assert f.tick(3., airborne()) == []
    assert f.tick(3.1, replace(airborne(), offboard=False, mode_detail="unknown")) == ["disable_planner", "cancel"]
    assert f.state == State.FAULT_LATCHED
    assert f.reason == "unknown"
    assert f.recovery_attempts == 0


@pytest.mark.parametrize("change", [
    dict(pose_age=.3), dict(landed=None), dict(connected=False), dict(sensors_ready=False),
    dict(map_ready=False), dict(sending=False),
    dict(position=(2., 0., .2)), dict(velocity=(2., 0., 0.)),
])
def test_recovery_readback_rechecks_current_conditions(change):
    f = executing(RECOVERY)
    assert f.tick(3.1, replace(airborne(), fault="planning_link")) == ["disable_planner", "recover", "hold_setpoint"]
    assert f.tick(3.2, airborne()) == ["hold_setpoint"]
    assert f.tick(4.2, airborne()) == ["hold_setpoint", "offboard"]
    assert f.tick(4.3, replace(airborne(), **change)) == ["disable_planner", "cancel"]
    assert f.state == State.FAULT_LATCHED
    assert f.reason == "recovery_conditions_lost"
    assert f.recovery_revoked


def test_unknown_mode_and_unqualified_auto_land_never_recover():
    for detail in ("unknown", "auto:land", "auto:rtl"):
        f = executing(RECOVERY)
        assert f.tick(3.1, replace(airborne(), offboard=False, mode_detail=detail)) == ["disable_planner", "cancel"]
        assert f.state == State.FAULT_LATCHED
        assert f.recovery_attempts == 0
    f = executing(RECOVERY)
    assert f.tick(3.1, replace(airborne(), mode_detail="manual", fault="sensor_link")) == ["disable_planner", "cancel"]
    assert f.reason == "manual_mode"


def test_recognized_sensor_fault_can_stay_recovering_after_auto_land():
    f = executing(RECOVERY)
    assert f.tick(3.1, replace(airborne(), fault="sensor_link")) == ["disable_planner", "recover", "hold_setpoint"]
    assert f.tick(3.2, replace(AUTO_LAND, fault="")) == ["hold_setpoint"]
    assert f.state == State.RECOVERING


def test_recovery_braking_setpoint_has_bounded_acceleration():
    f = executing(RECOVERY)
    o = replace(airborne(), position=(2., 0., 1.5), velocity=(.8, 0., 0.), fault="sensor_link")
    assert f.tick(3.1, o) == ["disable_planner", "recover", "hold_setpoint"]
    assert f.hold_setpoint(3.1) == ((2., 0., 1.5), (.8, 0., 0.), (-1., 0., 0.))
    p, v, a = f.hold_setpoint(3.5)
    assert p[0] == pytest.approx(2.24)
    assert v[0] == pytest.approx(.4)
    assert a == (-1., 0., 0.)
    p, v, a = f.hold_setpoint(3.9)
    assert p[0] == pytest.approx(2.32)
    assert v == (0., 0., 0.) and a == (0., 0., 0.)
    assert "braking" in f._recovery_ready(3.5, replace(o, fault=""))[1]


def test_brief_offboard_boolean_drop_does_not_latch_when_mode_is_offboard():
    f = executing(RECOVERY)
    assert f.tick(3.1, replace(airborne(), offboard=False, mode_detail="offboard")) == []
    assert f.state == State.EXECUTING


@pytest.mark.parametrize("detail", ["auto:land", "auto:rtl"])
@pytest.mark.parametrize("intention,age,current", [
    ("auto:land", 0., None), ("auto:rtl", 0., None),
    ("unknown", 0., None), ("offboard", 2., None),
    ("offboard", 0., "offboard"),
])
def test_auto_mode_requires_fresh_matching_px4_user_intention(detail, intention, age, current):
    f = executing(RECOVERY)
    o = replace(airborne(), mode_detail=detail, current_mode_detail=current or detail,
                intended_mode_detail=intention, current_mode_age_s=age,
                offboard=False, fault="sensor_link")
    f.tick(3.1, o)
    assert f.state == State.FAULT_LATCHED
    assert f.reason == "manual_mode"
    assert f.recovery_revoked


@pytest.mark.parametrize("cause,age", [("", 0.), ("offboard_link", 2.), ("unknown", 0.)])
def test_auto_recovery_requires_authoritative_px4_fault_observation(cause, age):
    f = recovering()
    o = replace(AUTO_LAND, px4_failsafe_cause=cause, px4_safety_age_s=age)
    actions = f.tick(4.2, o)
    assert "offboard" not in actions
    if cause == "unknown":
        assert f.state == State.FAULT_LATCHED
    else:
        assert "px4_failsafe_cause_not_verified" in f.recovery_detail


def test_stale_sensor_fault_retires_output_then_waits_without_requesting_control():
    f = executing(RECOVERY)
    lost = replace(airborne(), pose_age=.4, sensors_ready=False, fault="sensor_link")
    assert f.tick(3., lost) == ["disable_planner", "recover"]
    assert f.state == State.RECOVERING
    assert f.tick(3.2, lost) == []
    assert f.recovery_request_at is None
    fresh = airborne()
    assert f.tick(3.3, fresh) == ["hold_setpoint"]
    assert f.recovery_hold == fresh.position
    assert f.recovery_request_at is None
    assert "offboard" not in f.tick(4.2, fresh)
    assert "offboard" in f.tick(4.31, fresh)


def test_verified_auto_land_can_recover_while_landed_state_is_landing():
    f = executing(RECOVERY)
    landing = replace(AUTO_LAND, landed=4)
    assert "recover" in f.tick(3., landing)
    assert f.tick(3.1, landing) == ["hold_setpoint"]
    assert "offboard" in f.tick(4.11, landing)
    low = replace(landing, position=(0., 0., .5))
    assert "offboard" not in f.tick(4.2, low)
    assert f.recovery_request_at is None or f.state == State.RECOVERING


def test_new_recovery_budget_requires_a_continuous_healthy_interval():
    f = recovering()
    f.goal = (20., 0., 1.5)
    f.tick(3.5, AUTO_LAND)
    f.tick(4.51, AUTO_LAND)
    f.tick(4.6, airborne())
    assert f.recovery_total_attempts == 1
    f.tick(4.7, airborne())
    f.tick(5., replace(airborne(), sensors_ready=False))
    assert f.recovery_attempts == 1
    f.tick(5.1, airborne())
    f.tick(6.11, airborne())
    assert f.recovery_attempts == 0
    assert f.recovery_total_attempts == 1
    f.tick(6.2, replace(airborne(), fault="setpoint_link"))
    assert f.recovery_attempts == 1
    assert f.recovery_total_attempts == 2


def test_authoritative_unknown_px4_fault_closes_offboard_recovery():
    f = executing(RECOVERY)
    unknown = replace(airborne(), px4_safety_mode="offboard", px4_safety_age_s=.1,
                      px4_failsafe_cause="unknown", fault="sensor_link")
    assert f.tick(3., unknown) == ["disable_planner", "cancel"]
    assert f.reason == "unknown"
    f.land(3.1)
    assert any(e["state"] == "FAULT_LATCHED" and e["reason"] == "unknown" for e in f.events)


@pytest.mark.parametrize("fields", [dict(frame_reset_known=False), dict(frame_reset_age_s=1.01),
                                    dict(frame_reset_age_s=float("inf")), dict(frame_reset_age_s=-.1)])
def test_recovery_requires_fresh_reset_counter_observation(fields):
    f = Lifecycle(RECOVERY)
    f.start("s", (20., 0., 1.5), 0.)
    f.ground_z = 0.
    f.transition(State.EXECUTING, 1.)
    f.inflight_fault_actions(2., airborne(fault="sensor_link", **fields), "sensor_link")
    assert f.state == State.FAULT_LATCHED
    assert f.recovery_total_attempts == 0
    assert f.recovery_detail == "rejected:reset_observation_missing"


def test_missing_pose_timestamp_cannot_enter_sensor_recovery_wait():
    f = Lifecycle(RECOVERY)
    f.start("s", (20., 0., 1.5), 0.)
    f.ground_z = 0.
    f.transition(State.EXECUTING, 1.)
    f.inflight_fault_actions(2., airborne(fault="sensor_link", pose_age=float("inf")), "sensor_link")
    assert f.state == State.FAULT_LATCHED
    assert f.recovery_total_attempts == 0
