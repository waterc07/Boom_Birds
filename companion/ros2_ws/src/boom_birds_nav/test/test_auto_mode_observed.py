"""AUTO 子模式观测：Land 与 Return 必须可区分，且阈值只有一个定义点。

为什么单独一个文件：`ExecutionStatus.mode` 过去只有主模式，AUTO Land（sub=6）与
AUTO Return/RTL（sub=5）的 main_mode 都是 auto(4)，下游无法区分"正在降落"和
"正在返航"。恢复逻辑与验收都依赖这个区分，因此把观测路径单独锁住。
"""

import pytest

from boom_birds_control.px4_backend import (
    FakePx4Backend,
    PX4_CUSTOM_MAIN_MODE_AUTO,
    PX4_CUSTOM_MAIN_MODE_MANUAL,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
    VehicleState,
    px4_custom_main_mode_name,
    px4_custom_mode,
    px4_custom_sub_mode,
    px4_custom_sub_mode_name,
)
from boom_birds_control.px4_failsafe import (
    CONTRACT_TIMING_REFERENCE,
    FailsafeConfig,
    SignalId,
)
from boom_birds_control.runtime_config import DEFAULTS


# SITL 实测过的真实值：0x03040000 = main AUTO(4) / sub LOITER(3)。
# 历史上曾按 >>8 / >>16 解错，这里把它作为回归锚点保留。
SITL_OBSERVED_AUTO_LOITER = 0x03040000


def test_sitl_observed_value_decodes_as_auto_loiter():
    assert px4_custom_main_mode_name(SITL_OBSERVED_AUTO_LOITER) == "auto"
    assert px4_custom_sub_mode(SITL_OBSERVED_AUTO_LOITER) == 3
    assert px4_custom_sub_mode_name(SITL_OBSERVED_AUTO_LOITER) == "auto:loiter"


@pytest.mark.parametrize("main,sub,expected", [
    (PX4_CUSTOM_MAIN_MODE_AUTO, 6, "auto:land"),
    (PX4_CUSTOM_MAIN_MODE_AUTO, 5, "auto:rtl"),
    (PX4_CUSTOM_MAIN_MODE_AUTO, 3, "auto:loiter"),
    (PX4_CUSTOM_MAIN_MODE_AUTO, 2, "auto:takeoff"),
    (PX4_CUSTOM_MAIN_MODE_AUTO, 4, "auto:mission"),
    (PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0, "offboard"),
    (PX4_CUSTOM_MAIN_MODE_MANUAL, 0, "manual"),
    (PX4_CUSTOM_MAIN_MODE_AUTO, 42, "auto:42"),   # 未知子模式不编名字
])
def test_sub_mode_names(main, sub, expected):
    assert px4_custom_sub_mode_name(px4_custom_mode(main, sub)) == expected


def test_land_and_return_are_distinguishable():
    land = px4_custom_mode(PX4_CUSTOM_MAIN_MODE_AUTO, 6)
    rtl = px4_custom_mode(PX4_CUSTOM_MAIN_MODE_AUTO, 5)
    # 主模式相同 —— 这正是"只报主模式不够"的证据
    assert px4_custom_main_mode_name(land) == px4_custom_main_mode_name(rtl) == "auto"
    assert px4_custom_sub_mode_name(land) != px4_custom_sub_mode_name(rtl)


def test_unknown_custom_mode_is_none_not_guessed():
    assert px4_custom_sub_mode_name(None) is None
    assert px4_custom_sub_mode_name(px4_custom_mode(200, 0)) is None


def test_vehicle_state_predicates():
    land = VehicleState(custom_mode=px4_custom_mode(PX4_CUSTOM_MAIN_MODE_AUTO, 6),
                        custom_main_mode=PX4_CUSTOM_MAIN_MODE_AUTO, custom_sub_mode=6)
    rtl = VehicleState(custom_mode=px4_custom_mode(PX4_CUSTOM_MAIN_MODE_AUTO, 5),
                       custom_main_mode=PX4_CUSTOM_MAIN_MODE_AUTO, custom_sub_mode=5)
    off = VehicleState(custom_mode=px4_custom_mode(PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0),
                       custom_main_mode=PX4_CUSTOM_MAIN_MODE_OFFBOARD, custom_sub_mode=0)
    assert land.is_auto_land and not land.is_auto_return and not land.is_offboard
    assert rtl.is_auto_return and not rtl.is_auto_land and not rtl.is_offboard
    assert off.is_offboard and not off.is_auto_land and not off.is_auto_return
    assert land.mode_detail == "auto:land" and rtl.mode_detail == "auto:rtl"


def test_backend_reports_observed_mode_after_ack():
    backend = FakePx4Backend(link_up=True, auto_ack=True)
    assert backend.set_mode("auto:land")
    state = backend.read_vehicle_state()
    assert state.mode_detail == "auto:land" and state.is_auto_land
    assert backend.set_mode("auto:rtl")
    state = backend.read_vehicle_state()
    assert state.mode_detail == "auto:rtl" and state.is_auto_return


def test_command_ack_without_observation_is_not_treated_as_mode():
    """命令成功 ≠ 模式已确认：auto_ack=False 时观测仍是原模式。"""
    backend = FakePx4Backend(link_up=True, auto_ack=False)
    assert backend.set_mode("auto:land")
    state = backend.read_vehicle_state()
    assert state.commanded_mode_name == "auto:land"
    assert state.mode_detail == "manual"
    assert not state.is_auto_land


def test_failsafe_config_takes_thresholds_from_the_single_source():
    config = FailsafeConfig.from_runtime()
    assert config.timeouts[SignalId.SETPOINT] == DEFAULTS.setpoint_timeout_s
    assert config.timeouts[SignalId.VIO_POSE] == DEFAULTS.vio_timeout_s
    assert config.timeouts[SignalId.IMU] == DEFAULTS.imu_timeout_s
    assert config.timeouts[SignalId.CAMERA] == DEFAULTS.camera_timeout_s
    assert config.timeouts[SignalId.MAVLINK_LINK] == DEFAULTS.link_timeout_s
    assert config.timeouts[SignalId.PX4_HEARTBEAT] == DEFAULTS.heartbeat_timeout_s
    assert config.timeouts[SignalId.ODOM_EGO] == DEFAULTS.vio_timeout_s
    assert config.recovery_fresh_samples == DEFAULTS.recovery_required_samples


def test_contract_timing_reference_is_derived_not_duplicated():
    assert CONTRACT_TIMING_REFERENCE["pose_timeout_s"] == DEFAULTS.pose_timeout_s
    assert CONTRACT_TIMING_REFERENCE["depth_timeout_s"] == DEFAULTS.depth_timeout_s
