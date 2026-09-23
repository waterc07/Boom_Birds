"""契约 ↔ 节点参数一致性测试。

契约（config/contract.yaml）是话题/坐标系/时间约定的唯一来源；本测试确保
`mavlink_imu_node` 与 `mavlink_imu_core` 的默认值不会悄悄偏离契约，避免
「文档写一套、代码跑另一套」。

不连接任何设备；只读配置与代码常量。
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from boom_birds_nav.mavlink_imu_core import CONTRACT_IMU_FRAME, CONTRACT_IMU_TOPIC

CONTRACT = Path(__file__).resolve().parents[1] / "config" / "contract.yaml"


@pytest.fixture(scope="module")
def contract() -> dict:
    with CONTRACT.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _topic(contract: dict, name: str) -> dict:
    for item in contract["topics"]:
        if item["name"] == name:
            return item
    raise AssertionError(f"契约中缺少话题 {name}")


def test_imu_topic_matches_contract(contract):
    item = _topic(contract, CONTRACT_IMU_TOPIC)
    assert item["type"] == "sensor_msgs/Imu"
    assert item["frame_id"] == CONTRACT_IMU_FRAME
    assert item["units"]["angular_velocity"] == "rad/s"
    assert item["units"]["linear_acceleration"] == "m/s^2"
    # 契约的 QoS 与节点发布器一致（sensor data：best_effort / depth 5）
    assert item["qos"]["reliability"] == "best_effort"
    assert int(item["qos"]["depth"]) == 5


def test_diagnostic_topics_are_in_contract(contract):
    names = {item["name"] for item in contract["topics"]}
    assert "/boom_birds/imu/mavlink_status" in names
    assert "/boom_birds/imu/diagnostics" in names
    assert _topic(contract, "/boom_birds/imu/mavlink_status")["type"] == "std_msgs/String"
    assert _topic(contract, "/boom_birds/imu/diagnostics")["type"] == (
        "diagnostic_msgs/DiagnosticArray"
    )


def test_camera_imu_offset_is_documented_and_disabled_by_default(contract):
    from boom_birds_nav.mavlink_imu_core import MavlinkImuConfig

    timing = contract["timing"]
    assert float(timing["camera_imu_offset_s"]) == 0.0
    assert bool(timing["apply_camera_imu_offset"]) is False
    cfg = MavlinkImuConfig()
    assert cfg.camera_imu_offset_s == 0.0
    assert cfg.apply_camera_imu_offset is False


def test_clock_limits_match_contract_defaults(contract):
    from boom_birds_nav.mavlink_clock import ClockMapperConfig
    from boom_birds_nav.mavlink_imu_core import MavlinkImuConfig

    timing = contract["timing"]
    clock = ClockMapperConfig()
    imu = MavlinkImuConfig()
    assert clock.max_rtt_s == pytest.approx(float(timing["imu_max_rtt_s"]))
    assert clock.max_deviation_s == pytest.approx(float(timing["imu_max_offset_deviation_s"]))
    assert clock.sync_timeout_s == pytest.approx(float(timing["imu_max_offset_age_s"]))
    assert imu.ros_offset_stale_s == pytest.approx(float(timing["imu_max_offset_age_s"]))


def test_node_declares_the_contract_backed_parameters():
    """节点必须显式声明这些参数，否则 `-p` 会被静默忽略。"""
    import inspect

    from boom_birds_nav import mavlink_imu_node

    source = inspect.getsource(mavlink_imu_node.MavlinkImuNode.__init__)
    for name in (
        "connection", "baud", "target_system", "target_component", "device_id",
        "stream_rate_hz", "timesync_rate_hz", "max_rtt_s", "sync_timeout_s",
        "camera_imu_offset_s", "apply_camera_imu_offset", "imu_topic",
        "min_sample_age_s", "max_sample_age_s",
    ):
        assert f'declare_parameter("{name}"' in source, f"节点未声明参数 {name}"
