"""契约 ↔ 节点参数一致性测试。

``config/contract.yaml`` 是话题/坐标系/单位/无效值的**语义**来源；**数值**只来自
``config/runtime.yaml`` 的校验对象（单一来源检查见 ``test_config_single_source.py``）。
本测试确保 ``mavlink_imu_node`` / ``mavlink_imu_core`` / ``mavlink_clock`` 的实际默认值
与节点参数文件 ``config/mavlink_imu.yaml`` 及契约语义不偏离，避免"文档一套、代码一套"。

不连接任何设备；只读配置与代码常量。
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from boom_birds_sensing.mavlink_imu_core import CONTRACT_IMU_FRAME, CONTRACT_IMU_TOPIC

PKG = Path(__file__).resolve().parents[1]
CONTRACT = PKG.parent / "boom_birds_interfaces" / "config" / "contract.yaml"
MAVLINK_YAML = PKG.parent / "boom_birds_sensing" / "config" / "mavlink_imu.yaml"


@pytest.fixture(scope="module")
def contract() -> dict:
    with CONTRACT.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


@pytest.fixture(scope="module")
def imu_params() -> dict:
    """节点参数文件里的实际取值（launch/命令行之外的第二处声明）。"""
    with MAVLINK_YAML.open(encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    return data["boom_birds_mavlink_imu"]["ros__parameters"]


def _topic(contract: dict, name: str) -> dict:
    for item in contract["topics"]:
        if item["name"] == name:
            return item
    raise AssertionError(f"契约中缺少话题 {name}")


def _declared(contract: dict) -> dict:
    """契约 timing 的语义声明：{名称: 条目}。数值不在这里，只在 runtime.yaml。"""
    return {str(item.get("name", "")): item for item in contract["timing"]["declared"]}


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


def test_camera_imu_offset_is_declared_and_disabled_by_default(contract, imu_params):
    from boom_birds_sensing.mavlink_imu_core import MavlinkImuConfig

    entry = _declared(contract)["camera_imu_offset_s"]
    # 契约只声明语义与来源；数值来自 mavlink_imu.yaml / 节点默认值
    assert "mavlink_imu.yaml" in str(entry.get("source", ""))
    assert entry.get("invalid"), "契约必须写明未标定时的处理"
    assert float(imu_params["camera_imu_offset_s"]) == 0.0
    assert bool(imu_params["apply_camera_imu_offset"]) is False
    cfg = MavlinkImuConfig()
    assert cfg.camera_imu_offset_s == 0.0
    assert cfg.apply_camera_imu_offset is False


def test_clock_limits_match_node_defaults(contract, imu_params):
    from boom_birds_sensing.mavlink_clock import ClockMapperConfig
    from boom_birds_sensing.mavlink_imu_core import MavlinkImuConfig

    clock = ClockMapperConfig()
    imu = MavlinkImuConfig()
    assert clock.max_rtt_s == pytest.approx(float(imu_params["max_rtt_s"]))
    assert clock.max_deviation_s == pytest.approx(float(imu_params["max_offset_deviation_s"]))
    assert clock.sync_timeout_s == pytest.approx(float(imu_params["sync_timeout_s"]))
    assert imu.ros_offset_stale_s == pytest.approx(float(imu_params["sync_timeout_s"]))
    assert imu.max_sample_age_s == pytest.approx(float(imu_params["max_sample_age_s"]))
    assert imu.heartbeat_timeout_s == pytest.approx(float(imu_params["heartbeat_timeout_s"]))
    assert imu.pending_timeout_s == pytest.approx(float(imu_params["pending_timeout_s"]))
    assert imu.expected_rate_hz == pytest.approx(float(imu_params["expected_rate_hz"]))
    # 数值统一在 runtime.yaml / 节点默认值里；契约的 timing 只留语义
    timing = contract["timing"]
    assert "max_rtt_s" not in timing and "imu_max_rtt_s" not in timing


def test_node_declares_the_contract_backed_parameters():
    """节点必须显式声明这些参数，否则 `-p` 会被静默忽略。"""
    import inspect

    from boom_birds_sensing import mavlink_imu_node

    source = inspect.getsource(mavlink_imu_node.MavlinkImuNode.__init__)
    for name in (
        "connection", "baud", "target_system", "target_component", "device_id",
        "stream_rate_hz", "timesync_rate_hz", "max_rtt_s", "sync_timeout_s",
        "camera_imu_offset_s", "apply_camera_imu_offset", "imu_topic",
        "min_sample_age_s", "max_sample_age_s",
    ):
        assert f'declare_parameter("{name}"' in source, f"节点未声明参数 {name}"


def test_yaml_only_sets_declared_parameters(imu_params):
    """mavlink_imu.yaml 里的键必须是节点真正声明过的参数（拼错即静默失效）。"""
    import inspect

    from boom_birds_sensing import mavlink_imu_node

    source = inspect.getsource(mavlink_imu_node.MavlinkImuNode.__init__)
    undeclared = sorted(name for name in imu_params if f'declare_parameter("{name}"' not in source)
    assert not undeclared, f"mavlink_imu.yaml 设置了未声明的参数：{undeclared}"
