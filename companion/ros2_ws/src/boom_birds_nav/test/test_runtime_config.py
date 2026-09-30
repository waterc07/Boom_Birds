"""运行配置唯一来源的强制约束。

这些测试是"A1 配置统一"的可执行部分：
- 文件模板必须与代码默认值逐一相等；
- 场景只能覆盖原点；
- 校验必须真的拒绝坏值（类型、非有限值、非正阈值）；
- 高度量的参考系必须被登记，防止再出现"同一个 height 被两处按不同基准解释"。
"""

import math
from pathlib import Path

import pytest
import yaml

from boom_birds_control.runtime_config import (
    ALTITUDE_REFERENCE,
    DEFAULTS,
    SCENES,
    RuntimeConfig,
    config_path,
)


@pytest.mark.parametrize("kwargs", [
    {"pose_timeout_s": 0}, {"origin_x": math.nan}, {"world_frame": ""},
    {"recovery_attempts": 1.5}, {"takeoff_altitude_agl_m": -1.0},
    {"handoff_max_speed_m_s": 0.0}, {"recovery_enabled": 1},
    {"px4_takeoff_param_name": ""},
])
def test_bad_configuration(kwargs):
    with pytest.raises(ValueError):
        RuntimeConfig(**kwargs)


def test_unknown_key_rejected(tmp_path):
    path = tmp_path / "cfg.yaml"
    path.write_text("typo_timeout: 1", encoding="utf-8")
    with pytest.raises(TypeError):
        RuntimeConfig.load(path)


def test_origins_can_be_negative():
    assert RuntimeConfig(origin_x=-15.).origin_x == -15.


@pytest.mark.parametrize("values", [
    {"pose_timeout_s": True}, {"origin_x": "bad"}, {"world_frame": 1},
    {"recovery_attempts": True}, {"recovery_enabled": "yes"},
])
def test_wrong_parameter_types(values):
    with pytest.raises(ValueError):
        RuntimeConfig(**values)


def test_packaged_template_matches_code_defaults():
    """config/runtime.yaml 是模板，不是第二份定义：必须与默认值逐一相等。"""
    path = config_path()
    assert path.is_file(), f"缺少运行配置模板：{path}"
    assert RuntimeConfig.load(path) == DEFAULTS


def test_template_defines_versioned_scenes_with_expected_origins():
    scenes = RuntimeConfig.load_scenes(config_path())
    assert set(scenes) == {"local", "forest_30m", "recovery_local"}
    assert scenes["local"].origin == (0.0, 0.0, 0.0)
    # 30 m 森林场景：PX4 局部原点在 ROS global 的 (-15, 0, 0.1)
    assert scenes["forest_30m"].origin == (-15.0, 0.0, 0.1)


def test_module_scenes_come_from_the_template():
    assert set(SCENES) == {"local", "forest_30m", "recovery_local"}
    assert SCENES["forest_30m"].origin == (-15.0, 0.0, 0.1)


def test_scene_origin_override_is_legal(tmp_path):
    path = tmp_path / "runtime.yaml"
    path.write_text(
        yaml.safe_dump({
            **DEFAULTS.as_mapping(),
            "scenes": {"local": {"origin_x": 1.0}},
        }),
        encoding="utf-8",
    )
    scenes = RuntimeConfig.load_scenes(path)
    assert scenes["local"].origin == (1.0, 0.0, 0.0)


@pytest.mark.parametrize("non_origin", [
    {"pose_timeout_s": 5.0},      # 阈值不得按场景覆盖
    {"recovery_enabled": True},   # 恢复开关不得按场景覆盖
    {"depth_topic": "/other"},
])
def test_scene_may_only_override_origins(tmp_path, non_origin):
    path = tmp_path / "runtime.yaml"
    path.write_text(
        yaml.safe_dump({
            **DEFAULTS.as_mapping(),
            "scenes": {"local": dict(non_origin)},
        }),
        encoding="utf-8",
    )
    with pytest.raises(ValueError):
        RuntimeConfig.load_scenes(path)


def test_scenes_must_define_local(tmp_path):
    path = tmp_path / "runtime.yaml"
    path.write_text(
        yaml.safe_dump({**DEFAULTS.as_mapping(), "scenes": {"other": {}}}),
        encoding="utf-8",
    )
    with pytest.raises(ValueError):
        RuntimeConfig.load_scenes(path)


def test_declared_altitudes_have_a_reference_frame():
    """三个高度量独立命名，且每个都登记了参考系。"""
    names = {"takeoff_altitude_agl_m", "hold_lock_min_altitude_agl_m",
             "image_publish_min_altitude_agl_m", "recovery_min_altitude_agl_m"}
    assert names <= set(ALTITUDE_REFERENCE)
    fields = set(DEFAULTS.as_mapping())
    assert names <= fields
    # 不得再出现含糊的 "height" / "min_altitude" 名字
    assert not [f for f in fields if f.endswith("_height_m")]


def test_override_helper_stays_validated():
    derived = DEFAULTS.with_overrides(origin_x=-15.0, origin_z=0.1)
    assert derived.origin == (-15.0, 0.0, 0.1)
    assert derived.takeoff_altitude_agl_m == DEFAULTS.takeoff_altitude_agl_m
    with pytest.raises(ValueError):
        DEFAULTS.with_overrides(pose_timeout_s=0)


def test_recovery_is_disabled_by_default():
    """自动恢复默认关闭：真实设备配置不得默认启用。"""
    assert DEFAULTS.recovery_enabled is False
    assert DEFAULTS.recovery_attempts == 2
    assert DEFAULTS.mode_timeout_s == 3.0


def test_config_path_resolves_in_installed_layout():
    """安装后配置在 share/，不在模块旁边；解析必须找到它。"""
    path = config_path()
    assert path.is_file()
    assert path.name == "runtime.yaml"
    # 候选里必须包含 ament share 位置，否则安装后必然找不到
    from boom_birds_control.runtime_config import config_candidates

    names = [str(c) for c in config_candidates()]
    assert any("share" in n and "runtime.yaml" in n for n in names), names


def test_missing_config_fails_loudly(monkeypatch):
    """找不到配置必须明确失败，不允许静默退回只有 local 的场景表。"""
    from boom_birds_control import runtime_config as rc

    monkeypatch.setattr(rc, "config_candidates", lambda: [Path("/nonexistent/runtime.yaml")])
    with pytest.raises(RuntimeError) as excinfo:
        rc.config_path()
    assert "runtime.yaml" in str(excinfo.value)


def test_env_override_takes_priority(tmp_path, monkeypatch):
    """BOOM_BIRDS_RUNTIME_CONFIG 可显式指定配置。"""
    from boom_birds_control import runtime_config as rc

    target = tmp_path / "runtime.yaml"
    target.write_text(yaml.safe_dump(DEFAULTS.as_mapping()), encoding="utf-8")
    monkeypatch.setenv(rc.CONFIG_ENV_VAR, str(target))
    assert rc.config_path() == target
    assert rc.RuntimeConfig.load(rc.config_path()) == DEFAULTS


def test_recovery_scene_provides_landing_margin_without_changing_recovery_limits():
    scene = SCENES["recovery_local"]
    assert scene.takeoff_altitude_agl_m == 2.5
    assert scene.recovery_min_altitude_agl_m == DEFAULTS.recovery_min_altitude_agl_m
    assert scene.recovery_max_speed_m_s == DEFAULTS.recovery_max_speed_m_s
    assert scene.recovery_valid_duration_s == DEFAULTS.recovery_valid_duration_s
