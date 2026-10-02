"""单一配置来源（A2）回归：数值只允许有一个定义点。

四条断言：

(a) ``config/runtime.yaml`` 与 ``RuntimeConfig`` 代码默认值逐字段相等；
(b) ``config/px4_interface.yaml`` 的每个失效阈值等于 ``RuntimeConfig`` 对应字段，
    且不出现未登记的失效阈值键；
(c) ``config/contract.yaml`` 的 ``timing`` 只留语义声明，不再含数值阈值；
(d) ``launch/*.py`` 与 ``tools/*.sh`` 不再硬编码高度/接管距离字面量，
    起飞高度只能经 ``boom_birds_bringup.sih_params`` 下发，模式判定只在编排器。

``runtime.yaml`` 的所有者是 ``boom_birds_control``（见 (a0)/(a1)）；
其余被测配置仍属本包。两者放在同一个测试里核对，因为「单一来源」是跨包的属性。

只读文件与纯函数；不连接设备、不启动 SIH。
"""

from __future__ import annotations

import ast
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from boom_birds_control.runtime_config import RuntimeConfig, SCENES

PKG = Path(__file__).resolve().parents[1]
SRC_ROOT = PKG.parent                          # companion/ros2_ws/src
WS_ROOT = PKG.parents[1]                       # companion/ros2_ws
#: ``runtime.yaml`` 随其所有者迁移：定义它的（``RuntimeConfig`` 代码默认值）与
#: 读它的（``config_path()``）都在 ``boom_birds_control``，所以按**所有者包**定位，
#: 不再假设它躺在写这个测试的包里——nav 自拆包起只是兼容转发层。
RUNTIME_YAML = SRC_ROOT / "boom_birds_control" / "config" / "runtime.yaml"
PX4_INTERFACE_YAML = SRC_ROOT / "boom_birds_control" / "config" / "px4_interface.yaml"
CONTRACT_YAML = SRC_ROOT / "boom_birds_interfaces" / "config" / "contract.yaml"
MAVLINK_YAML = SRC_ROOT / "boom_birds_sensing" / "config" / "mavlink_imu.yaml"
REQUIREMENTS = WS_ROOT / "requirements-mavlink.txt"

#: px4_interface.yaml 的失效阈值 → RuntimeConfig 字段。
#: 新增阈值必须登记在这里，否则"未登记阈值"检查会失败。
FAILURE_THRESHOLDS = {
    "setpoint_timeout_s": "setpoint_timeout_s",
    "max_setpoint_age_s": "setpoint_timeout_s",     # 同一语义：PositionCommand 年龄上限
    "vio_timeout_s": "vio_timeout_s",
    "imu_timeout_s": "imu_timeout_s",
    "camera_timeout_s": "camera_timeout_s",
    "heartbeat_timeout_s": "heartbeat_timeout_s",
    "link_timeout_s": "link_timeout_s",
    "recovery_required_samples": "recovery_required_samples",
}

#: 名字像阈值、但不参与失效判定的节点本地参数：必须显式登记理由。
NON_THRESHOLD_PARAMETERS = {
    "read_timeout_s": "后端 read() 阻塞上限，不参与失效判定",
    "frame_alignment_required_samples": "航向核实所需样本数下限，属标定/核实参数",
}

#: 被禁的"高度/接管量"字段：这些数值只允许出现在 runtime.yaml 与读它的代码默认值里。
BANNED_FIELDS = (
    "takeoff_altitude_agl_m",
    "hold_lock_min_altitude_agl_m",
    "image_publish_min_altitude_agl_m",
    "recovery_min_altitude_agl_m",
    "handoff_max_distance_m",
    "handoff_max_speed_m_s",
)

#: 出现这些词的行才可能是"高度/接管判定"；其它行的同值数字（超时、分辨率、场景形状）不算。
BANNED_CONTEXT = re.compile(
    r"altitude|takeoff|handoff|接管|高度|MIS_TAKEOFF|NED\s*z|\bz\s*<=|\bagl\b", re.I)
NUMBER = re.compile(r"(?<![\w.])-?\d+(?:\.\d+)?(?![\w.])")

#: 模块名 → package.xml 依赖名。值为 None 表示该依赖在本地 rosdep 里没有可解析的键，
#: 由 requirements-mavlink.txt 固定版本（见 test_pymavlink_is_pinned_in_requirements）。
MODULE_DEPENDENCIES = {
    "mavros_msgs": "mavros_msgs",
    "rclpy": "rclpy",
    "cv_bridge": "cv_bridge",
    "message_filters": "message_filters",
    "sensor_msgs": "sensor_msgs",
    "sensor_msgs_py": "sensor_msgs_py",
    "geometry_msgs": "geometry_msgs",
    "nav_msgs": "nav_msgs",
    "std_msgs": "std_msgs",
    "std_srvs": "std_srvs",
    "builtin_interfaces": "builtin_interfaces",
    "diagnostic_msgs": "diagnostic_msgs",
    "quadrotor_msgs": "quadrotor_msgs",
    "boom_birds_interfaces": "boom_birds_interfaces",
    "boom_birds_control": "boom_birds_control",
    "boom_birds_sensing": "boom_birds_sensing",
    "boom_birds_sim": "boom_birds_sim",
    "boom_birds_bringup": "boom_birds_bringup",
    "stereo_depth": "stereo_depth",
    "launch": "launch",
    "launch_ros": "launch_ros",
    "ament_index_python": "ament_index_python",
    "numpy": "python3-numpy",
    "cv2": "python3-opencv",
    "yaml": "python3-yaml",
    "serial": "python3-serial",
    "pymavlink": None,
}


def _yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _px4_interface_params() -> dict:
    return _yaml(PX4_INTERFACE_YAML)["boom_birds_px4_interface"]["ros__parameters"]


def _numeric_leaves(node, trail=()):
    if isinstance(node, dict):
        for key, value in node.items():
            yield from _numeric_leaves(value, (*trail, str(key)))
    elif isinstance(node, list):
        for index, value in enumerate(node):
            yield from _numeric_leaves(value, (*trail, str(index)))
    elif isinstance(node, bool):
        return
    elif isinstance(node, (int, float)):
        yield ".".join(trail), node


# --------------------------------------------------------------------- (a)
def test_runtime_yaml_equals_code_defaults():
    assert RuntimeConfig.load(RUNTIME_YAML) == RuntimeConfig()
    assert SCENES == RuntimeConfig.load_scenes(RUNTIME_YAML)


def test_runtime_yaml_is_the_only_definition_in_source_tree():
    """(a0) 源码树里 runtime.yaml 只能有一份，且必须位于所有者包里。

    多一份就等于多一个改动点；「单一来源」首先是「只有一份文件」。
    """
    pkg_dirs = sorted(p for p in SRC_ROOT.glob("boom_birds_*") if p.is_dir())
    found = sorted(p for d in pkg_dirs for p in d.rglob("runtime.yaml")
                   if not {"build", "install", "log", "__pycache__"} & set(p.parts))
    assert found == [RUNTIME_YAML], f"runtime.yaml 出现多份或位置不符：{found}"


def _installed_share_config():
    """返回 ament share 里的 runtime.yaml；未安装（纯源码测试）时返回 None。"""
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return None
    try:
        path = Path(get_package_share_directory("boom_birds_control")) / "config" / "runtime.yaml"
    except Exception:
        return None
    return path if path.is_file() else None


def test_installed_runtime_yaml_matches_source():
    """(a1) 安装树里的 runtime.yaml 必须与源码一致：安装副本不是第二定义。

    安装只增不删——文件迁走后旧前缀里的副本会留下，并被 ``config_path()``
    的候选列表当成合法配置读走（拆包第五批实测），所以这里逐一比对内容。
    """
    from boom_birds_control.runtime_config import config_path

    installed = Path(config_path())
    assert installed.is_file(), f"config_path() 解析到不存在的文件：{installed}"
    assert _yaml(installed) == _yaml(RUNTIME_YAML), (
        f"安装树 {installed} 与源码 {RUNTIME_YAML} 内容分歧：安装副本已过期")
    # 上面这一条在"config_path() 先命中源码树"时是自反的（独立核验 F6）：所以再直接
    # 去 ament share 里取一次安装副本，两条独立路径都对齐才算真正一致。
    share = _installed_share_config()
    if share is not None:
        assert _yaml(share) == _yaml(RUNTIME_YAML), (
            f"share 目录 {share} 与源码 {RUNTIME_YAML} 内容分歧：安装副本已过期")


# --------------------------------------------------------------------- (b)
def test_px4_interface_failure_thresholds_match_runtime_config():
    params = _px4_interface_params()
    defaults = RuntimeConfig()
    assert not set(FAILURE_THRESHOLDS) & set(params), "节点模板不能覆盖共享失效阈值"
    import inspect
    from boom_birds_control.px4_interface_node import Px4InterfaceNode
    source = inspect.getsource(Px4InterfaceNode.__init__)
    for key, field in FAILURE_THRESHOLDS.items():
        assert f'declare_parameter("{key}", DEFAULTS.{field})' in source


def test_px4_interface_has_no_unregistered_thresholds():
    params = _px4_interface_params()
    suspicious = {key for key in params
                  if key.endswith("_timeout_s") or key.endswith("_samples")}
    unregistered = suspicious - set(FAILURE_THRESHOLDS) - set(NON_THRESHOLD_PARAMETERS)
    assert not unregistered, f"未登记的失效阈值（请登记或改名）：{sorted(unregistered)}"


# --------------------------------------------------------------------- (c)
def test_contract_timing_is_semantic_only():
    timing = _yaml(CONTRACT_YAML)["timing"]
    numeric = [f"{trail}={value}" for trail, value in _numeric_leaves(timing)]
    assert not numeric, f"contract.yaml 的 timing 又出现数值阈值：{numeric}"
    declared = [str(item.get("name", "")) for item in timing["declared"]]
    assert declared, "timing.declared 不能为空"
    declared_text = " ".join(declared)
    for name in ("pose_timeout_s", "depth_timeout_s", "command_timeout_s",
                 "status_timeout_s", "handoff_max_distance_m", "handoff_max_speed_m_s",
                 "mode_timeout_s", "camera_imu_offset_s"):
        assert name in declared_text, f"契约 timing 未声明 {name} 的语义"


def test_mavlink_imu_yaml_matches_node_defaults():
    """mavlink_imu.yaml 只能引用节点默认值，不能自成一套数字。"""
    from boom_birds_sensing.mavlink_clock import ClockMapperConfig
    from boom_birds_sensing.mavlink_imu_core import MavlinkImuConfig

    params = _yaml(MAVLINK_YAML)["boom_birds_mavlink_imu"]["ros__parameters"]
    clock, imu = ClockMapperConfig(), MavlinkImuConfig()
    assert float(params["max_rtt_s"]) == pytest.approx(clock.max_rtt_s)
    assert float(params["sync_timeout_s"]) == pytest.approx(clock.sync_timeout_s)
    assert float(params["max_offset_deviation_s"]) == pytest.approx(clock.max_deviation_s)
    assert float(params["camera_imu_offset_s"]) == pytest.approx(imu.camera_imu_offset_s)
    assert bool(params["apply_camera_imu_offset"]) is imu.apply_camera_imu_offset
    assert float(params["max_sample_age_s"]) == pytest.approx(imu.max_sample_age_s)
    assert float(params["heartbeat_timeout_s"]) == pytest.approx(imu.heartbeat_timeout_s)
    assert float(params["pending_timeout_s"]) == pytest.approx(imu.pending_timeout_s)
    assert float(params["expected_rate_hz"]) == pytest.approx(imu.expected_rate_hz)


# --------------------------------------------------------------------- (d)
def _banned_numbers() -> dict:
    defaults = RuntimeConfig()
    return {field: float(getattr(defaults, field)) for field in BANNED_FIELDS}


def _hardcoded_offenders(path: Path) -> list:
    """找出**代码里**硬编码的高度/接管字面量。

    只扫代码不扫注释：注释里的同一个数字是解释（"那时飞行器已经在 1.5 m 空中，
    拿第一条样本当地面会把 1.5 m 读成 AGL≈0"），不是第二个定义点；把注释也算违规
    只会逼人删掉有用的解释来讨好扫描器（第六批实测：SIH 工具的两行注释被误判）。
    真正的违规是能被执行的赋值/默认值，它们一定在 `#` 之前。
    """
    banned = _banned_numbers()
    offenders = []
    for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        code = line.split("#", 1)[0]
        if not BANNED_CONTEXT.search(code):
            continue
        for token in NUMBER.findall(code):
            value = float(token)
            for field, expected in banned.items():
                if abs(value - expected) < 1e-9:
                    offenders.append(f"{path.name}:{lineno} 硬编码 {field}={token}：{line.strip()}")
    return offenders


def test_launch_and_tools_have_no_hardcoded_altitude_or_handoff_literals():
    files = sorted(SRC_ROOT.glob("boom_birds_*/launch/*.py")) + sorted((WS_ROOT / "tools").glob("*.sh"))
    assert files, "没有找到 launch/tools 文件，扫描本身失效"
    offenders = [item for path in files for item in _hardcoded_offenders(path)]
    assert not offenders, "以下位置仍硬编码高度/接管字面量：\n" + "\n".join(offenders)


def test_takeoff_script_delegates_px4_params_to_sih_params():
    text = (WS_ROOT / "tools" / "run_px4_sitl_takeoff_visible.sh").read_text(encoding="utf-8")
    assert "boom_birds_bringup.sih_params" in text, "起飞脚本必须调用 sih_params"
    assert "MIS_TAKEOFF_ALT" not in text, "参数名只能来自 px4_takeoff_param_name"


def test_offboard_script_delegates_mode_and_gates_to_orchestrator():
    text = (WS_ROOT / "tools" / "run_px4_sitl_offboard_visible.sh").read_text(encoding="utf-8")
    assert "boom_birds_bringup.lifecycle_cli" in text, "接管判定必须委托给编排器状态"
    assert "handoff" not in text, "接管距离判定不应再出现在 shell"


def test_shell_never_switches_px4_mode_with_commander():
    for path in sorted((WS_ROOT / "tools").glob("*.sh")):
        text = path.read_text(encoding="utf-8")
        assert not re.search(r"px4-commander[^\n]*\bmode\b", text), (
            f"{path.name} 仍用 px4-commander 切模式；模式请求必须走 PX4 接口服务")


# ------------------------------------------------- sih_params 行为（纯函数）
def test_sih_params_emits_configured_takeoff_altitude():
    from boom_birds_bringup import sih_params

    config = RuntimeConfig()
    assignments = dict(sih_params.param_assignments(config))
    assert set(assignments) == {config.px4_takeoff_param_name,
                                sih_params.PX4_OFFBOARD_LOSS_ACTION_PARAM,
                                sih_params.PX4_OFFBOARD_LOSS_TIMEOUT_PARAM}
    assert float(assignments[config.px4_takeoff_param_name]) == pytest.approx(
        config.takeoff_altitude_agl_m)
    # 失联超时必须等于本地 PX4 版本缺省：显式下发是为了**留证不放大**
    # （提示词要求不得靠增大失联超时掩盖控制断流）。
    assert float(assignments[sih_params.PX4_OFFBOARD_LOSS_TIMEOUT_PARAM]) == pytest.approx(
        sih_params.PX4_OFFBOARD_LOSS_TIMEOUT_S)
    parts = sih_params.param_commands(config, param_bin="/px4/bin/px4-param")[0].split()
    assert parts[:2] == ["/px4/bin/px4-param", "set"]
    assert parts[2] == config.px4_takeoff_param_name
    assert float(parts[3]) == pytest.approx(config.takeoff_altitude_agl_m)


def test_sih_params_tracks_runtime_yaml(tmp_path):
    """改 runtime.yaml 的起飞高度，命令必须跟着变（不是代码里另写一份）。"""
    from boom_birds_bringup import sih_params

    data = _yaml(RUNTIME_YAML)
    data["takeoff_altitude_agl_m"] = 2.25
    path = tmp_path / "runtime.yaml"
    path.write_text(yaml.safe_dump(data, allow_unicode=True), encoding="utf-8")
    command = sih_params.param_commands(sih_params.load_runtime(path))[0]
    assert float(command.split()[3]) == pytest.approx(2.25)


def test_sih_params_get_is_whitelisted(capsys):
    from boom_birds_bringup import sih_params

    assert sih_params.main(["--get", "hold_lock_min_altitude_agl_m"]) == sih_params.EXIT_OK
    assert float(capsys.readouterr().out.strip()) == pytest.approx(
        RuntimeConfig().hold_lock_min_altitude_agl_m)
    assert sih_params.main(["--get", "not_a_field"]) == sih_params.EXIT_USAGE


def test_sih_params_apply_fails_when_binary_missing(tmp_path, capsys):
    from boom_birds_bringup import sih_params

    code = sih_params.main(["--apply", "--param-bin", str(tmp_path / "missing-px4-param")])
    assert code == sih_params.EXIT_APPLY_FAILED
    assert "找不到 PX4 参数二进制" in capsys.readouterr().err


def test_sih_params_apply_reports_runner_failure():
    from boom_birds_bringup import sih_params

    class Result:
        returncode = 1
        stdout = ""
        stderr = "param not found"

    original = sih_params._resolve_binary
    sih_params._resolve_binary = lambda _bin: _bin
    try:
        with pytest.raises(RuntimeError, match="param not found"):
            sih_params.apply_params(RuntimeConfig(), "/usr/bin/true",
                                    runner=lambda *a, **k: Result())
    finally:
        sih_params._resolve_binary = original


def test_sih_params_text_output_uses_config_values(capsys):
    from boom_birds_bringup import sih_params

    config = RuntimeConfig()
    assert sih_params.main([]) == sih_params.EXIT_OK
    out = capsys.readouterr().out
    assert (f"{config.px4_takeoff_param_name} = "
            f"{sih_params.format_value(config.takeoff_altitude_agl_m)}") in out
    assert "COM_OBL_RC_ACT" in out


# --------------------------------------------------------- 语法（可移植，CI 可直接跑）
def test_shell_scripts_parse():
    import subprocess

    for path in sorted((WS_ROOT / "tools").glob("*.sh")):
        result = subprocess.run(["bash", "-n", str(path)], capture_output=True, text=True)
        assert result.returncode == 0, f"{path.name} shell 语法错误：{result.stderr}"


def test_launch_files_are_valid_python():
    for path in sorted(SRC_ROOT.glob("boom_birds_*/launch/*.py")):
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def test_package_xml_parses():
    root = ET.parse(PKG / "package.xml").getroot()
    assert root.findtext("name") == "boom_birds_nav"


# --------------------------------------------------------- package.xml（A9）
#: 拆包后每个包都要为自己声明的依赖负责（pymavlink 的使用者已迁到 sensing）。
PACKAGE_DIRS = {p.name: p for p in sorted((WS_ROOT / "src").glob("boom_birds_*")) if p.is_dir()}


def _imported_top_level_modules(pkg_dir=None) -> set:
    modules = set()
    pkg_dir = PKG if pkg_dir is None else pkg_dir
    for path in sorted(list((pkg_dir / pkg_dir.name).glob("*.py")) + list((pkg_dir / "launch").glob("*.py"))):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                modules.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                modules.add(node.module.split(".")[0])
            elif isinstance(node, ast.Call):
                for keyword in node.keywords:
                    if keyword.arg == "package" and isinstance(keyword.value, ast.Constant):
                        modules.add(keyword.value.value)
                if getattr(node.func, "id", "") == "get_package_share_directory" and node.args and isinstance(node.args[0], ast.Constant):
                    modules.add(node.args[0].value)
    return modules


def _declared_dependencies(pkg_dir=None) -> set:
    pkg_dir = PKG if pkg_dir is None else pkg_dir
    root = ET.parse(pkg_dir / "package.xml").getroot()
    return {element.text.strip() for element in root
            if element.tag in ("depend", "exec_depend") and element.text}


@pytest.mark.parametrize("pkg_name", sorted(PACKAGE_DIRS))
def test_package_xml_declares_every_third_party_import(pkg_name):
    """每个包都要声明**自己**实际用到的第三方依赖。

    拆包后按包核对：修好 nav 不代表 sensing 也声明了 pymavlink。
    """
    pkg_dir = PACKAGE_DIRS[pkg_name]
    declared = _declared_dependencies(pkg_dir)
    missing = []
    covered_by_requirements = []
    for module in sorted(_imported_top_level_modules(pkg_dir)):
        # 包内绝对导入自己（``from boom_birds_x.y import z``）不是第三方依赖。
        # 这个坑一直存在，只是当时登记表里没有与包同名的条目；本轮把四个兄弟包
        # 加进 MODULE_DEPENDENCIES 后立刻暴露成"要求 <exec_depend>自己</exec_depend>"。
        if module == pkg_name or module not in MODULE_DEPENDENCIES:
            continue
        dependency = MODULE_DEPENDENCIES[module]
        if dependency is None:
            covered_by_requirements.append(module)
        elif dependency not in declared:
            missing.append(f"{module} → <exec_depend>{dependency}</exec_depend>")
    assert not missing, f"{pkg_name}/package.xml 缺少实际用到的依赖：\n" + "\n".join(missing)
    if pkg_name == "boom_birds_sensing":
        assert covered_by_requirements == ["pymavlink"], covered_by_requirements


@pytest.mark.parametrize("pkg_name", sorted(PACKAGE_DIRS))
def test_sibling_dependencies_are_declared_both_ways(pkg_name):
    """兄弟实现包的依赖必须双向一致：用了要声明，声明了要用。

    A9 原检查只覆盖"用了没声明"，而且 ``MODULE_DEPENDENCIES`` 里没有四个兄弟包，
    于是拆包时留在 control/sensing/sim 的 ``<depend>boom_birds_nav</depend>``
    （方向写反）和 nav 里**没声明**的四个转发目标同时存在也全绿（第六批实测）。
    ``boom_birds_interfaces`` 只出现在 launch/测试里，不算实现依赖，故排除。
    """
    pkg_dir = PACKAGE_DIRS[pkg_name]
    impl_siblings = {"boom_birds_control", "boom_birds_sensing", "boom_birds_sim", "boom_birds_bringup"}
    declared = {d for d in _declared_dependencies(pkg_dir) if d in impl_siblings and d != pkg_name}
    imported = {m for m in _imported_top_level_modules(pkg_dir) if m in impl_siblings and m != pkg_name}
    assert declared == imported, (
        f"{pkg_name}: package.xml 声明 {sorted(declared)}，代码实际 import {sorted(imported)}")


#: 深度量程的唯一定义点是 boom_birds_control/config/runtime.yaml + RuntimeConfig。
#: 凡是**绕开它写字面量**的地方，改基准字段时都会静默落后（独立核验 F1 实测：
#: depth_node.py 与 offline_chain.launch.py 各有一处硬编码 5.0）。
_DEPTH_RANGE_MODULES = ("max_depth_m", "depth_max_range_m")
_NUMBER = re.compile(r"(?<![\w.])-?\d+(?:\.\d+)?(?![\w.])")


def _depth_range_literals():
    """返回 (文件, 行号, 行) —— 提到量程字段又写了数字字面量的地方。"""
    candidates = list(SRC_ROOT.glob("boom_birds_*/launch/*.py"))
    for pkg in sorted((WS_ROOT / "src").glob("boom_birds_*")):
        candidates.extend(p for p in pkg.glob(f"{pkg.name}/*.py"))
    offenders = []
    for path in candidates:
        if path.name == "runtime_config.py":
            continue        # 定义点本身：DEFAULTS 就在这里
        for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            code = line.split("#", 1)[0]
            if not any(f in code for f in _DEPTH_RANGE_MODULES):
                continue
            if _NUMBER.search(code):
                offenders.append((str(path.relative_to(WS_ROOT)), lineno, line.strip()))
    return offenders


def test_depth_range_has_no_second_definition():
    """深度量程不得出现第二份字面量定义（单一来源的"改一处"承诺）。"""
    offenders = _depth_range_literals()
    assert not offenders, "深度量程出现字面量第二定义：\n" + "\n".join(
        f"{f}:{n}: {line}" for f, n, line in offenders)


def test_pymavlink_is_pinned_in_requirements():
    """pymavlink 没有可解析的 rosdep 键（rosdep resolve 报 no rule），
    因此由 requirements-mavlink.txt 固定版本，setup_python_env.sh --install 安装。"""
    text = REQUIREMENTS.read_text(encoding="utf-8")
    assert "pymavlink" in text, "requirements-mavlink.txt 必须钉住 pymavlink"
    assert "pyserial" in text, "requirements-mavlink.txt 必须钉住 pyserial"
