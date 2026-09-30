"""PX4 SIH 参数接线：数值的**唯一**来源是 ``RuntimeConfig``。

本模块把 ``RuntimeConfig`` 的字段翻译成 PX4 参数命令，供 SIH 脚本使用：

* 起飞高度 → ``RuntimeConfig.takeoff_altitude_agl_m``，参数名取
  ``RuntimeConfig.px4_takeoff_param_name``（默认 ``MIS_TAKEOFF_ALT``）；
* Offboard 信号丢失后的动作 → 本模块的 SIH 专用常量（见
  ``PX4_OFFBOARD_LOSS_ACTION_LAND``），它是 PX4 枚举的行为选择，不是本工程的
  时间/高度阈值，因此不放进 ``RuntimeConfig``。

为什么单独成模块：此前 SIH 脚本直接写 ``px4-param set MIS_TAKEOFF_ALT 1.5``，
与 launch / 生命周期各拿一套数字。现在 shell 只调用本模块，数值只有一个定义点
（``test/test_config_single_source.py`` 扫描脚本，禁止再出现这些字面量）。

用法::

    # 人读（默认）
    python3 -m boom_birds_nav.sih_params
    # 可直接粘贴到 px4-param 所在终端
    python3 -m boom_birds_nav.sih_params --format shell
    # 直接下发；参数二进制缺失或任一条失败即非零退出，不静默跳过
    python3 -m boom_birds_nav.sih_params --apply --param-bin /path/to/bin/px4-param
    # 给 shell 取单个字段（白名单字段，见 SHELL_FIELDS）
    python3 -m boom_birds_nav.sih_params --get takeoff_timeout_s
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

from boom_birds_control.runtime_config import DEFAULTS, RuntimeConfig, config_path

#: SIH 专用：Offboard 信号丢失后 PX4 的动作 = Land（PX4 ``COM_OBL_RC_ACT`` 枚举）。
#: 这是**行为选择**，不是本工程的高度/时间阈值，因此不放 RuntimeConfig；
#: 改动它等于改 SIH 的失效动作，必须同步 README 与 STATUS。
PX4_OFFBOARD_LOSS_ACTION_PARAM = "COM_OBL_RC_ACT"
PX4_OFFBOARD_LOSS_ACTION_LAND = 4
#: 允许的失联动作（PX4 ``COM_OBL_RC_ACT`` 枚举）：SIH 故障矩阵要分别验证
#: "进入 Land"与"进入 Return"之后的合格恢复，因此动作必须可显式选择并留证。
PX4_OFFBOARD_LOSS_ACTIONS = {"land": PX4_OFFBOARD_LOSS_ACTION_LAND, "rtl": 3}
PX4_OFFBOARD_LOSS_ACTION_DEFAULT = "land"

#: Offboard 信号丢失的**超时**（PX4 ``COM_OF_LOSS_T``）＝本地 PX4 版本缺省 1.0 s。
#: 显式写下并发下它，是为了**留证**：提示词要求"不增大失联超时掩盖控制断流"，
#: 因此这里必须等于版本缺省，不得因为控制断流难查就把它放大。
PX4_OFFBOARD_LOSS_TIMEOUT_PARAM = "COM_OF_LOSS_T"
PX4_OFFBOARD_LOSS_TIMEOUT_S = 1.0

DEFAULT_PARAM_BIN = "px4-param"

#: 允许 shell 通过 ``--get`` 读取的字段白名单：脚本只能引用登记过的字段，
#: 避免"随手从配置里取一个数"重新长出第二套语义。
SHELL_FIELDS = (
    "px4_takeoff_param_name",
    "takeoff_altitude_agl_m",
    "hold_lock_min_altitude_agl_m",
    "image_publish_min_altitude_agl_m",
    "recovery_min_altitude_agl_m",
    "handoff_max_distance_m",
    "handoff_max_speed_m_s",
    "goal_tolerance_m",
    "pose_timeout_s",
    "status_timeout_s",
    "planning_timeout_s",
    "precheck_timeout_s",
    "takeoff_timeout_s",
    "hold_ready_timeout_s",
    "mode_timeout_s",
)

EXIT_OK = 0
EXIT_APPLY_FAILED = 1
EXIT_USAGE = 2


def load_runtime(path=None) -> RuntimeConfig:
    """读取运行配置；``path`` 为 None 时用包内 ``config/runtime.yaml``。"""
    return RuntimeConfig.load(Path(path) if path is not None else config_path())


def format_value(value) -> str:
    """PX4 参数值/字段值的稳定文本形式（``1.5``、``1.0``、``10``）。"""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return value
    if isinstance(value, int):
        return str(value)
    return repr(float(value))


def _loss_action_value(obl_action: str) -> str:
    if obl_action not in PX4_OFFBOARD_LOSS_ACTIONS:
        raise ValueError(
            f"未知失联动作：{obl_action}（可用：{', '.join(PX4_OFFBOARD_LOSS_ACTIONS)}）")
    return str(PX4_OFFBOARD_LOSS_ACTIONS[obl_action])


def param_assignments(config: RuntimeConfig = DEFAULTS,
                      obl_action: str = PX4_OFFBOARD_LOSS_ACTION_DEFAULT) -> tuple:
    """``(PX4 参数名, 值)`` 列表；起飞高度只来自 ``config.takeoff_altitude_agl_m``。"""
    name = config.px4_takeoff_param_name
    if not isinstance(name, str) or not name.strip():
        raise ValueError("px4_takeoff_param_name 不能为空")
    return (
        (name, format_value(config.takeoff_altitude_agl_m)),
        (PX4_OFFBOARD_LOSS_ACTION_PARAM, _loss_action_value(obl_action)),
        (PX4_OFFBOARD_LOSS_TIMEOUT_PARAM, format_value(PX4_OFFBOARD_LOSS_TIMEOUT_S)),
    )


def param_commands(config: RuntimeConfig = DEFAULTS,
                   param_bin: str = DEFAULT_PARAM_BIN,
                   obl_action: str = PX4_OFFBOARD_LOSS_ACTION_DEFAULT) -> tuple:
    """可直接粘贴的 ``px4-param set`` 命令行。"""
    if not isinstance(param_bin, str) or not param_bin.strip():
        raise ValueError("param_bin 不能为空")
    return tuple(f"{param_bin} set {name} {value}"
                 for name, value in param_assignments(config, obl_action))


def _resolve_binary(param_bin: str) -> str:
    if Path(param_bin).is_file():
        return param_bin
    found = shutil.which(param_bin)
    if found:
        return found
    raise FileNotFoundError(f"找不到 PX4 参数二进制：{param_bin}")


def apply_params(config: RuntimeConfig = DEFAULTS, param_bin: str = DEFAULT_PARAM_BIN,
                 obl_action: str = PX4_OFFBOARD_LOSS_ACTION_DEFAULT,
                 runner=subprocess.run) -> tuple:
    """逐条下发 PX4 参数；任一条失败抛 ``RuntimeError``（调用方非零退出）。"""
    binary = _resolve_binary(param_bin)
    applied = []
    for name, value in param_assignments(config, obl_action):
        result = runner([binary, "set", name, value], capture_output=True, text=True)
        if result.returncode != 0:
            detail = (result.stderr or result.stdout or "").strip()
            raise RuntimeError(f"px4-param set {name} 失败（返回码 {result.returncode}）：{detail}")
        applied.append((name, value))
    return tuple(applied)


def readback_params(config: RuntimeConfig = DEFAULTS, param_bin: str = DEFAULT_PARAM_BIN,
                    obl_action: str = PX4_OFFBOARD_LOSS_ACTION_DEFAULT,
                    runner=subprocess.run) -> tuple:
    """回读已下发的 PX4 参数，供证据留档；任一条读不到抛 ``RuntimeError``。"""
    binary = _resolve_binary(param_bin)
    lines = []
    for name, _ in param_assignments(config, obl_action):
        result = runner([binary, "show", name], capture_output=True, text=True)
        if result.returncode != 0:
            detail = (result.stderr or result.stdout or "").strip()
            raise RuntimeError(f"px4-param show {name} 失败（返回码 {result.returncode}）：{detail}")
        lines.append((result.stdout or "").strip())
    return tuple(lines)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="从 RuntimeConfig 生成 / 下发 PX4 SIH 参数（数值唯一来源）")
    parser.add_argument("--scene", default="local")
    parser.add_argument("--format", choices=("text", "shell", "json"), default="text",
                        help="text=人读；shell=可粘贴的 px4-param 命令；json=机器读")
    parser.add_argument("--apply", action="store_true",
                        help="直接调用 --param-bin 下发（缺失或失败即非零退出）")
    parser.add_argument("--param-bin", default=DEFAULT_PARAM_BIN,
                        help=f"PX4 参数设置程序（默认 {DEFAULT_PARAM_BIN}）")
    parser.add_argument("--runtime-config", type=Path, default=None,
                        help="覆盖 config/runtime.yaml 路径（测试用）")
    parser.add_argument("--obl-action", choices=tuple(PX4_OFFBOARD_LOSS_ACTIONS),
                        default=PX4_OFFBOARD_LOSS_ACTION_DEFAULT,
                        help="PX4 失联动作（故障矩阵要分别验证 Land/Return 后的恢复）")
    parser.add_argument("--readback", action="store_true",
                        help="回读已下发的 PX4 参数并打印（留证用；失败即非零退出）")
    parser.add_argument("--get", metavar="FIELD", default=None,
                        help="只打印一个 RuntimeConfig 字段（白名单见 SHELL_FIELDS）")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = RuntimeConfig.load_scenes(args.runtime_config or config_path())[args.scene]
    except (OSError, ValueError, TypeError, KeyError) as exc:
        print(f"运行配置无效：{exc}", file=sys.stderr)
        return EXIT_USAGE
    source = str(args.runtime_config) if args.runtime_config is not None else str(config_path())
    if args.get is not None:
        if args.get not in SHELL_FIELDS:
            print(f"字段不在白名单：{args.get}（可用：{', '.join(SHELL_FIELDS)}）", file=sys.stderr)
            return EXIT_USAGE
        print(format_value(getattr(config, args.get)))
        return EXIT_OK
    if args.readback:
        try:
            lines = readback_params(config, args.param_bin, args.obl_action)
        except (OSError, RuntimeError, ValueError) as exc:
            print(f"回读 PX4 参数失败：{exc}", file=sys.stderr)
            return EXIT_APPLY_FAILED
        print("\n".join(lines))
        return EXIT_OK
    if args.apply:
        try:
            applied = apply_params(config, args.param_bin, args.obl_action)
        except (OSError, RuntimeError, ValueError) as exc:
            print(f"下发 PX4 参数失败：{exc}", file=sys.stderr)
            return EXIT_APPLY_FAILED
        print(f"已下发 {len(applied)} 条 PX4 参数（来源：{source}）")
        return EXIT_OK
    try:
        assignments = param_assignments(config, args.obl_action)
        commands = param_commands(config, args.param_bin, args.obl_action)
    except ValueError as exc:
        print(f"运行配置无效：{exc}", file=sys.stderr)
        return EXIT_USAGE
    if args.format == "shell":
        for command in commands:
            print(command)
    elif args.format == "json":
        print(json.dumps({"runtime_config": source,
                          "params": dict(assignments),
                          "commands": list(commands)},
                         ensure_ascii=False))
    else:
        for name, value in assignments:
            print(f"{name} = {value}")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
