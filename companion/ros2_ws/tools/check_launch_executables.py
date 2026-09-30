"""launch 文件引用的 boom_birds_* 入口点必须真的能被 ament 发现。

背景（本项目真实踩过两次）：构建成功 != 入口点装好。

* `lifecycle_node` 曾在 `setup.py` 里声明、被 `px4_sih_mission.launch.py` 引用，
  却只落到 `<prefix>/bin/`，没进 `<prefix>/lib/boom_birds_nav/`；
* 拆包后 nav 不再提供任何可执行文件，硬编码"必须找到 package=boom_birds_nav 的 Node"
  的旧扫描会直接失效报错。

因此本脚本按**所有** `boom_birds_*` 包核对：

1. 用 AST 取出各包 ``launch/*.py`` 里所有 ``Node(package="boom_birds_X", executable=...)``；
2. 每个名字必须在**该包** ``setup.py`` 的 ``console_scripts`` 里声明；
3. 每个名字必须出现在 ``ros2 pkg executables boom_birds_X`` 的输出里。

缺任何一项即非零退出，不跳过、不降级。
"""

from __future__ import annotations

import ast
import re
import subprocess
import sys
from pathlib import Path

SRC_DIR = Path(__file__).resolve().parents[1] / "src"
PACKAGE_PREFIX = "boom_birds_"


def _const(node) -> str:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return ""


def packages() -> dict:
    """{包名: 包目录}，只有带 setup.py 的才可能提供可执行文件。"""
    found = {}
    for path in sorted(SRC_DIR.glob(f"{PACKAGE_PREFIX}*")):
        if path.is_dir() and (path / "setup.py").is_file():
            found[path.name] = path
    if not found:
        raise SystemExit(f"在 {SRC_DIR} 下没有找到任何 {PACKAGE_PREFIX}* 包，扫描本身失效")
    return found


def launch_executables(pkgs: dict) -> dict:
    """{(包, 可执行文件): {launch 文件名}}。"""
    used = {}
    for pkg_name, pkg_dir in pkgs.items():
        for path in sorted((pkg_dir / "launch").glob("*.py")):
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call):
                    continue
                func = node.func
                name = func.id if isinstance(func, ast.Name) else getattr(func, "attr", "")
                if name != "Node":
                    continue
                kwargs = {kw.arg: kw.value for kw in node.keywords if kw.arg}
                package = _const(kwargs.get("package"))
                executable = _const(kwargs.get("executable"))
                if package in pkgs and executable:
                    used.setdefault((package, executable), set()).add(
                        f"{pkg_name}/launch/{path.name}")
    if not used:
        raise SystemExit("没有找到任何引用本项目包的 Node()，扫描本身失效")
    return used


def declared_entry_points(pkg_dir: Path) -> set:
    pattern = re.compile(r'"([A-Za-z_]\w*)\s*=\s*' + re.escape(pkg_dir.name) + r'\.[\w.]+:main"')
    return set(pattern.findall((pkg_dir / "setup.py").read_text(encoding="utf-8")))


def installed_executables(package: str) -> set:
    try:
        result = subprocess.run(["ros2", "pkg", "executables", package],
                                capture_output=True, text=True)
    except FileNotFoundError as exc:
        raise SystemExit(f"找不到 ros2 命令（需先 source ROS 与 install/setup.bash）：{exc}") from exc
    if result.returncode != 0:
        raise SystemExit(f"ros2 pkg executables {package} 失败（{result.returncode}）："
                         f"{result.stderr.strip()}")
    return {line.split()[-1] for line in result.stdout.splitlines() if line.strip()}


def main() -> int:
    pkgs = packages()
    used = launch_executables(pkgs)
    problems = []
    for (package, executable), where in sorted(used.items()):
        declared = declared_entry_points(pkgs[package])
        if executable not in declared:
            problems.append(f"{executable}：{package}/setup.py 未声明（launch: {', '.join(sorted(where))}）")
            continue
        if executable not in installed_executables(package):
            problems.append(f"{executable}：ros2 pkg executables 未发现（launch: {', '.join(sorted(where))}）")
    if problems:
        print("launch 入口点检查失败：", file=sys.stderr)
        for item in problems:
            print(f"  - {item}", file=sys.stderr)
        return 1
    total = len(used)
    print(f"launch 入口点检查通过：{total} 个引用覆盖 "
          f"{len({p for p, _ in used})} 个包")
    return 0


if __name__ == "__main__":
    sys.exit(main())
