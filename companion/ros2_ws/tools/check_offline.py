"""运行脱机测试并记录退出状态；未运行项目不计 PASS。"""
import argparse
import signal
import hashlib
import importlib.metadata
import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

def run_group(command, stream, cwd, timeout):
    process = subprocess.Popen(command, cwd=cwd, stdout=stream, stderr=subprocess.STDOUT,
                               start_new_session=True)
    try:
        process.wait(timeout=timeout)
        return process
    finally:
        # 清理本次创建的进程组，包含测试超时后遗留的 ROS 子进程。
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            pass
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()


ws = Path(__file__).resolve().parents[1]
root = ws.parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--out", type=Path, default=ws / "log/architecture_checks/report.json")
#: CI/受限环境可以显式排除需要 EGO fork C++ 产物的组；排除项会写进报告，
#: 既不算 PASS 也不静默消失（"跳过当通过"是本仓库明令禁止的）。
parser.add_argument("--exclude-groups", default="",
                    help="逗号分隔的组名；被排除的组不会运行，但会记入 report.json 的 excluded_groups")
args = parser.parse_args()
excluded = [n for n in (x.strip() for x in args.exclude_groups.split(",")) if n]
args.out = args.out.resolve()
if args.out.suffix != ".json":
    parser.error("--out 必须指向独立证据目录中的 report.json 文件")
if args.out.exists() or any(args.out.parent.glob("*.log")) or any(args.out.parent.glob("*.xml")):
    parser.error("证据目录已有报告或测试日志，拒绝覆盖；请指定新的 --out 路径")
args.out.parent.mkdir(parents=True, exist_ok=True)
build = Path(os.environ.get("BUILD_BASE", str(Path.home() / "bb_build/main/build")))
install = Path(os.environ.get("INSTALL_BASE", str(Path.home() / "bb_build/main/install")))
# 前置条件：安装前缀必须真的是本次构建用的那个。默认值是 ~/bb_build/main，而开发机
# 常把构建前缀覆盖成别处（architecture 等）；忘记一起覆盖就会去陈旧或不存在的安装树
# 里跑检查，表现成一串 "No module named boom_birds_*" / "Package not found"
# （本机实测，整组假失败）。这里明确失败，不让"看起来跑过了"通过。
if not (install / "setup.bash").is_file():
    sys.exit(f"安装树不存在：{install}——请用 INSTALL_BASE 指向本次构建的 install 前缀")
# 需要 JUnit 计数的组：全部是 pytest 入口。skipped 一律降级为 PARTIAL（不把跳过当通过）。
JUNIT_GROUPS = ("portable_behavior", "navigation", "control_layout", "sim_layout", "sensing_layout", "bringup_layout", "nav_forwarder", "config_single_source", "stereo_calibration")
checks = [
    ("portable_behavior", [sys.executable, "-m", "pytest", "-q",
        *[str(ws / "src/boom_birds_nav/test" / name) for name in (
            "test_lifecycle.py", "test_control_protocol.py", "test_runtime_config.py",
            "test_px4_frames.py", "test_depth_contract.py")],
        "--junitxml=" + str(args.out.parent / "portable_behavior.xml")]),
    ("navigation", [sys.executable, "-m", "pytest", str(ws / "src/boom_birds_nav/test"), "-q", "--junitxml=" + str(args.out.parent / "navigation.xml")]),
    # 单一配置来源（A2）：runtime.yaml ↔ 代码默认值、px4_interface.yaml ↔ RuntimeConfig、
    # contract.yaml 只留语义、launch/tools 无硬编码高度、package.xml 依赖齐全。
    ("config_single_source", [sys.executable, "-m", "pytest", str(ws / "src/boom_birds_nav/test/test_config_single_source.py"), "-q", "--junitxml=" + str(args.out.parent / "config_single_source.xml")]),
    # 拆包第四批：bringup 结构断言 + nav 只做兼容转发
    ("bringup_layout", [sys.executable, "-m", "pytest", str(ws / "src/boom_birds_bringup/test"), "-q", "--junitxml=" + str(args.out.parent / "bringup_layout.xml")]),
    ("nav_forwarder", [sys.executable, "-m", "pytest", str(ws / "src/boom_birds_nav/test/test_nav_is_forwarder.py"), "-q", "--junitxml=" + str(args.out.parent / "nav_forwarder.xml")]),
    # 拆包第三批：sensing 包的结构断言 + 不依赖 sim
    ("sensing_layout", [sys.executable, "-m", "pytest", str(ws / "src/boom_birds_sensing/test"), "-q", "--junitxml=" + str(args.out.parent / "sensing_layout.xml")]),
    # 拆包第二批：sim 包的结构断言 + 生产包不得依赖 sim
    ("sim_layout", [sys.executable, "-m", "pytest", str(ws / "src/boom_birds_sim/test"), "-q", "--junitxml=" + str(args.out.parent / "sim_layout.xml")]),
    # 拆包第一批：新包自带的结构断言（迁移到位、兼容转发是转发、生产包不依赖 sim）
    ("control_layout", [sys.executable, "-m", "pytest", str(ws / "src/boom_birds_control/test"), "-q", "--junitxml=" + str(args.out.parent / "control_layout.xml")]),
    ("stereo_calibration", [sys.executable, "-m", "pytest", str(ws / "src/stereo_depth/test_live_calibration.py"), "-q", "--junitxml=" + str(args.out.parent / "stereo_calibration.xml")]),
    # 消息包必须真的装好（缺包/未装会直接失败，不做静默跳过）。
    ("interfaces_import", [sys.executable, "-c",
                           "from boom_birds_interfaces.msg import ControlCommand, ExecutionStatus, PlannerRequest, PlannerStatus; "
                           "from boom_birds_interfaces.srv import Mission, VehicleAction; "
                           "print('boom_birds_interfaces ok')"]),
    # launch 引用的入口点必须真的能被 ros2 发现（构建成功 != 入口点装好）。
    ("launch_executables", [sys.executable, str(ws / "tools/check_launch_executables.py")]),
    ("map_behavior", [str(install / "plan_env/lib/plan_env/bb_grid_map_test")]),
    ("trajectory_validation", ["ctest", "--test-dir", str(build / "ego_planner"), "-R", "^trajectory_validation$", "--no-tests=error", "--output-on-failure"]),
]
def git_snapshot(directory):
    def git(*args):
        return subprocess.check_output(["git", *args], cwd=directory)
    manifest = {}
    for name in git("ls-files", "--cached", "--others", "--exclude-standard", "-z").split(b"\0"):
        if not name: continue
        path = directory / os.fsdecode(name)
        if path.is_file():
            manifest[os.fsdecode(name)] = hashlib.sha256(path.read_bytes()).hexdigest()
    return dict(sha=git("rev-parse", "HEAD").decode().strip(),
                status=git("status", "--short").decode(),
                diff_sha256=hashlib.sha256(git("diff", "--binary", "HEAD")).hexdigest(),
                files=manifest)

repositories = {"root": git_snapshot(root)}
for name in ("ego-planner-swarm", "open_vins"):
    repositories[name] = git_snapshot(ws / "src" / name)
submodules = subprocess.check_output(["git", "submodule", "status", "--recursive"], cwd=root, text=True)
versions = {name: importlib.metadata.version(name) for name in ("numpy", "pymavlink", "pyserial", "pytest")}
# 配置指纹必须覆盖**所有** boom_birds_* 包的 config：runtime.yaml 的所有者是
# boom_birds_control，只扫 nav 会让报告"看起来"记录了配置，实际上唯一真值不在里面
# （拆包第五批实测）。
configs = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
           for pkg in sorted((ws / "src").glob("boom_birds_*")) if pkg.is_dir()
           for p in sorted((pkg / "config").glob("*.yaml"))}
patch = subprocess.check_output(["git", "diff", "--binary", "HEAD"], cwd=root)
report = dict(repositories=repositories, submodules=submodules, python=sys.executable, versions=versions, config_sha256=configs,
              diff_sha256=hashlib.sha256(patch).hexdigest(), timestamp=datetime.datetime.now(datetime.timezone.utc).isoformat(),
              commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
              git_status=subprocess.check_output(["git", "status", "--short"], cwd=root, text=True),
              ros_domain_id=os.environ.get("ROS_DOMAIN_ID"), ov_install=os.environ.get("OV_INSTALL"),
              excluded_groups=excluded, checks=[])
try:
    for name, command in checks:
        if name in excluded:
            print(f"{name}: EXCLUDED (--exclude-groups)", flush=True)
            continue
        start = time.monotonic()
        wall_start = time.time()
        log = args.out.parent / (name + ".log")
        try:
            with log.open("w") as stream:
                result = run_group(command, stream, root, 600)
            status = "PASS" if result.returncode == 0 else "FAIL"
            detail = dict(returncode=result.returncode)
            if name in JUNIT_GROUPS:
                xml = args.out.parent / (name + ".xml")
                if xml.is_file() and xml.stat().st_mtime >= wall_start:
                    suites = ET.parse(xml).getroot().iter("testsuite")
                    counts = {key: 0 for key in ("tests", "failures", "errors", "skipped")}
                    for suite in suites:
                        for key in counts:
                            counts[key] += int(suite.get(key, "0"))
                    detail.update(counts)
                    if status == "PASS" and (counts["skipped"] or not counts["tests"]):
                        status = "PARTIAL"
                elif status == "PASS":
                    status = "FAIL"
                    detail["error"] = "未生成本次 JUnit 报告"
        except (OSError, subprocess.TimeoutExpired) as exc:
            status = "NOT RUN" if isinstance(exc, OSError) else "FAIL"
            detail = dict(error=str(exc))
        report["checks"].append(dict(name=name, status=status, command=command, log=str(log), seconds=round(time.monotonic()-start, 2), **detail))
        print(f"{name}: {status} ({log})", flush=True)
finally:
    after = {"root": git_snapshot(root)}
    for name in ("ego-planner-swarm", "open_vins"):
        after[name] = git_snapshot(ws / "src" / name)
    report["repositories_after"] = after
    report["source_unchanged"] = repositories == after
    report["changed_files_during_checks"] = {
        name: sorted(path for path in set(before["files"]) | set(after[name]["files"])
                     if before["files"].get(path) != after[name]["files"].get(path))
        for name, before in repositories.items()
    }
    if not report["source_unchanged"]:
        print("source_integrity: FAIL (源码在验收期间发生变化)", flush=True)
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(args.out, flush=True)
sys.exit(0 if report["source_unchanged"] and all(c["status"] == "PASS" for c in report["checks"]) else 1)
