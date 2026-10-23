"""核验已加载的运行安装树；不启动 ROS 节点或访问设备。"""
import argparse
import importlib
import json
from pathlib import Path

from ament_index_python.packages import get_package_prefix

REQUIRED = ("boom_birds_interfaces", "boom_birds_control", "boom_birds_sensing",
            "boom_birds_bringup", "stereo_depth", "ego_planner", "ov_msckf", "ov_core", "ov_init")
DEVELOPMENT = ("boom_birds_sim", "boom_birds_nav", "ov_eval", "map_generator", "mockamap")
REMOVED_SENSING = ("vio_calibration", "vio_calibration_cli", "vio_calibration_inputs",
                   "vio_calibration_record", "imu_noise_capture", "imu_noise_analysis",
                   "telemetry_journal", "platform_replay", "mavlink_imu_replay", "mavlink_imu_node",
                   "platform_synthetic", "image_scaling", "platform_worker")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--install", type=Path, required=True)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    install = args.install.resolve(strict=True)
    prefixes = {}
    errors = []
    for name in REQUIRED:
        prefix = Path(get_package_prefix(name)).resolve()
        prefixes[name] = str(prefix)
        if not prefix.is_relative_to(install):
            errors.append(f"加载了其他安装树：{name}={prefix}")
    packages = {p.parent.name for p in install.glob("*/share/*/package.xml")}
    for name in DEVELOPMENT:
        if name in packages: errors.append(f"安装了开发包：{name}")
    for name in ("boom_birds_control", "boom_birds_sensing", "boom_birds_bringup", "stereo_depth"):
        module = importlib.import_module(name)
        location = Path(module.__file__).resolve()
        if not location.is_relative_to(install): errors.append(f"源码遮蔽安装库：{name}={location}")
    sensing = Path(importlib.import_module("boom_birds_sensing").__file__).parent
    for name in REMOVED_SENSING:
        if (sensing / (name + ".py")).exists(): errors.append(f"残留开发模块：{name}")
    if (Path(importlib.import_module("stereo_depth").__file__).parent / "view.py").exists():
        errors.append("残留显示旋转模块 view.py")
    control = Path(importlib.import_module("boom_birds_control").__file__).parent
    if (control / "platform_runtime.py").exists(): errors.append("残留旧 platform_runtime 别名")
    egoLaunch = Path(prefixes["ego_planner"]) / "share/ego_planner/launch"
    for path in egoLaunch.glob("*.py"):
        if path.name != "boom_birds_offline.launch.py": errors.append(f"残留仿真 launch：{path}")
    ovShare = Path(prefixes["ov_msckf"]) / "share/ov_msckf"
    if (ovShare / "config").exists() or (ovShare / "launch").exists():
        errors.append("残留 OpenVINS 示例配置或启动文件")
    for path in install.rglob("*"):
        if path.is_file() and path.parent.parent.name == "lib":
            if path.name.startswith("test_") or path.name in ("run_simulation", "obj_generator", "sih_params", "vio_calibration", "platform_replay"):
                errors.append(f"残留开发可执行文件：{path}")
        if path.is_file() and (path.name.endswith("_test.yaml") or path.name == "attitude_sih.yaml"):
            errors.append(f"残留测试配置：{path}")
    from stereo_depth.core import Config, StereoProcessor
    processor = StereoProcessor(Config())
    calibration = Path(Config.calibration).resolve()
    if not calibration.is_relative_to(install): errors.append(f"默认标定来自安装树外：{calibration}")
    report = dict(status="FAIL" if errors else "PASS", install=str(install), prefixes=prefixes,
                  packages=sorted(packages), calibration=str(calibration), baseline_m=processor.baseline_m, errors=errors)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    raise SystemExit(bool(errors))


if __name__ == "__main__":
    main()
