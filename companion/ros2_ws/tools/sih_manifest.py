# ******************************************************************************
# @file           : sih_manifest.py
# @brief          : 记录当前源码、安装产物、配置和依赖。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

"""记录当前源码、安装产物、配置和依赖；不修改被测目录。"""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.evidence import git, gitSnapshot


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--forest-profile", default="reference_30m")
    args = parser.parse_args()
    import cv2
    import yaml
    from boom_birds_control.runtime_config import SCENES
    from dataclasses import asdict
    root = Path(__file__).resolve().parents[3]
    ws = root / "companion/ros2_ws"
    install = Path(os.environ["INSTALL_BASE"])
    binaries = {}
    for path in install.rglob("*"):
        if path.is_file() and path.suffix not in (".pyc",):
            binaries[str(path.relative_to(install))] = hashlib.sha256(path.read_bytes()).hexdigest()
    payload = dict(repositories={"root": gitSnapshot(root)},
                   runtime=asdict(SCENES[args.scene]), installed_files=binaries,
                   python=sys.executable, versions={name: importlib.metadata.version(name)
                   for name in ("numpy", "pymavlink", "pyserial", "pytest")},
                   rmw_environment={k: os.environ.get(k) for k in (
                       "RMW_IMPLEMENTATION", "RMW_FASTRTPS_PUBLICATION_MODE",
                       "RMW_FASTRTPS_USE_QOS_FROM_XML", "FASTDDS_DEFAULT_PROFILES_FILE")},
                   control_publication_default="ASYNCHRONOUS",
                   user_site_disabled=os.environ.get("PYTHONNOUSERSITE"),
                   command=sys.argv)
    if args.scene == "forest_30m":
        from boom_birds_sim.forest_scenes import loadProfile
        payload["forest_geometry"] = dict(profile=args.forest_profile, **loadProfile(args.forest_profile))
    for name in ("ego-planner-swarm", "open_vins"):
        payload["repositories"][name] = gitSnapshot(ws / "src" / name)
    px4 = Path(os.environ.get("PX4_SOURCE", str(Path.home() / "PX4-Autopilot")))
    px4Build = Path(os.environ.get("PX4_BUILD", str(px4 / "build/px4_sitl_default")))
    binary = px4Build / "bin/px4"
    payload["px4"] = dict(source=str(px4), sha=git(px4, "rev-parse", "HEAD").decode().strip(),
        status=git(px4, "status", "--short").decode(),
        diff_sha256=hashlib.sha256(git(px4, "diff", "HEAD", "--binary")).hexdigest(),
        binary=str(binary), binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    payload["versions"].update(opencv=cv2.__version__, pyyaml=yaml.__version__)
    with args.output.open("x") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2)


if __name__ == "__main__":
    main()
