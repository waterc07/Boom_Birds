"""记录当前源码、安装产物、配置和依赖；不修改被测目录。"""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys


def git(directory, *args):
    return subprocess.check_output(["git", *args], cwd=directory)


def repository(directory):
    names = git(directory, "ls-files", "--cached", "--others", "--exclude-standard", "-z")
    files = {}
    for name in names.split(b"\0"):
        path = directory / os.fsdecode(name)
        if name and path.is_file():
            files[os.fsdecode(name)] = hashlib.sha256(path.read_bytes()).hexdigest()
    return dict(sha=git(directory, "rev-parse", "HEAD").decode().strip(),
                status=git(directory, "status", "--short").decode(),
                diff_sha256=hashlib.sha256(git(directory, "diff", "HEAD", "--binary")).hexdigest(), files=files)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--scene", required=True)
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
    payload = dict(repositories={"root": repository(root)},
                   runtime=asdict(SCENES[args.scene]), installed_files=binaries,
                   python=sys.executable, versions={name: importlib.metadata.version(name)
                   for name in ("numpy", "pymavlink", "pyserial", "pytest")},
                   user_site_disabled=os.environ.get("PYTHONNOUSERSITE"),
                   command=sys.argv)
    for name in ("ego-planner-swarm", "open_vins"):
        payload["repositories"][name] = repository(ws / "src" / name)
    px4 = Path(os.environ.get("PX4_SOURCE", str(Path.home() / "PX4-Autopilot")))
    px4_build = Path(os.environ.get("PX4_BUILD", str(px4 / "build/px4_sitl_default")))
    binary = px4_build / "bin/px4"
    payload["px4"] = dict(source=str(px4), sha=git(px4, "rev-parse", "HEAD").decode().strip(),
        status=git(px4, "status", "--short").decode(),
        diff_sha256=hashlib.sha256(git(px4, "diff", "HEAD", "--binary")).hexdigest(),
        binary=str(binary), binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    payload["versions"].update(opencv=cv2.__version__, pyyaml=yaml.__version__)
    with args.output.open("x") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2)


if __name__ == "__main__":
    main()
