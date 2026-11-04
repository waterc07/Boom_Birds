"""RMUC2027 场地配置校验；本模块不授权或启动飞行任务。"""

import hashlib
import json
import math
from pathlib import Path
import yaml


def packagePath(root, relative):
    root = Path(root).resolve()
    path = (root / relative).resolve()
    if not path.is_relative_to(root):
        raise ValueError(f"path leaves simulation package: {relative}")
    if not path.exists():
        raise FileNotFoundError(path)
    return path


def bindLocalGoals(position, yaw, offsets):
    values = [*position, yaw, *(value for offset in offsets for value in offset)]
    if len(position) != 3 or any(len(offset) != 3 for offset in offsets) or not all(math.isfinite(v) for v in values):
        raise ValueError("finite position, yaw and 3D offsets required")
    c, s = math.cos(yaw), math.sin(yaw)
    return [[position[0] + c*x - s*y, position[1] + s*x + c*y, position[2] + z]
            for x, y, z in offsets]


def loadArena(root, mapFile="config/arena/rmuc2027.yaml"):
    root = Path(root).resolve()
    config = yaml.safe_load(packagePath(root, mapFile).read_text())
    if config.get("schema_version") != 1:
        raise ValueError("unsupported arena schema")
    model = packagePath(root, config["model_directory"])
    manifestPath = packagePath(root, config["manifest"])
    manifest = json.loads(manifestPath.read_text())
    assetRoot = packagePath(root, config["asset_root"])
    for record in manifest["files"]:
        path = packagePath(assetRoot, record["path"])
        if path.stat().st_size != record["bytes"] or hashlib.sha256(path.read_bytes()).hexdigest() != record["sha256"]:
            raise ValueError(f"arena asset changed: {record['path']}")
    scenarios = yaml.safe_load(packagePath(root, "config/arena/rmuc2027_scenarios.yaml").read_text())
    ids = [case["id"] for case in scenarios["cases"]]
    if len(set(ids)) != len(ids):
        raise ValueError("duplicate arena scenario id")
    for case in scenarios["cases"]:
        reference = case["reference_hold_pose_arena_xyz_yaw"]
        predicted = bindLocalGoals(reference[:3], reference[3], case["goal_offsets_start_flu_m"])
        expected = case["candidate_goals_arena_xyz_m"]
        if len(predicted) != len(expected) or any(len(goal) != 3 for goal in expected):
            raise ValueError(f"invalid goal count: {case['id']}")
        if any(not math.isfinite(b) or abs(a-b) > 1e-6 for p, q in zip(predicted, expected) for a, b in zip(p, q)):
            raise ValueError(f"arena/local goal mismatch: {case['id']}")
    reference = next(case for case in scenarios["cases"] if case["id"] == "S00_static")
    xyz, yaw = reference["reference_hold_pose_arena_xyz_yaw"][:3], reference["reference_hold_pose_arena_xyz_yaw"][3]
    return dict(model=model, resource_path=model.parent, rig_pose=[*xyz, 0.0, 0.0, yaw],
                map_sha256=hashlib.sha256(manifestPath.read_bytes()).hexdigest(),
                scenarios_sha256=hashlib.sha256(packagePath(root, "config/arena/rmuc2027_scenarios.yaml").read_bytes()).hexdigest(),
                flight_execution_enabled=False, model_file_count=len(manifest["files"]))
