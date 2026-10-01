"""完整场景点云的静态可达性检查，仅用于 SIH 验收，不向规划器提供地图或路径。"""
from dataclasses import dataclass, asdict
from pathlib import Path
import argparse
import hashlib
import json
import math
import numpy as np
import yaml


@dataclass(frozen=True)
class GridSpec:
    resolution: float
    size: tuple
    ground: float
    inflation: float
    ceiling: float

    @classmethod
    def from_parameters(cls, parameters):
        get = lambda key: parameters["grid_map/" + key]
        return cls(float(get("resolution")),
                   tuple(float(get("map_size_" + axis)) for axis in "xyz"),
                   float(get("ground_height")), float(get("obstacles_inflation")),
                   float(get("virtual_ceil_height")))

    def geometry(self):
        values = (self.resolution, *self.size, self.ground, self.inflation, self.ceiling)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("非有限地图参数")
        if self.resolution <= 0 or min(self.size) <= 0 or self.inflation < 0:
            raise ValueError("无效地图尺寸或膨胀")
        shape = np.ceil(np.asarray(self.size) / self.resolution).astype(int)
        if math.prod(shape) > 32_000_000:
            raise ValueError("参考地图超过 3200 万体素")
        origin = np.array([-self.size[0] / 2, -self.size[1] / 2, self.ground])
        return shape, origin


def check_reachability(cloud, start, goal, spec, tolerance):
    from scipy.ndimage import maximum_filter, label, generate_binary_structure

    shape, origin = spec.geometry()
    cloud = np.asarray(cloud)
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    if cloud.ndim != 2 or cloud.shape[1] != 3 or not len(cloud) or not np.isfinite(cloud).all():
        raise ValueError("场景点云须为非空、有限 Nx3")
    if start.shape != (3,) or goal.shape != (3,) or not np.isfinite([start, goal]).all():
        raise ValueError("无效起终点")
    if not math.isfinite(tolerance) or tolerance < 0:
        raise ValueError("无效目标容差")
    index = lambda p: np.floor((p - origin) / spec.resolution).astype(int)
    si, gi = index(start), index(goal)
    if any(np.any(i < 0) or np.any(i >= shape) for i in (si, gi)):
        raise ValueError("起终点在地图外")
    ids = index(cloud)
    inside = ((ids >= 0) & (ids < shape)).all(axis=1)
    occupied = np.zeros(tuple(shape), dtype=bool)
    occupied[tuple(ids[inside].T)] = True
    steps = int(math.ceil(spec.inflation / spec.resolution))
    inflated = maximum_filter(occupied, size=2 * steps + 1, mode="constant")
    # 与 GridMap 的虚拟顶棚索引一致；参考检查不允许跨过顶棚。
    height = shape[2]
    if spec.ceiling > -0.5:
        height = int(math.floor((spec.ceiling - origin[2]) / spec.resolution)) - 1
        if not 0 < height <= shape[2]:
            raise ValueError("虚拟顶棚不在地图内")
    free = ~inflated[:, :, :height]
    point_free = lambda i: i[2] < height and bool(free[tuple(i)])
    result = dict(start=start.tolist(), goal=goal.tolist(), grid=asdict(spec),
                  point_count=len(cloud), points_inside_map=int(inside.sum()),
                  start_free=point_free(si), goal_free=point_free(gi),
                  goal_tolerance_m=tolerance, reachable_6=None, reachable_26=None,
                  tolerance_reachable_6=None, tolerance_reachable_26=None,
                  scope="complete raw-cloud voxel geometry; no sensor, dynamics or body-envelope proof")
    if not result["start_free"]:
        return dict(result, verdict="START_OCCUPIED")
    if not result["goal_free"]:
        return dict(result, verdict="GOAL_OCCUPIED")
    lo = np.maximum(index(goal - tolerance) - 1, 0)
    hi = np.minimum(index(goal + tolerance) + 1, np.array(free.shape) - 1)
    cells = np.stack(np.meshgrid(
        *(np.arange(lo[k], hi[k] + 1) for k in range(3)), indexing="ij"
    ), axis=-1).reshape(-1, 3)
    centers = origin + (cells + 0.5) * spec.resolution
    box_distance = np.linalg.norm(
        np.maximum(np.abs(centers - goal) - spec.resolution / 2, 0), axis=1)
    cells = cells[box_distance <= tolerance]
    for neighbors, structure in (
        (6, generate_binary_structure(3, 1)), (26, np.ones((3, 3, 3), dtype=int))
    ):
        components, _ = label(free, structure)
        component = components[tuple(si)]
        result[f"reachable_{neighbors}"] = bool(components[tuple(gi)] == component)
        result[f"tolerance_reachable_{neighbors}"] = bool(
            len(cells) and np.any(components[tuple(cells.T)] == component))
    verdict = ("REACHABLE" if result["reachable_6"] else
               "CORNER_ONLY" if result["reachable_26"] else "DISCONNECTED")
    return dict(result, verdict=verdict)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cloud", type=Path, required=True)
    parser.add_argument("--planner-parameters", type=Path, required=True)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--start", nargs=3, type=float)
    parser.add_argument("--goal", nargs=3, type=float, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        parser.error("报告已存在，拒绝覆盖")
    from boom_birds_control.runtime_config import SCENES
    config = SCENES[args.scene]
    dump = yaml.safe_load(args.planner_parameters.read_text())
    nodes = [v["ros__parameters"] for v in dump.values()
             if isinstance(v, dict) and "ros__parameters" in v]
    if len(nodes) != 1:
        parser.error("参数文件必须只包含一个规划节点")
    start = args.start or (np.asarray(config.origin) +
                          [0, 0, config.takeoff_altitude_agl_m]).tolist()
    result = check_reachability(np.load(args.cloud, allow_pickle=False), start, args.goal,
                               GridSpec.from_parameters(nodes[0]), config.goal_tolerance_m)
    result.update(scene=args.scene, start_source="explicit" if args.start else "configured takeoff point",
                  cloud_sha256=hashlib.sha256(args.cloud.read_bytes()).hexdigest(),
                  parameters_sha256=hashlib.sha256(args.planner_parameters.read_bytes()).hexdigest())
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("x") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result["verdict"] == "REACHABLE" else 5


if __name__ == "__main__":
    raise SystemExit(main())
