"""接管距离判定。不同阶段保留各自闸门，共用坐标计算。"""
import argparse
import math
from .px4_frames import LocalFrameAlignment

from boom_birds_control.runtime_config import DEFAULTS, SCENES

DEFAULT_HANDOFF_DISTANCE_M = DEFAULTS.handoff_max_distance_m


def handoff_allowed(command_ros, position_ros, max_distance_m=DEFAULT_HANDOFF_DISTANCE_M):
    if len(command_ros) != 3 or len(position_ros) != 3:
        return False
    if not math.isfinite(max_distance_m) or max_distance_m <= 0:
        return False
    if not all(math.isfinite(v) for v in (*command_ros, *position_ros)):
        return False
    return math.dist(command_ros, position_ros) <= max_distance_m


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--command", nargs=3, type=float, required=True)
    parser.add_argument("--ned", nargs=3, type=float, required=True)
    parser.add_argument("--origin", nargs=3, type=float, default=None)
    parser.add_argument("--scene", choices=tuple(SCENES), default="local")
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--max-distance", type=float, default=DEFAULT_HANDOFF_DISTANCE_M)
    args = parser.parse_args()
    try:
        position = LocalFrameAlignment(yaw_offset_rad=args.yaw, translation_m=tuple(args.origin) if args.origin is not None else SCENES[args.scene].origin).position_ned_to_ros(args.ned)
        allowed = handoff_allowed(args.command, position, args.max_distance)
    except ValueError:
        allowed = False
    if not allowed:
        parser.exit(1, "拒绝接管：坐标无效或设定点距离超限\n")


if __name__ == "__main__":
    main()
