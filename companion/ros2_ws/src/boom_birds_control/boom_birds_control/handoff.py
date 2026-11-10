"""接管距离判定。不同阶段保留各自闸门，共用坐标计算。"""
import math

from boom_birds_control.runtime_config import DEFAULTS

DEFAULT_HANDOFF_DISTANCE_M = DEFAULTS.handoff_max_distance_m


def handoffAllowed(commandRos, positionRos, maxDistanceM=DEFAULT_HANDOFF_DISTANCE_M):
    if len(commandRos) != 3 or len(positionRos) != 3:
        return False
    if not math.isfinite(maxDistanceM) or maxDistanceM <= 0:
        return False
    if not all(math.isfinite(v) for v in (*commandRos, *positionRos)):
        return False
    return math.dist(commandRos, positionRos) <= maxDistanceM
