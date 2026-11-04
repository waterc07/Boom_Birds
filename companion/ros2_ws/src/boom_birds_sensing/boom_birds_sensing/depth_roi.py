"""执行包络在校正左目上的投影；视场不足时不截断需求。"""
from dataclasses import dataclass
import math
from threading import Lock
import time
import numpy as np


@dataclass(frozen=True)
class RoiSelection:
    bounds: tuple | None
    reason: str


def projectEnvelope(points, intrinsics, size, margin=0.5):
    points = np.asarray(points, dtype=float)
    k = np.asarray(intrinsics, dtype=float)
    if (points.ndim != 2 or points.shape[1:] != (3,) or not 8 <= len(points) <= 256
            or not np.isfinite(points).all() or np.any(points[:, 2] <= 0)
            or k.shape != (3, 3) or not np.isfinite(k).all()
            or min(k[0, 0], k[1, 1]) <= 0 or not np.allclose(k[2], [0, 0, 1])
            or not math.isfinite(margin) or margin < 0):
        return RoiSelection(None, "invalid_envelope")
    projected = points @ k.T
    uv = projected[:, :2] / projected[:, 2:]
    low, high = uv.min(axis=0), uv.max(axis=0)
    center = (low + high) / 2
    half = (high - low) * (1 + margin) / 2
    low, high = np.floor(center - half), np.ceil(center + half) + 1
    width, height = size
    # 不把超出视场的运动包络裁成看似完整的小区域。
    if low[0] < 0 or low[1] < 0 or high[0] > width or high[1] > height:
        return RoiSelection(None, "envelope_outside_fov")
    if np.any(high - low < 2):
        return RoiSelection(None, "degenerate_envelope")
    return RoiSelection(tuple(int(v) for v in (*low, *high)), "demand")


class DemandRoi:
    def __init__(self, frame, margin=0.5, timeout=0.1):
        if (not math.isfinite(margin) or margin < 0
                or not math.isfinite(timeout) or timeout <= 0):
            raise ValueError("ROI margin/timeout invalid")
        self.frame, self.margin, self.timeout = frame, margin, timeout
        self.lock = Lock()
        self.latest = None

    def push(self, stamp, frame, points, received=None):
        received = time.monotonic() if received is None else received
        points = np.asarray(points, dtype=float).copy()
        with self.lock:
            if (frame != self.frame or not math.isfinite(stamp) or stamp <= 0
                    or not math.isfinite(received)
                    or self.latest is not None and stamp <= self.latest[0]):
                self.latest = None
                return
            self.latest = (stamp, received, points)

    def select(self, stamp, intrinsics, size, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            item = self.latest
        if item is None:
            return RoiSelection(None, "demand_missing")
        sourceStamp, received, points = item
        if (not math.isfinite(stamp) or not math.isfinite(now)
                or not 0 <= stamp - sourceStamp <= self.timeout
                or not 0 <= now - received <= self.timeout):
            return RoiSelection(None, "demand_stale_or_future")
        return projectEnvelope(points, intrinsics, size, self.margin)
