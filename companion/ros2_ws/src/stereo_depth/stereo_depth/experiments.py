"""实验匹配、位姿先验与 RKNN 残差修正；不输出传播深度。"""

from dataclasses import dataclass
import ctypes as ct
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np


@dataclass(frozen=True)
class DisparityPrior:
    disparity: np.ndarray
    previous_stamp: float
    current_stamp: float

    def forFrame(self, shape, stamp, maxAge=0.15):
        if (
            not np.isfinite([self.previous_stamp, self.current_stamp, stamp]).all()
            or not 0 < self.current_stamp - self.previous_stamp <= maxAge
            or abs(stamp - self.current_stamp) > 1e-6
            or self.disparity.shape != shape
        ):
            return None
        return self.disparity


def projectVioPrior(depth, p, q, T_current_previous, previous_stamp, current_stamp):
    """T_current_previous 将上一帧校正光学系点变到当前帧；要求已完成外参转换。"""
    depth = np.asarray(depth, dtype=np.float32)
    t = np.asarray(T_current_previous, dtype=np.float64)
    if (
        t.shape != (4, 4)
        or not np.isfinite(t).all()
        or not np.allclose(t[3], [0, 0, 0, 1])
        or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5)
        or not np.isclose(np.linalg.det(t[:3, :3]), 1, atol=1e-5)
    ):
        raise ValueError("VIO prior requires a finite rigid T_current_previous")
    h, w = depth.shape
    rows, cols = np.indices((h, w))
    valid = np.isfinite(depth) & (depth > 0)
    z = depth[valid].astype(np.float64)
    points = np.vstack(
        ((cols[valid] - p[0, 2]) * z / p[0, 0], (rows[valid] - p[1, 2]) * z / p[1, 1], z)
    )
    points = t[:3, :3] @ points + t[:3, 3, None]
    front = np.isfinite(points).all(axis=0) & (points[2] > 0)
    points = points[:, front]
    x = np.rint(p[0, 0] * points[0] / points[2] + p[0, 2]).astype(np.int64)
    y = np.rint(p[1, 1] * points[1] / points[2] + p[1, 2]).astype(np.int64)
    inside = (x >= 0) & (x < w) & (y >= 0) & (y < h)
    x, y, z = x[inside], y[inside], points[2, inside]
    # 近点覆盖远点；遮挡之外的新区域保持 NaN，回到全范围搜索。
    order = np.argsort(z, kind="stable")
    flat = y[order] * w + x[order]
    _, first = np.unique(flat, return_index=True)
    prior = np.full((h, w), np.nan, np.float32)
    prior.reshape(-1)[flat[first]] = (q[2, 3] / z[order[first]] - q[3, 3]) / q[3, 2]
    return DisparityPrior(prior, float(previous_stamp), float(current_stamp))


class CensusMatcher:
    def __init__(self, native, count, coarse=False, radius=4):
        self.native, self.count, self.coarse, self.radius = native, count, coarse, radius
        self.frames = 0
        self.stats = {}

    def computePair(self, a, b, prior=None):
        self.frames += 1
        guided = prior
        if guided is None and self.coarse:
            size = (a.shape[1] // 2, a.shape[0] // 2)
            ca = cv2.resize(a, size, interpolation=cv2.INTER_AREA)
            cb = cv2.resize(b, size, interpolation=cv2.INTER_AREA)
            d = self.native.census(ca, cb, self.count // 2).astype(np.float32) / 8
            d[d < 0] = np.nan
            guided = cv2.resize(d, (a.shape[1], a.shape[0]), interpolation=cv2.INTER_NEAREST)
        use = guided is not None and self.frames % 5 != 0
        fallback = False
        if use:
            finite = np.isfinite(guided) & (guided > 0) & (guided < self.count - 1)
            center = np.where(finite, guided, 0)
            lo = np.where(finite, np.maximum(0, np.floor(center - self.radius)), 0).astype(np.int16)
            hi = np.where(
                finite, np.minimum(self.count - 1, np.ceil(center + self.radius)), self.count - 1
            ).astype(np.int16)
            da = self.native.census(a, b, self.count, (lo, hi))
            values = da / 16
            risky = finite & ((da < 0) | (values <= lo) | (values >= hi))
            if np.any(risky):
                full = self.native.census(a, b, self.count)
                da[risky] = full[risky]
                fallback = True
        else:
            da = self.native.census(a, b, self.count)
        # 翻转双目后用同一个正视差核，B→A 转成负视差。
        db = self.native.census(np.fliplr(b).copy(), np.fliplr(a).copy(), self.count)
        db = -np.fliplr(db).copy()
        db[db > 0] = -16 * self.count
        self.stats = {
            "guided": bool(use),
            "full_fallback": fallback,
            "periodic_full": self.frames % 5 == 0,
        }
        return da, db


class ElasMatcher:
    def __init__(self, library, count):
        if not library:
            raise ValueError("ELAS requires elas_library")
        self.lib = ct.CDLL(str(Path(library).resolve(strict=True)))
        self.lib.bb_elas.argtypes = [
            ct.c_void_p,
            ct.c_void_p,
            ct.c_int,
            ct.c_int,
            ct.c_int,
            ct.c_void_p,
            ct.c_void_p,
        ]
        self.lib.bb_elas.restype = ct.c_int
        self.count = count

    def computePair(self, a, b, prior=None):
        if a.ndim != 2 or a.shape != b.shape or a.dtype != np.uint8 or b.dtype != np.uint8:
            raise ValueError("ELAS requires equal mono8 images")
        a, b = np.ascontiguousarray(a), np.ascontiguousarray(b)
        da, db = np.empty(a.shape, np.float32), np.empty(a.shape, np.float32)
        rc = self.lib.bb_elas(
            a.ctypes.data,
            b.ctypes.data,
            a.shape[1],
            a.shape[0],
            self.count,
            da.ctypes.data,
            db.ctypes.data,
        )
        if rc:
            raise RuntimeError(f"ELAS failed: {rc}")

        def q4(value, right=False):
            valid = np.isfinite(value) & (value >= 0) & (value < self.count)
            out = np.full(value.shape, -16 * self.count if right else -16, np.int16)
            out[valid] = np.rint(value[valid] * (-16 if right else 16)).astype(np.int16)
            return out

        return q4(da), q4(db, True)


class RknnResidualRefiner:
    """模型输入 NHWC [L/255,R/255,d/N]；输出 NHWC [delta_px,confidence]。"""

    def __init__(self, model, manifest, count):
        meta = json.loads(Path(manifest).read_text())
        data = Path(model).read_bytes()
        if meta.get("sha256") != hashlib.sha256(data).hexdigest():
            raise ValueError("RKNN model SHA256 mismatch")
        if meta.get("contract") != "stereo-residual-nhwc-v1" or meta.get("shape") != [
            1,
            240,
            320,
            3,
        ]:
            raise ValueError("unsupported RKNN residual contract")
        self.maxDelta = float(meta["max_delta_px"])
        self.minConf = float(meta["min_confidence"])
        if not 0 < self.maxDelta <= 1 or not 0 < self.minConf <= 1:
            raise ValueError("invalid RKNN correction limits")
        from rknnlite.api import RKNNLite

        self.runtime = RKNNLite()
        self.count = count
        try:
            if self.runtime.loadRknn(str(model)) != 0 or self.runtime.initRuntime() != 0:
                raise RuntimeError("RKNN initialization failed")
        except Exception:
            self.runtime.release()
            raise

    def refine(self, left, right, disparity):
        d = disparity.astype(np.float32) / 16
        inputs = np.stack(
            [
                left.astype(np.float32) / 255,
                right.astype(np.float32) / 255,
                np.maximum(d, 0) / self.count,
            ],
            axis=-1,
        )[None]
        outputs = self.runtime.inference(
            inputs=[np.ascontiguousarray(inputs)], dataFormat=["nhwc"]
        )
        if outputs is None or len(outputs) != 1:
            raise RuntimeError("RKNN residual output missing")
        value = np.asarray(outputs[0])
        if value.shape != (1, *left.shape, 2):
            raise ValueError("invalid RKNN output shape")
        delta, confidence = value[0, :, :, 0], value[0, :, :, 1]
        accept = (
            (d > 0) & np.isfinite(delta) & np.isfinite(confidence) & (confidence >= self.minConf)
        )
        accept &= (confidence <= 1) & (np.abs(delta) <= self.maxDelta)
        out = disparity.copy()
        corrected = d + delta
        accept &= (corrected > 0) & (corrected < self.count - 1)
        out[accept] = np.rint(corrected[accept] * 16).astype(np.int16)
        return out

    def close(self):
        self.runtime.release()


class PosePriorBuffer:
    """有界校正相机位姿缓存；丢失、重置、跳变时回到当前帧全范围匹配。"""

    def __init__(self, tolerance=0.02, maxAge=0.15):
        from collections import deque
        from threading import Lock

        self.poses = deque(maxlen=32)
        self.lock = Lock()
        self.previous = None
        self.active = None
        self.tolerance, self.maxAge = tolerance, maxAge
        self.world = None
        self.generation = 0

    def push(self, stamp, pose, world):
        pose = np.asarray(pose, dtype=np.float64)
        if (
            not np.isfinite(stamp)
            or pose.shape != (4, 4)
            or not np.isfinite(pose).all()
            or not np.allclose(pose[3], [0, 0, 0, 1])
            or not world
            or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(pose[:3, :3]), 1, atol=1e-5)
        ):
            raise ValueError("invalid calibrated camera pose")
        with self.lock:
            if self.world != world or (self.poses and stamp <= self.poses[-1][0]):
                self.poses.clear()
                self.previous = None
                self.active = None
                self.generation += 1
            self.world = world
            self.poses.append((float(stamp), pose.copy()))

    def prepare(self, stamp, p, q):
        with self.lock:
            self.active = None
            if not self.poses:
                self.previous = None
                return None
            sample = min(self.poses, key=lambda item: abs(item[0] - stamp))
            if abs(sample[0] - stamp) > self.tolerance:
                self.previous = None
                return None
            pose = sample[1]
            self.active = (float(stamp), pose.copy(), self.generation)
            previous = self.previous
            if previous is None:
                return None
            oldStamp, oldPose, depth, generation = previous
            dt = stamp - oldStamp
            if generation != self.generation or not 0 < dt <= self.maxAge:
                self.previous = None
                return None
            relative = np.linalg.inv(pose) @ oldPose
            rotation = np.arccos(np.clip((np.trace(relative[:3, :3]) - 1) / 2, -1, 1))
            if np.linalg.norm(relative[:3, 3]) > 0.5 or rotation > np.pi / 6:
                self.previous = None
                return None
            return projectVioPrior(depth, p, q, relative, oldStamp, stamp)

    def record(self, depth):
        with self.lock:
            if self.active is None or self.active[2] != self.generation:
                self.previous = None
                return
            stamp, pose, generation = self.active
            self.previous = (stamp, pose, np.asarray(depth, dtype=np.float32).copy(), generation)
            self.active = None
