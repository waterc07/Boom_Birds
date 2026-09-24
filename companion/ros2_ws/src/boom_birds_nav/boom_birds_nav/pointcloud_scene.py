"""TEST-ONLY：把 EGO mockamap 世界点云投影成合成双目所需的深度底图。"""

from __future__ import annotations

import cv2
import numpy as np

from .synthetic import F_EFF, SCALE, SYNTH_CX, SYNTH_CY, SYNTH_DEPTH_SIZE


def depth_from_world_cloud(points_world: np.ndarray, camera_pose_world: np.ndarray,
                           *, resolution_m: float = 0.1,
                           size: tuple[int, int] = SYNTH_DEPTH_SIZE) -> np.ndarray:
    """点云 z-buffer + 体素投影覆盖；未观测像素保持 NaN。"""
    width, height = size
    depth = np.full((height, width), np.nan, dtype=np.float32)
    points = np.asarray(points_world, dtype=np.float32).reshape(-1, 3)
    if not len(points):
        return depth
    pose = np.asarray(camera_pose_world, dtype=np.float64)
    xyz = (points - pose[:3, 3]) @ pose[:3, :3]
    z = xyz[:, 2]
    valid = np.isfinite(xyz).all(axis=1) & (z > 0.15) & (z < 20.0)
    xyz, z = xyz[valid], z[valid]
    if not len(z):
        return depth
    u = np.rint(F_EFF * xyz[:, 0] / z + SYNTH_CX * SCALE).astype(np.int32)
    v = np.rint(F_EFF * xyz[:, 1] / z + SYNTH_CY * SCALE).astype(np.int32)
    inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    u, v, z = u[inside], v[inside], z[inside]
    if not len(z):
        return depth
    flat = np.full(width * height, np.inf, dtype=np.float32)
    np.minimum.at(flat, v * width + u, z)
    sparse = flat.reshape(height, width)
    # mockamap 的采样间距约 0.1 m；覆盖采样间隙，近点优先并保持空白为未知。
    inverse = np.where(np.isfinite(sparse), 1.0 / sparse, 0.0)
    span = max(3, min(13, 2 * int(round(F_EFF * resolution_m / (2.0 * max(float(np.median(z)), 0.5)))) + 1))
    inverse = cv2.dilate(inverse, np.ones((span, span), np.uint8))
    observed = inverse > 0.0
    depth[observed] = 1.0 / inverse[observed]
    return depth
