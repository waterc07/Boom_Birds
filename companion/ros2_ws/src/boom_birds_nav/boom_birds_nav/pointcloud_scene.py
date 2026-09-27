"""TEST-ONLY：把 EGO mockamap 世界点云投影成合成双目所需的深度底图。"""

from __future__ import annotations

import cv2
import numpy as np

from .synthetic import F_EFF, SCALE, SYNTH_CX, SYNTH_CY, SYNTH_DEPTH_SIZE


def depth_from_world_cloud(points_world: np.ndarray, camera_pose_world: np.ndarray,
                           *, resolution_m: float = 0.1,
                           size: tuple[int, int] = SYNTH_DEPTH_SIZE,
                           splat_span_px: int | None = None) -> np.ndarray:
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
    if splat_span_px is not None and (splat_span_px < 1 or splat_span_px % 2 != 1):
        raise ValueError("splat_span_px 必须是正奇数")
    inverse = np.zeros((height, width), dtype=np.float32)
    # 近处点的角间距更大。按距离分层覆盖采样间隙，避免用全图中位距离
    # 让近处障碍仍留出洞；各层取最近深度，保留遮挡关系。
    layers = ((0.15, 2.0), (2.0, 4.0), (4.0, 8.0), (8.0, 20.0))
    for near, far in layers:
        selected = (z >= near) & (z < far)
        if not np.any(selected):
            continue
        flat = np.full(width * height, np.inf, dtype=np.float32)
        np.minimum.at(flat, v[selected] * width + u[selected], z[selected])
        sparse = flat.reshape(height, width)
        layer = np.where(np.isfinite(sparse), 1.0 / sparse, 0.0)
        reference_depth = max(0.5, np.sqrt(near * far))
        span = (int(splat_span_px) if splat_span_px is not None else
                max(5, min(41, 2 * int(np.ceil(
                    0.75 * F_EFF * resolution_m / reference_depth)) + 1)))
        np.maximum(inverse, cv2.dilate(layer, np.ones((span, span), np.uint8)), out=inverse)
    observed = inverse > 0.0
    depth[observed] = 1.0 / inverse[observed]
    return depth
