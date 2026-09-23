"""确定性合成测试数据（TEST-ONLY）。

用途与边界：
- 这里的相机模型、几何与阈值都是**测试**用的，不代表真机参数，也不得写进设备配置。
- A 类（精确几何）：用解析关系给出预期值，验证投影、变换与地图；
- B 类（双目算法）：用独立渲染的左右图驱动真实 StereoSGBM，只报告统计量。

合成标定几何（TEST-ONLY，与真机标定无关）：
    focal = 690.39204848 px（每目 1280×960、零畸变，故校正映射退化为纯缩放）
    principal = (627.44939423, 464.60451508)
    baseline = 0.06772 m（右目相对左目沿 -x）
深度图尺度：320×240，f_eff = focal/4，b = 0.06772 m。

实测限制（2026-09-22，本机）：
- StereoSGBM 在本配置下输出 1/16 px 量化视差，无亚像素插值；
  因此 3 m 处约 6.4 cm 量化台阶，深度误差受此限制，不能要求毫米级。
- 匹配有效性受一致性检查与斑点滤波影响，平面场景有效率约 0.41（含边界 99 px 不可测区）。
这些数字是测试预期，不是真机精度。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

SYNTH_FOCAL = 690.39204848
SYNTH_CX = 627.44939423
SYNTH_CY = 464.60451508
SYNTH_BASELINE_M = 0.06772
SYNTH_IMAGE_SIZE = (1280, 960)          # 每目标定分辨率
SYNTH_DEPTH_SIZE = (320, 240)           # 深度/XYZ 输出分辨率
SCALE = SYNTH_DEPTH_SIZE[0] / SYNTH_IMAGE_SIZE[0]   # 0.25
F_EFF = SYNTH_FOCAL * SCALE


@dataclass
class Plane:
    """世界系中的矩形障碍面，法向 +x 朝向相机。"""

    x: float
    y: float
    z: float
    half_y: float
    half_z: float


@dataclass
class SyntheticScene:
    """相机沿世界 +x 观察，重力方向为世界 -z。

    相机光学系：x 右 = 世界 -y，y 下 = 世界 -z，z 前 = 世界 +x。
    """

    wall_x: float = 3.0
    camera_z: float = 1.5
    planes: list = field(default_factory=list)

    def obstacle(self) -> Plane:
        return self.planes[0]

    def depth_map(self, width: int = SYNTH_DEPTH_SIZE[0], height: int = SYNTH_DEPTH_SIZE[1]) -> np.ndarray:
        """解析深度图：每像素的光轴深度（米）。"""
        f = F_EFF
        cx = SYNTH_CX * SCALE
        cy = SYNTH_CY * SCALE
        depth = np.full((height, width), float(self.wall_x), dtype=np.float32)
        us = np.arange(width, dtype=np.float32)[None, :]
        vs = np.arange(height, dtype=np.float32)[:, None]
        for plane in self.planes:
            x_c = (us - cx) * plane.x / f
            y_c = (vs - cy) * plane.x / f
            world_y = -x_c
            world_z = self.camera_z - y_c
            inside = (np.abs(world_y - plane.y) <= plane.half_y) & (np.abs(world_z - plane.z) <= plane.half_z)
            depth = np.where(inside & (plane.x < depth), plane.x, depth)
        return depth

    def camera_pose_world(self, y_offset: float = 0.0, z_offset: float = 0.0):
        """相机在 world 中的位姿 T_W_Crect。

        相机 +z 轴指向世界 +x，+x 轴指向世界 -y，+y 轴指向世界 -z。
        """
        R = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
        p = np.array([0.0, y_offset, self.camera_z + z_offset])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = p
        return T


def default_scene(world_x_offset: float = 0.0) -> SyntheticScene:
    """默认场景：3 m 墙 + 1.8 m 处 1.2 m 宽 × 0.8 m 高的障碍。

    world_x_offset 用于"换坐标系"测试：把整个环境沿世界 x 平移，同时相机原点也平移，
    因此相机相对环境的几何不变，而障碍在**世界坐标**下出现在新的位置。
    这样可以区分"地图冻结"与"地图清空"——旧坐标系的障碍不应出现在新地图里。
    """
    off = float(world_x_offset)
    return SyntheticScene(
        wall_x=3.0 + off,
        camera_z=1.5,
        planes=[Plane(x=1.8 + off, y=0.0, z=1.5, half_y=0.6, half_z=0.4)],
    )


def texture(height: int, width: int, seed: int = 7, blur_sigma: float = 2.5) -> np.ndarray:
    """确定性宽带纹理（模糊随机噪声）。

    说明：周期性格纹会让 SGBM 的左右一致性检查大量失败（实测有效率趋近 0），
    因此这里用非周期宽带纹理；模糊半径控制可匹配的特征尺度。
    """
    rng = np.random.default_rng(seed)
    raw = rng.integers(0, 256, size=(height, width), dtype=np.uint8)
    img = cv2.GaussianBlur(raw, (0, 0), blur_sigma)
    lo, hi = float(img.min()), float(img.max())
    if hi - lo < 1.0:
        return np.full((height, width), 128, dtype=np.uint8)
    return np.clip((img.astype(np.float32) - lo) * (200.0 / (hi - lo)) + 28.0, 0, 255).astype(np.uint8)


def render_stereo(scene: SyntheticScene, width: int = SYNTH_DEPTH_SIZE[0], height: int = SYNTH_DEPTH_SIZE[1]):
    """渲染左右图（深度图尺度、零畸变、已校正）。

    对每像素用解析深度反投影到世界，再投影到右目像素并采样纹理；
    返回 (left, right, depth_gt)，三者为深度图尺度。
    """
    f = F_EFF
    cx = SYNTH_CX * SCALE
    cy = SYNTH_CY * SCALE
    depth = scene.depth_map(width, height)
    tex = texture(height, width)
    left = tex.copy()
    right = np.zeros_like(left)
    us = np.arange(width, dtype=np.float32)[None, :]
    x_c = (us - cx) * depth / f
    u_r = np.rint((x_c + SYNTH_BASELINE_M) * f / depth + cx).astype(np.int32)
    valid = (u_r >= 0) & (u_r < width)
    cols = np.clip(u_r, 0, width - 1)
    rows = np.broadcast_to(np.arange(height, dtype=np.int32)[:, None], (height, width))
    right[valid] = tex[rows[valid], cols[valid]]
    return left, right, depth


def synth_calibration_dict() -> dict:
    """合成标定参数字典（TEST-ONLY），与 render_stereo 的前向模型一致。"""
    K = np.array([[SYNTH_FOCAL, 0.0, SYNTH_CX], [0.0, SYNTH_FOCAL, SYNTH_CY], [0.0, 0.0, 1.0]], dtype=float)
    D = np.zeros((1, 5), dtype=float)
    R = np.eye(3, dtype=float)
    T = np.array([[-SYNTH_BASELINE_M], [0.0], [0.0]], dtype=float)
    E = np.zeros((3, 3), dtype=float)
    E[1, 2] = -SYNTH_BASELINE_M
    E[2, 1] = SYNTH_BASELINE_M
    F = np.zeros((3, 3), dtype=float)
    F[1, 2] = 1.0
    F[2, 1] = -1.0
    Q = np.eye(4, dtype=float)
    Q[2, 3] = SYNTH_FOCAL
    Q[3, 2] = -1.0 / SYNTH_BASELINE_M
    return {
        "K1": K.copy(),
        "K2": K.copy(),
        "D1": D.copy(),
        "D2": D.copy(),
        "R": R,
        "T": T,
        "E": E,
        "F": F,
        "R1": R.copy(),
        "R2": R.copy(),
        "P1": np.hstack([K, np.zeros((3, 1))]),
        "P2": np.hstack([K, np.array([[-SYNTH_FOCAL * SYNTH_BASELINE_M], [0.0], [0.0]])]),
        "Q": Q,
        "image_size": np.array(SYNTH_IMAGE_SIZE, dtype=np.int64),
        "square_size_m": np.array(0.02, dtype=float),
    }


def write_synth_calibration(path: str) -> dict:
    d = synth_calibration_dict()
    np.savez(path, **d)
    return d


def disparity_quantization_m(z: float) -> float:
    """1/16 px 视差量化导致的深度台阶（米），用于给出误差上界的解析预期。"""
    d = F_EFF * SYNTH_BASELINE_M / z
    return abs(F_EFF * SYNTH_BASELINE_M / (d - 1.0 / 16.0) - z)


def expected_depth_from_disparity(disparity_px: float) -> float:
    return F_EFF * SYNTH_BASELINE_M / float(disparity_px)


def imu_samples_identity(rate_hz: float, duration_s: float, gravity: float = 9.81, seed: int = 11):
    """静止 IMU 采样（TEST-ONLY）：比力 = -重力在 IMU 系的投影，角速度为 0。

    仅用于验证接口与数据通路，不代表任何真机 IMU 特性。
    """
    rng = np.random.default_rng(seed)
    dt = 1.0 / rate_hz
    n = int(round(duration_s * rate_hz))
    for i in range(n):
        yield i * dt, np.zeros(3), np.array([0.0, 0.0, -gravity]) + rng.normal(0, 1e-3, 3)

def _cli(argv=None) -> int:
    """命令行入口：生成合成标定（TEST-ONLY）。

    用途：launch 之前先把合成标定写到磁盘，避免"节点先启动、文件后生成"的竞态。
        python3 -m boom_birds_nav.synthetic --write-calibration /tmp/x/synth.npz
    """
    import argparse

    ap = argparse.ArgumentParser(description="生成 Boom_Birds 合成标定（TEST-ONLY）")
    ap.add_argument("--write-calibration", required=True, help="输出 npz 路径")
    args = ap.parse_args(argv)
    path = Path(args.write_calibration)
    path.parent.mkdir(parents=True, exist_ok=True)
    write_synth_calibration(str(path))
    print(f"已写出合成标定（TEST-ONLY）：{path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(_cli())
