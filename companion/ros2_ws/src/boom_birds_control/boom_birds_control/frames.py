"""位姿、四元数与齐次变换工具（纯函数，可离线测试）。

记号约定：T_A_B 表示「把 B 系坐标变换到 A 系」，即
    p_A = R_A_B @ p_B + p_A_B
齐次形式 T_A_B = [[R_A_B, p_A_B], [0, 0, 0, 1]]。

与 OpenVINS/ROS 相关的方向已在本仓库源码中核实：
- OpenVINS 发布的 poseimu/odomimu 是「IMU 在 global 中的位姿」，即 T_W_I
  （ROS2Visualizer.cpp:601-611 直接使用 state->_imu->quat()/pos()）。
- Kalibr/OpenVINS 的 T_imu_cam 字段即 T_I_C0（VioManagerOptions.h:263-269）。
- cv2.stereoRectify 的 R1 把「校正后坐标」映射回原图坐标，故
  T_C0_Crect 的旋转为 R1（见 depth_core.camera_rect_transform 的说明）。
"""

from __future__ import annotations

import math

import numpy as np


def quat_to_rot(q) -> np.ndarray:
    """四元数 (x, y, z, w) → 旋转矩阵 R_A_B（Hamilton，主动旋转）。"""
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise ValueError("四元数模长过小，无法归一化")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rot_to_quat(R: np.ndarray):
    """旋转矩阵 → 四元数 (x, y, z, w)。"""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    tr = float(np.trace(R))
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    return q / np.linalg.norm(q)


def make_transform(R: np.ndarray, p) -> np.ndarray:
    """由旋转与平移构造 T_A_B。"""
    T = np.eye(4, dtype=float)
    T[:3, :3] = np.asarray(R, dtype=float).reshape(3, 3)
    T[:3, 3] = np.asarray(p, dtype=float).reshape(3)
    return T


def transform_from_quat_pos(q, p) -> np.ndarray:
    return make_transform(quat_to_rot(q), p)


def invert_transform(T: np.ndarray) -> np.ndarray:
    """T_A_B → T_B_A 的解析逆。"""
    T = np.asarray(T, dtype=float).reshape(4, 4)
    R = T[:3, :3]
    t = T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def compose(*transforms: np.ndarray) -> np.ndarray:
    """按 T_A_C = T_A_B @ T_B_C 顺序左乘复合。"""
    if not transforms:
        raise ValueError("compose 至少需要一个变换")
    out = np.asarray(transforms[0], dtype=float).reshape(4, 4)
    for T in transforms[1:]:
        out = out @ np.asarray(T, dtype=float).reshape(4, 4)
    return out


def rotate_covariance(R: np.ndarray, cov: np.ndarray) -> np.ndarray:
    """把 3×3 协方差从被旋转前的坐标系变到旋转后的坐标系：C' = R C Rᵀ。"""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    cov = np.asarray(cov, dtype=float).reshape(3, 3)
    return R @ cov @ R.T


def transform_covariance_6x6(T: np.ndarray, cov6: np.ndarray) -> np.ndarray:
    """对 ROS 顺序 (x,y,z,r,p,y) 的 6×6 协方差整体做刚体旋转。"""
    cov6 = np.asarray(cov6, dtype=float).reshape(6, 6)
    R = np.asarray(T, dtype=float).reshape(4, 4)[:3, :3]
    B = np.zeros((6, 6))
    B[:3, :3] = R
    B[3:, 3:] = R
    return B @ cov6 @ B.T


def body_velocity_from_imu(v_w_i, omega_i, p_i_b, R_w_i) -> np.ndarray:
    """杆臂修正后的机体原点世界速度。

    v_W_B = v_W_I + R_W_I (ω_I × p_I_B)
    p_I_B 是机体原点在 IMU 系中的位置。两原点重合时 p_I_B = 0，退化为 v_W_B = v_W_I。
    """
    v_w_i = np.asarray(v_w_i, dtype=float).reshape(3)
    omega_i = np.asarray(omega_i, dtype=float).reshape(3)
    p_i_b = np.asarray(p_i_b, dtype=float).reshape(3)
    R_w_i = np.asarray(R_w_i, dtype=float).reshape(3, 3)
    return v_w_i + R_w_i @ np.cross(omega_i, p_i_b)


def is_rotation(R: np.ndarray, tol: float = 1e-6) -> bool:
    R = np.asarray(R, dtype=float).reshape(3, 3)
    return bool(np.allclose(R @ R.T, np.eye(3), atol=tol) and abs(np.linalg.det(R) - 1.0) < tol)
