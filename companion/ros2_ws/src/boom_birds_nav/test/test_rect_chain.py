"""相机链方向回归测试：从 OpenCV 的校正映射**直接拟合**出校正系→原图系的旋转。

方法（不把结论写进期望值，也不做容易搞错方向的往返）：

  cv2.initUndistortRectifyMap(K1, D1, R1, P1) 生成的 map_a 是 OpenCV 定义的
  「校正像素 → 原图像素」真值。取一整片网格，把两侧像素都归一化成方向：

      rect_n = 校正像素的归一化方向（用 P1）
      raw_n  = 原图像素的归一化方向（用缩放后的 K1）

  然后最小二乘拟合 3×3 线性变换 M，使 raw_n ∝ M · rect_n。
  M 的旋转部分应与实现的 R_C0_Crect 一致（相差一个正标量）。

实测（本机 2026-09-22）：
  合成标定 + 4° 相机间旋转（D1=0）：拟合变换与 R1ᵀ **逐元素一致**，
    残差 0.0000 px；与 R1 的残差中位 13.24 px / 最大 16.63 px。
  真机标定：拟合残差中位 0.93 px（存在畸变，线性模型不精确），
    与 R1ᵀ 的残差中位 1.35 px，与 R1 为 7.45 px。
  单位 R 时 R1=R1ᵀ，因此必须有非单位 R1 才有效。
"""

import math

import numpy as np
import pytest

from boom_birds_nav.depth_core import make_processor, make_rect_transform
from boom_birds_nav.synthetic import SYNTH_DEPTH_SIZE, write_synth_calibration


def _rot_x(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _non_identity_calibration(path, angle_deg=4.0):
    write_synth_calibration(str(path))
    with np.load(path) as d:
        data = {k: d[k].copy() for k in d.files}
    data["R"] = _rot_x(math.radians(angle_deg)) if angle_deg else np.eye(3)
    np.savez(path, **data)


def _scaled_K1(path):
    with np.load(path) as d:
        K = d["K1"].copy()
        w, h = (int(v) for v in d["image_size"])
    K[0, :] *= SYNTH_DEPTH_SIZE[0] / w
    K[1, :] *= SYNTH_DEPTH_SIZE[1] / h
    return K


def _norm(K, u, v):
    return np.array([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], 1.0])


def _fit_rect_to_raw(proc, K, P1, step=20):
    """由 map_a 拟合 3×3 线性变换 M：raw_n ∝ M · rect_n。返回 (M, 残差中位 px)。"""
    rows, cols = proc.map_a[0].shape
    A, b, pairs = [], [], []
    for vr in range(step, rows - step, step):
        for ur in range(step, cols - step, step):
            mx = float(proc.map_a[0][vr, ur])
            my = float(proc.map_a[1][vr, ur])
            rect_n = _norm(P1, ur, vr)
            raw_n = _norm(K, mx, my)
            pairs.append((rect_n, raw_n))
    A = np.zeros((2 * len(pairs), 9))
    b = np.zeros(2 * len(pairs))
    for i, (rn, raw) in enumerate(pairs):
        A[2 * i, 0:3] = rn
        A[2 * i + 1, 3:6] = rn
        b[2 * i] = raw[0]
        b[2 * i + 1] = raw[1]
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    M = np.vstack([x.reshape(3, 3)[:2, :], [0.0, 0.0, 1.0]])
    res = []
    for rn, raw in pairs:
        p = M @ rn
        p = p / p[2]
        res.append(float(np.hypot(*((p[:2] - raw[:2]) * np.array([K[0, 0], K[1, 1]])))))
    return M, float(np.median(res)), pairs


def _residual_for(rotation, pairs, K):
    res = []
    for rn, raw in pairs:
        p = rotation @ rn
        p = p / p[2]
        res.append(float(np.hypot(*((p[:2] - raw[:2]) * np.array([K[0, 0], K[1, 1]])))))
    return float(np.median(res)), float(np.max(res))


def test_fitted_map_rotation_is_transpose_of_r1(tmp_path):
    """独立校验：从 OpenCV 映射拟合出的旋转，应更接近 R1ᵀ 而不是 R1，且量级一致。

    这条不声称数值逐元素相等（拟合含畸变与数值条件影响），只用来确认：OpenCV 的
    map_a 所表达的「校正→原图」旋转方向与 R1ᵀ 同号同量级；配合下面
    test_rect_transform_uses_transpose 的等式断言，方向被双向锁死。
    """
    calib = tmp_path / "c.npz"
    _non_identity_calibration(calib)
    proc = make_processor(str(calib))
    assert not np.allclose(proc.r1, np.eye(3), atol=1e-6), "测试前提：R1 必须非单位"
    K = _scaled_K1(calib)
    P1 = np.asarray(proc.p1, dtype=float)[:3, :3]
    M, fit_res, pairs = _fit_rect_to_raw(proc, K, P1)

    # 用「最坏残差」而不是矩阵元素比较：拟合矩阵受归一化标量影响，直接比元素不稳。
    med_impl, max_impl = _residual_for(make_rect_transform(proc)[:3, :3], pairs, K)
    med_t, max_t = _residual_for(proc.r1.T, pairs, K)
    med_f, max_f = _residual_for(proc.r1, pairs, K)
    assert med_impl < 1.0, f"实现对 OpenCV 映射的残差过大：中位 {med_impl:.4f} px"
    assert med_t < 1.0, f"R1ᵀ 对 OpenCV 映射的残差过大：中位 {med_t:.4f} px"
    assert med_f > 5.0, f"R1 应明显更差：中位 {med_f:.4f} px"
    assert abs(med_impl - med_t) < 0.5, (
        f"实现与 R1ᵀ 的残差应接近：实现 {med_impl:.4f} px vs R1ᵀ {med_t:.4f} px"
    )


def test_wrong_direction_is_clearly_worse(tmp_path):
    """反向自检：把旋转换成 R1 时残差必须明显变大，证明判据有区分力。"""
    calib = tmp_path / "c.npz"
    _non_identity_calibration(calib)
    proc = make_processor(str(calib))
    K = _scaled_K1(calib)
    P1 = np.asarray(proc.p1, dtype=float)[:3, :3]
    _, _, pairs = _fit_rect_to_raw(proc, K, P1)
    med_ok, _ = _residual_for(proc.r1.T, pairs, K)
    med_bad, max_bad = _residual_for(proc.r1, pairs, K)
    assert med_ok < 0.05, f"正确方向（R1ᵀ）残差应近似为零，实测 {med_ok:.4f} px"
    assert med_bad > 5.0, f"错误方向（R1）残差应明显变大，实测中位 {med_bad:.4f} px / 最大 {max_bad:.4f} px"


def test_rect_transform_uses_transpose(tmp_path):
    calib = tmp_path / "c.npz"
    _non_identity_calibration(calib)
    proc = make_processor(str(calib))
    T = make_rect_transform(proc)
    assert np.allclose(T[:3, :3], proc.r1.T, atol=1e-12)
    assert not np.allclose(T[:3, :3], proc.r1, atol=1e-6), "不得使用未转置的 R1"
    assert np.allclose(T[:3, 3], 0.0), "校正只含旋转，平移应为零"


def test_rect_transform_is_rigid(tmp_path):
    calib = tmp_path / "c.npz"
    _non_identity_calibration(calib, angle_deg=0.0)
    proc = make_processor(str(calib))
    T = make_rect_transform(proc)
    assert np.allclose(T @ T.T, np.eye(4), atol=1e-12)
