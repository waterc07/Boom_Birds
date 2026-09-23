"""原始 CameraInfo 的跨分辨率缩放口径（P1-1 回归）。

背景：`raw_camera_info` 曾把**米制物理基线**也乘上分辨率缩放比，于是右目
`P[0][3] = -fx·B` 在 scale=0.5 时变成正确值的 1/4，下游三角化尺度整体错掉。
正确口径：内参（像素量）随分辨率缩放，物理基线（米制刚体量）不缩放。
"""

from __future__ import annotations

import numpy as np
import pytest

from boom_birds_nav.stereo_source import raw_camera_info

CALIB_W, CALIB_H = 1280, 960
BASELINE_M = 0.067672          # 与仓库当前标定同量级（T 的模）


def _calibration(image_size=(CALIB_W, CALIB_H)):
    k = [[900.0, 0.0, 640.0], [0.0, 900.0, 480.0], [0.0, 0.0, 1.0]]
    return {
        "image_size": np.array(image_size, dtype=int),
        "K1": np.array(k, dtype=float),
        "K2": np.array(k, dtype=float),
        "D1": np.zeros(5),
        "D2": np.zeros(5),
        # A→B 正视差水平双目：右目在左目 -x 侧
        "T": np.array([-BASELINE_M, 0.0, 0.0], dtype=float),
    }


def test_physical_baseline_is_never_scaled():
    """基线是米制刚体量：无论怎么缩放分辨率，返回的 baseline_m 都必须不变。"""
    calib = _calibration()
    _, b_full, s_full = raw_camera_info(calib, "left", "cam0", CALIB_W, CALIB_H, scale="auto")
    _, b_half, s_half = raw_camera_info(calib, "right", "cam1", CALIB_W // 2, CALIB_H // 2,
                                        scale="auto")
    assert s_full == pytest.approx(1.0)
    assert s_half == pytest.approx(0.5)
    assert b_full == pytest.approx(BASELINE_M, rel=1e-12)
    assert b_half == pytest.approx(BASELINE_M, rel=1e-12), (
        f"基线被缩放了：{b_half} != {BASELINE_M}"
    )


def test_right_p_matrix_baseline_term_scales_once_not_twice():
    """P[0][3] = -fx_scaled · B_物理 ⇒ 相对全分辨率只变一次（scale^1），不是 scale^2。"""
    calib = _calibration()
    info_full, b, _ = raw_camera_info(calib, "right", "cam1", CALIB_W, CALIB_H, scale="auto")
    info_half, _, _ = raw_camera_info(calib, "right", "cam1", CALIB_W // 2, CALIB_H // 2,
                                      scale="auto")
    fx_full = float(calib["K2"][0][0])
    fx_half = fx_full * 0.5

    p_full = np.array(info_full.p).reshape(3, 4)
    p_half = np.array(info_half.p).reshape(3, 4)

    assert p_full[0, 3] == pytest.approx(-fx_full * b, rel=1e-12)
    assert p_half[0, 3] == pytest.approx(-fx_half * b, rel=1e-12), (
        "右目 P[0][3] 应为 -fx_scaled·B；当前值看起来把基线也缩了"
    )
    # 比值恰好是 scale（一次方），排除平方衰减
    assert p_half[0, 3] / p_full[0, 3] == pytest.approx(0.5, rel=1e-12)
    wrong_sq = -fx_half * b * 0.5
    assert not np.isclose(p_half[0, 3], wrong_sq, rtol=1e-9), "出现 scale^2 的基线衰减"


def test_left_has_no_baseline_term_and_k_scales():
    calib = _calibration()
    info, _, scale = raw_camera_info(calib, "left", "cam0", CALIB_W // 2, CALIB_H // 2,
                                     scale="auto")
    p = np.array(info.p).reshape(3, 4)
    k = np.array(info.k).reshape(3, 3)
    assert scale == pytest.approx(0.5)
    assert p[0, 3] == pytest.approx(0.0), "左目不应带基线项"
    assert k[0, 0] == pytest.approx(450.0), "fx 应随分辨率缩放"
    assert k[0, 2] == pytest.approx(320.0), "cx 应随分辨率缩放"
    assert k[2, 2] == pytest.approx(1.0), "K[2][2] 恒为 1"
    # 畸变系数是无量纲的，不随分辨率缩放
    assert list(info.d) == pytest.approx([0.0] * 5)


def test_scale_one_rejects_resolution_mismatch():
    """显式 scale=1.0 时必须把尺寸不匹配暴露出来，而不是静默发错内参。"""
    calib = _calibration()
    with pytest.raises(RuntimeError):
        raw_camera_info(calib, "left", "cam0", CALIB_W // 2, CALIB_H // 2, scale=1.0)
