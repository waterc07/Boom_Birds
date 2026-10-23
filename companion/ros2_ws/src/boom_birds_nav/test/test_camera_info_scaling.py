"""原始 CameraInfo 的跨分辨率缩放口径（P1-1 回归）。

背景：`rawCameraInfo` 曾把**米制物理基线**也乘上分辨率缩放比，于是右目
`P[0][3] = -fx·B` 在 scale=0.5 时变成正确值的 1/4，下游三角化尺度整体错掉。
正确口径：内参（像素量）随分辨率缩放，物理基线（米制刚体量）不缩放。
"""

from __future__ import annotations

import numpy as np
import pytest

from boom_birds_sensing.stereo_source import rawCameraInfo

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
    _, bFull, sFull = rawCameraInfo(calib, "left", "cam0", CALIB_W, CALIB_H, scale="auto")
    _, bHalf, sHalf = rawCameraInfo(calib, "right", "cam1", CALIB_W // 2, CALIB_H // 2,
                                        scale="auto")
    assert sFull == pytest.approx(1.0)
    assert sHalf == pytest.approx(0.5)
    assert bFull == pytest.approx(BASELINE_M, rel=1e-12)
    assert bHalf == pytest.approx(BASELINE_M, rel=1e-12), (
        f"基线被缩放了：{bHalf} != {BASELINE_M}"
    )


def test_right_p_matrix_baseline_term_scales_once_not_twice():
    """P[0][3] = -fx_scaled · B_物理 ⇒ 相对全分辨率只变一次（scale^1），不是 scale^2。"""
    calib = _calibration()
    infoFull, b, _ = rawCameraInfo(calib, "right", "cam1", CALIB_W, CALIB_H, scale="auto")
    infoHalf, _, _ = rawCameraInfo(calib, "right", "cam1", CALIB_W // 2, CALIB_H // 2,
                                      scale="auto")
    fxFull = float(calib["K2"][0][0])
    fxHalf = fxFull * 0.5

    pFull = np.array(infoFull.p).reshape(3, 4)
    pHalf = np.array(infoHalf.p).reshape(3, 4)

    assert pFull[0, 3] == pytest.approx(-fxFull * b, rel=1e-12)
    assert pHalf[0, 3] == pytest.approx(-fxHalf * b, rel=1e-12), (
        "右目 P[0][3] 应为 -fx_scaled·B；当前值看起来把基线也缩了"
    )
    # 比值恰好是 scale（一次方），排除平方衰减
    assert pHalf[0, 3] / pFull[0, 3] == pytest.approx(0.5, rel=1e-12)
    wrongSq = -fxHalf * b * 0.5
    assert not np.isclose(pHalf[0, 3], wrongSq, rtol=1e-9), "出现 scale^2 的基线衰减"


def test_left_has_no_baseline_term_and_k_scales():
    calib = _calibration()
    info, _, scale = rawCameraInfo(calib, "left", "cam0", CALIB_W // 2, CALIB_H // 2,
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
        rawCameraInfo(calib, "left", "cam0", CALIB_W // 2, CALIB_H // 2, scale=1.0)

# ---------------------------------------------------------------- A3：物理基线语义
# 三路内参必须满足同一个米制口径：右目 P[0][3] = -fx_当前分辨率 · B_物理，
# 物理基线不随分辨率缩放，左目不得带基线项。
from boom_birds_sensing.camera_geometry import (          # noqa: E402
    GeometryError,
    stereoBaselineTerm,
    validateProjectionMatrix,
)


@pytest.mark.parametrize("width, height, scale", [
    (CALIB_W, CALIB_H, 1.0),
    (960, 720, 0.75),
    (CALIB_W // 2, CALIB_H // 2, 0.5),
])
def test_right_p_matrix_satisfies_the_metric_baseline_identity(width, height, scale):
    calib = _calibration()
    info, baselineM, gotScale = rawCameraInfo(calib, "right", "cam1", width, height,
                                                  scale="auto")
    assert gotScale == pytest.approx(scale, rel=1e-12)
    assert baselineM == pytest.approx(BASELINE_M, rel=1e-12), "基线不随分辨率缩放"
    p = np.array(info.p).reshape(3, 4)
    fx = float(calib["K2"][0][0]) * scale
    assert p[0, 0] == pytest.approx(fx, rel=1e-12)
    assert p[0, 3] == pytest.approx(stereoBaselineTerm(fx, baselineM), rel=1e-12)
    validateProjectionMatrix(info.p, width, height, side="right", baseline_m=baselineM)


def test_left_raw_info_has_no_baseline_term_and_passes_strict_validation():
    calib = _calibration()
    info, baselineM, _ = rawCameraInfo(calib, "left", "cam0", CALIB_W // 2, CALIB_H // 2,
                                          scale="auto")
    assert np.array(info.p).reshape(3, 4)[0, 3] == 0.0, "左目不应带基线项"
    validateProjectionMatrix(info.p, CALIB_W // 2, CALIB_H // 2, side="left")
    assert baselineM == pytest.approx(BASELINE_M, rel=1e-12)


def test_stereo_pair_stays_consistent_within_one_output_size():
    """同一标定、同一次输出尺寸下，左右两路只能相差一个基线项。"""
    calib = _calibration()
    left, baselineL, _ = rawCameraInfo(calib, "left", "cam0", 960, 720, scale="auto")
    right, baselineR, _ = rawCameraInfo(calib, "right", "cam1", 960, 720, scale="auto")
    assert baselineL == pytest.approx(baselineR, rel=1e-12), "左右必须报告同一物理基线"
    lp = np.array(left.p).reshape(3, 4)
    rp = np.array(right.p).reshape(3, 4)
    np.testing.assert_allclose(lp[0, :3], rp[0, :3], rtol=0, atol=1e-12)
    np.testing.assert_allclose(lp[1:, :], rp[1:, :], rtol=0, atol=1e-12)
    assert rp[0, 3] == pytest.approx(stereoBaselineTerm(lp[0, 0], baselineL), rel=1e-12)


def test_scaling_the_baseline_too_would_be_detected():
    """反向自检：把基线也乘 scale 的错误矩阵必须被严格校验拒绝。"""
    calib = _calibration()
    info, baselineM, _ = rawCameraInfo(calib, "right", "cam1", CALIB_W // 2, CALIB_H // 2,
                                          scale="auto")
    p = list(info.p)
    p[3] = -p[0] * baselineM * 0.5          # 旧 bug：基线被再缩一次
    with pytest.raises(GeometryError):
        validateProjectionMatrix(p, CALIB_W // 2, CALIB_H // 2, side="right",
                                   baseline_m=baselineM)