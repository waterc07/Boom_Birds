"""深度契约测试：单位、无效值、H×W 对应关系与内参同源。"""

import numpy as np
import pytest

from boom_birds_nav.depth_core import (
    annotate_validity,
    make_processor,
    process_stitched,
    rectified_camera_info,
    xyz_to_compact,
    xyz_to_structured,
)
from boom_birds_nav.synthetic import (
    F_EFF,
    SYNTH_BASELINE_M,
    default_scene,
    disparity_quantization_m,
    render_stereo,
    write_synth_calibration,
)


@pytest.fixture(scope="module")
def synth(tmp_path_factory):
    calib = tmp_path_factory.mktemp("synth") / "synth_candidate.npz"
    write_synth_calibration(str(calib))
    processor = make_processor(str(calib))
    scene = default_scene()
    left, right, depth_gt = render_stereo(scene)
    stitched = np.hstack([left, right])[:, :, None].repeat(3, axis=2)
    result = process_stitched(processor, stitched)
    return {"processor": processor, "depth_gt": depth_gt, "result": result, "calib": str(calib)}


def test_missing_calibration_fails_loudly(tmp_path):
    with pytest.raises(FileNotFoundError):
        make_processor(str(tmp_path / "missing.npz"))


def test_intrinsics_come_from_same_calibration(synth):
    """内参必须与深度计算同源：来自 stereoRectify(alpha=0) 的 P1，而不是原始标定 K。

    实测值（合成标定，2026-09-22）：fx=fy=172.5980149526744、cx=156.8623504638672、
    cy=116.15113067626953。alpha=0 会在边缘裁掉少量视野，因此 fx 略小于 K/4=172.5980121。
    """
    info = rectified_camera_info(synth["processor"])
    assert abs(info["fx"] - 172.5980149526744) < 1e-9
    assert abs(info["fy"] - 172.5980149526744) < 1e-9
    assert abs(info["cx"] - 156.8623504638672) < 1e-6
    assert abs(info["cy"] - 116.15113067626953) < 1e-6
    assert abs(info["baseline_m"] - SYNTH_BASELINE_M) < 1e-9
    assert (info["width"], info["height"]) == (320, 240)


def test_xyz_shape_and_nan_contract(synth):
    result = synth["result"]
    assert result.xyz.shape == (240, 320, 3)
    assert result.valid.shape == (240, 320)
    invalid = ~result.valid
    assert invalid.any()
    assert np.isnan(result.xyz[invalid]).all()
    assert np.isfinite(result.xyz[result.valid]).all()
    assert (result.xyz[:, :, 2][result.valid] > 0).all()


def test_structured_cloud_keeps_pixel_layout(synth):
    result = synth["result"]
    pts, is_dense = xyz_to_structured(result.xyz, result.valid)
    assert pts.shape == (240 * 320,)
    assert is_dense is False
    good = int(np.argwhere(result.valid.reshape(-1))[0][0])
    bad = int(np.argwhere((~result.valid).reshape(-1))[0][0])
    assert np.isfinite(pts["x"][good])
    assert np.isnan(pts["x"][bad]) and np.isnan(pts["y"][bad]) and np.isnan(pts["z"][bad])


def test_compact_cloud_only_valid_points(synth):
    result = synth["result"]
    pts, is_dense = xyz_to_compact(result.xyz, result.valid)
    assert is_dense is True
    assert pts.shape[0] == int(result.valid.sum())
    assert np.isfinite(np.stack([pts["x"], pts["y"], pts["z"]], axis=1)).all()


def test_depth_publish_conversion_and_masks():
    depth = np.array([[np.nan, 0.1, 1.0, 4.0, 9.0]], dtype=np.float32)
    valid = np.array([[True, True, True, True, True]])
    out, invalid, over = annotate_validity(depth, valid, max_depth_m=5.0, min_depth_m=0.2)
    assert out[0, 0] == 0.0 and out[0, 1] == 0.0 and out[0, 4] == 0.0
    assert out[0, 2] == pytest.approx(1.0) and out[0, 3] == pytest.approx(4.0)
    assert invalid.tolist() == [[True, True, False, False, True]]
    assert over.tolist() == [[False, False, False, False, True]]
    assert not np.isnan(out).any()


def test_match_failure_is_not_treated_as_free_space():
    out, invalid, over = annotate_validity(np.array([[2.0]], dtype=np.float32), np.array([[False]]), 5.0, 0.2)
    assert invalid[0, 0] and out[0, 0] == 0.0 and not over[0, 0]


def test_geometry_matches_analytic_expectation(synth):
    result = synth["result"]
    depth_gt = synth["depth_gt"]
    z = result.xyz[:, :, 2]
    m = 20
    core = np.zeros_like(result.valid)
    core[m:-m, m:-m] = True
    for expect in (3.0, 1.8):
        sel = result.valid & core & (np.abs(depth_gt - expect) < 1e-3)
        assert sel.sum() > 1000, f"有效样本过少：{sel.sum()}"
        err = np.abs(z[sel] - expect)
        floor = disparity_quantization_m(expect)
        # 阈值来自实测（2026-09-22，本机）：墙(3 m) median 0.078 m / P95 0.167 m；
        # 障碍(1.8 m) median 0.148 m / P95 0.538 m。上界取实测的 ~1.5 倍，
        # 既防止回归，也不假装能优于 1/16 px 视差量化给出的物理下界。
        if expect > 2.0:
            med_limit, p95_limit = 0.12, 0.25
        else:
            med_limit, p95_limit = 0.22, 0.80
        assert np.median(err) <= med_limit, (expect, float(np.median(err)), floor)
        assert np.percentile(err, 95) <= p95_limit, (expect, float(np.percentile(err, 95)), floor)


def test_xyz_z_equals_depth_channel(synth):
    result = synth["result"]
    assert np.allclose(np.nan_to_num(result.depth, nan=-1.0), np.nan_to_num(result.xyz[:, :, 2], nan=-1.0))
