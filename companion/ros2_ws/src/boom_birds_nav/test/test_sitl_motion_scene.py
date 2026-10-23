"""TEST-ONLY 运动场景的几何回归。"""

import numpy as np

from boom_birds_sim.sitl_truth_source import nedToRosPosition, px4AttitudeToRosRotation
from boom_birds_sim.synthetic import defaultScene


def test_camera_moves_toward_and_past_obstacle():
    scene = defaultScene()
    center = (120, 157)
    nearBefore = float(scene.depthMap()[center])
    assert abs(nearBefore - 1.8) < 1e-5

    scene.camera_x = 0.5
    nearAfter = float(scene.depthMap()[center])
    assert abs(nearAfter - 1.3) < 1e-5

    scene.camera_x = 2.0
    assert abs(float(scene.depthMap()[center]) - 1.0) < 1e-5


def test_scene_y_and_z_follow_camera():
    scene = defaultScene()
    depth = scene.depthMap()
    assert np.isclose(depth[120, 157], 1.8)
    scene.camera_y_offset = 2.0
    assert np.isclose(scene.depthMap()[120, 157], 3.0)
    scene.camera_y_offset = 0.0
    scene.camera_z = 0.0
    assert np.isclose(scene.depthMap()[120, 157], 3.0)


def test_px4_ned_to_project_world():
    assert nedToRosPosition((1.0, 2.0, -3.0)) == (1.0, -2.0, 3.0)
    assert nedToRosPosition((0.0, 0.0, 0.0), (-15.0, 0.0, 0.1)) == (-15.0, 0.0, 0.1)
    np.testing.assert_allclose(px4AttitudeToRosRotation(0.0, 0.0, 0.0), np.eye(3))


def test_stereo_depth_changes_with_attitude():
    scene = defaultScene()
    straight = float(scene.depthMap()[120, 157])
    bodyRotation = px4AttitudeToRosRotation(0.0, 0.2, 0.0)
    opticalFromBody = scene.cameraPoseWorld()[:3, :3]
    scene.camera_rotation = bodyRotation @ opticalFromBody
    angled = float(scene.depthMap()[120, 157])
    assert np.isfinite(angled)
    assert abs(angled - straight) > 0.01

# ---------------------------------------------------------------- A3：几何同源

import pytest                                     # noqa: E402

from boom_birds_sensing.camera_geometry import (      # noqa: E402
    depthGeometryMismatch,
    expectedDepthGeometry,
    stereoBaselineTerm,
    validateProjectionMatrix,
)
from boom_birds_sim.synthetic import (            # noqa: E402
    SYNTH_BASELINE_M,
    writeSynthCalibration,
)


def test_sitl_scene_output_size_comes_from_one_calibration(tmp_path):
    """SIH 运动场景的深度输出尺寸必须与 EGO 内参来自同一标定、同一次输出。"""
    calib = tmp_path / "synth.npz"
    writeSynthCalibration(str(calib))
    depth = defaultScene().depthMap()
    expected = expectedDepthGeometry(calib)
    assert (expected["width"], expected["height"]) == (depth.shape[1], depth.shape[0])
    assert float(expected["baseline_m"]) == pytest.approx(SYNTH_BASELINE_M, rel=1e-9)
    assert depthGeometryMismatch(expected, calib) == ""
    # 内参或尺寸变化必须被发现，不得静默沿用旧几何
    assert depthGeometryMismatch(dict(expected, fx=expected["fx"] * 1.05), calib) != ""
    assert depthGeometryMismatch(dict(expected, width=depth.shape[1] // 2), calib) != ""


def test_sitl_scene_geometry_preserves_the_metric_baseline_at_any_output_scale(tmp_path):
    """跨分辨率只缩放 fx/fy/cx/cy；物理基线与右目基线项口径不变。"""
    calib = tmp_path / "synth.npz"
    writeSynthCalibration(str(calib))
    full = expectedDepthGeometry(calib, 1.0)
    for scale in (0.75, 0.5):
        scaled = expectedDepthGeometry(calib, scale)
        assert scaled["baseline_m"] == full["baseline_m"], "物理基线不随分辨率缩放"
        assert scaled["fx"] == full["fx"] * (scaled["width"] / full["width"])
        # 右目投影项仍等于 -fx_当前分辨率 · B_物理
        assert stereoBaselineTerm(scaled["fx"], scaled["baseline_m"]) == \
            -scaled["fx"] * scaled["baseline_m"]
        validateProjectionMatrix(
            [scaled["fx"], 0.0, scaled["cx"], 0.0,
             0.0, scaled["fy"], scaled["cy"], 0.0,
             0.0, 0.0, 1.0, 0.0],
            scaled["width"], scaled["height"], side="left")