"""TEST-ONLY 运动场景的几何回归。"""

import numpy as np

from boom_birds_sim.sitl_truth_source import ned_to_ros_position, px4_attitude_to_ros_rotation
from boom_birds_sim.synthetic import default_scene


def test_camera_moves_toward_and_past_obstacle():
    scene = default_scene()
    center = (120, 157)
    near_before = float(scene.depth_map()[center])
    assert abs(near_before - 1.8) < 1e-5

    scene.camera_x = 0.5
    near_after = float(scene.depth_map()[center])
    assert abs(near_after - 1.3) < 1e-5

    scene.camera_x = 2.0
    assert abs(float(scene.depth_map()[center]) - 1.0) < 1e-5


def test_scene_y_and_z_follow_camera():
    scene = default_scene()
    depth = scene.depth_map()
    assert np.isclose(depth[120, 157], 1.8)
    scene.camera_y_offset = 2.0
    assert np.isclose(scene.depth_map()[120, 157], 3.0)
    scene.camera_y_offset = 0.0
    scene.camera_z = 0.0
    assert np.isclose(scene.depth_map()[120, 157], 3.0)


def test_px4_ned_to_project_world():
    assert ned_to_ros_position((1.0, 2.0, -3.0)) == (1.0, -2.0, 3.0)
    assert ned_to_ros_position((0.0, 0.0, 0.0), (-15.0, 0.0, 0.1)) == (-15.0, 0.0, 0.1)
    np.testing.assert_allclose(px4_attitude_to_ros_rotation(0.0, 0.0, 0.0), np.eye(3))


def test_stereo_depth_changes_with_attitude():
    scene = default_scene()
    straight = float(scene.depth_map()[120, 157])
    body_rotation = px4_attitude_to_ros_rotation(0.0, 0.2, 0.0)
    optical_from_body = scene.camera_pose_world()[:3, :3]
    scene.camera_rotation = body_rotation @ optical_from_body
    angled = float(scene.depth_map()[120, 157])
    assert np.isfinite(angled)
    assert abs(angled - straight) > 0.01

# ---------------------------------------------------------------- A3：几何同源

import pytest                                     # noqa: E402

from boom_birds_sensing.camera_geometry import (      # noqa: E402
    depth_geometry_mismatch,
    expected_depth_geometry,
    stereo_baseline_term,
    validate_projection_matrix,
)
from boom_birds_sim.synthetic import (            # noqa: E402
    SYNTH_BASELINE_M,
    write_synth_calibration,
)


def test_sitl_scene_output_size_comes_from_one_calibration(tmp_path):
    """SIH 运动场景的深度输出尺寸必须与 EGO 内参来自同一标定、同一次输出。"""
    calib = tmp_path / "synth.npz"
    write_synth_calibration(str(calib))
    depth = default_scene().depth_map()
    expected = expected_depth_geometry(calib)
    assert (expected["width"], expected["height"]) == (depth.shape[1], depth.shape[0])
    assert float(expected["baseline_m"]) == pytest.approx(SYNTH_BASELINE_M, rel=1e-9)
    assert depth_geometry_mismatch(expected, calib) == ""
    # 内参或尺寸变化必须被发现，不得静默沿用旧几何
    assert depth_geometry_mismatch(dict(expected, fx=expected["fx"] * 1.05), calib) != ""
    assert depth_geometry_mismatch(dict(expected, width=depth.shape[1] // 2), calib) != ""


def test_sitl_scene_geometry_preserves_the_metric_baseline_at_any_output_scale(tmp_path):
    """跨分辨率只缩放 fx/fy/cx/cy；物理基线与右目基线项口径不变。"""
    calib = tmp_path / "synth.npz"
    write_synth_calibration(str(calib))
    full = expected_depth_geometry(calib, 1.0)
    for scale in (0.75, 0.5):
        scaled = expected_depth_geometry(calib, scale)
        assert scaled["baseline_m"] == full["baseline_m"], "物理基线不随分辨率缩放"
        assert scaled["fx"] == full["fx"] * (scaled["width"] / full["width"])
        # 右目投影项仍等于 -fx_当前分辨率 · B_物理
        assert stereo_baseline_term(scaled["fx"], scaled["baseline_m"]) == \
            -scaled["fx"] * scaled["baseline_m"]
        validate_projection_matrix(
            [scaled["fx"], 0.0, scaled["cx"], 0.0,
             0.0, scaled["fy"], scaled["cy"], 0.0,
             0.0, 0.0, 1.0, 0.0],
            scaled["width"], scaled["height"], side="left")