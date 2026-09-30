"""EGO 世界点云投影到合成双目的几何回归。"""

import pytest
import numpy as np

from boom_birds_sim.pointcloud_scene import depth_from_world_cloud
from boom_birds_sim.synthetic import (F_EFF, SYNTH_BASELINE_M, SyntheticScene,
                                      render_stereo_from_pointcloud_depth)


def test_world_cloud_moves_with_camera_and_ignores_points_behind_it():
    scene = SyntheticScene()
    cloud = np.array([[2.0, 0.0, 1.5], [3.0, 0.0, 1.5], [-1.0, 0.0, 1.5]])
    center = (116, 157)
    initial = depth_from_world_cloud(cloud, scene.camera_pose_world())
    assert np.isclose(initial[center], 2.0)
    assert np.isnan(initial[0, 0])

    scene.camera_x = 1.0
    moved = depth_from_world_cloud(cloud, scene.camera_pose_world())
    assert np.isclose(moved[center], 1.0)
    left, right = render_stereo_from_pointcloud_depth(moved)
    assert left.shape == right.shape == (240, 320)
    assert np.any(right)


def test_near_surface_projects_left_texture_to_right_pixel():
    depth = np.full((240, 320), np.nan, dtype=np.float32)
    depth[116, 220] = 2.0
    left, right = render_stereo_from_pointcloud_depth(depth)
    right_col = round(220 - F_EFF * SYNTH_BASELINE_M / 2.0)
    assert right[116, right_col] == left[116, 220]


def test_empty_or_behind_cloud_is_unknown():
    scene = SyntheticScene()
    empty = depth_from_world_cloud(np.empty((0, 3)), scene.camera_pose_world())
    behind = depth_from_world_cloud(np.array([[-1.0, 0.0, 1.5]]), scene.camera_pose_world())
    assert np.isnan(empty).all()
    assert np.isnan(behind).all()

# ---------------------------------------------------------------- A3：几何同源与 NaN

from boom_birds_sensing.camera_geometry import (      # noqa: E402
    depth_geometry_mismatch,
    expected_depth_geometry,
)
from boom_birds_sim.synthetic import (            # noqa: E402
    SYNTH_DEPTH_SIZE,
    write_synth_calibration,
)


def test_scene_depth_output_size_matches_the_calibration_geometry(tmp_path):
    """EGO 内参与合成场景的深度输出必须同标定、同尺寸（三路同步的 Python 侧断言）。"""
    calib = tmp_path / "synth.npz"
    write_synth_calibration(str(calib))
    depth = SyntheticScene().depth_map()
    assert depth.shape == (SYNTH_DEPTH_SIZE[1], SYNTH_DEPTH_SIZE[0])
    expected = expected_depth_geometry(calib)
    assert (expected["width"], expected["height"]) == (depth.shape[1], depth.shape[0])
    assert depth_geometry_mismatch(expected, calib) == ""
    # 尺寸或内参变化必须被发现，不得静默沿用旧几何
    assert depth_geometry_mismatch(dict(expected, width=expected["width"] // 2), calib) != ""
    assert depth_geometry_mismatch(dict(expected, fx=expected["fx"] * 1.05), calib) != ""


def test_unobserved_pixels_stay_nan_and_never_become_a_surface():
    """未观测像素保持 NaN：不得被当成 0 米表面参与前向投影。"""
    scene = SyntheticScene()
    depth = depth_from_world_cloud(np.array([[2.0, 0.0, 1.5]]), scene.camera_pose_world())
    assert np.isnan(depth[0, 0]), "无点云覆盖的像素必须保持 NaN"
    assert np.isfinite(depth[116, 157])

    empty = np.full((240, 320), np.nan, dtype=np.float32)
    left_nan, right_nan = render_stereo_from_pointcloud_depth(empty)
    one = np.full((240, 320), np.nan, dtype=np.float32)
    one[116, 220] = 2.0
    left_one, right_one = render_stereo_from_pointcloud_depth(one)
    # 左目纹理只由尺寸决定；全 NaN 深度不产生任何额外表面
    assert np.array_equal(left_nan, left_one)
    assert not np.array_equal(right_nan, right_one), "有效深度必须产生前向投影"
