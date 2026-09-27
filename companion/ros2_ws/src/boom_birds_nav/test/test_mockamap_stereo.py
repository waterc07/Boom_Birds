"""EGO 世界点云投影到合成双目的几何回归。"""

import numpy as np

from boom_birds_nav.pointcloud_scene import depth_from_world_cloud
from boom_birds_nav.synthetic import (F_EFF, SYNTH_BASELINE_M, SyntheticScene,
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
