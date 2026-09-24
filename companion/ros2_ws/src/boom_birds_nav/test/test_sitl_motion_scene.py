"""TEST-ONLY 运动场景的几何回归。"""

import numpy as np

from boom_birds_nav.sitl_truth_source import ned_to_ros_position, px4_attitude_to_ros_rotation
from boom_birds_nav.synthetic import default_scene


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
