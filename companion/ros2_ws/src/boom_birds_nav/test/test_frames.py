"""位姿/变换纯函数测试：用已知刚体变换做数值断言（不依赖 ROS）。"""

import math

import numpy as np

from boom_birds_nav.frames import (
    body_velocity_from_imu,
    compose,
    invert_transform,
    is_rotation,
    make_transform,
    quat_to_rot,
    rot_to_quat,
    rotate_covariance,
    transform_covariance_6x6,
)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def test_quat_rot_roundtrip():
    R = rot_z(-1.1) @ np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)
    assert np.allclose(quat_to_rot(rot_to_quat(R)), R, atol=1e-12)


def test_compose_and_inverse_match_analytic():
    T_a_b = make_transform(rot_z(math.pi / 2), [1.0, 2.0, 3.0])
    p_a = T_a_b[:3, :3] @ np.array([1.0, 0.0, 0.0]) + T_a_b[:3, 3]
    assert np.allclose(p_a, [1.0, 3.0, 3.0], atol=1e-12)
    T_b_a = invert_transform(T_a_b)
    assert np.allclose(T_b_a @ T_a_b, np.eye(4), atol=1e-12)
    assert is_rotation(T_b_a[:3, :3])


def test_compose_order_is_left_to_right():
    T_w_i = make_transform(rot_z(0.0), [1.0, 0.0, 0.0])
    T_i_c = make_transform(rot_z(math.pi / 2), [0.0, 1.0, 0.0])
    T_w_c = compose(T_w_i, T_i_c)
    p_w = T_w_c[:3, :3] @ np.array([1.0, 0.0, 0.0]) + T_w_c[:3, 3]
    assert np.allclose(p_w, [1.0, 2.0, 0.0], atol=1e-12)   # Rz(90°)·(1,0,0)=(0,1,0)，再加平移 (0,1,0)


def test_rotate_covariance_known_case():
    out = rotate_covariance(rot_z(math.pi / 2), np.diag([1.0, 4.0, 9.0]))
    assert np.allclose(out, np.diag([4.0, 1.0, 9.0]), atol=1e-12)


def test_transform_covariance_6x6_block_structure():
    T = make_transform(rot_z(math.pi / 2), np.zeros(3))
    cov6 = np.zeros((6, 6))
    cov6[0, 0] = 1.0
    cov6[3, 3] = 2.0
    out = transform_covariance_6x6(T, cov6)
    assert abs(out[1, 1] - 1.0) < 1e-12 and abs(out[4, 4] - 2.0) < 1e-12
    assert abs(out[0, 0]) < 1e-12 and np.allclose(out, out.T)


def test_body_velocity_lever_arm():
    v = body_velocity_from_imu(np.zeros(3), np.array([0.0, 0.0, 1.0]), np.array([0.5, 0.0, 0.0]), np.eye(3))
    assert np.allclose(v, [0.0, 0.5, 0.0], atol=1e-12)
    v0 = body_velocity_from_imu(np.array([1.0, 2.0, 3.0]), np.array([0.0, 0.0, 1.0]), np.zeros(3), np.eye(3))
    assert np.allclose(v0, [1.0, 2.0, 3.0], atol=1e-12)


def test_nonzero_lever_arm_changes_velocity():
    a = body_velocity_from_imu(np.zeros(3), np.array([0.0, 0.0, 1.0]), np.zeros(3), np.eye(3))
    b = body_velocity_from_imu(np.zeros(3), np.array([0.0, 0.0, 1.0]), np.array([0.3, 0.0, 0.0]), np.eye(3))
    assert np.linalg.norm(b - a) > 0.29
