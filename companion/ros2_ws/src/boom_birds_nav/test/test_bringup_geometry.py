"""接管与内参回归：非零航向、非整数缩放、无效消息必须拒绝。

另外锁定 A3 的几何契约：

* 三路内参（stereo_source 原始内参 / depth_node 深度内参 / EGO grid_map）
  必须来自**同一标定**与**同一次实际输出尺寸**，不一致必须拒绝并诊断；
* 跨分辨率只缩放 ``fx/fy/cx/cy``，米制物理基线不缩放，
  右目恒有 ``P[0][3] = -fx_当前分辨率 · B_物理``；
* 没有内参（或没有深度）时必须处于"未就绪"，不得回退到默认值或上一次的内参。
"""
from types import SimpleNamespace as NS
import math
import numpy as np
import pytest

from boom_birds_sensing.camera_geometry import (
    DEPTH_FRAME,
    GEOMETRY_TOLERANCE_PX,
    GeometryError,
    calibration_identity,
    camera_message_geometry,
    camera_message_intrinsics,
    depth_geometry_mismatch,
    depth_geometry_ready,
    ego_intrinsics,
    expected_depth_geometry,
    scaled_camera_info,
    stereo_baseline_term,
    validate_projection_matrix,
)
from boom_birds_control.handoff import handoff_allowed
from boom_birds_control.px4_frames import LocalFrameAlignment
from boom_birds_sim.synthetic import SYNTH_BASELINE_M, SYNTH_DEPTH_SIZE, write_synth_calibration

BASELINE_M = 0.067672
LEFT_P = [170., 0., 157., 0., 0., 170., 117., 0., 0., 0., 1., 0.]


def test_scaled_geometry_preserves_rays_and_baseline():
    info = dict(width=320, height=240, fx=170., fy=170., cx=157., cy=117., baseline_m=.067)
    out = scaled_camera_info(info, .731)
    assert (out["width"], out["height"]) == (234, 175)
    for u, v in ((0, 0), (100, 80), (319, 239)):
        assert (u * 234 / 320 - out["cx"]) / out["fx"] == pytest.approx((u-info["cx"])/info["fx"])
        assert (v * 175 / 240 - out["cy"]) / out["fy"] == pytest.approx((v-info["cy"])/info["fy"])
    assert out["baseline_m"] == info["baseline_m"]
    assert info["width"] == 320


@pytest.mark.parametrize("scale", [0, -.5, 1.1, math.nan, math.inf])
def test_bad_scale(scale):
    with pytest.raises(ValueError):
        scaled_camera_info({}, scale)


def test_nonzero_yaw_and_translation_handoff():
    alignment = LocalFrameAlignment(yaw_offset_rad=math.pi/2, translation_m=(-15., 2., .1))
    # R.T @ (1, 2, -3) + t = (-17, 1, 3.1)
    position = alignment.position_ned_to_ros((1., 2., -3.))
    np.testing.assert_allclose(position, (-17., 1., 3.1))
    assert handoff_allowed((-17., 1., 3.1), position)
    assert not handoff_allowed((-14., 0., 3.1), position)
    assert handoff_allowed((.5, 0, 0), (0, 0, 0))
    assert not handoff_allowed((.5001, 0, 0), (0, 0, 0))
    assert not handoff_allowed((math.nan, 0, 0), (0, 0, 0))
    assert not handoff_allowed((0, 0, 0), (0, 0, 0), math.nan)


def message():
    return NS(header=NS(frame_id="cam0_rect", stamp=NS(sec=10, nanosec=0)),
              width=240, height=180, p=[129., 0, 117., 0, 0, 129., 87., 0, 0, 0, 1, 0])


def test_live_geometry_validation():
    msg = message()
    assert float(camera_message_intrinsics(msg, 10.5)["fx"]) == 129.
    for now in (9., 13., math.nan):
        with pytest.raises(ValueError):
            camera_message_intrinsics(msg, now)
    msg.header.frame_id = "global"
    with pytest.raises(ValueError):
        camera_message_intrinsics(msg, 10.5)
    msg = message(); msg.p[0] = math.nan
    with pytest.raises(ValueError):
        camera_message_intrinsics(msg, 10.5)


# --------------------------------------------------------------- 三路同步 / 物理基线


@pytest.mark.parametrize("scale", [1.0, 0.75, 0.5])
def test_right_projection_term_is_fx_times_the_physical_baseline(scale):
    """``P[0][3] = -fx_当前分辨率 · B_物理``：基线是米制量，只随 fx 变一次。"""
    fx = 170.0 * scale
    matrix = list(LEFT_P)
    matrix[0] = fx                       # fx 随分辨率缩放
    matrix[3] = stereo_baseline_term(fx, BASELINE_M)
    assert validate_projection_matrix(matrix, 320, 240, side="right",
                                      baseline_m=BASELINE_M) == pytest.approx(-fx * BASELINE_M)
    # 把基线也缩放的旧 bug（scale²）必须被识别为不一致
    with pytest.raises(GeometryError):
        validate_projection_matrix(matrix, 320, 240, side="right",
                                   baseline_m=BASELINE_M * 0.5)


def test_left_projection_matrix_must_not_carry_a_baseline_term():
    validate_projection_matrix(LEFT_P, 320, 240, side="left")
    right_like = list(LEFT_P)
    right_like[3] = -11.5
    with pytest.raises(GeometryError):
        validate_projection_matrix(right_like, 320, 240, side="left")


def test_right_matrix_requires_an_explicit_physical_baseline():
    matrix = list(LEFT_P)
    matrix[3] = stereo_baseline_term(170.0, BASELINE_M)
    with pytest.raises(GeometryError):
        validate_projection_matrix(matrix, 320, 240, side="right")


@pytest.mark.parametrize("mutate", [
    lambda p: p.__setitem__(5, math.nan),          # 非有限值
    lambda p: p.__setitem__(1, 2.0),               # 倾斜项
    lambda p: p.__setitem__(11, 1.0),              # 齐次项
    lambda p: p.__setitem__(10, 0.9),              # P[10] != 1
    lambda p: p.__setitem__(2, 400.0),             # 主点在图像外
    lambda p: p.__setitem__(6, -1.0),              # 主点为负
    lambda p: p.__setitem__(0, 0.0),               # fx 非正
    lambda p: p.__setitem__(0, -3.0),              # fx 为负
])
def test_projection_matrix_field_guards(mutate):
    matrix = list(LEFT_P)
    mutate(matrix)
    with pytest.raises(GeometryError):
        validate_projection_matrix(matrix, 320, 240, side="left")


def test_projection_matrix_requires_twelve_entries():
    with pytest.raises(GeometryError):
        validate_projection_matrix(LEFT_P[:11], 320, 240, side="left")


def test_output_size_is_the_authoritative_actual_size():
    """给定了本次实际输出尺寸就以它为准；与 scale 推出的不一致必须拒绝。"""
    info = dict(width=320, height=240, fx=170., fy=170., cx=157., cy=117.)
    assert scaled_camera_info(info, .731, output_size=(234, 175)) == scaled_camera_info(info, .731)
    for wrong in ((240, 175), (234, 180), (100, 100)):
        with pytest.raises(GeometryError):
            scaled_camera_info(info, .731, output_size=wrong)


def test_scaled_camera_info_recomputes_the_right_baseline_term_once():
    """带 P 的右目几何在缩放后必须重算基线项，且基线本身不变。"""
    info = dict(width=320, height=240, fx=170., fy=170., cx=157., cy=117.,
                baseline_m=BASELINE_M)
    info["p"] = list(LEFT_P)
    info["p"][3] = stereo_baseline_term(info["fx"], BASELINE_M)
    out = scaled_camera_info(info, 0.5, side="right")
    assert out["baseline_m"] == pytest.approx(BASELINE_M, rel=1e-12)
    assert out["fx"] == pytest.approx(85.0)
    assert out["p"][3] == pytest.approx(-out["fx"] * BASELINE_M, rel=1e-12)
    # 恰好变一次（scale^1），不是 scale²
    assert out["p"][3] / info["p"][3] == pytest.approx(0.5, rel=1e-12)
    # 左目在同一份几何下必须没有基线项
    assert scaled_camera_info(info, 0.5, side="left")["p"][3] == 0.0


def test_scaled_camera_info_rejects_geometry_that_leaves_the_image():
    info = dict(width=320, height=240, fx=170., fy=170., cx=157., cy=117.)
    bad = dict(info, cx=321.0)          # 缩放后主点会跑到图像外
    with pytest.raises(GeometryError):
        scaled_camera_info(bad, 0.5)
    with pytest.raises(GeometryError):
        scaled_camera_info(dict(info, cy=241.0), 0.5)
    with pytest.raises(GeometryError):
        scaled_camera_info(dict(info, fx=math.nan), 1.0)


def test_depth_message_must_match_the_actual_output_size_and_be_the_left_camera():
    msg = message()
    assert camera_message_geometry(msg, 10.5)["width"] == 240
    assert camera_message_geometry(msg, 10.5, expected_size=(240, 180))["height"] == 180
    with pytest.raises(GeometryError):
        camera_message_geometry(msg, 10.5, expected_size=(320, 240))
    # 深度图必须是校正后左目：右目的 P[0][3] 会让反投影整体偏一个基线
    msg.p[3] = stereo_baseline_term(129.0, BASELINE_M)
    with pytest.raises(GeometryError):
        camera_message_geometry(msg, 10.5)
    # 非 cam0_rect 的光学帧不得当作深度几何
    msg = message(); msg.header.frame_id = "cam1_rect"
    with pytest.raises(GeometryError):
        camera_message_geometry(msg, 10.5, frame_id=DEPTH_FRAME)


def test_depth_geometry_ready_semantics():
    """需求 3：没有内参、字段不全或非法都必须是"未就绪"。"""
    assert not depth_geometry_ready(None)
    assert not depth_geometry_ready({})
    ready = dict(width=240, height=180, fx=129., fy=129., cx=117., cy=87., frame_id=DEPTH_FRAME)
    assert depth_geometry_ready(ready)
    assert not depth_geometry_ready({k: ready[k] for k in ("width", "height", "fx", "fy")})
    assert not depth_geometry_ready(dict(ready, fx=0.0))
    assert not depth_geometry_ready(dict(ready, cx=999.0))
    assert not depth_geometry_ready(dict(ready, width=0))
    assert not depth_geometry_ready(dict(ready, cx=math.nan))
    # ego_intrinsics 只接受合法内参，绝不"猜"默认值
    assert ego_intrinsics(ready) == {"fx": "129.0", "fy": "129.0", "cx": "117.0", "cy": "87.0"}
    with pytest.raises(GeometryError):
        ego_intrinsics(dict(ready, fy=-1.0))


# --------------------------------------------------------------- 标定身份与同源检查


def test_calibration_identity_is_stable_and_content_bound(tmp_path):
    good = tmp_path / "synth.npz"
    write_synth_calibration(str(good))
    first = calibration_identity(good)
    assert first == calibration_identity(good)
    assert first["image_size"] == (1280, 960)
    assert first["baseline_m"] == pytest.approx(SYNTH_BASELINE_M, rel=1e-9)
    assert len(first["sha256"]) == 64

    other = tmp_path / "synth2.npz"
    write_synth_calibration(str(other))
    with np.load(other) as data:
        payload = {key: data[key] for key in data.files}
    payload["K1"] = np.asarray(payload["K1"], dtype=float) * 1.01
    np.savez(other, **payload)
    changed = calibration_identity(other)
    assert changed["sha256"] != first["sha256"]
    assert changed["fx_left"] != pytest.approx(first["fx_left"])


def test_expected_depth_geometry_matches_the_calibration_output_size(tmp_path):
    calib = tmp_path / "synth.npz"
    write_synth_calibration(str(calib))
    expected = expected_depth_geometry(calib)
    assert (expected["width"], expected["height"]) == SYNTH_DEPTH_SIZE
    assert expected["frame_id"] == DEPTH_FRAME
    assert expected["baseline_m"] == pytest.approx(SYNTH_BASELINE_M, rel=1e-9)
    # 深度内参必须通过严格的左目投影矩阵校验
    validate_projection_matrix(
        [expected["fx"], 0.0, expected["cx"], 0.0,
         0.0, expected["fy"], expected["cy"], 0.0,
         0.0, 0.0, 1.0, 0.0],
        expected["width"], expected["height"], side="left")
    assert depth_geometry_ready(expected)
    # 与自身比较必须同源
    assert depth_geometry_mismatch(expected, calib) == ""
    # 任一不一致都必须被发现，而不是静默沿用
    assert depth_geometry_mismatch(dict(expected, fx=expected["fx"] + 1.0), calib) != ""
    assert depth_geometry_mismatch(dict(expected, width=expected["width"] // 2), calib) != ""
    assert depth_geometry_mismatch(dict(expected, frame_id="cam1_rect"), calib) != ""
    # 容差本身必须是像素级的小量，不能靠放宽阈值"通过"
    assert 0.0 < GEOMETRY_TOLERANCE_PX <= 1e-2
    assert depth_geometry_mismatch(
        dict(expected, fx=expected["fx"] + GEOMETRY_TOLERANCE_PX / 2.0), calib) == ""


def test_scaled_expected_geometry_follows_the_output_scale(tmp_path):
    calib = tmp_path / "synth.npz"
    write_synth_calibration(str(calib))
    full = expected_depth_geometry(calib, 1.0)
    half = expected_depth_geometry(calib, 0.5)
    assert (half["width"], half["height"]) == (full["width"] // 2, full["height"] // 2)
    assert half["fx"] == pytest.approx(full["fx"] / 2.0, rel=1e-9)
    assert half["baseline_m"] == pytest.approx(full["baseline_m"], rel=1e-12), "基线不随分辨率缩放"
    assert depth_geometry_mismatch(half, calib, 1.0) != "", "缩放后的几何不能当作全分辨率几何"


def test_installed_launch_requires_live_geometry():
    import importlib.util
    from pathlib import Path
    from launch import LaunchContext
    from ament_index_python.packages import get_package_share_directory
    launch_path = Path(get_package_share_directory("boom_birds_sim")) / "launch/px4_sitl_motion.launch.py"
    spec = importlib.util.spec_from_file_location("motion_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(goal_x="1", goal_y="2", goal_z="3")
    include, = module.planner_from_calibration(context, "/unused/ego.launch.py")
    actual = dict(include.launch_arguments)
    assert actual == dict(use_camera_info="true", goal_x="1", goal_y="2", goal_z="3")


def test_camera_geometry_cli_receives_live_info():
    import subprocess
    import sys
    import time
    import rclpy
    from sensor_msgs.msg import CameraInfo
    from conftest import child_env
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node("geometry_test_publisher", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    publisher = node.create_publisher(CameraInfo, "/boom_birds/depth/camera_info", 10)
    process = subprocess.Popen([sys.executable, "-m", "boom_birds_sensing.camera_geometry", "--timeout", "5"],
                               env=child_env(), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        deadline = time.monotonic() + 8
        while process.poll() is None and time.monotonic() < deadline:
            msg = CameraInfo()
            msg.header.frame_id = "cam0_rect"
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.width, msg.height = 240, 180
            msg.p = [129., 0., 117., 0., 0., 129., 87., 0., 0., 0., 1., 0.]
            publisher.publish(msg)
            executor.spin_once(timeout_sec=.05)
        out, err = process.communicate(timeout=2)
        assert process.returncode == 0, err
        assert out.splitlines() == ["fx:=129.0", "fy:=129.0", "cx:=117.0", "cy:=87.0"]
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


def test_camera_geometry_cli_rejects_missing_info():
    import subprocess
    import sys
    from conftest import child_env
    result = subprocess.run([sys.executable, "-m", "boom_birds_sensing.camera_geometry", "--timeout", ".2"],
                            env=child_env(ROS_DOMAIN_ID=193), capture_output=True, text=True, timeout=10)
    assert result.returncode != 0
    assert "未收到" in result.stderr


def test_camera_geometry_cli_rejects_geometry_from_another_calibration(tmp_path):
    """三路同源：深度 CameraInfo 与给定标定不同源时 EGO 不得启动。"""
    import subprocess
    import sys
    import time
    import rclpy
    from sensor_msgs.msg import CameraInfo
    from conftest import child_env
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    calib = tmp_path / "synth.npz"
    write_synth_calibration(str(calib))

    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node("geometry_test_publisher_foreign", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    publisher = node.create_publisher(CameraInfo, "/boom_birds/depth/camera_info", 10)
    # 标定推导出的深度输出是 320x240；这里发一路 240x180，属于另一套输出几何。
    process = subprocess.Popen(
        [sys.executable, "-m", "boom_birds_sensing.camera_geometry",
         "--timeout", "4", "--calibration", str(calib)],
        env=child_env(), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        deadline = time.monotonic() + 6
        while process.poll() is None and time.monotonic() < deadline:
            msg = CameraInfo()
            msg.header.frame_id = "cam0_rect"
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.width, msg.height = 240, 180
            msg.p = [129., 0., 117., 0., 0., 129., 87., 0., 0., 0., 1., 0.]
            publisher.publish(msg)
            executor.spin_once(timeout_sec=.05)
        out, err = process.communicate(timeout=2)
        assert process.returncode != 0, out
        assert "未收到" in err and "不同源" in err, err
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
