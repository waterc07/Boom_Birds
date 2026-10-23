"""防止模拟标定轴向/基线错误，以及评估对齐掩盖单目尺度错误。"""

import numpy as np
from scipy.spatial.transform import Rotation
from boom_birds_sim.gazebo_fixture import R_CAMERA_IMU, BASELINE
from boom_birds_sim.gazebo_evaluate import poseErrors


def test_optical_axes_and_positive_disparity():
    r = R_CAMERA_IMU
    np.testing.assert_allclose(r.T @ r, np.eye(3))
    assert np.linalg.det(r) == 1
    np.testing.assert_allclose(r.T @ np.array([1.0, 0.0, 0.0]), [0.0, 0.0, 1.0])
    np.testing.assert_allclose(r.T @ np.array([0.0, -1.0, 0.0]), [1.0, 0.0, 0.0])
    cameraSeparation = r.T @ np.array([0.0, -BASELINE, 0.0])
    np.testing.assert_allclose(cameraSeparation, [BASELINE, 0.0, 0.0])
    point = np.array([0.2, 0.1, 3.0])
    # 右相机坐标减去相机中心间距，左 x 像素应更大。
    assert 420 * point[0] / point[2] > 420 * (point[0] - BASELINE) / point[2]


def test_first_pose_alignment_does_not_fit_scale():
    t = np.linspace(0, 10, 101)
    p = np.column_stack([0.1 * t, np.sin(t) * 0.1, np.zeros_like(t)])
    q = np.tile([0.0, 0.0, 0.0, 1.0], (len(t), 1))
    truth = np.column_stack([t, p, q])
    rotation = Rotation.from_euler("z", 0.4)
    estimate = np.column_stack(
        [
            t,
            rotation.inv().apply(p - [3.0, 2.0, 1.0]),
            np.tile(rotation.inv().as_quat(), (len(t), 1)),
        ]
    )
    result, _ = poseErrors(truth, estimate)
    assert result["position_rmse_m"] < 1e-12
    assert result["orientation_p95_deg"] < 1e-10
    estimate[:, 1:4] *= 2
    result, _ = poseErrors(truth, estimate)
    assert result["position_rmse_m"] > 0.5
    assert not result["scale_fitted"]


def test_imu_consistency_rejects_velocity_only_motion():
    from boom_birds_sim.gazebo_fixture import checkImuTruth

    t = np.linspace(0, 20, 2001)
    positions = np.column_stack([0.2 * np.sin(t), 0.3 * np.cos(t), 1.5 + 0.1 * np.sin(2 * t)])
    truth = np.column_stack([t, positions, np.tile([0.0, 0.0, 0.0, 1.0], (len(t), 1))])
    u = np.linspace(0, 20, 4001)
    accel = np.column_stack([-0.2 * np.sin(u), -0.3 * np.cos(u), 9.81 - 0.4 * np.sin(2 * u)])
    imu = np.column_stack([u, accel, np.zeros((len(u), 3))])
    assert checkImuTruth(truth, imu)["pass_check"]
    # 只改运动速度、没有物理加速度时，模拟 IMU 只剩重力。
    imu[:, 1:4] = [0.0, 0.0, 9.81]
    assert not checkImuTruth(truth, imu)["pass_check"]


def test_replay_config_preserves_inputs_and_calibration(tmp_path):
    from boom_birds_sim.gazebo_replay import prepareReplayConfig

    source = tmp_path / "source"
    out = tmp_path / "replay"
    source.mkdir()
    out.mkdir()
    template = """relative_config_imu: "imu.yaml"
relative_config_imucam: "cam.yaml"
save_total_state: false
filepath_est: "est.txt"
filepath_std: "std.txt"
filepath_gt: "gt.txt"
zupt_chi2_multipler: 0
zupt_max_disparity: 0.5
use_stereo: false
"""
    (source / "estimator_config.yaml").write_text(template)
    for name in ("kalibr_imu_chain.yaml", "kalibr_imucam_chain.yaml"):
        (source / name).write_text("calibration-test")
    config = prepareReplayConfig(source, out, "noiseless", "low-parallax")
    assert (source / "estimator_config.yaml").read_text() == template
    assert (out / "kalibr_imu_chain.yaml").read_text() == "calibration-test"
    assert "use_stereo: false" in config.read_text()
    assert "zupt_max_disparity: 0.001" in config.read_text()
    assert "fi_max_baseline: 200" in config.read_text()


def test_diagnostics_distinguish_propagation_and_visual_updates(tmp_path):
    from boom_birds_sim.gazebo_evaluate import estimatorDiagnostics

    path = tmp_path / "openvins.log"
    path.write_text("[TIME]: seconds for MSCKF update (0 feats)\n[ZUPT]: accepted\n")
    assert estimatorDiagnostics(tmp_path)["msckf_nonzero_update_frames"] == 0
    with path.open("a") as stream:
        stream.write("[TIME]: seconds for MSCKF update (4 feats)\nbg = 0.01,0,0 | ba = 0,0.02,0\n")
    result = estimatorDiagnostics(tmp_path)
    assert result["msckf_nonzero_update_frames"] == 1
    assert result["msckf_updated_features_total"] == 4
    assert result["final_accel_bias_m_s2"] == [0.0, 0.02, 0.0]


def test_matrix_rejects_missing_combinations_and_imu_only_output(tmp_path, monkeypatch):
    import json, hashlib
    import boom_birds_sim.gazebo_matrix as matrix

    source = tmp_path / "input"
    out = tmp_path / "output"
    source.mkdir()
    out.mkdir()
    manifest = {key: 0 for key in matrix.GEOMETRY}
    manifest.update(scene="range_3m", motion_profile="nominal", sensor_noise="disabled baseline")
    (source / "manifest.json").write_text(json.dumps(manifest))
    a, f = matrix.MOTION_PROFILES["nominal"]
    (source / "sensor_world.sdf").write_text(
        f'<sdf><world><model name="front_wall"><pose>3.2 0 0</pose></model><model name="bb_rig"><plugin name="boom_birds::SensorMotion"><amplitudes>{" ".join(map(str,a))}</amplitudes><frequencies>{" ".join(map(str,f))}</frequencies></plugin></model></world></sdf>'
    )
    for name in ("kalibr_imu_chain.yaml", "kalibr_imucam_chain.yaml"):
        (source / name).write_text("same")
        (out / name).write_text("same")
    (out / "estimator_config.yaml").write_text("same")
    replay = dict(
        zupt_mode="noiseless",
        feature_profile="low-parallax",
        depth_profile="budget",
        sgbm_block_size=5,
        truth_published=False,
        estimator_config_sha256=hashlib.sha256(b"same").hexdigest(),
    )
    (out / "replay.json").write_text(json.dumps(replay))
    (out / "processes.json").write_text(json.dumps(dict(openvins=0, depth=0, playback=0)))
    monkeypatch.setattr(
        matrix,
        "evaluate",
        lambda *_: {"pass": True, "gates": {}, "vio": {"visual_updates_observed": False}},
    )
    result = matrix.summarize([(source, out)])
    assert not result["pass_matrix"]
    assert not result["runs"][0]["pass_case"]
    assert any("missing combinations" in e for e in result["errors"])


def test_fixture_manifest_uses_requested_duration(tmp_path):
    import pytest
    from boom_birds_sim.gazebo_fixture import prepare

    estimator = tmp_path / "estimator.yaml"
    imu = tmp_path / "imu.yaml"
    plugin = tmp_path / "motion.so"
    estimator.write_text("%YAML:1.0\nuse_stereo: true\nmax_cameras: 2\n")
    imu.write_text("%YAML:1.0\nimu0:\n  rostopic: /imu0\n")
    plugin.write_bytes(b"fixture-only-plugin-placeholder")
    result = prepare(tmp_path / "capture", estimator, imu, plugin, duration_sim_s=12.0)
    assert result["duration_sim_s"] == 12.0
    assert result["ground_truth_role"] == "evaluation only"
    for duration in (0.0, -1.0, float("nan"), float("inf")):
        with pytest.raises(ValueError, match="duration_sim_s"):
            prepare(tmp_path / "invalid", estimator, imu, plugin, duration_sim_s=duration)
    assert not (tmp_path / "invalid").exists()
