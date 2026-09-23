"""外参配置与失效行为测试：缺配置必须显式失败；相机链与手工矩阵乘法一致。"""

import pathlib
import tempfile

import numpy as np
import pytest
import yaml

from boom_birds_nav.config_io import ConfigError, load_extrinsics
from boom_birds_nav.frames import invert_transform, make_transform, quat_to_rot, rot_to_quat


def _write(tmp_path, data, name="extr.yaml"):
    p = tmp_path / name
    p.write_text(yaml.safe_dump(data, allow_unicode=True), encoding="utf-8")
    return str(p)


def test_missing_file_raises(tmp_path):
    with pytest.raises(ConfigError):
        load_extrinsics(str(tmp_path / "nope.yaml"))


def test_missing_T_I_C0_raises(tmp_path):
    with pytest.raises(ConfigError):
        load_extrinsics(_write(tmp_path, {"source": "test"}))


def test_missing_T_I_B_requires_explicit_assumption(tmp_path):
    data = {"source": "test", "T_I_C0": np.eye(4).tolist()}
    with pytest.raises(ConfigError):
        load_extrinsics(_write(tmp_path, data))
    data["assume_origin_coincident"] = True
    cfg = load_extrinsics(_write(tmp_path, data, "ok.yaml"))
    assert np.allclose(cfg["T_I_B"], np.eye(4))
    assert np.allclose(cfg["p_I_B"], np.zeros(3))


def test_non_rotation_matrix_rejected(tmp_path):
    bad = np.eye(4)
    bad[0, 0] = 2.0
    with pytest.raises(ConfigError):
        load_extrinsics(_write(tmp_path, {"source": "t", "T_I_C0": bad.tolist(), "assume_origin_coincident": True}))


def test_lever_arm_must_match_T_I_B(tmp_path):
    T = np.eye(4)
    T[:3, 3] = [0.1, 0.0, 0.0]
    with pytest.raises(ConfigError):
        load_extrinsics(_write(tmp_path, {
            "source": "t", "T_I_C0": np.eye(4).tolist(), "T_I_B": T.tolist(), "p_I_B_m": [0.2, 0.0, 0.0],
        }))
    cfg = load_extrinsics(_write(tmp_path, {
        "source": "t", "T_I_C0": np.eye(4).tolist(), "T_I_B": T.tolist(),
    }, "ok2.yaml"))
    assert np.allclose(cfg["p_I_B"], [0.1, 0.0, 0.0])


def test_package_configs_load():
    root = pathlib.Path(__file__).resolve().parents[1] / "config"
    for name in ("extrinsics_synthetic_test.yaml", "extrinsics_chain_test.yaml"):
        cfg = load_extrinsics(str(root / name))
        assert "TEST-ONLY" in cfg["source"]
        assert np.allclose(cfg["T_I_C0"][3], [0, 0, 0, 1])


def test_camera_chain_matches_manual_product(tmp_path):
    from boom_birds_nav.depth_core import camera_rect_transform, make_processor
    from boom_birds_nav.synthetic import write_synth_calibration

    calib = tmp_path / "c.npz"
    write_synth_calibration(str(calib))
    proc = make_processor(str(calib))
    T_w_i = make_transform(quat_to_rot(rot_to_quat(np.eye(3))), [0.1, 0.2, 0.3])
    T_i_c0 = make_transform(
        np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]), [0.05, 0.01, -0.02]
    )
    T_manual = T_w_i @ T_i_c0 @ make_transform(proc.r1, np.zeros(3))
    assert np.allclose(camera_rect_transform(proc, T_w_i, T_i_c0), T_manual, atol=1e-12)
    assert np.allclose(invert_transform(T_manual) @ T_manual, np.eye(4), atol=1e-12)


def test_r1_identity_for_synthetic_calibration():
    from boom_birds_nav.depth_core import make_processor
    from boom_birds_nav.synthetic import write_synth_calibration

    with tempfile.TemporaryDirectory() as d:
        calib = pathlib.Path(d) / "c.npz"
        write_synth_calibration(str(calib))
        proc = make_processor(str(calib))
        assert np.allclose(proc.r1, np.eye(3), atol=1e-9)
