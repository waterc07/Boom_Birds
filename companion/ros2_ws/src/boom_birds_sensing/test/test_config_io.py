import numpy as np
import pytest
import yaml

from boom_birds_sensing.config_io import ConfigError, loadExtrinsics


@pytest.mark.parametrize("header", ["", "%YAML:1.0\n---\n", "%YAML 1.1\n---\n"])
def test_extrinsics_accept_opencv_and_standard_yaml_headers(tmp_path, header):
    camera = np.eye(4)
    camera[:3, 3] = [.07, -.03, -.04]
    body = np.eye(4)
    body[:3, 3] = [.01, .02, .03]
    path = tmp_path / "extrinsics.yaml"
    path.write_text(header + yaml.safe_dump({
        "T_I_C0": camera.tolist(), "T_I_B": body.tolist(), "source": "TEST-ONLY fixture"}))
    result = loadExtrinsics(str(path))
    np.testing.assert_allclose(result["T_I_C0"], camera)
    np.testing.assert_allclose(result["T_I_B"], body)
    np.testing.assert_allclose(result["p_I_B"], body[:3, 3])


def test_opencv_header_does_not_hide_missing_body_measurement(tmp_path):
    path = tmp_path / "extrinsics.pending.yaml"
    path.write_text("%YAML:1.0\n---\n" + yaml.safe_dump({
        "T_I_C0": np.eye(4).tolist(), "T_I_B": None, "source": "camera calibration only"}))
    with pytest.raises(ConfigError, match="T_I_B 必须是 4"):
        loadExtrinsics(str(path))


def test_opencv_header_does_not_hide_malformed_yaml(tmp_path):
    path = tmp_path / "bad.yaml"
    path.write_text("%YAML:1.0\n---\nT_I_C0: [1\n")
    with pytest.raises(yaml.YAMLError):
        loadExtrinsics(str(path))
