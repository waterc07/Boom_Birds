"""显式对比度处理保留 Tag 身份、几何与像素尺寸门限。"""
from copy import deepcopy
from pathlib import Path
import cv2
import numpy as np
import pytest
import yaml
from boom_birds_sensing.platform_observation import BoardDetector

PROFILE = Path(__file__).resolve().parents[2] / "boom_birds_sim/config/platform_landing_test.yaml"


def profile():
    return yaml.safe_load(PROFILE.read_text())


def markerImage(config, pixels=64, marker_id=7):
    width, height = config["camera"]["image_size"]
    image = np.full((height, width), 255, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.drawMarker(dictionary, marker_id, pixels)
    x, y = (width-pixels)//2, (height-pixels)//2
    image[y:y+pixels, x:x+pixels] = marker
    return image


def test_dim_marker_preserves_identity_and_pose_after_explicit_contrast():
    config = profile()
    config["detector"] = {"corner_refinement_window_px": 2}
    reference = BoardDetector(config, test_only=True).observe(markerImage(config), 1.)
    config["detector"]["image_preprocessing"] = "percentile"
    dark = markerImage(config)//8 + 20
    result = BoardDetector(config, test_only=True).observe(dark, 1.)
    assert reference.valid and result.valid, (reference.reason, result.reason)
    assert result.tag_ids == reference.tag_ids == (7,)
    np.testing.assert_allclose(result.pose(), reference.pose(), atol=.02)


def test_contrast_keeps_pixel_size_gate_and_rejects_blank_input():
    config = profile()
    config["detector"] = {"corner_refinement_window_px": 2, "image_preprocessing": "percentile"}
    config["quality"]["min_edge_px"] = 64.
    detector = BoardDetector(config, test_only=True)
    result = detector.observe(markerImage(config, pixels=32)//8+20, 1.)
    assert not result.valid and result.reason == "tag_too_small"
    assert not detector.observe(np.full_like(markerImage(config), 25), 1.).valid


@pytest.mark.parametrize("options", [
    None, {"unknown": 1}, {"corner_refinement_window_px": 0},
    {"corner_refinement_window_px": 2.5}, {"corner_refinement_window_px": 8},
    {"image_preprocessing": "unsupported"},
])
def test_invalid_detector_options_are_rejected(options):
    config = profile()
    config["detector"] = deepcopy(options)
    with pytest.raises(ValueError):
        BoardDetector(config, test_only=True)
