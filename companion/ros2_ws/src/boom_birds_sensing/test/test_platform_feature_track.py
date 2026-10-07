from pathlib import Path
import cv2
import numpy as np
import pytest
import yaml
from boom_birds_sensing.platform_feature_track import BoardFeatureTrack, TrackConfig
from boom_birds_sensing.platform_observation import BoardDetector
from boom_birds_sensing.platform_synthetic import render_board


def fixture():
    p = Path(__file__).parents[2]/"boom_birds_control/config/platform_landing_test.yaml"
    detector = BoardDetector(yaml.safe_load(p.read_text()), test_only=True)
    # Plane directly below body FLU, with a small tilt to avoid planar ambiguity.
    pose = np.eye(4)
    pose[:3, :3] = cv2.Rodrigues(np.array([.08, .05, .1]))[0]
    pose[2, 3] = -1.
    image = render_board(detector, pose)
    observation = detector.observe(image, 10.)
    assert observation.valid, observation.reason
    return detector, image, observation


def test_center_continues_after_full_marker_border_removed():
    detector, image, observation = fixture()
    tracker = BoardFeatureTrack(detector, TrackConfig(roi_radius_px=50))
    seeded = tracker.seed(image, observation, 1.)
    assert seeded["valid"]
    # Keep center texture but remove the marker border: decoder cannot recover ID.
    partial = np.full_like(image, 255)
    center = np.rint(seeded["landing_pixel"]).astype(int)
    x, y = center
    partial[y-60:y+61, x-60:x+61] = image[y-60:y+61, x-60:x+61]
    moved = cv2.warpAffine(partial, np.float32([[1, 0, 3], [0, 1, -2]]),
                           (image.shape[1], image.shape[0]), borderValue=255)
    assert not detector.observe(moved, 10.02).valid
    result = tracker.update(moved, 10.02)
    assert result["valid"], result
    assert result["inherited_tag_ids"] == list(observation.tag_ids)
    assert np.linalg.norm(np.array(result["landing_pixel"])-np.array(seeded["landing_pixel"])-[3, -2]) < 1.
    assert result["metric_pose_available"] is False


@pytest.mark.parametrize("failure", ["gap", "reorder", "blank", "identity_expired", "outside_height", "wrong_id"])
def test_loss_requires_new_decoding(failure):
    detector, image, observation = fixture()
    tracker = BoardFeatureTrack(detector, TrackConfig(max_identity_age_s=.03))
    assert tracker.seed(image, observation, 1.)["valid"]
    if failure == "outside_height":
        result = tracker.seed(image, observation, .2)
    elif failure == "wrong_id":
        from dataclasses import replace
        result = tracker.seed(image, replace(observation, valid=False, reason="wrong_id"), 1.)
    else:
        stamp = {"gap": 11., "reorder": 10., "blank": 10.02, "identity_expired": 10.04}[failure]
        result = tracker.update(np.full_like(image, 255) if failure == "blank" else image, stamp)
    assert not result["valid"]
    assert tracker.update(image, 11.02)["reason"] == "decode_required"
