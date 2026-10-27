"""Multi-scale boards retain only markers that satisfy the pixel-size gate."""
from copy import deepcopy
from pathlib import Path
import cv2
import numpy as np
import yaml
from boom_birds_sensing.platform_observation import BoardDetector

ROOT=Path(__file__).resolve().parents[1]
PROFILE=ROOT.parent/"boom_birds_sim/config/platform_landing_test.yaml"


def fixture():
    p=yaml.safe_load(PROFILE.read_text())
    small=deepcopy(p["board"]["tags"][0])
    small.update(id=8,size_m=.005)
    small["T_platform_tag"][0][3]=.25
    p["board"]["tags"].append(small)
    d=BoardDetector(p,test_only=True)
    rvec=np.array([3.,.15,.1]);tvec=np.array([.03,.01,1.])
    detections=[(tag["id"],cv2.projectPoints(d.tagCorners(tag),rvec,tvec,d.K,d.D)[0].reshape(4,2))
                for tag in p["board"]["tags"]]
    return d,detections


def test_small_known_marker_does_not_discard_large_valid_marker():
    d,detections=fixture()
    large=d.estimate(detections[:1],1.,"TEST_ONLY")
    combined=d.estimate(detections,1.,"TEST_ONLY")
    assert large.valid and combined.valid,(large.reason,combined.reason)
    assert combined.tag_ids==(7,)
    assert combined.quality["ignored_small_ids"]==[8]
    assert np.allclose(combined.T_body_platform,large.T_body_platform)


def test_minimum_marker_count_is_applied_after_size_filtering():
    d,detections=fixture()
    d.profile["quality"]["min_tags"]=2
    result=d.estimate(detections,1.,"TEST_ONLY")
    assert not result.valid and result.reason=="tag_too_small"


def test_only_small_marker_still_fails_closed():
    d,detections=fixture()
    result=d.estimate(detections[1:],1.,"TEST_ONLY")
    assert not result.valid and result.reason=="tag_too_small"
