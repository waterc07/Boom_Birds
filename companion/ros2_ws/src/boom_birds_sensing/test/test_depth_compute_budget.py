from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from boom_birds_sensing.depth_core import makeProcessor, processPair, processStitched
from boom_birds_sensing.depth_node import DepthNode
from boom_birds_sim.synthetic import writeSynthCalibration
from stereo_depth.core import selectNumDisparities


@pytest.fixture
def calibration(tmp_path):
    path = tmp_path / "candidate.npz"
    writeSynthCalibration(str(path))
    return str(path)


def texturedPair(disparity=24):
    rng = np.random.default_rng(7)
    left = rng.integers(0, 256, (480, 640), dtype=np.uint8)
    right = np.zeros_like(left)
    right[:, :-disparity] = left[:, disparity:]
    return left, right


@pytest.mark.parametrize("count", [48, 64, 96])
def test_z_only_matches_xyz_mask_and_depth(calibration, count):
    processor = makeProcessor(calibration, count)
    left, right = texturedPair()
    full = processPair(processor, left, right)
    z = processPair(processor, left, right, with_xyz=False)
    assert z.xyz is None and z.depth.shape == (240, 320)
    assert full.valid.any() and (~full.valid).any()
    np.testing.assert_array_equal(z.valid, full.valid)
    np.testing.assert_allclose(z.depth, full.depth, rtol=1e-6, atol=1e-7)
    assert np.isnan(z.depth[~z.valid]).all()


def test_pair_path_matches_stitched_path(calibration):
    processor = makeProcessor(calibration)
    left, right = texturedPair()
    pair = processPair(processor, left, right)
    stitched = processStitched(processor, np.hstack([left, right]))
    np.testing.assert_array_equal(pair.disparity, stitched.disparity)
    np.testing.assert_array_equal(pair.valid, stitched.valid)
    np.testing.assert_allclose(pair.xyz, stitched.xyz)


def test_z_only_skips_full_reprojection(calibration, monkeypatch):
    processor = makeProcessor(calibration)
    def forbidden(*args, **kwargs):
        raise AssertionError("Z-only must not allocate XYZ")
    monkeypatch.setattr(cv2, "reprojectImageTo3D", forbidden)
    left, right = texturedPair()
    result = processPair(processor, left, right, with_xyz=False)
    assert result.valid.any()


def test_range_selection_preserves_required_near_distance(calibration):
    reference = makeProcessor(calibration)
    for count in (48, 64, 96):
        # 留一个视差像素距离门限余量，自动档可包含该量程。
        minimum = reference.q[2, 3] / (reference.q[3, 2] * (count - 2))
        processor = makeProcessor(calibration, 0, minimum)
        assert processor.num_disparities == count
        assert processor.nearestDepthM < minimum
        with pytest.raises(ValueError, match="无法覆盖"):
            makeProcessor(calibration, count - 16, minimum)
    with pytest.raises(ValueError, match="超过 96"):
        makeProcessor(calibration, 0, .01)


@pytest.mark.parametrize("count,minimum", [(0, 0), (47, 0), (-16, 0), (320, 0), (True, 0), (48.5, 0), (64, float("nan")), (64, -.1)])
def test_invalid_disparity_configuration_rejected(calibration, count, minimum):
    reference = makeProcessor(calibration)
    with pytest.raises(ValueError):
        selectNumDisparities(reference.q, count, minimum)


def test_xyz_demand_tracks_subscribers():
    node = DepthNode.__new__(DepthNode)
    node.publish_xyz = node.publishCompact = True
    node.publish_aux_without_subscribers = False
    node.pub_xyz = SimpleNamespace(get_subscription_count=lambda: 0)
    node.pub_xyz_valid = SimpleNamespace(get_subscription_count=lambda: 0)
    assert not node._needsXyz()
    node.pub_xyz_valid = SimpleNamespace(get_subscription_count=lambda: 1)
    assert node._needsXyz()
    node.publishCompact = False
    assert not node._needsXyz()
    node.publish_aux_without_subscribers = True
    assert node._needsXyz()


def test_existing_calibration_z_matches_xyz():
    from stereo_depth.core import Config, StereoProcessor
    processor = StereoProcessor(Config(num_disparities=0, required_min_depth_m=.2))
    assert processor.num_disparities == 64
    assert processor.nearestDepthM < .2
    left, right = texturedPair()
    full = processPair(processor, left, right)
    z = processPair(processor, left, right, with_xyz=False)
    np.testing.assert_array_equal(z.valid, full.valid)
    np.testing.assert_allclose(z.depth, full.depth, rtol=1e-6, atol=1e-7)
