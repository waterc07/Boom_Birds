from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from boom_birds_sensing.depth_core import make_processor, process_pair, process_stitched
from boom_birds_sensing.depth_node import DepthNode
from boom_birds_sim.synthetic import write_synth_calibration
from stereo_depth.core import select_num_disparities


@pytest.fixture
def calibration(tmp_path):
    path = tmp_path / "candidate.npz"
    write_synth_calibration(str(path))
    return str(path)


def textured_pair(disparity=24):
    rng = np.random.default_rng(7)
    left = rng.integers(0, 256, (480, 640), dtype=np.uint8)
    right = np.zeros_like(left)
    right[:, :-disparity] = left[:, disparity:]
    return left, right


@pytest.mark.parametrize("count", [48, 64, 96])
def test_z_only_matches_xyz_mask_and_depth(calibration, count):
    processor = make_processor(calibration, count)
    left, right = textured_pair()
    full = process_pair(processor, left, right)
    z = process_pair(processor, left, right, with_xyz=False)
    assert z.xyz is None and z.depth.shape == (240, 320)
    assert full.valid.any() and (~full.valid).any()
    np.testing.assert_array_equal(z.valid, full.valid)
    np.testing.assert_allclose(z.depth, full.depth, rtol=1e-6, atol=1e-7)
    assert np.isnan(z.depth[~z.valid]).all()


def test_pair_path_matches_stitched_path(calibration):
    processor = make_processor(calibration)
    left, right = textured_pair()
    pair = process_pair(processor, left, right)
    stitched = process_stitched(processor, np.hstack([left, right]))
    np.testing.assert_array_equal(pair.disparity, stitched.disparity)
    np.testing.assert_array_equal(pair.valid, stitched.valid)
    np.testing.assert_allclose(pair.xyz, stitched.xyz)


def test_z_only_skips_full_reprojection(calibration, monkeypatch):
    processor = make_processor(calibration)
    def forbidden(*args, **kwargs):
        raise AssertionError("Z-only must not allocate XYZ")
    monkeypatch.setattr(cv2, "reprojectImageTo3D", forbidden)
    left, right = textured_pair()
    result = process_pair(processor, left, right, with_xyz=False)
    assert result.valid.any()


def test_range_selection_preserves_required_near_distance(calibration):
    reference = make_processor(calibration)
    for count in (48, 64, 96):
        # 留一个视差像素距离门限余量，自动档可包含该量程。
        minimum = reference.q[2, 3] / (reference.q[3, 2] * (count - 2))
        processor = make_processor(calibration, 0, minimum)
        assert processor.num_disparities == count
        assert processor.nearest_depth_m < minimum
        with pytest.raises(ValueError, match="无法覆盖"):
            make_processor(calibration, count - 16, minimum)
    with pytest.raises(ValueError, match="超过 96"):
        make_processor(calibration, 0, .01)


@pytest.mark.parametrize("count,minimum", [(0, 0), (47, 0), (-16, 0), (320, 0), (True, 0), (48.5, 0), (64, float("nan")), (64, -.1)])
def test_invalid_disparity_configuration_rejected(calibration, count, minimum):
    reference = make_processor(calibration)
    with pytest.raises(ValueError):
        select_num_disparities(reference.q, count, minimum)


def test_xyz_demand_tracks_subscribers():
    node = DepthNode.__new__(DepthNode)
    node.publish_xyz = node.publish_compact = True
    node.publish_aux_without_subscribers = False
    node.pub_xyz = SimpleNamespace(get_subscription_count=lambda: 0)
    node.pub_xyz_valid = SimpleNamespace(get_subscription_count=lambda: 0)
    assert not node._needs_xyz()
    node.pub_xyz_valid = SimpleNamespace(get_subscription_count=lambda: 1)
    assert node._needs_xyz()
    node.publish_compact = False
    assert not node._needs_xyz()
    node.publish_aux_without_subscribers = True
    assert node._needs_xyz()


def test_existing_calibration_z_matches_xyz():
    from stereo_depth.core import Config, StereoProcessor
    processor = StereoProcessor(Config(num_disparities=0, required_min_depth_m=.2))
    assert processor.num_disparities == 64
    assert processor.nearest_depth_m < .2
    left, right = textured_pair()
    full = process_pair(processor, left, right)
    z = process_pair(processor, left, right, with_xyz=False)
    np.testing.assert_array_equal(z.valid, full.valid)
    np.testing.assert_allclose(z.depth, full.depth, rtol=1e-6, atol=1e-7)
