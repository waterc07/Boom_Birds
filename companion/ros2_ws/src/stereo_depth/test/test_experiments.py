from dataclasses import replace
import hashlib
import json
import os
import sys
from types import SimpleNamespace
import cv2
import numpy as np
import pytest
from stereo_depth.core import Config, StereoProcessor
from stereo_depth.experiments import DisparityPrior, projectVioPrior, RknnResidualRefiner
from boom_birds_sim.synthetic import writeSynthCalibration


@pytest.fixture
def config(tmp_path):
    calibration = tmp_path / "candidate.npz"
    writeSynthCalibration(str(calibration))
    return Config(
        calibration=calibration,
        num_disparities=64,
        native_library=os.environ.get("BB_NATIVE_LIBRARY", ""),
    )


def pair(d=12, seed=7):
    left = np.random.default_rng(seed).integers(0, 256, (240, 320), dtype=np.uint8)
    right = np.zeros_like(left)
    right[:, :-d] = left[:, d:]
    return left, right


@pytest.mark.parametrize("xyz", [False, True])
@pytest.mark.parametrize("mode", ["lut", "native"])
def test_q4_matches_reference_random_and_boundary_values(config, xyz, mode):
    p = StereoProcessor(config)
    q = StereoProcessor(replace(config, postprocess=mode))
    rng = np.random.default_rng(51)
    a = rng.integers(-32768, 32767, (240, 320), dtype=np.int16)
    b = rng.integers(-32768, 32767, (240, 320), dtype=np.int16)
    # exact LR threshold, half-pixel ties, d=0, d=N-1, negative sentinel, border
    a[20:30, :] = rng.choice([0, 8, 16, 24, 32, 1007, 1008, 1024, -16], (10, 320))
    target = np.clip(np.rint(p.cols - a.astype(np.float32) / 16).astype(int), 0, 319)
    b[p.rows, target] = -a
    ref, rm = p.reconstructQ4(a, b, with_xyz=xyz)
    out, vm = q.reconstructQ4(a, b, with_xyz=xyz)
    np.testing.assert_array_equal(vm, rm)
    np.testing.assert_allclose(out, ref, rtol=2e-6, atol=1e-7)
    assert np.isnan(out[~vm]).all()


@pytest.mark.parametrize("mode", ["numpy", "lut", "native"])
def test_snapshot_not_overwritten_by_reuse(config, mode):
    p = StereoProcessor(replace(config, reuse_buffers=True, fixed_maps=True, postprocess=mode))
    first = p.processPair(*pair(), with_xyz=False)
    saved = [v.copy() for v in first[:-1]]
    p.processPair(*pair(20, 9), with_xyz=True)
    for before, after in zip(saved, first[:-1]):
        np.testing.assert_array_equal(before, after)
    assert len(p._scratch) <= 10


@pytest.mark.parametrize("channels", [1, 3])
def test_fixed_maps_and_reuse_do_not_change_images(config, channels):
    a, b = pair()
    image = np.hstack([a, b])
    if channels == 3:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    base = StereoProcessor(config).processImage(image)
    optimized = StereoProcessor(replace(config, fixed_maps=True, reuse_buffers=True)).processImage(
        image
    )
    for a, b in zip(base[:-1], optimized[:-1]):
        np.testing.assert_array_equal(a, b)


@pytest.mark.parametrize("matcher", ["census", "coarse_census"])
def test_census_known_shift_and_occlusion(config, matcher):
    p = StereoProcessor(replace(config, matcher=matcher, experimental=True, postprocess="native"))
    # 已校正的人工平移，避免标定 resize 把输入视差改成另一尺度。
    a, b = pair(12)
    result = p.matchRectified(a, b, with_xyz=False)
    d, mask = result[2], result[4]
    assert mask[10:-10, 80:-10].mean() > 0.8
    assert abs(np.median(d[mask]) - 12) < 0.1
    assert not mask[:4].any() and not mask[:, :12].any()
    assert np.isnan(result[3][~mask]).all()
    # 无纹理和边界不形成有效自由空间。
    empty = p.matchRectified(np.zeros_like(a), np.zeros_like(b), with_xyz=False)
    assert not empty[4].any()


def test_prior_motion_projection_and_stale_fallback(config):
    p = StereoProcessor(config)
    depth = np.full((240, 320), 1, np.float32)
    prior = projectVioPrior(depth, p.p1, p.q, np.eye(4), 1, 1.05)
    expected = (p.q[2, 3] - p.q[3, 3]) / p.q[3, 2]
    np.testing.assert_allclose(prior.disparity, expected)
    assert prior.forFrame(depth.shape, 1.05) is not None
    assert prior.forFrame(depth.shape, 1.06) is None
    assert DisparityPrior(prior.disparity, 1, 2).forFrame(depth.shape, 2) is None
    t = np.eye(4)
    t[0, 3] = 1 / p.p1[0, 0]
    shifted = projectVioPrior(depth, p.p1, p.q, t, 1, 1.05)
    assert np.isnan(shifted.disparity[:, 0]).all()
    assert np.isfinite(shifted.disparity[:, 1:]).all()
    t[0, 0] = 2
    with pytest.raises(ValueError):
        projectVioPrior(depth, p.p1, p.q, t, 1, 1.05)


def test_prior_is_consumed_once_and_revalidates_current_pair(config):
    p = StereoProcessor(replace(config, matcher="census", experimental=True, postprocess="native"))
    a, b = pair(12)
    p.setVioPrior(DisparityPrior(np.full(a.shape, 12, np.float32), 1, 1.05), 1.05)
    guided = p.matchRectified(a, b, with_xyz=False)
    assert p.alternative.stats["guided"]
    full = p.matchRectified(a, b, with_xyz=False)
    assert not p.alternative.stats["guided"]
    assert np.median(guided[2][guided[4]]) == 12
    assert guided[4][10:-10, 80:-10].mean() > 0.8
    p.setVioPrior(DisparityPrior(np.full(a.shape, 40, np.float32), 1, 1.05), 1.2)
    stale = p.matchRectified(a, b, with_xyz=False)
    assert not p.alternative.stats["guided"]
    np.testing.assert_array_equal(stale[4], full[4])
    blank = p.matchRectified(np.zeros_like(a), np.zeros_like(b), with_xyz=False)
    assert not blank[4].any()


@pytest.mark.parametrize(
    "options",
    [
        {"matcher": "census"},
        {"matcher": "elas"},
        {"rknn_model": "x"},
        {"postprocess": "bad"},
        {"matcher": "bad"},
    ],
)
def test_invalid_or_unapproved_experiment_rejected(config, options):
    with pytest.raises(ValueError):
        StereoProcessor(replace(config, **options))


def test_native_rejects_wrong_shape_or_type(config):
    p = StereoProcessor(replace(config, postprocess="native"))
    a = np.zeros((240, 320), np.int16)
    with pytest.raises(ValueError):
        p.reconstructQ4(a.astype(np.float32), a)
    with pytest.raises(ValueError):
        p.reconstructQ4(a[:2], a[:2])


def fakeModel(tmp_path, monkeypatch, loadCode=0):
    model = tmp_path / "model.rknn"
    model.write_bytes(b"mock-model-for-contract-tests")
    meta = tmp_path / "model.json"
    meta.write_text(
        json.dumps(
            {
                "sha256": hashlib.sha256(model.read_bytes()).hexdigest(),
                "contract": "stereo-residual-nhwc-v1",
                "shape": [1, 240, 320, 3],
                "max_delta_px": 1,
                "min_confidence": 0.8,
            }
        )
    )

    class Runtime:
        def __init__(self):
            self.released = False
            self.output = np.zeros((1, 240, 320, 2), np.float32)

        def loadRknn(self, path):
            return loadCode

        def initRuntime(self):
            return 0

        def inference(self, inputs, dataFormat):
            assert inputs[0].shape == (1, 240, 320, 3) and inputs[0].dtype == np.float32
            return [self.output]

        def release(self):
            self.released = True

    runtime = Runtime()
    monkeypatch.setitem(sys.modules, "rknnlite", SimpleNamespace())
    monkeypatch.setitem(sys.modules, "rknnlite.api", SimpleNamespace(RKNNLite=lambda: runtime))
    return model, meta, runtime


def test_rknn_contract_and_correction_limits(tmp_path, monkeypatch):
    model, meta, rt = fakeModel(tmp_path, monkeypatch)
    refine = RknnResidualRefiner(model, meta, 64)
    a, b = pair()
    d = np.full(a.shape, 12 * 16, np.int16)
    d[0, 0] = -16
    rt.output[..., 0] = 0.5
    rt.output[..., 1] = 0.9
    rt.output[0, 0, 1, 0] = 2
    rt.output[0, 0, 2, 1] = 0.1
    rt.output[0, 0, 3, 0] = np.nan
    out = refine.refine(a, b, d)
    assert out[10, 10] == 200
    np.testing.assert_array_equal(out[0, :4], d[0, :4])
    refine.close()
    assert rt.released
    model.write_bytes(b"changed")
    with pytest.raises(ValueError, match="SHA256"):
        RknnResidualRefiner(model, meta, 64)


def test_rknn_runtime_released_when_init_fails(tmp_path, monkeypatch):
    model, meta, rt = fakeModel(tmp_path, monkeypatch, 1)
    with pytest.raises(RuntimeError):
        RknnResidualRefiner(model, meta, 64)
    assert rt.released


def test_pose_prior_buffer_loss_reset_jump_and_recovery(config):
    from stereo_depth.experiments import PosePriorBuffer

    p = StereoProcessor(config)
    cache = PosePriorBuffer()
    depth = np.ones((240, 320), np.float32)
    assert cache.prepare(1, p.p1, p.q) is None
    cache.push(1, np.eye(4), "world")
    assert cache.prepare(1, p.p1, p.q) is None
    cache.record(depth)
    cache.push(1.05, np.eye(4), "world")
    assert cache.prepare(1.05, p.p1, p.q) is not None
    cache.record(depth)
    assert cache.prepare(2, p.p1, p.q) is None
    cache.push(2, np.eye(4), "world")
    assert cache.prepare(2, p.p1, p.q) is None
    cache.record(depth)
    cache.push(2.05, np.eye(4), "reset_world")
    assert cache.prepare(2.05, p.p1, p.q) is None
    cache.record(depth)
    jump = np.eye(4)
    jump[0, 3] = 1
    cache.push(2.1, jump, "reset_world")
    assert cache.prepare(2.1, p.p1, p.q) is None
    cache.push(1, np.eye(4), "reset_world")
    assert cache.prepare(1, p.p1, p.q) is None


def test_odometry_prior_rejects_imu_frame_and_uses_xyzw():
    from boom_birds_sensing.depth_node import DepthNode
    from nav_msgs.msg import Odometry
    from stereo_depth.experiments import PosePriorBuffer

    node = DepthNode.__new__(DepthNode)
    node.frame_id = "cam0_rect"
    node.priorBuffer = PosePriorBuffer()
    node.get_logger = lambda: SimpleNamespace(warn=lambda *a: None)
    msg = Odometry()
    msg.header.stamp.sec = 1
    msg.header.frame_id = "world"
    msg.pose.pose.orientation.w = 1
    msg.child_frame_id = "imu"
    node._onPriorPose(msg)
    assert not node.priorBuffer.poses
    msg.child_frame_id = "cam0_rect"
    node._onPriorPose(msg)
    np.testing.assert_allclose(node.priorBuffer.poses[-1][1], np.eye(4))
    msg.pose.covariance[0] = 10
    node._onPriorPose(msg)
    assert len(node.priorBuffer.poses) == 1
