"""RAW 输入、单目留出求解和采集门控；不打开真实设备。"""
from argparse import Namespace
from pathlib import Path
import json
import time
import cv2
import numpy as np
import pytest
from calibration_core import Board, is_novel
from mono_calibration_core import calibrate, coverage
from mipi_raw import unpack_raw10, demosaic
from live_mipi_calibration import MonoApp


def synthetic():
    board = Board()
    size = (1640, 1232)
    k = np.array([[1200., 0, 820], [0, 1190, 616], [0, 0, 1]])
    d = np.array([-.12, .025, .001, -.002, 0])
    rng = np.random.default_rng(35)
    samples = []
    for _ in range(1000):
        rv = rng.uniform([-.65, -.65, -.5], [.65, .65, .5])
        z = rng.uniform(.36, .95)
        center = rng.uniform([270, 230], [1370, 1000])
        rotation = cv2.Rodrigues(rv)[0]
        tv = np.r_[(center-np.array([820, 616]))/np.array([1200, 1190])*z, z]
        tv -= rotation @ np.array([.10, .07, 0])
        p = cv2.projectPoints(board.objects(), rv, tv, k, d)[0]
        if (p.min() > 15 and np.all(p.reshape(-1, 2) < np.array(size)-15)):
            p += rng.normal(0, .04, p.shape).astype(np.float32)
            samples.append((p,))
        if len(samples) == 60:
            break
    return samples, size, board, k


def test_raw10_roundtrip_with_row_padding():
    rng = np.random.default_rng(44)
    pixels = rng.integers(0, 1024, (6, 12), dtype=np.uint16)
    groups = pixels.reshape(6, 3, 4)
    packed = np.zeros((6, 3, 5), np.uint8)
    packed[:, :, :4] = groups >> 2
    for index in range(4):
        packed[:, :, 4] |= ((groups[:, :, index] & 3) << (2*index)).astype(np.uint8)
    padded = np.pad(packed.reshape(6, 15), ((0, 0), (0, 1)), constant_values=255)
    np.testing.assert_array_equal(unpack_raw10(padded.tobytes(), 12, 6, 16), pixels)
    with pytest.raises(ValueError):
        unpack_raw10(padded.tobytes()[:-1], 12, 6, 16)


def test_mono_intrinsics_holdout_and_no_stereo_fields():
    samples, size, board, expected = synthetic()
    assert coverage(samples, size, board)['ready']
    assert len(coverage(samples, size, board)['grids']) == 1
    assert not is_novel(samples[0], samples, size, board)
    model, report = calibrate(samples, size, board)
    np.testing.assert_allclose(model['K'], expected, atol=3)
    assert report['train_rms_px'] < .1
    assert max(report['holdout_rms_px']) < .1
    assert not set(report['train_indices']) & set(report['holdout_indices'])
    assert 'baseline_mm' not in report and 'K2' not in model
    # 留出点改变不应参与 K/D 拟合，但必须触发留出误差警告。
    modified = [(p[0].copy(),) for p in samples]
    rng = np.random.default_rng(3)
    for index in report['holdout_indices']:
        modified[index][0][:] += rng.normal(0, 4, modified[index][0].shape)
    model2, report2 = calibrate(modified, size, board)
    np.testing.assert_array_equal(model2['K'], model['K'])
    assert report2['status'] == 'CANDIDATE_NEEDS_REVIEW'


def test_refuse_insufficient_and_nonfinite():
    samples, size, board, _ = synthetic()
    with pytest.raises(ValueError):
        calibrate(samples[:5], size, board)
    samples[0][0][0, 0, 0] = np.nan
    with pytest.raises(ValueError, match='角点'):
        calibrate(samples, size, board)


def make_app(tmp_path):
    return MonoApp(Namespace(width=1640, height=1232, columns=11, rows=8,
                             square_mm=20, stream_fps=8, output=str(tmp_path)))


def test_start_confirmation_freshness_exposure_lock_checkpoint(tmp_path):
    app = make_app(tmp_path)
    app.received_at = time.monotonic()
    app.camera.set_exposure = lambda *args: None
    app.command(dict(action='exposure', exposure=1500, gain=192))
    with pytest.raises(ValueError, match='确认'):
        app.command(dict(action='start', confirmed=False))
    app.command(dict(action='start', confirmed=True))
    with pytest.raises(ValueError, match='锁定'):
        app.command(dict(action='exposure', exposure=1500, gain=192))
    app.command(dict(action='pause'))
    with pytest.raises(ValueError, match='锁定'):
        app.command(dict(action='exposure', exposure=1500, gain=192))
    samples, _, _, _ = synthetic()
    app.samples = samples[:1]
    app.sample_sequences = [5]
    app.source_times = [10.]
    app.checkpoint()
    saved = app.session_dir
    with np.load(saved/'observations.npz') as record:
        assert record['corners'].shape == (1, 88, 1, 2)
        assert record['image_size'].tolist() == [1640, 1232]
    assert app.status()['camera_count'] == 1
    app.command(dict(action='reset'))
    assert saved.exists() and app.samples == [] and app.source_times == []
    app.received_at = time.monotonic()-4
    with pytest.raises(ValueError, match='实时'):
        app.command(dict(action='start', confirmed=True))


def test_capture_drains_raw_without_demosaic(tmp_path, monkeypatch):
    app = make_app(tmp_path)
    class Camera:
        def __init__(self):
            self.count = 0
            self.closed = False
        def configure(self):
            pass
        def read(self, stop):
            self.count += 1
            if self.count > 100:
                stop.set()
                return None
            return bytes([self.count])
        def close(self):
            self.closed = True
    app.camera = Camera()
    def forbidden(*args):
        raise AssertionError('采集线程不能等待去马赛克')
    monkeypatch.setattr('live_mipi_calibration.demosaic', forbidden)
    app.capture_loop()
    assert app.sequence == 100 and app.packet == bytes([100])
    assert app.error is None and app.camera.closed and app.decoded is None


def test_preview_resize_does_not_change_calibration_image(tmp_path):
    app = make_app(tmp_path)
    original = np.zeros((1232, 1640, 3), np.uint8)
    jpeg = app.preview_jpeg(original)
    preview = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert preview.shape == (616, 820, 3)
    assert original.shape == (1232, 1640, 3)


def test_explicit_rggb_red_channel(monkeypatch):
    raw = np.full((12, 12), 64, np.uint16)
    raw[0::2, 0::2] = 1023
    monkeypatch.setattr('mipi_raw.unpack_raw10', lambda *args: raw)
    image = demosaic(b'', 12, 12, 15)
    assert np.all(image[2:-2, 2:-2, 2] > 250)
    assert np.all(image[2:-2, 2:-2, :2] < 2)
