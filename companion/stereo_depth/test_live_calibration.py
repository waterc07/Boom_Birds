"""无硬件回归：合成相机几何、棋盘检测、采样门槛与 HTTP 状态机。"""
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from http.server import ThreadingHTTPServer

import cv2
import numpy as np

from calibration_core import Board, align_pair, calibrate, coverage, detect, is_novel, pair_quality
from live_calibration import CalibrationApp, Handler
from depth_preview import Config, StereoProcessor


def synthetic_samples(count=35):
    rng = np.random.default_rng(27)
    board, size = Board(), (1280, 960)
    k = np.array([[1000., 0, 640], [0, 1010, 480], [0, 0, 1]])
    d = np.array([-.07, .015, .001, -.001, 0.])
    stereo_r = cv2.Rodrigues(np.array([.004, -.009, .003]))[0]
    stereo_t = np.array([[-.065], [.0004], [.0008]])
    samples = []
    for i in range(count):
        rv = rng.uniform(-.5, .5, 3)
        t = np.array([[rng.uniform(-.19, .06)], [rng.uniform(-.16, .03)], [rng.uniform(.48, .9)]])
        rb = cv2.Rodrigues(stereo_r @ cv2.Rodrigues(rv)[0])[0]
        tb = stereo_r @ t + stereo_t
        a = cv2.projectPoints(board.objects(), rv, t, k, d)[0]
        b = cv2.projectPoints(board.objects(), rb, tb, k, d)[0]
        pair = tuple(np.float32(p + rng.normal(0, .04, p.shape)) for p in (a, b))
        samples.append(pair)
    return board, size, samples


class GeometryTests(unittest.TestCase):
    def test_board_units(self):
        b = Board()
        self.assertEqual(b.objects().shape, (88, 3))
        np.testing.assert_allclose(b.objects()[-1], [.2, .14, 0], atol=1e-7)

    def test_detect_full_resolution_and_reverse_order(self):
        b = Board()
        image = np.full((960, 1280), 235, np.uint8)
        for y in range(9):
            for x in range(12):
                image[240+y*40:240+(y+1)*40, 360+x*40:360+(x+1)*40] = 255 if (x+y)%2 else 0
        corners = detect(image, b)
        self.assertIsNotNone(corners)
        self.assertEqual(corners.shape, (88, 1, 2))
        a, c = align_pair(corners, corners[::-1], b)
        np.testing.assert_allclose(a, c)
        self.assertGreater(float(a[:, :, 0].max()), 790)
        widths, reasons = pair_quality([image, image], [a, c], (1280, 960), b)
        self.assertFalse(reasons)
        self.assertLess(max(widths), 3.5)

    def test_repeat_and_coverage_rejection(self):
        b, size, samples = synthetic_samples()
        self.assertFalse(is_novel(samples[0], [samples[0]], size, b))
        self.assertTrue(is_novel(samples[10], [samples[0]], size, b))
        cov = coverage([samples[0]]*30, size, b)
        self.assertFalse(cov['ready'])
        self.assertGreater(len(cov['missing']), 1)
        with self.assertRaises(ValueError):
            calibrate(samples[:8], size, b)

    def test_metric_stereo_and_unseen_views(self):
        b, size, samples = synthetic_samples()
        model, report = calibrate(samples, size, b)
        self.assertAlmostEqual(float(np.linalg.norm(model['T'])), .065, delta=.001)
        self.assertAlmostEqual(model['K1'][0, 0], 1000, delta=15)
        self.assertLess(report['holdout_vertical_p95_px'], .5)
        self.assertTrue(set(report['train_indices']).isdisjoint(report['holdout_indices']))
        self.assertEqual(len(report['holdout_indices']), 7)
        self.assertTrue(np.isfinite(model['Q']).all())
        self.assertLess(model['T'][0, 0], 0)
        json.dumps(report, allow_nan=False)


class DepthIntegrationTests(unittest.TestCase):
    def test_old_and_high_resolution_intrinsic_scaling(self):
        legacy_path = Path(__file__).resolve().parent / 'calibration/20260911_202047/baseline_65mm.npz'
        old = StereoProcessor(Config(calibration=legacy_path))
        self.assertEqual(old.capture_size, (1280, 480))
        self.assertAlmostEqual(old.baseline_m, .065, places=6)
        with np.load(legacy_path) as archive:
            model = {key: archive[key].copy() for key in archive.files}
        for key in ('K1', 'K2'):
            model[key][:2, :] *= 2
        model['image_size'] = np.array([1280, 960])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'high.npz'
            np.savez(path, **model)
            high = StereoProcessor(Config(calibration=path))
            self.assertEqual(high.capture_size, (2560, 960))
            np.testing.assert_allclose(old.q, high.q, atol=1e-8)
            for a, b in zip(old.map_a, high.map_a):
                np.testing.assert_allclose(a, b, atol=1e-5)
            ok, jpg = cv2.imencode('.jpg', np.zeros((960, 2560, 3), np.uint8))
            self.assertTrue(ok)
            a, b = high.rectify(jpg)
            self.assertEqual(a.shape, (240, 320, 3))
            self.assertEqual(b.shape, (240, 320, 3))
            with self.assertRaises(RuntimeError):
                old.rectify(jpg)
            model['T'] *= 1.02
            np.savez(path, **model)
            self.assertAlmostEqual(StereoProcessor(Config(calibration=path)).baseline_m, .0663, places=6)


class WebTests(unittest.TestCase):
    def setUp(self):
        args = SimpleNamespace(width=2560, height=960, columns=11, rows=8, square_mm=20, stream_fps=15)
        self.app = CalibrationApp(args)
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.server.app = self.app
        self.worker = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.worker.start()
        self.base = f'http://127.0.0.1:{self.server.server_port}'

    def tearDown(self):
        self.app.stop.set()
        self.server.shutdown()
        self.server.server_close()
        self.worker.join()

    def post(self, body, **headers):
        req = Request(self.base+'/command', data=json.dumps(body).encode(),
                      headers={'Content-Type':'application/json', **headers})
        return urlopen(req, timeout=3)

    def test_status_page_and_guardrails(self):
        with urlopen(self.base+'/') as response:
            self.assertIn('双目校准台'.encode(), response.read())
        with urlopen(self.base+'/status') as response:
            state = json.load(response)
        self.assertEqual(state['mode'], 'focus')
        with self.assertRaises(HTTPError) as error:
            self.post({'action':'start', 'confirmed':True})
        self.assertEqual(error.exception.code, 400)
        self.app.received_at = time.monotonic()
        with self.assertRaises(HTTPError):
            self.post({'action':'start', 'confirmed':False})
        with self.post({'action':'start', 'confirmed':True}) as response:
            self.assertEqual(response.status, 200)
        self.assertEqual(self.app.mode, 'collect')
        with self.assertRaises(HTTPError):
            self.post({'action':'solve'})
        self.assertEqual(self.app.mode, 'collect')
        with self.post({'action':'pause'}):
            pass
        self.assertEqual(self.app.mode, 'paused')
        with self.assertRaises(HTTPError) as error:
            self.post({'action':'reset'}, Origin='http://example.org')
        self.assertEqual(error.exception.code, 403)
        self.app.mode = 'solving'
        with self.assertRaises(HTTPError):
            self.post({'action':'reset'})

    def test_latest_frame_endpoint_and_staleness_header(self):
        with self.assertRaises(HTTPError) as error:
            urlopen(self.base+'/frame/raw.jpg')
        self.assertEqual(error.exception.code, 503)
        for sequence, payload in [(1, b'first'), (2, b'latest')]:
            self.app.streams['raw'] = (sequence, payload)
            self.app.stream_times['raw'] = time.monotonic() - .02
        with urlopen(self.base+'/frame/raw.jpg') as response:
            self.assertEqual(response.read(), b'latest')
            self.assertEqual(response.headers['X-Frame-Sequence'], '2')
            self.assertLess(float(response.headers['X-Source-Age-Ms']), 300)
            self.assertEqual(response.headers['Cache-Control'], 'no-store')
        self.app.error = 'camera disconnected'
        with self.assertRaises(HTTPError) as error:
            urlopen(self.base+'/frame/raw.jpg')
        self.assertEqual(error.exception.code, 503)

    def test_corner_checkpoint_no_photographs(self):
        b, size, samples = synthetic_samples(2)
        with tempfile.TemporaryDirectory() as directory:
            self.app.session_dir = Path(directory)
            self.app.samples = samples
            self.app.sample_sequences = [10, 20]
            self.app.checkpoint()
            json.dumps(self.app.status(), allow_nan=False)
            self.assertEqual([p.name for p in Path(directory).iterdir()], ['observations.npz'])
            with np.load(Path(directory)/'observations.npz') as data:
                np.testing.assert_allclose(data['corners_a'], [p[0] for p in samples])
                self.assertAlmostEqual(float(data['square_size_m']), .02)
            self.app.command({'action':'reset'})
            self.assertTrue((Path(directory)/'observations.npz').exists())
            self.assertEqual(self.app.samples, [])

    def test_solver_writes_report_and_rectification_maps(self):
        board, size, samples = synthetic_samples()
        with tempfile.TemporaryDirectory() as directory:
            self.app.session_dir = Path(directory)
            self.app.samples = samples
            self.app.mode = 'solving'
            self.app.solve()
            self.assertEqual(self.app.mode, 'review', self.app.message)
            self.assertIsNotNone(self.app.maps)
            self.assertTrue((Path(directory)/'candidate.npz').is_file())
            report = json.loads((Path(directory)/'report.json').read_text())
            self.assertEqual(len(report['holdout_indices']), 7)
            json.dumps(self.app.status(), allow_nan=False)

    def test_preview_waits_for_first_real_packet(self):
        worker = threading.Thread(target=self.app.preview_loop, daemon=True)
        worker.start()
        time.sleep(.1)
        self.assertIsNone(self.app.error)
        ok, packet = cv2.imencode('.jpg', np.zeros((960, 2560, 3), np.uint8))
        self.assertTrue(ok)
        with self.app.condition:
            self.app.packet = packet
            self.app.sequence = 1
            self.app.received_at = time.monotonic()
            self.app.condition.notify_all()
            self.assertTrue(self.app.condition.wait_for(lambda: self.app.decoded_sequence == 1, 3))
        self.app.stop.set()
        with self.app.condition:
            self.app.condition.notify_all()
        worker.join(3)
        self.assertIsNone(self.app.error)


if __name__ == '__main__':
    cv2.setNumThreads(2)
    unittest.main(verbosity=2)
