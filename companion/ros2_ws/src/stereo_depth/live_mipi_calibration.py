"""IMX219 单目标定网页，默认 localhost:8083；不修改既有双目标定。"""
from collections import deque
from datetime import datetime
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse
import argparse
import json
import signal
import threading
import time
import cv2
import numpy as np
from calibration_core import Board, detect, align_pair, pair_quality, is_novel
from mono_calibration_core import calibrate, coverage, pose_error
from live_calibration import CalibrationApp, Handler
from mipi_raw import RawCamera, demosaic

ROOT = Path(__file__).resolve().parent


class MonoApp(CalibrationApp):
    def __init__(self, args):
        super().__init__(args)
        self.size = (args.width, args.height)
        self.camera = RawCamera(args)
        self.detection = dict(found=[False], sharpness=[None], motion_px=None)
        self.message = '先调整曝光/增益、检查清晰度；固定镜头并确认实测棋盘尺寸。'
        self.source_times = []
        self.latest_raw = None
        self.packet_received = 0
        self.decode_ms = 0
        self.preview_size = (820, 616)

    def capture_loop(self):
        times = deque(maxlen=60)
        try:
            self.camera.configure()
            while not self.stop.is_set():
                payload = self.camera.read(self.stop)
                if payload is None:
                    return
                # RAW 管道读完的主机时间；没有曝光时间戳，不能用于相机—IMU同步。
                now = time.monotonic()
                times.append(now)
                with self.condition:
                    self.sequence += 1
                    self.packet = payload
                    self.packet_received = now
                    self.capture_fps = (len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0
                    self.condition.notify_all()
        except Exception as exc:
            self.fail(exc)
        finally:
            self.camera.close()

    def preview_jpeg(self, image):
        preview = cv2.resize(image, self.preview_size, interpolation=cv2.INTER_AREA)
        ok, jpg = cv2.imencode('.jpg', preview, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            raise RuntimeError('预览 JPEG 编码失败')
        return jpg.tobytes()

    def preview_loop(self):
        sequence = -1
        times = deque(maxlen=30)
        try:
            while not self.stop.is_set():
                began = time.monotonic()
                with self.condition:
                    ready = self.condition.wait_for(lambda: self.stop.is_set() or
                        (self.packet is not None and self.sequence != sequence), 4)
                    if self.stop.is_set():
                        return
                    if not ready:
                        raise RuntimeError('超过 4 秒没有新 RAW 帧')
                    sequence, payload, received = self.sequence, self.packet, self.packet_received
                # RAW 取帧保持单槽；转图可跳过旧帧，检测/保存仍使用完整传感器尺寸。
                image = demosaic(payload, *self.size, self.camera.stride)
                jpeg = self.preview_jpeg(image)
                times.append(time.monotonic())
                with self.condition:
                    self.decoded = (image, received)
                    self.latest_raw = payload
                    self.decoded_sequence = sequence
                    self.received_at = received
                    self.streams['raw'] = (sequence, jpeg)
                    self.stream_times['raw'] = received
                    self.decode_ms = (time.monotonic()-began)*1000
                    self.preview_fps = (len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0
                    self.condition.notify_all()
                self.stop.wait(max(0, 1/self.args.stream_fps-(time.monotonic()-began)))
        except Exception as exc:
            self.fail(exc)

    def checkpoint(self):
        if self.session_dir is None:
            self.session_dir = Path(self.args.output) / ('mono_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            self.session_dir.mkdir(parents=True, exist_ok=False)
        temp = self.session_dir / 'observations.tmp.npz'
        np.savez_compressed(temp, corners=np.array([p[0] for p in self.samples]),
                            image_size=self.size, board=self.board.pattern,
                            square_size_m=self.board.square_m, sequences=self.sample_sequences,
                            received_monotonic_s=self.source_times)
        temp.replace(self.session_dir / 'observations.npz')
        (self.session_dir / 'source.json').write_text(
            json.dumps(self.camera.metadata, ensure_ascii=False, indent=2), encoding='utf-8')

    def detection_loop(self):
        sequence, previous, previous_at, stable_since, last_accept = -1, None, None, None, 0
        while not self.stop.is_set():
            try:
                with self.condition:
                    self.condition.wait_for(lambda: self.stop.is_set() or
                        (self.decoded is not None and self.decoded_sequence != sequence), 2)
                    if self.stop.is_set():
                        return
                    if self.decoded is None or self.decoded_sequence == sequence:
                        continue
                    sequence, (image, received) = self.decoded_sequence, self.decoded
                    generation, mode = self.generation, self.mode
                    raw_payload = self.latest_raw
                    model, maps = self.model, self.maps
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                began = time.monotonic()
                points = detect(gray, self.board, thorough=mode != 'focus')
                reasons, motion, widths = [], None, [None]
                if points is not None:
                    points = align_pair(points, points, self.board)[0]
                    sample = (points,)
                    widths, reasons = pair_quality([gray], sample, self.size, self.board)
                    if previous is not None and received - previous_at < 2:
                        motion = float(np.sqrt(np.mean((points - previous) ** 2)))
                    if motion is None or motion > 2.5 or reasons:
                        stable_since = None
                    elif stable_since is None:
                        stable_since = received
                    previous, previous_at = points, received
                else:
                    previous, previous_at, stable_since = None, None, None
                    reasons.append('请让完整棋盘进入画面，保留白边，避免反光')
                preview = image.copy()
                if points is not None:
                    cv2.drawChessboardCorners(preview, self.board.pattern, points, True)
                jpg = self.preview_jpeg(preview)
                corrected = cv2.remap(image, *maps, cv2.INTER_LINEAR) if maps is not None else None
                validation = dict(rms_px=pose_error(points, model, self.board), sequence=sequence) if model is not None and points is not None else None
                with self.condition:
                    if generation != self.generation:
                        previous, previous_at, stable_since = None, None, None
                        continue
                    self.detect_ms = (time.monotonic()-began)*1000
                    self.detection = dict(found=[points is not None], sharpness=widths, motion_px=motion)
                    self.streams['detected'] = (sequence, jpg)
                    self.stream_times['detected'] = received
                    if corrected is not None:
                        self.streams['rectified'] = (sequence, self.preview_jpeg(corrected))
                        self.stream_times['rectified'] = received
                    self.validation = validation
                    if self.mode == mode == 'collect':
                        if reasons:
                            self.message = '；'.join(reasons)
                        elif time.monotonic() - received > 2:
                            self.message = '检测图像过期，暂不收样'
                        elif stable_since is None or received - stable_since < .7:
                            self.message = '保持静止约 1 秒，等待自动收样'
                        elif not is_novel(sample, self.samples, self.size, self.board):
                            self.message = '此姿态已采集，请改变位置、距离或倾角'
                        elif received - last_accept >= 1.2:
                            index = len(self.samples)
                            self.samples.append((points.copy(),))
                            self.sample_sequences.append(sequence)
                            self.source_times.append(received)
                            self.checkpoint()
                            # 保留用于检测的无损图和对应 RAW，便于重做角点检查。
                            if not cv2.imwrite(str(self.session_dir / f'sample_{index:03d}.png'), image):
                                raise RuntimeError('样本图保存失败')
                            (self.session_dir / f'sample_{index:03d}.raw').write_bytes(raw_payload)
                            last_accept = received
                            self.message = f'已收集 {index+1} 组，请移动到下一个姿态'
                            if len(self.samples) >= 60:
                                self.mode = 'paused'
                    self.condition.notify_all()
            except Exception as exc:
                self.fail(exc)
                return
            self.stop.wait(max(0, .5 - (time.monotonic()-began)))

    def status(self):
        # 基类的互斥锁和状态契约保留；覆盖与结果只有一目。
        with self.condition:
            return dict(mode=self.mode, error=self.error, message=self.message,
                        sequence=self.sequence, age_s=time.monotonic()-self.received_at if self.received_at else None,
                        preview_size=list(self.preview_size), decode_ms=round(self.decode_ms, 1),
                        capture_fps=round(self.capture_fps, 1), decode_fps=round(self.preview_fps, 1),
                        detection_ms=round(self.detect_ms), detection=self.detection,
                        validation=self.validation, report=self.report,
                        stream_fps_limit=self.args.stream_fps, capture_size=list(self.size),
                        image_size=list(self.size), board=list(self.board.pattern),
                        square_mm=self.board.square_m*1000, camera_count=1,
                        session_dir=str(self.session_dir) if self.session_dir else None,
                        coverage=coverage(self.samples, self.size, self.board),
                        source=self.camera.metadata.copy(),
                        exposure_locked=bool(self.samples) or self.mode != 'focus')

    def command(self, data):
        action = data.get('action')
        with self.condition:
            if action == 'exposure':
                if self.mode != 'focus' or self.samples:
                    raise ValueError('采集开始后曝光锁定；需重置后调整')
                self.camera.set_exposure(int(data['exposure']), int(data['gain']))
                self.generation += 1
                return
            if action == 'solve':
                if self.mode == 'solving' or self.error:
                    raise ValueError('正在求解或相机发生错误')
                cov = coverage(self.samples, self.size, self.board)
                if not cov['ready']:
                    raise ValueError('；'.join(cov['missing']))
                self.mode = 'solving'
                self.message = '计算单目内参和畸变，保留 20% 姿态检查'
                threading.Thread(target=self.solve, daemon=True).start()
                return
            super().command(data)
            if action == 'reset':
                self.source_times = []
                self.message = '新一轮：可调整曝光和焦点；旧样本目录保留。'

    def solve(self):
        try:
            with self.condition:
                samples, directory = list(self.samples), self.session_dir
            model, report = calibrate(samples, self.size, self.board)
            maps = cv2.initUndistortRectifyMap(model['K'], model['D'], None,
                                             model['K'], self.size, cv2.CV_32FC1)
            report.update(source=self.camera.metadata.copy(), created_at=datetime.now().isoformat(),
                          opencv_version=cv2.__version__)
            np.savez_compressed(directory / 'candidate.npz', **model)
            (directory / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
            with self.condition:
                self.model, self.maps, self.report = model, maps, report
                self.mode = 'review'
                self.message = '候选内参已保存。摆放未采集的新姿态，检查实时重投影和去畸变画面。'
        except Exception as exc:
            with self.condition:
                self.mode, self.message = 'paused', f'计算失败，样本保留：{exc}'


class MonoHandler(Handler):
    def do_GET(self):
        if urlparse(self.path).path == '/crop.jpg':
            from urllib.parse import parse_qs
            query = parse_qs(urlparse(self.path).query)
            try:
                x, y = float(query['x'][0]), float(query['y'][0])
                if not np.isfinite([x, y]).all() or not 0 <= x <= 1 or not 0 <= y <= 1:
                    raise ValueError('裁剪位置不合法')
                app = self.server.app
                with app.condition:
                    if app.decoded is None:
                        raise ValueError('没有实时图像')
                    image, received = app.decoded
                px = max(0, min(image.shape[1]-320, int(x*image.shape[1])-160))
                py = max(0, min(image.shape[0]-240, int(y*image.shape[0])-120))
                ok, jpg = cv2.imencode('.jpg', image[py:py+240, px:px+320])
                if not ok:
                    raise ValueError('裁剪编码失败')
                return self.send(jpg.tobytes(), 'image/jpeg',
                                 headers={'X-Source-Age-Ms': (time.monotonic()-received)*1000})
            except (KeyError, ValueError, IndexError):
                return self.send({'error': '无有效裁剪图'}, status=400)
        if urlparse(self.path).path == '/':
            return self.send((ROOT / 'mono_calibration.html').read_bytes(), 'text/html; charset=utf-8')
        super().do_GET()


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device', default='/dev/video0')
    p.add_argument('--media', default='/dev/media2')
    p.add_argument('--subdev', default='/dev/v4l-subdev2')
    p.add_argument('--width', type=int, default=1640)
    p.add_argument('--height', type=int, default=1232)
    p.add_argument('--columns', type=int, default=11)
    p.add_argument('--rows', type=int, default=8)
    p.add_argument('--square-mm', type=float, default=20)
    p.add_argument('--exposure', type=int, default=1600)
    p.add_argument('--gain', type=int, default=128)
    p.add_argument('--stream-fps', type=float, default=8)
    p.add_argument('--port', type=int, default=8083)
    p.add_argument('--output', default=str(ROOT / 'calibration'))
    a = p.parse_args()
    if (a.width, a.height) != (1640, 1232) or not 1 <= a.port <= 65535 or not 1 <= a.stream_fps <= 15:
        p.error('当前 MIPI 入口限定 1640×1232；端口/预览频率不合法')
    if not 3 <= a.columns <= 20 or not 3 <= a.rows <= 20 or a.columns == a.rows or not 0 < a.square_mm < 1000:
        p.error('棋盘参数不合法')
    return a


def main():
    args = parse_args()
    cv2.setNumThreads(2)
    app = MonoApp(args)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), MonoHandler)
    server.app = app
    workers = [threading.Thread(target=f, daemon=True) for f in (app.capture_loop, app.preview_loop, app.detection_loop)]
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    try:
        for worker in workers:
            worker.start()
        print(f'MIPI 标定 http://127.0.0.1:{args.port}', flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        app.stop.set()
        with app.condition:
            app.condition.notify_all()
        for worker in workers:
            worker.join(timeout=4)
        app.camera.close()
        server.server_close()


if __name__ == '__main__':
    main()
