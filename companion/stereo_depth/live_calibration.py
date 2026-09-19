"""USB 双目实时调焦与引导标定。python3 live_calibration.py，浏览器端口 8082。"""
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse
import argparse
import json
import signal
import threading
import time

import cv2
import numpy as np

from calibration_core import Board, align_pair, calibrate, coverage, detect, epipolar_errors, is_novel, pair_quality

ROOT = Path(__file__).resolve().parent


class CalibrationApp:
    """采集、原图预览、检测和求解互相独立；只保留最新图像，样本仅为角点。"""

    def __init__(self, args):
        self.args = args
        self.board = Board(args.columns, args.rows, args.square_mm / 1000)
        self.size = (args.width // 2, args.height)
        self.condition = threading.Condition(threading.RLock())
        self.stop = threading.Event()
        self.packet = None
        self.sequence = 0
        self.received_at = 0
        self.decoded = None
        self.decoded_sequence = 0
        self.streams = {}
        self.stream_times = {}
        self.samples = []
        self.sample_sequences = []
        self.mode = 'focus'
        self.error = None
        self.message = '先查看原图，在实际工作距离调好两目焦点；再确认棋盘尺寸开始采集。'
        self.detection = dict(found=[False, False], sharpness=[None, None], motion_px=None)
        self.model = None
        self.maps = None
        self.report = None
        self.validation = None
        self.session_dir = None
        self.generation = 0
        self.capture_fps = 0
        self.preview_fps = 0
        self.detect_ms = 0

    def fail(self, exc):
        with self.condition:
            self.error = str(exc)
            self.message = str(exc)
            self.condition.notify_all()

    def capture_loop(self):
        camera = cv2.VideoCapture(self.args.device, cv2.CAP_V4L2)
        times = deque(maxlen=60)
        try:
            if not camera.isOpened():
                raise RuntimeError('无法打开相机；请停止深度程序并检查设备和 video 组权限')
            camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.args.width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.args.height)
            camera.set(cv2.CAP_PROP_FPS, 60)
            if not camera.set(cv2.CAP_PROP_CONVERT_RGB, 0):
                raise RuntimeError('相机后端不支持原始 MJPEG 采集')
            while not self.stop.is_set():
                ok, packet = camera.read()
                if not ok or packet is None or packet.size < 4:
                    raise RuntimeError('相机取帧失败；重新接好后重启标定程序')
                now = time.monotonic()
                times.append(now)
                with self.condition:
                    self.sequence += 1
                    self.received_at = now
                    self.packet = packet.reshape(-1).copy()
                    self.capture_fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0
                    self.streams['raw'] = (self.sequence, self.packet.tobytes())
                    self.stream_times['raw'] = now
                    self.condition.notify_all()
        except Exception as exc:
            self.fail(exc)
        finally:
            camera.release()

    def preview_loop(self):
        sequence = -1
        times = deque(maxlen=30)
        try:
            while not self.stop.is_set():
                began = time.monotonic()
                with self.condition:
                    ready = self.condition.wait_for(
                        lambda: (self.packet is not None and self.sequence != sequence) or self.stop.is_set(), 8 if sequence == -1 else 3)
                    if self.stop.is_set():
                        return
                    if not ready or self.packet is None:
                        raise RuntimeError('相机超过 3 秒未提供图像')
                    sequence, packet, received = self.sequence, self.packet, self.received_at
                    maps = self.maps
                image = cv2.imdecode(packet, cv2.IMREAD_COLOR)
                if image is None or image.shape[:2] != (self.args.height, self.args.width):
                    raise RuntimeError('实际 JPEG 尺寸与请求不一致；拒绝用错误尺寸标定')
                with self.condition:
                    self.decoded = (image, received)
                    self.decoded_sequence = sequence
                    self.condition.notify_all()
                # 原图 /raw 是设备 JPEG 原字节；下列流仅提供校正观察。
                if maps is not None:
                    w = self.size[0]
                    a = cv2.remap(image[:, :w], *maps[0], cv2.INTER_LINEAR)
                    b = cv2.remap(image[:, w:], *maps[1], cv2.INTER_LINEAR)
                    rectified = np.hstack((a, b))
                    for y in range(0, self.size[1], 60):
                        cv2.line(rectified, (0, y), (self.args.width - 1, y), (80, 220, 160), 1)
                    ok, jpg = cv2.imencode('.jpg', rectified, [cv2.IMWRITE_JPEG_QUALITY, 90])
                    if ok:
                        with self.condition:
                            self.streams['rectified'] = (sequence, jpg.tobytes())
                            self.stream_times['rectified'] = received
                times.append(time.monotonic())
                with self.condition:
                    self.preview_fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0
                    self.condition.notify_all()
                # 检测无需逐帧解码 60 FPS；仅取最新 JPEG，原图传输不依赖此线程。
                self.stop.wait(max(0, .125 - (time.monotonic() - began)))
        except Exception as exc:
            self.fail(exc)

    def checkpoint(self):
        """调用方持锁；只写紧凑角点，原子替换，避免半写文件。"""
        if self.session_dir is None:
            self.session_dir = ROOT / 'calibration' / ('live_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            self.session_dir.mkdir(parents=True, exist_ok=False)
        temp = self.session_dir / 'observations.tmp.npz'
        np.savez_compressed(temp, corners_a=np.array([s[0] for s in self.samples]),
                            corners_b=np.array([s[1] for s in self.samples]), image_size=self.size,
                            board=self.board.pattern, square_size_m=self.board.square_m,
                            sequences=self.sample_sequences)
        temp.replace(self.session_dir / 'observations.npz')

    def detection_loop(self):
        sequence, previous, stable_since, last_accept = -1, None, None, 0
        previous_time = None
        while not self.stop.is_set():
            try:
                with self.condition:
                    ready = self.condition.wait_for(
                        lambda: (self.decoded is not None and self.decoded_sequence != sequence) or self.stop.is_set(), 3)
                    if self.stop.is_set():
                        return
                    if not ready or self.decoded is None:
                        continue
                    sequence, (image, received) = self.decoded_sequence, self.decoded
                    generation, mode = self.generation, self.mode
                began = time.monotonic()
                w = self.size[0]
                grays = [cv2.cvtColor(eye, cv2.COLOR_BGR2GRAY) for eye in (image[:, :w], image[:, w:])]
                pair = [detect(gray, self.board, thorough=mode != 'focus') for gray in grays]
                found = [p is not None for p in pair]
                widths, reasons, motion = [None, None], [], None
                if all(found):
                    pair = align_pair(*pair, self.board)
                    widths, reasons = pair_quality(grays, pair, self.size, self.board)
                    if previous is not None and previous_time is not None and received - previous_time < 2:
                        motion = max(float(np.sqrt(np.mean((a - b)**2))) for a, b in zip(pair, previous))
                    if motion is None or motion > 2.5 or reasons:
                        stable_since = None
                    elif stable_since is None:
                        stable_since = received
                    previous, previous_time = pair, received
                else:
                    previous, previous_time, stable_since = None, None, None
                    reasons.append('请让完整棋盘同时进入 A、B 两目；保留白边，避免反光')
                preview = image.copy()
                for eye, pts in enumerate(pair):
                    if pts is not None:
                        shifted = pts.copy()
                        shifted[:, 0, 0] += eye * w
                        cv2.drawChessboardCorners(preview, self.board.pattern, shifted, True)
                ok, jpg = cv2.imencode('.jpg', preview, [cv2.IMWRITE_JPEG_QUALITY, 90])
                with self.condition:
                    if generation != self.generation:
                        previous, stable_since = None, None
                        continue
                    self.detect_ms = (time.monotonic() - began) * 1000
                    self.detection = dict(found=found, sharpness=widths, motion_px=motion,
                                          sequence=sequence, age_s=time.monotonic() - received)
                    if ok:
                        self.streams['detected'] = (sequence, jpg.tobytes())
                        self.stream_times['detected'] = received
                    if self.mode == 'collect' and mode == 'collect':
                        if reasons:
                            self.message = '；'.join(reasons)
                        elif time.monotonic() - received > 2:
                            self.message = '检测延迟过高，暂不收样；请保持棋盘清晰，避免后台高负载'
                        elif stable_since is None or received - stable_since < .7:
                            self.message = '保持棋盘静止约 1 秒，等待自动收样'
                        elif not is_novel(pair, self.samples, self.size, self.board):
                            self.message = '该姿态已采集。请改变位置、距离或倾斜角度'
                        elif received - last_accept >= 1.2:
                            self.samples.append(tuple(p.copy() for p in pair))
                            self.sample_sequences.append(sequence)
                            self.checkpoint()
                            last_accept = received
                            self.message = f'已收集 {len(self.samples)} 组；请移动到下一个姿态'
                            if len(self.samples) >= 60:
                                self.mode = 'paused'
                                self.message = '已收集 60 组并暂停，请检查覆盖情况后计算标定'
                    if self.model is not None and all(found):
                        errors = epipolar_errors(pair, self.model, self.size)
                        self.validation = dict(p95_px=float(np.percentile(errors, 95)),
                                               median_px=float(np.median(errors)), sequence=sequence,
                                               received_at=received)
                    elif self.model is not None:
                        self.validation = None
                    self.condition.notify_all()
            except Exception as exc:
                self.fail(exc)
                return
            self.stop.wait(.35 if mode == 'focus' else .08)

    def status(self):
        with self.condition:
            age = time.monotonic() - self.received_at if self.received_at else None
            return dict(mode=self.mode, error=self.error, message=self.message, sequence=self.sequence,
                        age_s=age, capture_fps=round(self.capture_fps, 1), decode_fps=round(self.preview_fps, 1),
                        detection_ms=round(self.detect_ms), detection=self.detection, validation=self.validation,
                        stream_fps_limit=self.args.stream_fps,
                        capture_size=[self.args.width, self.args.height], image_size=list(self.size),
                        board=list(self.board.pattern), square_mm=self.board.square_m * 1000,
                        coverage=coverage(self.samples, self.size, self.board), report=self.report,
                        session_dir=str(self.session_dir) if self.session_dir else None)

    def command(self, data):
        action = data.get('action')
        with self.condition:
            if self.mode == 'solving':
                raise ValueError('计算中，请等待完成')
            if action == 'reset':
                self.samples, self.sample_sequences = [], []
                self.model, self.maps, self.report, self.validation = None, None, None, None
                self.streams.pop('rectified', None)
                self.session_dir = None
                self.mode = 'focus'
                self.generation += 1
                self.message = '本轮已重置，已保存的角点记录仍保留。重新调焦后开始新一轮。'
            elif action == 'start':
                if self.error or self.received_at == 0 or time.monotonic() - self.received_at > 3:
                    raise ValueError('没有有效的实时相机图像')
                if self.model is not None:
                    raise ValueError('已有计算结果；若需重新收样，请重置本轮')
                if data.get('confirmed') is not True:
                    raise ValueError('请确认方格尺寸、平整度和焦点已固定')
                if len(self.samples) >= 60:
                    raise ValueError('已达到 60 组，请计算或重置本轮')
                self.mode = 'collect'
                self.generation += 1
                self.message = '自动采集中：先从中心开始，然后按覆盖指引移动棋盘'
            elif action == 'pause':
                self.mode = 'paused'
                self.message = '采集已暂停，已有角点保留；镜头位置不要改变'
            elif action == 'solve':
                if self.error:
                    raise ValueError('请先排除相机或检测错误')
                cov = coverage(self.samples, self.size, self.board)
                if not cov['ready']:
                    raise ValueError('；'.join(cov['missing']))
                self.mode = 'solving'
                self.message = '准备计算，视频继续显示；计算可能需要数分钟'
                threading.Thread(target=self.solve, name='calibration-solver', daemon=True).start()
            else:
                raise ValueError('未知操作')

    def solve(self):
        def progress(message):
            with self.condition:
                self.message = message
        try:
            with self.condition:
                samples = list(self.samples)
                directory = self.session_dir
            model, report = calibrate(samples, self.size, self.board, progress)
            maps = [cv2.initUndistortRectifyMap(model[f'K{i}'], model[f'D{i}'], model[f'R{i}'],
                                               model[f'P{i}'], self.size, cv2.CV_32FC1) for i in (1, 2)]
            report['created_at'] = datetime.now().isoformat()
            report['opencv_version'] = cv2.__version__
            np.savez_compressed(directory / 'candidate.npz', **model)
            (directory / 'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
            with self.condition:
                self.model, self.maps, self.report = model, maps, report
                self.mode = 'review'
                self.message = '候选标定已保存。切换校正视图，再摆放未采集的新姿态检查水平对齐。'
        except Exception as exc:
            with self.condition:
                self.mode = 'paused'
                self.message = f'计算失败，角点仍保留：{exc}'


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def send(self, value, content_type='application/json; charset=utf-8', status=200, headers=None):
        if not isinstance(value, bytes):
            value = json.dumps(value, ensure_ascii=False, allow_nan=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(value)))
        self.send_header('Cache-Control', 'no-store')
        for name, header_value in (headers or {}).items():
            self.send_header(name, str(header_value))
        self.end_headers()
        self.wfile.write(value)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == '/':
            return self.send((ROOT / 'calibration.html').read_bytes(), 'text/html; charset=utf-8')
        if path == '/status':
            return self.send(self.server.app.status())
        if path.startswith('/frame/') and path.endswith('.jpg'):
            kind = path[len('/frame/'):-4]
            if kind not in ('raw', 'detected', 'rectified'):
                return self.send({'error': 'Unknown view'}, status=404)
            app = self.server.app
            with app.condition:
                sequence, jpeg = app.streams.get(kind, (-1, None))
                received = app.stream_times.get(kind, 0)
                error = app.error
            if jpeg is None or error:
                return self.send({'error': error or 'Waiting for frame'}, status=503)
            return self.send(jpeg, 'image/jpeg', headers={
                'X-Frame-Sequence': sequence,
                'X-Source-Age-Ms': round((time.monotonic() - received) * 1000, 1)})
        if path.startswith('/stream/') and path.split('/')[-1] in ('raw', 'detected', 'rectified'):
            return self.stream(path.split('/')[-1])
        if path == '/report.json':
            report = self.server.app.status()['report']
            return self.send(report or {'error': '尚未计算标定'}, status=200 if report else 404)
        self.send({'error': 'Not found'}, status=404)

    def do_POST(self):
        # 浏览器只允许本站 JSON 操作，不接受跨站表单误触。
        if self.headers.get('Origin') and urlparse(self.headers['Origin']).netloc != self.headers.get('Host'):
            return self.send({'error': 'Origin mismatch'}, status=403)
        if self.path != '/command' or self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
            return self.send({'error': 'Expected /command JSON'}, status=400)
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 4096:
                raise ValueError('请求大小不合法')
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict):
                raise ValueError('请求必须为对象')
            self.server.app.command(data)
            self.send({'ok': True})
        except (ValueError, TypeError) as exc:
            self.send({'error': str(exc)}, status=400)

    def stream(self, kind):
        app, sequence, sent_at = self.server.app, -1, 0
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.connection.settimeout(5)
        try:
            while not app.stop.is_set():
                with app.condition:
                    app.condition.wait_for(lambda: app.streams.get(kind, (-1,))[0] != sequence
                                           or app.error or app.stop.is_set(), 2)
                    if app.error or app.stop.is_set():
                        return
                    current, jpeg = app.streams.get(kind, (-1, None))
                if jpeg is None or current == sequence:
                    continue
                # 网络预览限速，设备仍持续采集；不把 60 FPS 宣称为浏览器帧率。
                app.stop.wait(max(0, 1 / self.server.app.args.stream_fps - (time.monotonic() - sent_at)))
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                 + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                self.wfile.flush()
                sequence, sent_at = current, time.monotonic()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--port', type=int, default=8082)
    parser.add_argument('--width', type=int, default=2560, help='左右拼接的总宽度')
    parser.add_argument('--height', type=int, default=960)
    parser.add_argument('--columns', type=int, default=11, help='每行内角点数')
    parser.add_argument('--rows', type=int, default=8, help='内角点行数')
    parser.add_argument('--square-mm', type=float, default=20)
    parser.add_argument('--stream-fps', type=float, default=15, help='浏览器预览上限，不改变相机请求帧率')
    args = parser.parse_args()
    if (args.width < 640 or args.width % 2 or args.height < 240 or args.height % 2
            or not 1 <= args.port <= 65535
            or not 1 <= args.stream_fps <= 60 or not 0 < args.square_mm < 1000
            or not 3 <= args.rows <= 20 or not 3 <= args.columns <= 20 or args.rows == args.columns):
        parser.error('尺寸、端口或棋盘参数不合法；使用非正方形内角点阵列')
    return args


def main():
    args = parse_args()
    cv2.setNumThreads(2)
    app = CalibrationApp(args)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    server.app = app
    workers = [threading.Thread(target=f, daemon=True) for f in
               (app.capture_loop, app.preview_loop, app.detection_loop)]
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    try:
        for worker in workers:
            worker.start()
        print(f'标定预览 http://127.0.0.1:{args.port} ; Ctrl+C 停止', flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        app.stop.set()
        with app.condition:
            app.condition.notify_all()
        server.server_close()
        for worker in workers:
            worker.join(timeout=3)


if __name__ == '__main__':
    main()
