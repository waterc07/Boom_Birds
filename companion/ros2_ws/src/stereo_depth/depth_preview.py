"""树莓派 USB 双目深度与 XYZ 预览。

阅读顺序：Config → StereoProcessor → PreviewApp → RequestHandler → main。
采集线程只保留最新 JPEG，计算线程处理每个选中的新帧，HTTP 线程提供预览与保存。
运行方式及坐标约定见 README.md；导入本模块不会打开相机或启动服务。
"""

from collections import deque
from dataclasses import dataclass
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import argparse
import json
import os
import signal
import threading
import time

import cv2
import numpy as np

def _resolve_root():
    """标定与输出目录的根。

    默认是当前文件所在目录（仓库开发布局，行为与既有版本一致）。
    安装到 ROS 2 share 目录后，源码目录不可写也不再位于仓库内，此时回退到
    ament share 路径，保证已安装包不依赖当前工作目录。
    """
    here = Path(__file__).resolve().parent
    if (here / "calibration").is_dir():
        return here
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("stereo_depth"))
    except Exception:
        return here


ROOT = _resolve_root()
DEPTH_SIZE = (320, 240)    # 每目计算分辨率，不能仅改此值而忽略标定尺度。
PREVIEW_RANGE_M = (0.15, 3.0)


@dataclass(frozen=True)
class Config:
    """集中定义运行参数；分辨率固定，避免误用不匹配的标定。"""

    device: str = "/dev/video0"
    port: int = 8081
    threads: int = 4
    reduced_decode: bool = True
    calibration: Path = ROOT / "calibration/live_20260916_210120_642136/candidate.npz"
    output_dir: Path = ROOT / "depth_outputs"


@dataclass(frozen=True)
class CapturedFrame:
    sequence: int
    received_at: float  # 主机完成取帧的单调时钟时间，不是传感器曝光时间。
    packet: np.ndarray


@dataclass(frozen=True)
class DepthFrame:
    """发布后不再修改内部数组，HTTP 保存时可安全读取同一帧的完整快照。"""

    source: CapturedFrame
    rectified: np.ndarray
    disparity: np.ndarray
    xyz: np.ndarray
    valid: np.ndarray
    preview: np.ndarray
    timings: dict

    @property
    def depth(self):
        return self.xyz[:, :, 2]


class StereoProcessor:
    """只负责几何与图像运算，可用已保存的 JPEG 离线调用，不依赖相机和 HTTP。"""

    def __init__(self, config):
        self.config = config
        if not config.calibration.is_file():
            raise FileNotFoundError(
                f"标定文件不存在：{config.calibration}；请从树莓派取回标定目录、重新标定，"
                "或用 --calibration 指定已有的 candidate.npz")
        with np.load(config.calibration) as archive:
            calibration = {key: archive[key].copy() for key in archive.files}
        image_size = tuple(int(v) for v in calibration["image_size"])
        if len(image_size) != 2 or min(image_size) < 240 or any(v % 2 for v in image_size):
            raise ValueError("标定图像尺寸必须为有效偶数像素尺寸")
        self.capture_size = (image_size[0] * 2, image_size[1])
        self.baseline_m = float(np.linalg.norm(calibration["T"]))
        if not np.isfinite(self.baseline_m) or not .001 < self.baseline_m < 1:
            raise ValueError("标定基线必须以米为单位且处于合理范围")
        translation = calibration["T"].reshape(3)
        if translation[0] >= 0 or abs(translation[1]) > abs(translation[0]) * .2:
            raise ValueError("标定不符合 A→B 正视差水平双目，请检查左右顺序")

        # 按标定的原始每目尺寸缩放内参；采集也使用该模式，避免假定不同模式同视场。
        # T 的单位仍是米，不能随图像尺寸一起缩放。
        k_a = calibration["K1"].copy()
        k_b = calibration["K2"].copy()
        for k in (k_a, k_b):
            k[0, :] *= DEPTH_SIZE[0] / image_size[0]
            k[1, :] *= DEPTH_SIZE[1] / image_size[1]
        r_a, r_b, p_a, p_b, self.q, _, _ = cv2.stereoRectify(
            k_a, calibration["D1"], k_b, calibration["D2"], DEPTH_SIZE,
            calibration["R"], calibration["T"],
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
        )
        # 保留校正结果：R1 是「校正后坐标 → 原图坐标」的旋转，用于把相机位姿
        # 从原始光学系变换到校正光学系；P1 是 DEPTH_SIZE 尺度下、alpha=0 的重投影内参，
        # ROS 侧据此发布 CameraInfo，保证地图内参与深度计算同源。
        self.r1, self.r2 = r_a, r_b
        self.p1, self.p2 = p_a, p_b
        self.map_a = cv2.initUndistortRectifyMap(
            k_a, calibration["D1"], r_a, p_a, DEPTH_SIZE, cv2.CV_32FC1)
        self.map_b = cv2.initUndistortRectifyMap(
            k_b, calibration["D2"], r_b, p_b, DEPTH_SIZE, cv2.CV_32FC1)
        self.num_disparities = 96  # 必须为 16 的倍数；改变它会影响最近可测距离。
        parameters = dict(
            numDisparities=self.num_disparities, blockSize=5,
            P1=8 * 25, P2=32 * 25, disp12MaxDiff=1,
            uniquenessRatio=12, speckleWindowSize=80, speckleRange=2,
            preFilterCap=31, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        self.matcher_a = cv2.StereoSGBM_create(minDisparity=0, **parameters)
        self.matcher_b = cv2.StereoSGBM_create(
            minDisparity=1 - self.num_disparities, **parameters)
        self.rows, self.cols = np.indices(DEPTH_SIZE[::-1], dtype=np.int32)

    def rectify(self, packet):
        """半尺寸 JPEG 解码可省去完整解码；完整解码选项用于对照。"""
        mode = cv2.IMREAD_REDUCED_COLOR_2 if self.config.reduced_decode else cv2.IMREAD_COLOR
        image = cv2.imdecode(packet.reshape(-1), mode)
        divisor = 2 if self.config.reduced_decode else 1
        expected = (self.capture_size[1] // divisor, self.capture_size[0] // divisor)
        if image is None or image.shape[:2] != expected:
            raise RuntimeError("JPEG 尺寸不符合预期，请检查相机输出模式")
        return self._rectify_resized(image)

    def rectify_image(self, image):
        """对已解码的 BGR 拼接图做校正；供 ROS 2 节点复用同一套几何运算。

        入参 image 是完整的左右拼接 BGR 数组（宽 = 2 × 每目宽），不是单个目。
        与 rectify() 的唯一区别是本方法不经过 JPEG 解码，尺寸/缩放/校正路径完全一致，
        避免 ROS 侧为复用而重新编码 JPEG。
        """
        if image is None or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("rectify_image 需要已解码的 BGR 拼接图（H×W×3）")
        return self._rectify_resized(image)

    def _rectify_resized(self, image):
        """把任意尺寸的拼接 BGR 图缩放到 DEPTH_SIZE 并做极线校正，返回 (a, b)。"""
        if image.shape[:2] != (DEPTH_SIZE[1], DEPTH_SIZE[0] * 2):
            image = cv2.resize(image, (640, 240), interpolation=cv2.INTER_AREA)
        a = cv2.remap(image[:, :320], *self.map_a, cv2.INTER_LINEAR)
        b = cv2.remap(image[:, 320:], *self.map_b, cv2.INTER_LINEAR)
        return a, b

    def reconstruct(self, disparity_a, disparity_b):
        """执行左右一致性检查，再输出校正 A 目光学坐标系的 XYZ（米）。"""
        # A 目横坐标 u 对应 B 目 u-d；B→A 视差符号与 A→B 相反。
        target_cols = np.rint(self.cols - disparity_a).astype(int)
        inside = (target_cols >= 0) & (target_cols < DEPTH_SIZE[0])
        sampled_b = disparity_b[self.rows, np.clip(target_cols, 0, DEPTH_SIZE[0] - 1)]
        xyz = cv2.reprojectImageTo3D(disparity_a, self.q)
        valid = (
            inside & (disparity_a > 0) & (disparity_a < self.num_disparities - 1)
            & (sampled_b > -self.num_disparities)
            & (np.abs(disparity_a + sampled_b) <= 1)
            & np.isfinite(xyz).all(axis=2) & (xyz[:, :, 2] > 0)
        )
        xyz[~valid] = np.nan  # 三个通道同时无效，不能把失败匹配解释为零距离。
        return xyz, valid

    def process(self, source):
        start = time.monotonic()
        a, b = self.rectify(source.packet)
        gray_a = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY)
        gray_b = cv2.cvtColor(b, cv2.COLOR_BGR2GRAY)
        prepared = time.monotonic()
        # OpenCV SGBM 返回 16 倍定点视差，转 XYZ 前必须恢复像素单位。
        disparity_a = self.matcher_a.compute(gray_a, gray_b).astype(np.float32) / 16
        disparity_b = self.matcher_b.compute(gray_b, gray_a).astype(np.float32) / 16
        matched = time.monotonic()
        xyz, valid = self.reconstruct(disparity_a, disparity_b)
        near, far = PREVIEW_RANGE_M
        normalized = np.uint8(np.clip(
            (np.nan_to_num(xyz[:, :, 2], nan=near) - near) / (far - near), 0, 1) * 255)
        color = cv2.applyColorMap(255 - normalized, cv2.COLORMAP_TURBO)
        color[~valid] = 0
        preview = np.hstack([a, color])  # 不在树莓派上放大，交给浏览器显示。
        end = time.monotonic()
        timings = {
            "prep_ms": (prepared - start) * 1000,
            "match_ms": (matched - prepared) * 1000,
            "post_ms": (end - matched) * 1000,
            "compute_ms": (end - start) * 1000,
        }
        return DepthFrame(source, a, disparity_a, xyz, valid, preview, timings)


def save_frame(frame, processor):
    """保存同一结果帧及其原始 JPEG；metadata 最后写入，标志保存步骤完成。"""
    folder = processor.config.output_dir / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    folder.mkdir(parents=True, exist_ok=False)
    raw = cv2.imdecode(frame.source.packet.reshape(-1), cv2.IMREAD_COLOR)
    if raw is None:
        raise RuntimeError("原始 JPEG 解码失败")
    (folder / "raw.jpg").write_bytes(frame.source.packet.tobytes())
    for name, image in {
        "raw": raw, "rectified": frame.rectified, "preview": frame.preview,
        "valid": frame.valid.astype(np.uint8) * 255,
    }.items():
        if not cv2.imwrite(str(folder / f"{name}.png"), image):
            raise RuntimeError(f"写入 {name}.png 失败")
    for name, array in {
        "xyz_m": frame.xyz, "depth_m": frame.depth,
        "disparity_px": frame.disparity, "Q": processor.q,
    }.items():
        np.save(folder / f"{name}.npy", array)
    metadata = {
        "calibration": str(processor.config.calibration),
        "status": "EXPERIMENTAL_NOT_DISTANCE_VALIDATED",
        "xyz_unit": "m", "xyz_shape": list(frame.xyz.shape),
        "xyz_frame": "rectified A camera optical frame: X right, Y down, Z forward",
        "xyz_invalid": "NaN in all three coordinates",
        "depth_unit": "m", "invalid": "NaN",
        "depth_definition": "rectified camera optical-axis Z",
        "input": "direct USB MJPEG capture", "depth_size": list(DEPTH_SIZE),
        "capture_size": list(processor.capture_size),
        "baseline_m": processor.baseline_m,
        "reduced_decode": processor.config.reduced_decode,
        "valid_fraction": float(frame.valid.mean()),
        "fetch_compute_seconds": frame.timings["compute_ms"] / 1000,
        "timings_ms": frame.timings, "preview_range_m": list(PREVIEW_RANGE_M),
        "capture_sequence": frame.source.sequence,
        "received_monotonic_s": frame.source.received_at,
        "timestamp_note": "host receive/save times are not sensor exposure timestamps",
    }
    (folder / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    return folder


class PreviewApp:
    """管理线程共享状态；锁内只交换引用，耗时的计算、编码和磁盘写入放在锁外。"""

    def __init__(self, config):
        self.config = config
        self.processor = StereoProcessor(config)
        self.condition = threading.Condition()
        self.stopping = threading.Event()
        self.latest_source = None
        self.latest_result = None
        self.jpeg = None
        self.error = None
        self.sequence = 0
        self.metrics = {}

    def fail(self, exception):
        with self.condition:
            self.error = str(exception)
            self.condition.notify_all()
        print(f"ERROR: {exception}", flush=True)

    def capture_loop(self):
        camera = cv2.VideoCapture(self.config.device, cv2.CAP_V4L2)
        try:
            if not camera.isOpened():
                raise RuntimeError("无法打开相机，请检查 video 组权限及设备占用")
            camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.processor.capture_size[0])
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.processor.capture_size[1])
            camera.set(cv2.CAP_PROP_FPS, 60)
            if not camera.set(cv2.CAP_PROP_CONVERT_RGB, 0):
                raise RuntimeError("相机后端不支持原始 MJPEG 采集")
            sequence = 0
            while not self.stopping.is_set():
                ok, packet = camera.read()
                if not ok or packet.size < 4:
                    raise RuntimeError("相机取帧失败")
                sequence += 1
                source = CapturedFrame(sequence, time.monotonic(), packet.copy())
                with self.condition:
                    # 不排队：处理速度低于相机时，覆盖尚未处理的旧帧以降低积压。
                    self.latest_source = source
                    self.condition.notify_all()
        except Exception as exception:
            self.fail(exception)
        finally:
            camera.release()

    def processing_loop(self):
        last_sequence = 0
        output_times = deque(maxlen=30)
        try:
            while not self.stopping.is_set():
                with self.condition:
                    ready = self.condition.wait_for(
                        lambda: self.stopping.is_set() or self.error is not None
                        or (self.latest_source is not None
                            and self.latest_source.sequence != last_sequence), timeout=2)
                    if self.stopping.is_set() or self.error is not None:
                        return
                    if not ready:
                        raise RuntimeError("相机超过 2 秒未提供新帧")
                    source = self.latest_source
                last_sequence = source.sequence
                frame = self.processor.process(source)
                output_times.append(time.monotonic())
                fps = ((len(output_times) - 1) / (output_times[-1] - output_times[0])
                       if len(output_times) > 1 else 0.0)
                # FPS 是深度生成速率；计时包含相邻结果之间的编码/调度，不是显示器 FPS。
                encode_start = time.monotonic()
                label = (f"FPS {fps:.1f} | 320x240 | valid {frame.valid.mean()*100:.1f}%"
                         f" | compute {frame.timings['compute_ms']:.0f} ms")
                cv2.putText(frame.preview, label, (12, 26), cv2.FONT_HERSHEY_SIMPLEX,
                            .38, (0, 255, 255), 1, cv2.LINE_AA)
                ok, encoded = cv2.imencode(".jpg", frame.preview, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if not ok:
                    raise RuntimeError("预览 JPEG 编码失败")
                metrics = {key: round(value, 2) for key, value in frame.timings.items()}
                metrics.update(fps=round(fps, 2),
                               encode_ms=round((time.monotonic() - encode_start) * 1000, 2))
                with self.condition:
                    self.latest_result = frame
                    self.jpeg = encoded.tobytes()
                    self.metrics = metrics
                    self.sequence += 1
                    self.condition.notify_all()
        except Exception as exception:
            self.fail(exception)

    def health(self):
        with self.condition:
            return dict(sequence=self.sequence, error=self.error,
                        reduced_decode=self.config.reduced_decode,
                        calibration=str(self.config.calibration),
                        baseline_m=self.processor.baseline_m,
                        capture_size=list(self.processor.capture_size), **self.metrics)


PAGE = """<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>双目 XYZ 与深度预览</title>
<style>body{background:#141820;color:#eee;font:18px sans-serif;margin:24px}
img{width:100%;max-width:1280px}button{padding:12px;font-size:18px}p{line-height:1.6}</style>
<h2>双目 XYZ 与深度试验预览</h2>
<p>基线及采集尺寸由所选标定决定 · XYZ 与深度 320×240 · 不限输出帧率。左：校正 A 目；右：深度。</p>
<p>近暖远冷，色标 0.15–3 m，超界饱和；黑色无效。距离尚未独立验证。</p>
<img src="/stream" alt="校正图像与深度预览">
<p><button onclick="save()">保存 XYZ、深度与原图</button></p>
<p id="status">将有纹理的物体放在约 0.3–2 m 处。黑区不代表没有障碍物。</p>
<script>async function save(){try{const r=await fetch('/capture',{method:'POST'});
const j=await r.json();document.getElementById('status').textContent=j.path||j.error;}
catch(e){document.getElementById('status').textContent=String(e);}}</script></html>"""


class RequestHandler(BaseHTTPRequestHandler):
    """只处理 HTTP；相机和匹配算法不在请求线程中运行。"""

    @property
    def app(self):
        return self.server.app

    def send_bytes(self, data, content_type="application/json", status=200):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def send_json(self, value, status=200):
        self.send_bytes(json.dumps(value).encode(), status=status)

    def do_GET(self):
        if self.path == "/":
            self.send_bytes(PAGE.encode(), "text/html; charset=utf-8")
        elif self.path == "/health":
            self.send_json(self.app.health())
        elif self.path == "/frame.jpg":
            with self.app.condition:
                jpeg = None if self.app.error else self.app.jpeg
            self.send_bytes(jpeg or b"No live frame", "image/jpeg" if jpeg else "text/plain",
                            status=200 if jpeg else 503)
        elif self.path == "/stream":
            self.stream_frames()
        else:
            self.send_bytes(b"Not found", "text/plain", status=404)

    def stream_frames(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        sequence = -1
        try:
            while not self.app.stopping.is_set():
                with self.app.condition:
                    self.app.condition.wait_for(
                        lambda: self.app.sequence != sequence or self.app.error is not None
                        or self.app.stopping.is_set(), timeout=5)
                    if self.app.error is not None or self.app.stopping.is_set():
                        return
                    jpeg, current = self.app.jpeg, self.app.sequence
                if jpeg is None or current == sequence:
                    sequence = current
                    continue
                sequence = current
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass  # 用户关闭或刷新页面，不是相机故障。

    def do_POST(self):
        if self.path != "/capture":
            return self.send_json({"error": "Not found"}, status=404)
        with self.app.condition:
            frame = None if self.app.error else self.app.latest_result
        if frame is None:
            return self.send_json({"error": "No live depth"}, status=503)
        try:
            folder = save_frame(frame, self.app.processor)
            self.send_json({"path": str(folder)})
        except Exception as exception:
            self.send_json({"error": str(exception)}, status=500)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--threads", type=int, choices=range(1, 5), default=4)
    parser.add_argument("--full-decode", action="store_true", help="完整 JPEG 解码后缩小，用于对照")
    parser.add_argument("--calibration", type=Path, default=Config.calibration,
                        help="标定 NPZ；采集尺寸自动匹配标定的 image_size")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("端口必须在 1–65535 范围内")
    return Config(device=args.device, port=args.port, threads=args.threads,
                  calibration=args.calibration,
                  reduced_decode=not (args.full_decode or os.getenv("STEREO_FULL_DECODE") == "1"))


def main():
    config = parse_args()
    cv2.setNumThreads(config.threads)
    app = PreviewApp(config)
    # 先绑定端口再打开相机，重复启动失败时不会抢占已有相机。
    server = ThreadingHTTPServer(("127.0.0.1", config.port), RequestHandler)
    server.app = app
    workers = [threading.Thread(target=app.capture_loop, name="camera", daemon=True),
               threading.Thread(target=app.processing_loop, name="depth", daemon=True)]

    def handle_stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, handle_stop)
    try:
        for worker in workers:
            worker.start()
        print(f"Preview: http://127.0.0.1:{config.port} ; Ctrl+C 停止", flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        app.stopping.set()
        with app.condition:
            app.condition.notify_all()
        server.server_close()
        for worker in workers:
            worker.join(timeout=2)


if __name__ == "__main__":
    main()
