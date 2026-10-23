"""ROS 2 只订阅采集器；浏览器提示动作，不创建飞控服务或控制发布器。"""

from __future__ import annotations

import json
import math
import queue
import shutil
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

from .vio_calibration import DatasetWriter, atomicJson, phaseAt

PAGE = """<!doctype html><html lang="zh"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>相机—IMU标定采集</title>
<style>body{max-width:900px;margin:24px auto;padding:0 16px;font:17px system-ui;background:#f5f5f2;color:#202824}
img{width:100%;max-width:640px;background:#ddd}pre{white-space:pre-wrap}button{padding:12px 20px;margin:12px 8px 12px 0}#phase{font-size:24px}small{display:block}</style>
<h1>棋盘格＋飞控 IMU</h1><p>11×8 内角点 · 20 mm 方格。固定棋盘格，移动整机。全程未解锁。</p>
<img id="preview" alt="等待左目原图；预览与保存图像方向一致">
<p id="phase">等待数据</p><p id="instruction"></p>
<button id="start" onclick="act('start')" disabled>开始动作采集</button>
<button onclick="act('stop')">停止并保存</button>
<small>传感器、棋盘格及同步检查通过后才能开始；USB供电会话不检查电池。运动约70秒；结束后自动保存。此页不提供解锁、电机或飞行控制。</small>
<pre id="details"></pre><script>
async function act(name){await fetch('/'+name,{method:'POST'});}
async function tick(){try{let s=await(await fetch('/status')).json();
document.getElementById('phase').textContent=s.state+' · '+(s.phase?.name||'')+' '+(s.phase?.remaining_s??'');
document.getElementById('instruction').textContent=s.error||s.blockers?.join('；')||s.phase?.instruction||'';
document.getElementById('start').disabled=s.state!=='READY';
document.getElementById('details').textContent=JSON.stringify(s,null,2);
document.getElementById('preview').src='/preview.jpg?t='+Date.now();}catch(e){document.getElementById('instruction').textContent='连接中断，采集状态未知，请勿继续动作。';document.getElementById('start').disabled=true;}}
setInterval(tick,1000);tick();</script></html>"""


def detectBoard(gray, columns, rows):
    """在检测线程生成预览，原始数据由另一线程写盘。"""
    cv2.setNumThreads(1)
    found, corners = cv2.findChessboardCornersSB(
        gray, (columns, rows), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    view = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if found:
        cv2.drawChessboardCorners(view, (columns, rows), corners, True)
    ok, jpeg = cv2.imencode(".jpg", view)
    return bool(found), jpeg.tobytes() if ok else None


def runRecord(session, bind="127.0.0.1", port=8084):
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, Imu, BatteryState
    from mavros_msgs.msg import State
    from std_msgs.msg import String

    session = Path(session).resolve()
    manifest = json.loads((session / "session.json").read_text())
    if (session / "capture.sqlite3").exists():
        raise ValueError("会话已采集，请 prepare 新目录")
    checkBattery = manifest.get("battery_check_required", True)
    if bind != "127.0.0.1":
        raise ValueError("界面只绑定127.0.0.1，通过SSH转发访问")
    # 棋盘检测与写盘和 ROS 回调共用 CPU，避免 OpenCV 默认线程池抢占回调。
    cv2.setNumThreads(1)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    class Recorder(Node):
        def __init__(self):
            super().__init__("boom_birds_vio_calibration")
            self.bridge = CvBridge()
            self.jobs = queue.Queue(maxsize=2048)
            self.samples = {
                name: deque(maxlen=1000) for name in ("cam0", "cam1", "imu")
            }
            self.connected = False
            self.stateAt = self.batteryAt = self.gateAt = 0.0
            self.battery = None
            self.gate = {}
            self.board = dict(found=False, checked_mono=0.0)
            self.shared = dict(state="WAITING", blockers=["等待传感器"], phase=None)
            self.preview = b""
            self.started = None
            self.beginRequest = False
            self.stopRequest = False
            self.error = None
            self.done = False
            self.lastTick = time.monotonic()
            self.max_tick_gap_s = 0.0
            self.workerDone = threading.Event()
            self.worker = threading.Thread(target=self.work, daemon=True)
            self.worker.start()
            self.previewJob = None
            self.previewDone = threading.Event()
            self.previewWorker = threading.Thread(target=self.detect, daemon=True)
            self.previewWorker.start()
            self.qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.subs = [
                self.create_subscription(
                    Imu, manifest["imu_topic"], self.onImu, self.qos
                ),
                self.create_subscription(
                    State, "/mavros/state", self.onState, self.qos
                ),
                self.create_subscription(
                    BatteryState, "/mavros/battery", self.onBattery, self.qos
                ),
                self.create_subscription(
                    String, "/boom_birds/imu/mavlink_status", self.onGate, self.qos
                ),
            ]
            for i, topic in enumerate(manifest["image_topics"]):
                self.subs.append(
                    self.create_subscription(
                        Image, topic, lambda m, i=i: self.onImage(i, m), self.qos
                    )
                )
            self.create_timer(0.2, self.tick)

        def push(self, item):
            try:
                self.jobs.put_nowait(item)
            except queue.Full:
                self.fail("写盘队列已满，采集停止；不会静默丢帧")

        def fail(self, reason):
            if self.error is None:
                self.error = reason
                self.shared = dict(self.shared, state="ABORTED", blockers=[reason])

        def stamp(self, msg, key):
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            now = self.get_clock().now().nanoseconds
            history = self.samples[key]
            if stamp <= 0 or (history and stamp <= history[-1][1]):
                if self.started is not None:
                    self.fail(key + " 时间戳回退或重复")
                return None
            if not -1_000_000 <= now - stamp <= 500_000_000:
                if self.started is not None:
                    self.fail(f"{key} 采样时间过期或超前：age={(now - stamp) * 1e-9:.6f}s")
                return None
            history.append((time.monotonic(), stamp))
            return stamp, now

        def onImage(self, i, msg):
            stamps = self.stamp(msg, f"cam{i}")
            if stamps is None:
                return
            if (msg.width, msg.height) != (manifest["width"], manifest["height"]):
                self.fail("图像尺寸与会话内参不一致")
                return
            try:
                gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8").copy()
            except Exception as exc:
                self.fail("图像解码失败：" + str(exc))
                return
            if i == 0:
                # 检测仅保留最新帧；原始采集仍逐帧入写盘队列。
                self.previewJob = (stamps[1], gray)
            if self.started is not None:
                self.push(("image", i, *stamps, gray))

        def onImu(self, msg):
            stamps = self.stamp(msg, "imu")
            if stamps is None:
                return
            values = tuple(
                float(getattr(v, k))
                for v in (msg.angular_velocity, msg.linear_acceleration)
                for k in ("x", "y", "z")
            )
            if (
                not np.isfinite(values).all()
                or msg.angular_velocity_covariance[0] < 0
                or msg.linear_acceleration_covariance[0] < 0
            ):
                self.fail("IMU 数值或测量有效性非法")
                return
            if self.started is not None:
                self.push(("imu", *stamps, values))

        def onState(self, msg):
            self.stateAt = time.monotonic()
            age = self.get_clock().now().nanoseconds - (
                msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            )
            self.connected = bool(msg.connected) and 0 <= age <= 3_000_000_000
            if msg.armed:
                self.fail("观察到飞控已解锁，立即停止采集")

        def onBattery(self, msg):
            self.batteryAt = time.monotonic()
            self.battery = float(msg.percentage)
            if checkBattery and math.isfinite(self.battery) and 0 <= self.battery <= 0.2:
                self.fail("飞控上报电量低于或等于20%")

        def onGate(self, msg):
            try:
                self.gate = json.loads(msg.data)
                self.gateAt = time.monotonic()
            except ValueError:
                pass

        def blockers(self):
            now = time.monotonic()
            reasons = []
            if not self.connected or now - self.stateAt > 3:
                reasons.append("飞控连接/未解锁状态未确认")
            if checkBattery and (
                self.battery is None
                or not math.isfinite(self.battery)
                or not 0.2 < self.battery <= 1
                or now - self.batteryAt > 10
            ):
                reasons.append("等待新鲜且高于20%的电量数据")
            syncAge = self.gate.get("last_good_sync_age_s")
            if (
                now - self.gateAt > 2
                or self.gate.get("backend") != "mavros"
                or not self.gate.get("connected")
                or type(syncAge) not in (int, float)
                or not math.isfinite(syncAge)
                or not 0 <= syncAge + (now - self.gateAt) <= 1
            ):
                reasons.append("IMU 时间映射未就绪或过期")
            rates = {}
            for key, history in self.samples.items():
                recent = [(t, s) for t, s in history if now - t < 5]
                rate = (
                    (len(recent) - 1) * 1e9 / (recent[-1][1] - recent[0][1])
                    if len(recent) > 1 and recent[-1][1] > recent[0][1]
                    else 0.0
                )
                rates[key] = round(rate, 2)
                minimum = manifest["gates"][
                    "min_imu_hz" if key == "imu" else "min_image_hz"
                ]
                if len(recent) < 5 or now - recent[-1][0] > 0.5 or rate < minimum:
                    reasons.append(key + " 数据未到或频率不足")
            if not self.board["found"] or now - self.board["checked_mono"] > 2:
                reasons.append("左目未识别到完整11×8棋盘格")
            if shutil.disk_usage(session).free < 2 * 1024**3:
                reasons.append("可用磁盘不足2 GiB")
            return reasons, rates

        def tick(self):
            if self.done:
                return
            reasons, rates = self.blockers()
            now = time.monotonic()
            self.max_tick_gap_s = max(self.max_tick_gap_s, now - self.lastTick)
            self.lastTick = now
            timing = dict(max_tick_gap_s=round(self.max_tick_gap_s, 4),
                stream_receive_age_s={k: round(now - v[-1][0], 4) if v else None
                    for k, v in self.samples.items()}, write_queue=self.jobs.qsize())
            # 运动中棋盘短暂丢失保留原始数据，最终由识别比例筛查；通信失效立即中止。
            critical = [r for r in reasons if "棋盘格" not in r]
            if self.started is not None and critical:
                self.fail("；".join(critical))
            if self.error or self.stopRequest:
                self.done = True
                self.shared = dict(
                    self.shared, state="ABORTED", blockers=[self.error or "用户停止"], timing=timing
                )
                return
            if self.beginRequest and self.started is None:
                self.beginRequest = False
                if not reasons:
                    self.started = now
                    self.push(("begin", self.get_clock().now().nanoseconds, manifest))
            if self.started is None:
                self.shared = dict(
                    state="READY" if not reasons else "WAITING",
                    blockers=reasons,
                    power_source=manifest.get("power_source", "battery"),
                    battery_check_required=checkBattery,
                    rates=rates,
                    timing=timing,
                    battery_percent=(
                        round(self.battery * 100, 1)
                        if self.battery is not None
                        and math.isfinite(self.battery)
                        and 0 <= self.battery <= 1
                        else None
                    ),
                    board=self.board,
                    phase=None,
                )
            else:
                elapsed = now - self.started
                phase = phaseAt(elapsed, manifest["phases"])
                self.shared = dict(
                    state="RECORDING",
                    blockers=reasons,
                    power_source=manifest.get("power_source", "battery"),
                    battery_check_required=checkBattery,
                    rates=rates,
                    timing=timing,
                    battery_percent=(round(self.battery * 100, 1)
                        if self.battery is not None and math.isfinite(self.battery)
                        and 0 <= self.battery <= 1 else None),
                    board=self.board,
                    phase=phase,
                )
                if phase["name"] == "完成":
                    self.done = True
                    self.shared["state"] = "SAVING"
            # fsync 可能阻塞 SD 卡，不能在 ROS 回调中执行。
            self.push(("status", dict(self.shared)))

        def detect(self):
            while not self.previewDone.is_set():
                job = self.previewJob
                if job is not None:
                    received, gray = job
                    try:
                        board = manifest["board"]
                        found, jpeg = detectBoard(gray,
                            board["columns"], board["rows"])
                        self.board = dict(found=found, checked_mono=time.monotonic())
                        if self.started is not None and not self.done:
                            self.push(("board", received, found))
                        if jpeg:
                            self.preview = jpeg
                    except Exception as exc:
                        self.fail("棋盘检测异常：" + str(exc))
                self.previewDone.wait(.5)

        def work(self):
            writer = None
            lastCommit = 0.0
            try:
                while not self.workerDone.is_set() or not self.jobs.empty():
                    try:
                        item = self.jobs.get(timeout=0.1)
                    except queue.Empty:
                        if writer and time.monotonic() - lastCommit > 0.25:
                            writer.commit()
                            lastCommit = time.monotonic()
                        continue
                    kind, *data = item
                    if kind == "status":
                        atomicJson(session / "status.json", data[0])
                    elif kind == "begin":
                        writer = DatasetWriter(session)
                        writer.event(data[0], "started", data[1])
                    elif kind == "imu" and writer:
                        writer.imu(*data)
                    elif kind == "image":
                        cam, stamp, received, gray = data
                        if writer:
                            writer.image(cam, stamp, received, gray)
                    elif kind == "board" and writer:
                        writer.event(data[0], "board", dict(found=data[1]))
                    if writer and time.monotonic() - lastCommit > 0.25:
                        writer.commit()
                        lastCommit = time.monotonic()
            except Exception as exc:
                self.fail("数据写盘/棋盘检测异常：" + str(exc))
            finally:
                if writer:
                    try:
                        writer.event(
                            time.time_ns(),
                            "abort" if self.error or self.stopRequest else "finished",
                            dict(
                                reason=self.error
                                or ("用户停止" if self.stopRequest else ""),
                                synthetic=manifest["synthetic"],
                            ),
                        )
                        writer.close()
                    except Exception as exc:
                        self.fail("收尾写盘失败：" + str(exc))

    node = Recorder()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/":
                payload, kind = PAGE.encode(), "text/html; charset=utf-8"
            elif path == "/status":
                payload, kind = (
                    json.dumps(
                        node.shared, ensure_ascii=False, allow_nan=False
                    ).encode(),
                    "application/json",
                )
            elif path == "/preview.jpg":
                payload, kind = node.preview, "image/jpeg"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", kind)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            if self.path == "/start":
                if node.shared["state"] != "READY":
                    self.send_error(409, "not ready")
                    return
                node.beginRequest = True
            elif self.path == "/stop":
                node.stopRequest = True
            else:
                self.send_error(404)
                return
            self.send_response(202)
            self.end_headers()

        def log_message(self, *args):
            pass

    server = None
    try:
        server = ThreadingHTTPServer((bind, port), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        print(f"只读传感器采集界面：http://127.0.0.1:{port}；等待点击开始", flush=True)
        printed = None
        while rclpy.ok() and not node.done:
            executor.spin_once(timeout_sec=0.05)
            phase = node.shared.get("phase")
            name = phase["name"] if phase else node.shared["state"]
            if name != printed:
                print(json.dumps(node.shared, ensure_ascii=False), flush=True)
                printed = name
    except (KeyboardInterrupt, ExternalShutdownException):
        node.stopRequest = True
    except Exception as exc:
        node.fail("采集异常：" + str(exc))
    finally:
        if not node.done and not node.error:
            node.stopRequest = True
        if server:
            server.shutdown()
            server.server_close()
        node.previewDone.set()
        node.previewWorker.join(timeout=10)
        if node.previewWorker.is_alive():
            node.fail("棋盘检测线程未在10秒内退出")
        node.workerDone.set()
        node.worker.join(timeout=30)
        if node.worker.is_alive():
            node.fail("写盘线程未在30秒内退出")
        node.shared["state"] = (
            "ABORTED" if node.error or node.stopRequest else "COMPLETE"
        )
        node.shared["error"] = node.error
        if node.preview:
            (session / "preview.jpg").write_bytes(node.preview)
        atomicJson(session / "status.json", node.shared)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 1 if node.error or node.stopRequest else 0
