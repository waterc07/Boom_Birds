"""标定专用遥测入口；串口出口按消息和参数白名单过滤。"""

from __future__ import annotations

import fcntl
import json
import math
import os
import select
import signal
import socket
import subprocess
import sys
import termios
import threading
import time
from pathlib import Path

import yaml

from .vio_calibration import atomic_json

# PX4 Onboard 静态采集档：保留心跳、状态、电池、时间同步和 IMU。
PROFILE = {
    111: 100000,
    112: -1,
    105: 20000,
    32: 500000,
    30: 500000,
    141: -1,
    132: -1,
    265: -1,
    330: -1,
    331: -1,
    246: -1,
    31: -1,
    83: -1,
    147: 1000000,
    263: -1,
    225: -1,
    230: -1,
    245: 1000000,
    285: -1,
    284: -1,
    281: -1,
    33: -1,
    124: -1,
    49: -1,
    24: -1,
    25: -1,
    242: -1,
    12920: -1,
    62: -1,
    12901: -1,
    12904: -1,
    12918: -1,
    106: -1,
    4: -1,
    87: -1,
    85: -1,
    339: -1,
    65: -1,
    29: -1,
    36: -1,
    1: 1000000,
    2: 1000000,
    74: -1,
    241: -1,
    231: -1,
    254: -1,
    350: -1,
    250: -1,
    251: -1,
}


def allowed(message, originals=None):
    kind = message.get_type()
    if kind == "TIMESYNC":
        return True
    if kind != "COMMAND_LONG":
        return False
    if message.target_system not in (0, 1) or message.target_component not in (0, 1):
        return False
    p = [getattr(message, "param" + str(i)) for i in range(1, 8)]
    if not all(math.isfinite(x) for x in p):
        return False
    if message.command == 512:
        return p[0] == 148 and all(x == 0 for x in p[1:6]) and p[6] in (0, 1)
    if message.target_system != 1 or message.target_component != 1:
        return False
    mid = int(p[0])
    if mid not in PROFILE or mid != p[0] or any(x != 0 for x in p[2:]):
        return False
    if message.command == 510:
        return p[1] == 0
    if message.command == 511:
        return p[1] == PROFILE[mid] or (
            originals is not None and mid in originals and p[1] == originals[mid]
        )
    return False


class TelemetryLink:
    def __init__(self, device, out, baud=115200, check_battery=True):
        from pymavlink.dialects.v20 import common as mav

        if baud != 115200:
            raise ValueError("该标定入口只验证过115200，不自动改飞控波特率")
        self.check_battery = check_battery
        self.mav = mav
        self.path = Path(out)
        self.path.mkdir(exist_ok=False)
        self.rx = (self.path / "serial-rx.bin").open("wb", buffering=0)
        self.tx = (self.path / "serial-tx.bin").open("wb", buffering=0)
        self.audit = (self.path / "wire.jsonl").open("w", buffering=1)
        self.fd = None
        self.sock = None
        self.thread = None
        self.stop = threading.Event()
        self.originals = {}
        self.modified = []
        self.restored = {}
        self.intervals = {}
        self.armed = False
        self.hb = None
        self.battery = None
        self.battery_at = None
        self.error = None
        self.parser = mav.MAVLink(None)
        self.parser.robust_parsing = True
        self.encoder = mav.MAVLink(None, srcSystem=255, srcComponent=191)
        try:
            self.fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            a = termios.tcgetattr(self.fd)
            a[0] = a[1] = a[3] = 0
            a[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            a[4] = a[5] = termios.B115200
            a[6][termios.VMIN] = a[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, a)
            termios.tcflush(self.fd, termios.TCIFLUSH)
            time.sleep(0.2)
            termios.tcflush(self.fd, termios.TCIFLUSH)
            end = time.monotonic() + 12
            while (self.hb is None or (self.check_battery and self.battery is None)) and time.monotonic() < end:
                self.poll(0.1, check=False)
            self.check()
            if self.check_battery and self.battery["battery_remaining"] < 30:
                raise RuntimeError("启动标定要求电量至少30%")
        except BaseException as exc:
            self.error = str(exc)
            self.close()
            raise

    def log(self, action, msg):
        self.audit.write(
            json.dumps(
                dict(mono_ns=time.monotonic_ns(), action=action, message=msg.to_dict())
            )
            + "\n"
        )

    def send(self, msg, packet=None):
        if not allowed(msg, self.originals):
            self.log("BLOCKED", msg)
            return False
        data = msg.pack(self.encoder) if packet is None else packet
        self.encoder.seq = (self.encoder.seq + 1) % 256
        view = memoryview(data)
        deadline = time.monotonic() + 1
        while view:
            try:
                n = os.write(self.fd, view)
                view = view[n:]
            except BlockingIOError:
                if time.monotonic() > deadline:
                    raise RuntimeError("串口发送超时")
                select.select([], [self.fd], [], 0.05)
        self.tx.write(data)
        self.log("SENT", msg)
        return True

    def consume(self, data, forward=None):
        self.rx.write(data)
        for msg in self.parser.parse_buffer(data) or []:
            kind = msg.get_type()
            if kind == "BAD_DATA" or kind.startswith("UNKNOWN_"):
                continue
            if forward:
                forward(bytes(msg.get_msgbuf()))
            if msg.get_srcSystem() != 1 or msg.get_srcComponent() != 1:
                continue
            if kind == "HEARTBEAT":
                self.hb = time.monotonic()
                self.armed |= bool(msg.base_mode & 128)
            elif kind == "BATTERY_STATUS":
                self.battery = msg.to_dict()
                self.battery_at = time.monotonic()
            elif kind == "MESSAGE_INTERVAL":
                self.intervals[msg.message_id] = msg.interval_us
            elif kind == "TIMESYNC" and msg.tc1 == 0 and forward is None:
                self.send(self.encoder.timesync_encode(time.time_ns(), msg.ts1))

    def check(self):
        if self.armed:
            raise RuntimeError("观察到已解锁状态")
        if self.hb is None or time.monotonic() - self.hb > 3.5:
            raise RuntimeError("飞控心跳过期")
        if not self.check_battery:
            return
        if self.battery is None or time.monotonic() - self.battery_at > 10:
            raise RuntimeError("电量数据过期")
        if (
            self.battery["charge_state"] in (2, 3, 4, 5, 6)
            or not 20 < self.battery["battery_remaining"] <= 100
        ):
            raise RuntimeError("飞控电量低或上报电池故障")

    def poll(self, timeout=0.05, check=True):
        if select.select([self.fd], [], [], timeout)[0]:
            self.consume(os.read(self.fd, 65536))
        if check:
            self.check()

    def command(self, command, mid, value=0):
        msg = self.encoder.command_long_encode(
            1, 1, command, 0, mid, value, 0, 0, 0, 0, 0
        )
        if not self.send(msg):
            raise RuntimeError("遥测命令被白名单拒绝")

    def get_interval(self, mid):
        self.intervals.pop(mid, None)
        self.command(510, mid)
        end = time.monotonic() + 1.5
        while mid not in self.intervals and time.monotonic() < end:
            self.poll(check=False)
        if mid not in self.intervals:
            raise RuntimeError("遥测间隔回读超时：" + str(mid))
        return self.intervals[mid]

    def configure(self):
        for mid in PROFILE:
            self.check()
            self.originals[mid] = self.get_interval(mid)
        atomic_json(self.path / "original-intervals.json", self.originals)
        for mid, value in PROFILE.items():
            self.check()
            if self.originals[mid] == value:
                continue
            self.modified.append(mid)
            self.command(511, mid, value)
            if self.get_interval(mid) != value:
                raise RuntimeError("遥测间隔设置未生效：" + str(mid))

    def relay(self, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))
        parser = self.mav.MAVLink(None)
        parser.robust_parsing = True
        try:
            last_sync = 0
            while not self.stop.is_set():
                ready, _, _ = select.select([self.fd, self.sock], [], [], 0.02)
                if self.fd in ready:
                    self.consume(
                        os.read(self.fd, 65536),
                        lambda packet: self.sock.sendto(
                            packet, ("127.0.0.1", port - 40)
                        ),
                    )
                if self.sock in ready:
                    data, peer = self.sock.recvfrom(65536)
                    if peer != ("127.0.0.1", port - 40):
                        continue
                    for msg in parser.parse_buffer(data) or []:
                        if msg.get_type() != "BAD_DATA":
                            self.send(msg, bytes(msg.get_msgbuf()))
                self.check()
                if time.monotonic() - last_sync >= 1:
                    for stream in (self.rx, self.tx, self.audit):
                        stream.flush()
                        os.fsync(stream.fileno())
                    last_sync = time.monotonic()
        except BaseException as exc:
            self.error = str(exc)
            self.stop.set()

    def start(self, port=14680):
        self.thread = threading.Thread(target=self.relay, args=(port,), daemon=True)
        self.thread.start()

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=3)
        if self.fd is not None:
            for mid in reversed(self.modified):
                try:
                    self.command(511, mid, self.originals[mid])
                    self.restored[mid] = self.get_interval(mid) == self.originals[mid]
                except BaseException as exc:
                    self.restored[mid] = str(exc)
            # 不恢复旧 canonical/echo 设置，避免空闲串口自动回显飞控数据。
            fcntl.ioctl(self.fd, termios.TIOCNXCL)
            os.close(self.fd)
            self.fd = None
        if self.sock:
            self.sock.close()
        for stream in (self.rx, self.tx, self.audit):
            stream.close()
        atomic_json(
            self.path / "report.json",
            dict(
                armed_seen=self.armed,
                battery_check_required=self.check_battery,
                restore=self.restored,
                all_restored=len(self.restored) == len(self.modified)
                and all(v is True for v in self.restored.values()),
                battery=self.battery,
                error=self.error,
                tx_policy="TIMESYNC; whitelisted GET/SET_MESSAGE_INTERVAL; AUTOPILOT_VERSION request only",
            ),
        )


def result_payload(session, path):
    """采集退出后的只读页面，不连接 ROS 或串口。"""
    from .vio_calibration_record import PAGE

    session = Path(session)
    if path == "/":
        return PAGE.encode(), "text/html; charset=utf-8"
    if path == "/status":
        return (session / "status.json").read_bytes(), "application/json"
    if path == "/preview.jpg" and (session / "preview.jpg").exists():
        return (session / "preview.jpg").read_bytes(), "image/jpeg"
    return None


def serve_result(session, port, seconds=600):
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            payload = result_payload(session, self.path.split("?", 1)[0])
            if payload is None:
                self.send_error(404)
                return
            data, kind = payload
            self.send_response(200)
            self.send_header("Content-Type", kind)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            self.send_error(409, "capture ended; prepare a new session")

        def log_message(self, *args):
            pass

    with ThreadingHTTPServer(("127.0.0.1", port), Handler) as server:
        server.timeout = 0.5
        deadline = time.monotonic() + seconds
        print(f"采集已结束，硬件输入已关闭；只读结果页保留 {seconds}s", flush=True)
        while time.monotonic() < deadline:
            server.handle_request()


def capture(session, device="/dev/ttyAMA0", port=8084, maximum_seconds=180,
            usb_powered=False):
    import numpy as np

    session = Path(session).resolve()
    manifest = json.loads((session / "session.json").read_text())
    if (session / "capture.sqlite3").exists():
        raise ValueError("会话已有采集数据")
    if maximum_seconds < sum(x[1] for x in manifest["phases"]) + 10:
        raise ValueError("最长运行时间不足")
    manifest["battery_check_required"] = not usb_powered
    manifest["power_source"] = "usb" if usb_powered else "battery"
    atomic_json(session / "session.json", manifest)
    # 不与现有串口使用者争用；不停止其他服务。
    check = subprocess.run(["fuser", device], capture_output=True, text=True)
    if check.returncode == 0:
        raise RuntimeError("串口已被占用：" + check.stdout.strip())
    if check.returncode not in (0, 1):
        raise RuntimeError("无法检查串口占用：" + check.stderr.strip())
    with np.load(session / "source-stereo.npz", allow_pickle=False) as data:
        w, h = [int(x) for x in data["image_size"]]
    scale = manifest["width"] / w
    if not np.isclose(scale, manifest["height"] / h) or scale not in (0.5, 1.0):
        raise ValueError("采集入口仅支持原图或二分之一缩图")
    params = dict(
        mode="v4l2",
        device="/dev/v4l/by-id/usb-Generic_USB_Camera_20000001-video-index0",
        calibration_file=str(session / "source-stereo.npz"),
        capture_width=2 * w,
        capture_height=h,
        capture_fps=60,
        capture_buffers=2,
        rate_hz=20.0,
        raw_output_scale=scale,
        raw_decode_divisor=2 if scale == 0.5 else 1,
        opencv_threads=2,
        image_queue_depth=2,
        publish_queue_depth=1,
        publish_raw_without_subscribers=False,
        publish_stitched=False,
        publish_mjpeg=False,
    )
    path = session / "camera-params.yaml"
    path.write_text(
        yaml.safe_dump(dict(boom_birds_stereo_source=dict(ros__parameters=params)))
    )
    children = []
    link = None
    error = None
    exit_code = 1

    def launch(name, argv):
        log = (session / (name + ".log")).open("w")
        p = subprocess.Popen(
            argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
        )
        children.append((name, p, log))
        return p

    old_handler = signal.getsignal(signal.SIGTERM)

    def terminate(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate)
    try:
        link = TelemetryLink(device, session / "link", check_battery=not usb_powered)
        link.configure()
        link.start()
        launch(
            "mavros",
            [
                "ros2",
                "run",
                "mavros",
                "mavros_node",
                "--ros-args",
                "-p",
                "fcu_url:=udp://127.0.0.1:14640@127.0.0.1:14680",
                "-p",
                'gcs_url:=""',
                "-p",
                "plugin_allowlist:=['sys_status','sys_time','command','imu']",
                "-p",
                "tgt_system:=1",
                "-p",
                "tgt_component:=1",
                "-p",
                "fcu_protocol:=v2.0",
            ],
        )
        for name in ("mavros_config_node", "mavros_imu_node"):
            launch(
                name,
                [
                    sys.executable,
                    "-c",
                    "from boom_birds_sensing." + name + " import main; main()",
                ],
            )
        launch(
            "camera",
            [
                sys.executable,
                "-m",
                "boom_birds_sensing.stereo_source",
                "--ros-args",
                "--params-file",
                str(path),
            ],
        )
        recorder = launch(
            "recorder",
            [
                sys.executable,
                "-m",
                "boom_birds_sensing.vio_calibration_cli",
                "record",
                "--session",
                str(session),
                "--port",
                str(port),
            ],
        )
        print(
            f"采集预览：http://127.0.0.1:{port}；就绪后点击开始，最长等待/采集共{maximum_seconds}s",
            flush=True,
        )
        start = time.monotonic()
        while recorder.poll() is None:
            if link.error:
                raise RuntimeError(link.error)
            if any(
                p.poll() is not None for name, p, _ in children if name != "recorder"
            ):
                raise RuntimeError("输入进程提前退出，检查本会话日志")
            if time.monotonic() - start > maximum_seconds:
                raise RuntimeError("超过采集入口最长运行时间")
            time.sleep(0.1)
        exit_code = recorder.returncode
        if exit_code and (session / "status.json").exists():
            final_status = json.loads((session / "status.json").read_text())
            error = final_status.get("error") or "采集器异常退出"
    except (KeyboardInterrupt, Exception) as exc:
        error = str(exc) or "采集被中断"
    finally:
        for name, p, log in reversed(children):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
                try:
                    p.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGTERM)
                    try:
                        p.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(p.pid, signal.SIGKILL)
                        p.wait()
            log.close()
        if link:
            link.close()
        signal.signal(signal.SIGTERM, old_handler)
        report = dict(
            error=error,
            exit_codes={name: p.returncode for name, p, _ in children},
            input_profile="UNARMED_CALIBRATION_ONLY",
            current_release_changed=False,
        )
        if link and (
            len(link.restored) != len(link.modified)
            or not all(v is True for v in link.restored.values())
        ):
            report["error"] = "部分遥测间隔恢复未确认；禁止继续测试"
        atomic_json(session / "inputs.json", report)
        if report["error"] and not (session / "status.json").exists():
            atomic_json(session / "status.json", dict(state="ABORTED",
                error=report["error"], blockers=[report["error"]], phase=None))
    if (session / "status.json").exists():
        try:
            serve_result(session, port)
        except (KeyboardInterrupt, OSError) as exc:
            print("结果页已关闭：" + str(exc), flush=True)
    return 1 if report["error"] else exit_code
