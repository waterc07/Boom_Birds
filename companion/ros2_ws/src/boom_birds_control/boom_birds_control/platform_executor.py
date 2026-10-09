"""Single-owner output arbiter and compute-release callbacks."""
from dataclasses import replace
from collections import deque
from dataclasses import asdict, is_dataclass
import json
import math
from pathlib import Path
import queue
import threading
from .platform_landing import PlatformLanding, VELOCITY_YAW_RATE_MASK


class PlatformExecutor:
    def __init__(self, backend, profile, *, test_only=False, revoke_navigation,
                 release_compute, native_land, disarm, trace=None):
        self.core = PlatformLanding(profile, test_only=test_only)
        self.backend = backend
        self.revoke_navigation = revoke_navigation
        self.release_compute = release_compute
        self.native_land = native_land
        self.disarm = disarm
        self.owner = "navigation"
        self.revoked = False
        self.release_requested = False
        self.release_verified = False
        self.land_requested = False
        self.disarm_requested = False
        self.release_error = ""
        self.runtime = RuntimeOptions(profile)
        self.history = self.runtime.history
        self.trace = trace
        self.history_evicted = 0
        self.release_state = "IDLE"
        self.release_attempts = 0
        self.release_since = self.release_last_try = None
        self.release_token = None

    def navigation_attitude(self, setpoint):
        if self.owner != "navigation":
            return False
        return self.backend.send_attitude_setpoint(setpoint)

    def navigation_position(self, setpoint, mask):
        if self.owner != "navigation":
            return False
        return self.backend.send_setpoint(setpoint, mask)

    def request(self, intent="land"):
        self.core.request(intent)

    def tick(self, now, observation, range_sample, flight, feedback=None, *, navigation_ready=False):
        # 捕获阶段仍由导航持有输出；合格样本积累后撤销导航，下一步才预发零速度。
        if self.core.state == "ACQUIRE" and navigation_ready and self.core.good >= self.core.cfg.acquire_samples:
            if not self.revoked:
                self.revoked = self.revoke_navigation() is True
                if not self.revoked:
                    self.core.cancel("navigation_revoke_failed")
        out = self.core.step(now, observation, range_sample, flight, feedback,
                             navigation_revoked=self.revoked, navigation_ready=navigation_ready, release_ack=self.release_verified)
        if out.state == "PREPARE" and self.owner == "navigation":
            self.owner = "platform"
            switch = getattr(self.backend, "activate_platform_velocity", None)
            if switch is not None and not switch(out.token):
                self.core.cancel("backend_handoff_failed")
                out = self.core.step(now)
        if out.velocity_ned is not None:
            sent = self.backend.send_setpoint(out.setpoint(), VELOCITY_YAW_RATE_MASK)
            self.core.note_sent(out, now, sent)
            if not sent:
                out = self.core.step(now)
        self._release_tick(now, out)
        if self.release_verified and self.core.confirmed and self.core.intent == "land":
            # 资源退出回读不依赖下一帧 Tag；失标期间也必须如实反映已停止的计算。
            self.core.released = True
            out = replace(out, vio_required=False, release_compute=False)
        if out.request_native_land and not self.land_requested:
            self.owner = "fallback"
            self.land_requested = self.native_land(out.reason) is True
        if out.request_disarm and not self.disarm_requested:
            self.disarm_requested = self.disarm() is True
        if out.state == "COMPLETE":
            self.owner = "none"
        record = dict(now=now, owner=self.owner, output=out,
                      release_requested=self.release_requested, release_verified=self.release_verified,
                      release_state=self.release_state, release_attempts=self.release_attempts,
                      release_error=self.release_error)
        if len(self.history) == self.history.maxlen:
            self.history_evicted += 1
        self.history.append(record)
        if self.trace:
            self.trace.put(record)
        return out

    def _release_tick(self, now, out):
        eligible = (out.release_compute and out.state in ("ALIGN", "DESCEND", "FLARE", "TOUCHDOWN")
                    and self.owner == "platform" and self.core.confirmed)
        if self.release_verified:
            self.release_state = "RELEASED"
            return
        if not eligible:
            if self.release_state in ("RETRY", "WAIT_RESULT"):
                self.release_state = "CANCELLED"
            return
        if self.release_state in ("FAILED", "CANCELLED"):
            return
        if self.release_since is None:
            self.release_since = now
            self.release_token = out.token
        if now - self.release_since >= self.runtime.release_timeout_s:
            self.release_state, self.release_error = "FAILED", "compute_release_timeout"
            return
        if self.release_state == "WAIT_RESULT":
            return
        if self.release_last_try is not None and now - self.release_last_try < self.runtime.release_retry_s:
            return
        if self.release_attempts >= self.runtime.release_max_attempts:
            self.release_state, self.release_error = "FAILED", "compute_release_attempt_limit"
            return
        self.release_requested = True
        self.release_attempts += 1
        self.release_last_try = now
        try:
            # True 是退出已确认，None 是异步等待，False 是未确认；调用方须避免阻塞控制周期。
            result = self.release_compute(out.token)
            if result is True:
                self.release_verified = True
                self.release_state, self.release_error = "RELEASED", ""
            elif result is None:
                self.release_state, self.release_error = "WAIT_RESULT", ""
            else:
                self.release_state, self.release_error = "RETRY", "compute_release_not_confirmed"
        except Exception as exc:
            self.release_state, self.release_error = "RETRY", str(exc)

    def release_result(self, token, success):
        # 异步结果只能确认仍有效的原 token；取消后的迟到响应不得开放 released。
        if (self.release_state != "WAIT_RESULT" or token != self.release_token
                or token != self.core.token or self.owner != "platform"
                or self.core.state not in ("ALIGN", "DESCEND", "FLARE", "TOUCHDOWN")):
            return False
        self.release_verified = success is True
        self.release_state = "RELEASED" if self.release_verified else "RETRY"
        self.release_error = "" if self.release_verified else "compute_release_not_confirmed"
        return True

    def close(self):
        return self.trace.close() if self.trace else True

    def cancel(self, reason="cancelled"):
        self.owner = "fallback"
        self.core.cancel(reason)


class SegmentedTrace:
    """控制线程只入队；磁盘写入失败和队满均计数，不阻塞控制。"""
    def __init__(self, directory, *, queue_capacity=256, segment_records=1000):
        if type(queue_capacity) is not int or queue_capacity < 1:
            raise ValueError("trace_queue_capacity")
        if type(segment_records) is not int or segment_records < 1:
            raise ValueError("trace_segment_records")
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.queue = queue.Queue(maxsize=queue_capacity)
        self.segment_records = segment_records
        self.dropped = self.errors = self.written = 0
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()

    def put(self, record):
        if self.stop.is_set():
            self.dropped += 1
            return False
        try:
            self.queue.put_nowait(record)
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def _write(self):
        stream = None
        count = 0
        index = 0
        try:
            while not self.stop.is_set() or not self.queue.empty():
                try:
                    record = self.queue.get(timeout=.05)
                except queue.Empty:
                    continue
                try:
                    if stream is None or count >= self.segment_records:
                        if stream:
                            stream.close()
                        stream = (self.directory / f"trace-{index:06d}.jsonl").open("x")
                        index += 1
                        count = 0
                    stream.write(json.dumps(record, allow_nan=False, default=lambda o: asdict(o) if is_dataclass(o) else str(o)) + "\n")
                    count += 1
                    self.written += 1
                except (OSError, ValueError, TypeError):
                    self.errors += 1
                    if stream:
                        stream.close()
                        stream = None
                finally:
                    self.queue.task_done()
        finally:
            if stream:
                stream.close()

    def close(self, timeout=2.):
        self.stop.set()
        self.thread.join(timeout=timeout)
        return not self.thread.is_alive()

    def stats(self):
        return dict(written=self.written, dropped=self.dropped, errors=self.errors,
                    pending=self.queue.qsize(), writer_alive=self.thread.is_alive())

class RuntimeOptions:
    def __init__(self, profile):
        values = profile.get("runtime", {})
        allowed = dict(history_capacity=1000, release_retry_s=.2,
                       release_timeout_s=2., release_max_attempts=5,
                       trace_queue_capacity=256, trace_segment_records=1000)
        if not isinstance(values, dict) or set(values) - set(allowed):
            raise ValueError("runtime_options")
        for name, default in allowed.items():
            value = values.get(name, default)
            if isinstance(default, int):
                if type(value) is not int or value < 1:
                    raise ValueError(name)
            elif type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(name)
            setattr(self, name, value)
        self.history = deque(maxlen=self.history_capacity)
