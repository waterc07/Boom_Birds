"""平台控制的有界诊断与非阻塞分段记录。"""
from collections import deque
from dataclasses import asdict, is_dataclass
import json
import math
from pathlib import Path
import queue
import threading


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
