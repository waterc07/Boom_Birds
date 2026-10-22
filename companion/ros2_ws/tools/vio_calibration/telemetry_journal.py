"""串口审计异步落盘；队列满或写盘失败时使采集失败。"""
import os
from pathlib import Path
import queue
import threading
import time


class TelemetryJournal:
    def __init__(self, path, capacity=4096, sync_interval=1.):
        path = Path(path)
        self.jobs = queue.Queue(maxsize=capacity)
        self.stopping = threading.Event()
        self.error = None
        self.sync_interval = sync_interval
        self.max_queue = 0
        self.max_sync_s = 0.
        self.written = 0
        self.streams = {
            "rx": (path / "serial-rx.bin").open("wb"),
            "tx": (path / "serial-tx.bin").open("wb"),
            "audit": (path / "wire.jsonl").open("wb"),
        }
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def write(self, kind, data):
        if self.error is not None:
            return
        if self.stopping.is_set():
            self.error = "telemetry journal closed"
            return
        try:
            self.jobs.put_nowait((kind, data))
            self.max_queue = max(self.max_queue, self.jobs.qsize())
        except queue.Full:
            self.error = "telemetry journal queue full"

    def _sync(self):
        started = time.monotonic()
        for stream in self.streams.values():
            stream.flush()
            os.fsync(stream.fileno())
        self.max_sync_s = max(self.max_sync_s, time.monotonic() - started)

    def _run(self):
        last_sync = time.monotonic()
        try:
            while not self.stopping.is_set() or not self.jobs.empty():
                try:
                    kind, data = self.jobs.get(timeout=.05)
                except queue.Empty:
                    pass
                else:
                    self.streams[kind].write(data)
                    self.written += 1
                if time.monotonic() - last_sync >= self.sync_interval:
                    self._sync()
                    last_sync = time.monotonic()
            self._sync()
        except BaseException as exc:
            self.error = self.error or str(exc) or type(exc).__name__
        finally:
            for stream in self.streams.values():
                try:
                    stream.close()
                except OSError as exc:
                    self.error = self.error or str(exc)

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=10)
        if self.thread.is_alive():
            self.error = self.error or "telemetry journal drain timeout"
        return dict(complete=self.error is None and not self.thread.is_alive(),
                    error=self.error, written=self.written,
                    pending=self.jobs.qsize(), max_queue=self.max_queue,
                    max_sync_s=self.max_sync_s)
