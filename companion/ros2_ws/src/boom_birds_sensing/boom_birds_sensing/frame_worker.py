"""在线观测只保留一帧待处理输入；回放另走逐帧接口。"""
from collections import deque
import threading
import time


class LatestFrameWorker:
    def __init__(self, process, *, processing_period=0., fatal_errors=False, name=None):
        self.processing_period = processing_period
        self.fatal_errors = fatal_errors
        self.error = None
        self.process = process
        self.condition = threading.Condition()
        self.pending = None
        self.stopping = False
        self.received = self.replaced = self.processed = self.errors = 0
        self.durations = deque(maxlen=1000)
        self.thread = threading.Thread(target=self._run, name=name, daemon=True)
        self.thread.start()

    def submit(self, item):
        with self.condition:
            self.raise_if_failed()
            if self.stopping:
                return False
            self.received += 1
            if self.pending is not None:
                self.replaced += 1
            self.pending = item
            self.condition.notify()
            return True

    def raise_if_failed(self, message="frame worker failed"):
        if self.fatal_errors and self.error is not None:
            raise RuntimeError(message) from self.error

    def _run(self):
        next_at = 0.
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.stopping or self.pending is not None)
                if self.stopping:
                    return
                while not self.stopping:
                    remaining = next_at - time.monotonic()
                    if remaining <= 0:
                        break
                    self.condition.wait(timeout=remaining)
                if self.stopping:
                    return
                item, self.pending = self.pending, None
                next_at = time.monotonic() + self.processing_period
            start = time.monotonic()
            try:
                self.process(item)
                with self.condition:
                    self.processed += 1
            except Exception as exc:
                with self.condition:
                    self.errors += 1
                    self.error = exc
                    if self.fatal_errors:
                        self.stopping = True
                        self.pending = None
                        self.condition.notify_all()
                        return
            with self.condition:
                self.durations.append(time.monotonic()-start)

    def snapshot(self):
        with self.condition:
            return dict(received=self.received, replaced=self.replaced, processed=self.processed,
                        errors=self.errors, pending=self.pending is not None,
                        durations_s=tuple(self.durations))

    def close(self, timeout=2.):
        with self.condition:
            self.stopping = True
            self.pending = None
            self.condition.notify()
        self.thread.join(timeout=timeout)
        return not self.thread.is_alive()
