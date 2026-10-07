"""在线观测只保留一帧待处理输入；回放另走逐帧接口。"""
from collections import deque
import threading
import time


class LatestFrameWorker:
    def __init__(self, process):
        self.process = process
        self.condition = threading.Condition()
        self.pending = None
        self.stopping = False
        self.received = self.replaced = self.processed = self.errors = 0
        self.durations = deque(maxlen=1000)
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def submit(self, item):
        with self.condition:
            if self.stopping:
                return False
            self.received += 1
            if self.pending is not None:
                self.replaced += 1
            self.pending = item
            self.condition.notify()
            return True

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.stopping or self.pending is not None)
                if self.stopping:
                    return
                item, self.pending = self.pending, None
            start = time.monotonic()
            try:
                self.process(item)
                with self.condition:
                    self.processed += 1
            except Exception:
                # 观测失败由 freshness 闸门处理；不能让工作线程静默退出。
                with self.condition:
                    self.errors += 1
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
