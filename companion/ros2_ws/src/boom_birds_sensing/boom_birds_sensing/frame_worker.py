# ******************************************************************************
# @file           : frame_worker.py
# @brief          : 异步处理在线图像，仅保留最新的待处理帧。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

"""在线观测只保留一帧待处理输入；回放另走逐帧接口。"""
from collections import deque
import threading
import time


class LatestFrameWorker:
    def __init__(self, process, *, processingPeriod=0., fatalErrors=False, name=None):
        self.processingPeriod = processingPeriod
        self.fatalErrors = fatalErrors
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
            self.raiseIfFailed()
            if self.stopping:
                return False
            self.received += 1
            if self.pending is not None:
                self.replaced += 1
            self.pending = item
            self.condition.notify()
            return True

    def raiseIfFailed(self, message="frame worker failed"):
        if self.fatalErrors and self.error is not None:
            raise RuntimeError(message) from self.error

    def _run(self):
        nextAt = 0.
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.stopping or self.pending is not None)
                if self.stopping:
                    return
                while not self.stopping:
                    remaining = nextAt - time.monotonic()
                    if remaining <= 0:
                        break
                    self.condition.wait(timeout=remaining)
                if self.stopping:
                    return
                item, self.pending = self.pending, None
                nextAt = time.monotonic() + self.processingPeriod
            start = time.monotonic()
            try:
                self.process(item)
                with self.condition:
                    self.processed += 1
            except Exception as exc:
                with self.condition:
                    self.errors += 1
                    self.error = exc
                    if self.fatalErrors:
                        self.stopping = True
                        self.pending = None
                        self.condition.notify_all()
                        return
            with self.condition:
                self.durations.append(time.monotonic()-start)

    @staticmethod
    def _percentile(values, fraction: float):
        if not values:
            return None
        ordered = sorted(values)
        index = int(round(fraction * (len(ordered) - 1)))
        return ordered[min(len(ordered) - 1, max(0, index))]

    def snapshot(self, *, includeDurations: bool = True):
        with self.condition:
            stats = dict(received=self.received, replaced=self.replaced,
                         processed=self.processed, errors=self.errors,
                         pending=self.pending is not None)
            durations = tuple(self.durations)
        if includeDurations:
            stats["durations_s"] = durations
        else:
            # 诊断话题不携带 1000 条原始耗时，只给分位摘要（离线工具仍可用默认参数取全量）。
            stats["durations_p50_s"] = self._percentile(durations, .5)
            stats["durations_p95_s"] = self._percentile(durations, .95)
            stats["durations_max_s"] = max(durations) if durations else None
        return stats

    def close(self, timeout=2.):
        with self.condition:
            self.stopping = True
            self.pending = None
            self.condition.notify()
        self.thread.join(timeout=timeout)
        return not self.thread.is_alive()
