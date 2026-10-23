"""限频可读诊断；关键状态变化同周期发布。"""
import math


class DiagnosticThrottle:
    def __init__(self, rate_hz):
        if not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError("diagnostic_rate_hz must be finite and positive")
        self.period = 1.0 / rate_hz
        self.lastAt = -math.inf
        self.lastKey = None

    def due(self, now, key):
        if key == self.lastKey and 0 <= now - self.lastAt < self.period:
            return False
        self.lastAt, self.lastKey = now, key
        return True
