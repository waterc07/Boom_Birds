"""限频可读诊断；关键状态变化同周期发布。"""
import math


class DiagnosticThrottle:
    def __init__(self, rate_hz):
        if not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError("diagnostic_rate_hz must be finite and positive")
        self.period = 1.0 / rate_hz
        self.last_at = -math.inf
        self.last_key = None

    def due(self, now, key):
        if key == self.last_key and 0 <= now - self.last_at < self.period:
            return False
        self.last_at, self.last_key = now, key
        return True
