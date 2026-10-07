"""A stale trajectory during attitude navigation must not invalidate fresh EKF velocity."""
from collections import deque
import os
import threading
from types import SimpleNamespace
from boom_birds_control import platform_sih_feedback as module


def test_attitude_navigation_has_estimator_without_velocity_ack(monkeypatch):
    reader = module.SihVelocityFeedback.__new__(module.SihVelocityFeedback)
    reader.pid = os.getpid()
    reader.core = SimpleNamespace(sent_history={})
    reader.lock = threading.Lock()
    reader.latest = None
    reader.estimator_valid = False
    reader.raw = deque(maxlen=1000)
    reader.raw_evicted = 0
    class OnePass:
        done = False
        def is_set(self): return self.done
        def wait(self, delay): self.done = True
    reader.stop = OnePass()
    def listener(command, **kwargs):
        topic = command[1]
        texts = {
            "offboard_control_mode": "timestamp: 1 (0.01 seconds ago)\n attitude: True\n velocity: False\n",
            "vehicle_control_mode": "timestamp: 1 (5.0 seconds ago)\n flag_control_velocity_enabled: False\n",
            "vehicle_local_position": "timestamp: 1 (0.01 seconds ago)\n v_xy_valid: True\n v_z_valid: True\n",
            "trajectory_setpoint": "timestamp: 1 (5.0 seconds ago)\n velocity: [nan, nan, nan]\n",
        }
        return SimpleNamespace(returncode=0, stdout=texts[topic])
    monkeypatch.setattr(module, "verify_sih_process", lambda pid: True)
    monkeypatch.setattr(module.subprocess, "run", listener)
    reader._run()
    assert reader.snapshot() == (None, True)
