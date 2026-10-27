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
    reader.estimatorValid = False
    reader.raw = deque(maxlen=1000)
    reader.rawEvicted = 0
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
    monkeypatch.setattr(module, "verifySihProcess", lambda pid: True)
    monkeypatch.setattr(module.subprocess, "run", listener)
    reader._run()
    assert reader.snapshot() == (None, True)


def test_frozen_probe_cannot_refresh_estimator_or_ack(tmp_path, monkeypatch):
    import json
    from types import SimpleNamespace as NS
    reader = module.SihVelocityFeedback.__new__(module.SihVelocityFeedback)
    reader.lock = threading.Lock()
    reader.latest = None
    reader.estimatorValid = False
    reader.raw = deque(maxlen=1000)
    reader.rawEvicted = 0
    reader.core = NS(token="t", epoch=0, sent_history={42:(99.95,(0.,0.,0.))})
    reader.feedPath = tmp_path / "probe.json"
    samples = {
        "offboard_control_mode":"timestamp: 1 (0.01 seconds ago)\n velocity: True\n position: False\n attitude: False\n",
        "vehicle_control_mode":"timestamp: 1 (5.0 seconds ago)\n flag_control_velocity_enabled: True\n flag_control_position_enabled: False\n flag_control_offboard_enabled: True\n",
        "vehicle_local_position":"timestamp: 1 (0.01 seconds ago)\n v_xy_valid: True\n v_z_valid: True\n",
        "trajectory_setpoint":"timestamp: 1 (0.01 seconds ago)\n velocity: [0, 0, 0]\n"}
    monkeypatch.setattr(module.time, "monotonic", lambda:100.)
    reader.feedPath.write_text(json.dumps(dict(stamp=99.98,samples=samples)))
    data, stamp, sampleStamps, sampleEnds = reader._readSamples()
    reader._consume(data, sampledAt=stamp, sampleStamps=sampleStamps, sampleEnds=sampleEnds)
    assert reader.latest is not None and reader.estimatorValid
    assert abs(reader.latest.stamp - 99.97) < 1e-6
    # 不更新文件；即使 listener 输出中仍写着 0.01 s，也必须清除旧状态。
    class OnePass:
        done = False
        def is_set(self): return self.done
        def wait(self, delay): self.done = True
    for stamp in (1., 100.1, None, float("nan")):
        reader.feedPath.write_text(json.dumps(dict(stamp=stamp,samples=samples)))
        reader.stop = OnePass()
        reader._runFile()
        assert reader.snapshot() == (None, False)
    reader._consume(samples, sampledAt=99.8)
    assert reader.snapshot() == (None, False)


def test_target_correlation_uses_target_time_not_older_control_mode(tmp_path, monkeypatch):
    # 控制模式早于当前速度目标；不能把模式采样时刻作为目标回执时刻。
    reader = module.SihVelocityFeedback.__new__(module.SihVelocityFeedback)
    reader.lock = threading.Lock(); reader.latest = None; reader.estimatorValid = False
    reader.raw = deque(maxlen=1000); reader.rawEvicted = 0
    reader.core = SimpleNamespace(token="t", epoch=0, sent_history={3:(99.95,(.1,0.,0.))})
    monkeypatch.setattr(module.time, "monotonic", lambda:100.)
    samples = {
        "offboard_control_mode":"timestamp: 1 (0.1 seconds ago)\n velocity: True\n position: False\n attitude: False\n",
        "vehicle_control_mode":"timestamp: 1 (5.0 seconds ago)\n flag_control_velocity_enabled: True\n flag_control_position_enabled: False\n flag_control_offboard_enabled: True\n",
        "vehicle_local_position":"timestamp: 1 (0.01 seconds ago)\n v_xy_valid: True\n v_z_valid: True\n",
        "trajectory_setpoint":"timestamp: 1 (0.01 seconds ago)\n velocity: [0.1, 0, 0]\n"}
    reader._consume(samples)
    assert reader.latest is not None and reader.latest.sequence == 3
    assert abs(reader.latest.stamp-99.99)<1e-6


def test_listener_interval_can_contain_send_of_changing_target(monkeypatch):
    reader = module.SihVelocityFeedback.__new__(module.SihVelocityFeedback)
    reader.lock = threading.Lock(); reader.latest = None; reader.estimatorValid = False
    reader.raw = deque(maxlen=1000); reader.rawEvicted = 0
    reader.core = SimpleNamespace(token="t", epoch=0, sent_history={3:(99.95,(.1,0.,0.))})
    monkeypatch.setattr(module.time, "monotonic", lambda:100.)
    samples = {
        "offboard_control_mode":"timestamp: 1 (0.01 seconds ago)\n velocity: True\n position: False\n attitude: False\n",
        "vehicle_control_mode":"timestamp: 1 (5.0 seconds ago)\n flag_control_velocity_enabled: True\n flag_control_position_enabled: False\n flag_control_offboard_enabled: True\n",
        "vehicle_local_position":"timestamp: 1 (0.01 seconds ago)\n v_xy_valid: True\n v_z_valid: True\n",
        "trajectory_setpoint":"timestamp: 1 (0.01 seconds ago)\n velocity: [0.1, 0, 0]\n"}
    starts = {topic:99.95 for topic in samples}
    ends = {topic:99.97 for topic in samples}
    reader._consume(samples,sampledAt=99.98,sampleStamps=starts,sampleEnds=ends)
    assert reader.latest is not None and reader.latest.sequence == 3
    assert abs(reader.latest.stamp-99.96)<1e-6
    monkeypatch.setattr(module.time, "monotonic", lambda:100.2)
    reader._consume(samples,sampledAt=99.98,sampleStamps=starts,sampleEnds=ends)
    assert reader.snapshot() == (None, False)
