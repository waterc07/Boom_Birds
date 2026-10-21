from collections import deque
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
from boom_birds_sensing.mavros_imu_node import MavrosImuNode
from boom_birds_sensing import mavros_imu_node
from test_mavros_imu import imu, ready


def node_with_clock(monkeypatch):
    node = object.__new__(MavrosImuNode)
    clocks = [100., 1000.]
    monkeypatch.setattr(mavros_imu_node.time, "monotonic", lambda: clocks[0])
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=round(clocks[1]*1e9)))
    node.get_parameter = lambda _: NS(value=.02)
    node._pending_imu = deque()
    node._pending_timer = Mock()
    node._pending_timer.is_canceled.return_value = True
    node._future_buffered = 0
    node.gate = ready(hold_last_sync_on_rtt=True)
    accepted = []
    def receive(msg):
        result = node.gate.accept(msg, now_mono=clocks[0], now_ros=clocks[1], connected=True)
        if result: accepted.append(result[0])
    node._imu = receive
    return node, clocks, accepted


def test_small_future_sample_waits_without_changing_timestamp(monkeypatch):
    node, clocks, accepted = node_with_clock(monkeypatch)
    node._buffer_imu(imu(1000.004))
    assert not accepted and len(node._pending_imu) == 1
    clocks[:] = [100.003, 1000.003]
    node._drain_pending_imu()
    assert not accepted
    clocks[:] = [100.005, 1000.005]
    node._drain_pending_imu()
    assert accepted == [pytest.approx(1000.004)]
    assert not node._pending_imu and node.gate.rejected == 0


def test_future_queue_has_time_bound_and_does_not_allow_far_future(monkeypatch):
    node, clocks, accepted = node_with_clock(monkeypatch)
    node._buffer_imu(imu(1001.))
    assert not node._pending_imu and node.gate.reason == "imu_sample_age"
    node._buffer_imu(imu(1000.004))
    clocks[0] += .021  # ROS clock has stopped; a queued sample must not wait forever.
    node._drain_pending_imu()
    assert not node._pending_imu and not accepted
    assert node.gate.rejection_counts["imu_sample_age"] == 2


def test_future_queue_preserves_arrival_order_and_is_bounded(monkeypatch):
    node, clocks, accepted = node_with_clock(monkeypatch)
    node._buffer_imu(imu(1000.004))
    node._buffer_imu(imu(1000.003))
    clocks[:] = [100.005, 1000.005]
    node._drain_pending_imu()
    assert accepted == [pytest.approx(1000.004)]
    assert node.gate.reason == "imu_nonmonotonic"
    for _ in range(65): node._buffer_imu(imu(1000.01))
    assert len(node._pending_imu) == 64
    assert node.gate.reason == "imu_pending_overflow"
