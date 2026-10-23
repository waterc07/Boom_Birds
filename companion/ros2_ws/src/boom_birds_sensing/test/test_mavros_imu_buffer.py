from collections import deque
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
from boom_birds_sensing.mavros_imu_node import MavrosImuNode
from boom_birds_sensing import mavros_imu_node
from test_mavros_imu import imu, ready


def nodeWithClock(monkeypatch):
    node = object.__new__(MavrosImuNode)
    clocks = [100., 1000.]
    monkeypatch.setattr(mavros_imu_node.time, "monotonic", lambda: clocks[0])
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=round(clocks[1]*1e9)))
    node.get_parameter = lambda _: NS(value=.02)
    node._pendingImu = deque()
    node._pendingTimer = Mock()
    node._pendingTimer.isCanceled.return_value = True
    node._futureBuffered = 0
    node.gate = ready(hold_last_sync_on_rtt=True)
    accepted = []
    def receive(msg):
        result = node.gate.accept(msg, nowMono=clocks[0], nowRos=clocks[1], connected=True)
        if result: accepted.append(result[0])
    node._imu = receive
    return node, clocks, accepted


def test_small_future_sample_waits_without_changing_timestamp(monkeypatch):
    node, clocks, accepted = nodeWithClock(monkeypatch)
    node._bufferImu(imu(1000.004))
    assert not accepted and len(node._pendingImu) == 1
    clocks[:] = [100.003, 1000.003]
    node._drainPendingImu()
    assert not accepted
    clocks[:] = [100.005, 1000.005]
    node._drainPendingImu()
    assert accepted == [pytest.approx(1000.004)]
    assert not node._pendingImu and node.gate.rejected == 0


def test_future_queue_has_time_bound_and_does_not_allow_far_future(monkeypatch):
    node, clocks, accepted = nodeWithClock(monkeypatch)
    node._bufferImu(imu(1001.))
    assert not node._pendingImu and node.gate.reason == "imu_sample_age"
    node._bufferImu(imu(1000.004))
    clocks[0] += .021  # ROS clock has stopped; a queued sample must not wait forever.
    node._drainPendingImu()
    assert not node._pendingImu and not accepted
    assert node.gate.rejectionCounts["imu_sample_age"] == 2


def test_future_queue_preserves_arrival_order_and_is_bounded(monkeypatch):
    node, clocks, accepted = nodeWithClock(monkeypatch)
    node._bufferImu(imu(1000.004))
    node._bufferImu(imu(1000.003))
    clocks[:] = [100.005, 1000.005]
    node._drainPendingImu()
    assert accepted == [pytest.approx(1000.004)]
    assert node.gate.reason == "imu_nonmonotonic"
    for _ in range(65): node._bufferImu(imu(1000.01))
    assert len(node._pendingImu) == 64
    assert node.gate.reason == "imu_pending_overflow"
