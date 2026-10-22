from collections import Counter, deque
from types import SimpleNamespace as NS, MethodType

import pytest

from boom_birds_control.attitude_control import AttitudeController
from boom_birds_control.mavros_backend import MavrosPx4Backend
from boom_birds_control.px4_backend import ManualClock
from boom_birds_control.px4_interface_node import Px4InterfaceNode
from boom_birds_control.runtime_config import DEFAULTS


def odom(stamp, *, frame="global"):
    sec = int(stamp)
    return NS(header=NS(stamp=NS(sec=sec, nanosec=round((stamp-sec)*1e9)), frame_id=frame),
              child_frame_id=DEFAULTS.body_frame,
              pose=NS(pose=NS(position=NS(x=0., y=0., z=0.),
                              orientation=NS(x=0., y=0., z=0., w=1.))),
              twist=NS(twist=NS(linear=NS(x=0., y=0., z=0.))))


@pytest.fixture
def probe(monkeypatch):
    import boom_birds_control.px4_interface_node as module
    clock = ManualClock(100.)
    monkeypatch.setattr(module.time, "monotonic", clock)
    node = NS(attitude_controller=AttitudeController(), frame_world="global", require_session=True,
              _vio_feedback=None, _attitude_feedback_error="vio_missing",
              _pending_attitude_odom=deque(), _attitude_feedback_counters=Counter(),
              _last_vio_input_stamp_ns=None, _attitude_feedback_epoch=None, ros_shift=0.,
              accepted=[], noted=[])
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=round((clock()+node.ros_shift)*1e9)))
    backend = MavrosPx4Backend(node, clock=clock, control_mode="companion_attitude")
    backend._endpoint_verified = True
    state = NS(restart_epoch=0, frame_reset_epoch=0, armed=False)
    def vehicle_state():
        state.attitude_age_s = clock()-backend._attitude_history[-1][0] if backend._attitude_history else None
        return state
    backend.read_vehicle_state = vehicle_state
    node.backend = backend
    for name in ("_observation_time", "_queue_attitude_odom", "_drain_attitude_odom", "_observe_attitude_odom", "_on_odom"):
        setattr(node, name, MethodType(getattr(Px4InterfaceNode, name), node))
    def note(msg, *, observed=None):
        node.accepted.append(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
        node.noted.append(observed)
    node._on_odom_impl = note
    def advance(t):
        clock.set(t)
        backend._time_gate.count = 5
        backend._time_gate.last_sync = t
    advance(100.)
    return node, backend, state, clock, advance


def align(probe):
    node, backend, _, clock, advance = probe
    for i in range(10):
        advance(100.+i*.02)
        stamp = clock()-.03
        backend._attitude_history.append((stamp, (0., 0., 0.)))
        node._on_odom(odom(stamp))
    assert node.attitude_controller.ready


def test_delayed_attitude_pairs_original_sample_without_refresh(probe):
    align(probe)
    node, backend, _, clock, advance = probe
    advance(100.3)
    msg = odom(100.27)
    node._on_odom(msg)
    assert len(node._pending_attitude_odom) == 1
    old_feedback = node._vio_feedback
    advance(100.35)
    backend._attitude_history.append((100.27, (0., 0., 0.)))
    node._drain_attitude_odom()
    assert not node._pending_attitude_odom
    assert node._vio_feedback is not old_feedback
    assert node._vio_feedback[3] == pytest.approx(100.27)
    assert node.noted[-1] == pytest.approx(100.27)
    assert msg.header.stamp.nanosec == 270000000


def test_expired_wait_does_not_replace_feedback_and_gap_still_latches(probe):
    align(probe)
    node, backend, _, _, advance = probe
    old_feedback = node._vio_feedback
    advance(100.4)
    node._on_odom(odom(100.37))
    advance(100.6)
    node._drain_attitude_odom()
    assert not node._pending_attitude_odom
    assert node._vio_feedback is old_feedback
    assert node._attitude_feedback_counters["expired"] == 1
    backend._attitude_history.append((100.57, (0., 0., 0.)))
    node._on_odom(odom(100.57))
    assert node.attitude_controller.latched == "vio_gap_reference_invalid"
    assert node._vio_feedback is None


def test_queue_capacity_and_duplicates(probe):
    node, _, _, _, advance = probe
    for i in range(17):
        advance(100.+i*.001)
        node._on_odom(odom(99.97+i*.001))
    assert len(node._pending_attitude_odom) == 16
    assert node._attitude_feedback_counters["overflow"] == 1
    node._on_odom(odom(99.986))
    assert len(node._pending_attitude_odom) == 16
    assert node._attitude_feedback_counters["duplicate"] == 1


@pytest.mark.parametrize("change,reason", [
    ("clock", "vio_clock_reset"),
    ("stamp", "vio_clock_reset"),
    ("publisher", "vio_producer_changed"),
    ("restart", "flight_controller_reference_changed"),
    ("reset", "flight_controller_reference_changed"),
])
def test_pending_feedback_cannot_survive_reference_change(probe, change, reason):
    align(probe)
    node, _, state, _, advance = probe
    advance(100.3)
    node._on_odom(odom(100.27))
    assert node._pending_attitude_odom
    if change == "clock": node.ros_shift = .1
    elif change == "restart": state.restart_epoch += 1
    elif change == "reset": state.frame_reset_epoch += 1
    elif change == "stamp": node._on_odom(odom(100.26))
    else:
        node._vio_publisher_gid = b"old"
        node._on_odom(odom(100.28), NS(publisher_gid=b"new"))
    node._drain_attitude_odom()
    assert node.attitude_controller.latched == reason
    assert not node._pending_attitude_odom and node._vio_feedback is None


def test_future_sample_and_frame_mismatch_still_rejected(probe):
    node, backend, _, _, advance = probe
    node._on_odom(odom(100.04))
    assert not node._pending_attitude_odom and not node.accepted
    advance(100.1)
    backend._attitude_history.append((100.07, (0., 0., 0.)))
    node._on_odom(odom(100.07, frame="wrong"))
    assert node.attitude_controller.latched == "vio_world_or_body_frame_mismatch"
    assert not node._pending_attitude_odom


@pytest.mark.parametrize("delay,true_gap,ready", [(.08, False, True), (.3, False, False), (.08, True, False)])
def test_arrival_sequence_uses_production_queue(probe, delay, true_gap, ready):
    node, backend, _, _, advance = probe
    events = []
    for i in range(151):
        t = 100+i*.02
        if not (true_gap and 101<t<101.5): events.append((t+.03, 1, t))
    for i in range(61):
        t = 100+i*.05
        events.append((t+(delay if 101<=t<101.7 else .01), 0, t))
    events.extend((100+i*.02, 2, None) for i in range(165))
    for now, kind, stamp in sorted(events):
        advance(now)
        if kind == 0: backend._attitude_history.append((stamp, (0., 0., 0.)))
        elif kind == 1: node._on_odom(odom(stamp))
        else: node._drain_attitude_odom()
        assert len(node._pending_attitude_odom) <= 16
    assert node.attitude_controller.ready is ready
    if not ready: assert node.attitude_controller.latched == "vio_gap_reference_invalid"


def test_rejected_future_stamp_does_not_poison_input_order(probe):
    node, backend, _, _, advance = probe
    node._on_odom(odom(200.))
    assert node._last_vio_input_stamp_ns is None
    advance(100.1)
    backend._attitude_history.append((100.07, (0., 0., 0.)))
    node._on_odom(odom(100.07))
    assert node.accepted == [pytest.approx(100.07)]
    assert not node.attitude_controller.latched


def test_expired_old_delivery_does_not_invalidate_good_feedback(probe):
    align(probe)
    node, _, _, _, advance = probe
    old_feedback = node._vio_feedback
    advance(100.3)
    node._on_odom(odom(99.9))
    assert node._vio_feedback is old_feedback
    assert not node.attitude_controller.latched


def test_preemption_between_clock_reads_is_not_a_clock_reset(probe):
    node, backend, _, clock, advance = probe
    node._pending_attitude_odom.append((odom(99.97), 99.97))
    backend._attitude_history.append((99.97, (0., 0., 0.)))
    normal_clock = node.get_clock
    delayed = [False]
    def get_clock():
        def now():
            value = round(clock()*1e9)
            if not delayed[0]:
                delayed[0] = True
                advance(clock()+.08)
            return NS(nanoseconds=value)
        return NS(now=now)
    node.get_clock = get_clock
    node._drain_attitude_odom()
    assert not node.attitude_controller.latched
    assert node.accepted == [pytest.approx(99.97)]
    node.get_clock = normal_clock
