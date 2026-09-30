from dataclasses import replace
import math
import pytest
from boom_birds_control.control_protocol import Command, ControlIngress, EXECUTE, CANCEL, HOLD


def fresh():
    gate = ControlIngress()
    session = gate.open_session()
    return gate, Command(session, 1, 1, 100., .2, EXECUTE)


def test_reordering_duplicates_and_id_reuse():
    g, c = fresh()
    assert g.receive(c, 100., 10.)[0]
    assert not g.receive(c, 100.01, 10.01)[0]
    assert g.receive(replace(c, sequence=3, trajectory_id=2), 100.02, 10.02)[0]
    assert not g.receive(replace(c, sequence=2, trajectory_id=3), 100.03, 10.03)[0]
    assert not g.receive(replace(c, sequence=4), 100.03, 10.03)[0]
    assert g.current.trajectory_id == 2


def test_cancel_is_immediate_and_blocks_residual_commands():
    g, c = fresh()
    assert g.receive(c, 100., 10.)[0]
    assert g.receive(replace(c, sequence=2, kind=CANCEL), 105., 15.) == (True, "cancelled")
    assert g.active(105., 15.) is None
    assert not g.receive(replace(c, sequence=3, stamp=105.), 105., 15.)[0]
    assert g.receive(replace(c, sequence=4, trajectory_id=2, stamp=105., kind=HOLD), 105., 15.)[0]


def test_restart_old_session_and_old_cancel_cannot_touch_new_command():
    g, c = fresh()
    g.receive(c, 100., 10.)
    new_session = g.open_session()
    new = replace(c, session_id=new_session)
    assert g.receive(new, 100., 10.)[0]
    assert not g.receive(replace(c, sequence=999, kind=CANCEL), 100., 10.)[0]
    assert g.current == new
    restarted = ControlIngress()
    restarted.open_session()
    assert not restarted.receive(new, 100., 10.)[0]


@pytest.mark.parametrize("values", [dict(stamp=99.), dict(stamp=101.), dict(valid_for=0.), dict(valid_for=1.), dict(yaw=math.nan), dict(position=(math.inf, 0., 0.)), dict(frame=""), dict(kind=99), dict(trajectory_id=0)])
def test_invalid_commands(values):
    g, c = fresh()
    assert not g.receive(replace(c, **values), 100., 10.)[0]
    assert g.active(100., 10.) is None


def test_monotonic_deadline_and_clock_reset():
    g, c = fresh()
    g.receive(c, 100., 10.)
    assert g.active(100., 10.201) is None  # paused ROS clock cannot extend TTL
    assert not g.receive(replace(c, sequence=2), 100., 10.202)[0]
    g, c = fresh()
    g.receive(c, 100., 10.)
    assert g.active(99., 10.01) is None
    assert not g.session
