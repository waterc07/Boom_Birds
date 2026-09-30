import pytest
from types import SimpleNamespace
from boom_birds_control.px4_backend import MavlinkPx4Backend, ManualClock


def test_landed_observation_unknown_source_freshness_and_restart():
    clock = ManualClock()
    backend = MavlinkPx4Backend("udpin:127.0.0.1:0", clock=clock)
    def msg(state, system=1):
        return SimpleNamespace(get_msgId=lambda:245, get_srcSystem=lambda:system,
            get_srcComponent=lambda:1, landed_state=state)
    assert backend.read_vehicle_state().landed_state is None
    backend._handle_message(msg(2, system=9))
    assert backend.read_vehicle_state().landed_state is None
    backend._handle_message(msg(2))
    assert backend.read_vehicle_state().landed_state == 2
    assert backend.read_vehicle_state().landed_age_s == 0.
    clock.advance(1.2)
    assert backend.read_vehicle_state().landed_age_s > 1.
    backend._handle_message(msg(0))
    assert backend.read_vehicle_state().landed_state is None
    backend._handle_message(msg(1))
    assert backend.read_vehicle_state().landed_state == 1
    backend._mark_restart("test", clock(), 10000, 100)
    assert backend.read_vehicle_state().landed_state is None
    assert backend.read_vehicle_state().landed_age_s is None


def test_px4_intention_source_age_reset_and_estimator_reset():
    from boom_birds_control.px4_backend import px4_custom_mode
    clock = ManualClock()
    backend = MavlinkPx4Backend("udpin:127.0.0.1:0", clock=clock)
    def msg(msgid, system=1, **kw):
        return SimpleNamespace(get_msgId=lambda:msgid, get_srcSystem=lambda:system,
                              get_srcComponent=lambda:1, **kw)
    land, off = px4_custom_mode(4, 6), px4_custom_mode(6)
    backend._handle_message(msg(436, system=9, custom_mode=land, intended_custom_mode=off))
    assert backend.read_vehicle_state().intended_mode_detail is None
    backend._handle_message(msg(436, custom_mode=land, intended_custom_mode=off))
    state = backend.read_vehicle_state()
    assert (state.current_mode_detail, state.intended_mode_detail) == ("auto:land", "offboard")
    clock.advance(2.)
    assert backend.read_vehicle_state().current_mode_age_s == 2.
    assert backend.read_vehicle_state().frame_reset_age_s is None
    for counter in (255, 255, 0):
        backend._handle_message(msg(331, reset_counter=counter))
    assert backend.read_vehicle_state().frame_reset_epoch == 1
    assert backend.read_vehicle_state().frame_reset_age_s == 0.
    clock.advance(1.1)
    assert backend.read_vehicle_state().frame_reset_age_s == pytest.approx(1.1)
    backend._mark_restart("test", clock(), 10000, 100)
    assert backend.read_vehicle_state().intended_mode_detail is None
    assert backend.read_vehicle_state().frame_reset_age_s is None


def test_current_mode_prevents_old_heartbeat_revoking_a_confirmed_mode_change():
    from boom_birds_control.px4_backend import px4_custom_mode
    clock = ManualClock()
    backend = MavlinkPx4Backend("udpin:127.0.0.1:0", clock=clock)
    backend._hb_custom_mode = px4_custom_mode(4, 6)
    msg = SimpleNamespace(get_msgId=lambda:436, get_srcSystem=lambda:1,
                          get_srcComponent=lambda:1, custom_mode=px4_custom_mode(6),
                          intended_custom_mode=px4_custom_mode(6))
    backend._handle_message(msg)
    assert backend.read_vehicle_state().mode_detail == "offboard"
    clock.advance(backend.heartbeat_timeout_s + .01)
    assert backend.read_vehicle_state().mode_detail == "auto:land"
