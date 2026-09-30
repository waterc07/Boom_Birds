from boom_birds_control.sih_safety import parse_flags, REQUIRED
import pytest


def observation(**overrides):
    values = {name: False for name in REQUIRED}
    values["battery_warning"] = 0
    values.update(overrides)
    return "timestamp: 12000 (0.010000 seconds ago)\n" + "\n".join(f"  {k}: {v}" for k, v in values.items())


def test_flags_are_px4_observations_with_age_and_no_missing_fields():
    flags, age = parse_flags(observation(offboard_control_signal_lost=True))
    assert flags["offboard_control_signal_lost"] is True
    assert flags["battery_warning"] == 0
    assert age == .01


def test_truncated_or_missing_timestamp_observations_are_refused():
    with pytest.raises(ValueError):
        parse_flags("timestamp: 12000 (0.01 seconds ago)\n offboard_control_signal_lost: True")
    with pytest.raises(ValueError):
        parse_flags(observation().split("\n", 1)[1])


@pytest.mark.parametrize("remote_required,expected", [(True, "unknown"), (None, "unknown"), (False, "")])
def test_remote_id_requirement_must_be_observed(remote_required, expected):
    from boom_birds_control.sih_safety import classify_cause
    flags, _ = parse_flags(observation(remote_id_unhealthy=True))
    assert classify_cause(flags, set(), remote_required, True, "") == expected


def test_known_loss_is_retained_through_auto_mode_and_unknown_fault_supersedes_it():
    from boom_birds_control.sih_safety import classify_cause
    flags, _ = parse_flags(observation(auto_mission_missing=True, manual_control_signal_lost=True,
                                       remote_id_unhealthy=True, offboard_control_signal_lost=True))
    cause = classify_cause(flags, {"manual_control_signal_lost"}, False, False, "")
    assert cause == "offboard_link"
    flags["offboard_control_signal_lost"] = False
    assert classify_cause(flags, {"manual_control_signal_lost"}, False, False, cause) == cause
    assert classify_cause(flags, {"manual_control_signal_lost"}, False, True, cause) == ""
    flags["battery_warning"] = 1
    assert classify_cause(flags, {"manual_control_signal_lost"}, False, False, cause) == "unknown"
    flags["battery_warning"] = 0
    flags["new_unsupported_failure"] = True
    assert classify_cause(flags, {"manual_control_signal_lost"}, False, False, cause) == "unknown"


def test_new_manual_link_loss_cannot_be_exempted_by_an_unrelated_flag():
    from boom_birds_control.sih_safety import classify_cause
    flags, _ = parse_flags(observation(manual_control_signal_lost=True))
    assert classify_cause(flags, {"gcs_connection_lost"}, False, False, "") == "unknown"
