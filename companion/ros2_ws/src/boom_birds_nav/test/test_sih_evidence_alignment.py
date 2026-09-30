import importlib.util
import math
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location(
    "sih_matrix_row", Path(__file__).resolve().parents[3] / "tools/sih_matrix_row.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def status(origin=(-15., 2., .1), yaw=math.pi / 2):
    return {"data": {"frame_alignment": {"position_allowed": True,
            "translation_m": origin, "yaw_offset_rad": yaw}}}


def test_summary_uses_recorded_translation_and_heading():
    alignment = module.recorded_alignment([status()])
    assert alignment.position_ned_to_ros((2., 1., -1.)) == pytest.approx((-16., 0., 1.1))


def test_summary_refuses_missing_alignment():
    with pytest.raises(ValueError, match="missing or changed"):
        module.recorded_alignment([{"data": {}}])


def test_summary_refuses_a_coordinate_reset():
    with pytest.raises(ValueError, match="missing or changed"):
        module.recorded_alignment([status(), status(origin=(-14., 2., .1))])


def test_summary_refuses_px4_restart_even_if_origin_values_match():
    reset = status()
    reset["data"]["frame_alignment"].update(
        position_allowed=False, invalidated_after_px4_restart=True)
    with pytest.raises(ValueError, match="PX4 restart"):
        module.recorded_alignment([status(), reset])


def test_recovery_is_not_a_latch_and_brief_history_fault_is_preserved():
    history = [dict(t=1., state="RECOVERING", reason="sensor_link", detail="waiting"),
               dict(t=2., state="FAULT_LATCHED", reason="unknown", detail="revoked"),
               dict(t=2.1, state="LANDING", reason="", detail="")]
    rows = [dict(t=3., data=dict(state="LANDING", history=history)),
            dict(t=4., data=dict(state="COMPLETE", history=history))]
    events = module.mission_fault_events(rows, 0.)
    assert len(events) == 2
    assert [e["reason"] for e in events if e["state"] == "FAULT_LATCHED"] == ["unknown"]
    assert events[0]["t_rel_s"] == 1.


def test_center_clearance_is_point_distance_and_rejects_invalid_data():
    assert module.minimum_cloud_distance([(0., 0., 0.), (2., 0., 0.)],
                                          [(1., 0., 0.), (4., 0., 0.)]) == pytest.approx(1.)
    with pytest.raises(ValueError, match="invalid"):
        module.minimum_cloud_distance([(float("nan"), 0., 0.)], [(1., 0., 0.)])
