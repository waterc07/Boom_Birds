import pytest
from pymavlink.dialects.v20 import development as mav
from boom_birds_control.telemetry_budget import observationIntervals, budgetReport

SIZES = {mid: definition.unpacker.size for mid,definition in mav.mavlink_map.items()}


def test_feature_ekf_budget_rejects_115200_8000_and_preserves_requests():
    plan = budgetReport(115200, 8000, SIZES)
    assert plan["status"] == "BUDGET_INSUFFICIENT"
    streams = {row["message_id"]:row for row in plan["streams"]}
    assert streams[105]["requested_hz"] == 50
    assert streams[331]["requested_hz"] == streams[230]["requested_hz"] == 20
    assert plan["required_max_bytes_s"] > 115200/10*.8
    assert budgetReport(230400, 16000, SIZES)["status"] == "BUDGET_OK_UNVERIFIED"
    assert budgetReport(230400, 8000, SIZES)["status"] == "BUDGET_INSUFFICIENT"


def test_platform_and_signing_are_accounted_for():
    basic = budgetReport(230400, 16000, SIZES)
    platform = budgetReport(230400, 16000, SIZES, platform=True)
    signed = budgetReport(230400, 16000, SIZES, platform=True, signed=True)
    assert basic["required_max_bytes_s"] < platform["required_max_bytes_s"] < signed["required_max_bytes_s"]
    assert observationIntervals("companion_attitude")[331] == 1000000
    assert observationIntervals("companion_attitude", sih=True)[32] == 20000
    assert observationIntervals("companion_attitude", ekf=True)[331] == 50000


@pytest.mark.parametrize("baud,rate,reserve", [(0,8000,.2),(115200,0,.2),(115200,8000,1),
    (115200,float("nan"),.2),(115200,8000,-.1)])
def test_invalid_or_automatic_budget_is_not_treated_as_verified(baud,rate,reserve):
    with pytest.raises(ValueError):
        budgetReport(baud, rate, SIZES, reserve=reserve)
