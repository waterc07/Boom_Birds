from pathlib import Path
import pytest
from boom_birds_control.flight_trial import flightTrialApproved, validateTrialTask


def profile(tmp_path):
    record=tmp_path/"trial.md";record.write_text("User-authorized task-1 trial; not flight acceptance.")
    return {"hardware_verified":False,"flight_trial":{"authorized":True,"task":"tag_takeoff_land",
            "control_mode":"px4_velocity","record":str(record)}}


def test_trial_is_separate_from_acceptance_and_needs_record(tmp_path):
    p=profile(tmp_path)
    assert flightTrialApproved(p) and p["hardware_verified"] is False
    Path(p["flight_trial"]["record"]).unlink()
    assert not flightTrialApproved(p)
    assert not flightTrialApproved({})


@pytest.mark.parametrize("change",[{"name":"tag_return_turn"},{"height_m":1.0},{"max_speed_m_s":.4},{"max_accel_m_s2":.4}])
def test_trial_does_not_authorize_other_tasks_or_limits(tmp_path,change):
    p=profile(tmp_path)
    task=dict(name="tag_takeoff_land",height_m=.8,max_speed_m_s=.3,max_accel_m_s2=.3)
    validateTrialTask([p],task)
    task.update(change)
    with pytest.raises(ValueError,match="limited_to_task1"):
        validateTrialTask([p],task)


def test_trial_keeps_candidate_geometry_label_without_claiming_acceptance(tmp_path):
    import yaml
    from boom_birds_control.platform_model import validateProfile
    p=yaml.safe_load((Path(__file__).resolve().parents[2]/"boom_birds_sim/config/platform_landing_test.yaml").read_text())
    p["flight_trial"]=profile(tmp_path)["flight_trial"]
    p["hardware_verified"]=False
    validateProfile(p,test_only=False)
    assert p["synthetic"] is True and p["hardware_verified"] is False
    p.pop("flight_trial")
    with pytest.raises(ValueError,match="synthetic_profile"):
        validateProfile(p,test_only=False)
