"""Explicit task-1 flight-trial authorization, separate from hardware acceptance."""
from pathlib import Path


def flightTrialApproved(profile):
    trial = profile.get("flight_trial", {})
    return (isinstance(trial, dict) and trial.get("authorized") is True
            and trial.get("task") == "tag_takeoff_land"
            and trial.get("control_mode") == "px4_velocity"
            and isinstance(trial.get("record"), str) and bool(trial["record"])
            and Path(trial["record"]).is_file())


def validateTrialTask(profiles, task):
    if any(flightTrialApproved(p) for p in profiles):
        if (task.get("name") != "tag_takeoff_land" or task.get("height_m") != .8
                or task.get("max_speed_m_s", 99) > .3
                or task.get("max_accel_m_s2", 99) > .3):
            raise ValueError("flight_trial_limited_to_task1_0_8m")
