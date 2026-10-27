#!/usr/bin/env python3
"""Run the independent Tag task SIH matrix sequentially; preserve every case report."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

TASKS = ("tag_takeoff_land", "tag_return_turn", "tag_forward_land")
SUCCESS_SCENARIOS = ("nominal", "detour", "tag_temporary_loss")
#: 正常任务 + 交接/故障场景；tag_border_loss 只跑旧 VIO/LK 路径作历史对照。
SAFETY_CASES = (
    ("tag_takeoff_land", "handover_timeout", None, False),
    ("tag_takeoff_land", "handover_ack_mismatch", None, False),
    ("tag_takeoff_land", "feedback_interruption", None, False),
    ("tag_takeoff_land", "tag_temporary_loss", None, False),
    ("tag_takeoff_land", "tag_persistent_loss", None, False),
    ("tag_takeoff_land", "stale_observation", None, False),
    ("tag_takeoff_land", "wrong_tag_id", None, False),
    ("tag_takeoff_land", "cancel", None, False),
    ("tag_forward_land", "detour", 90., False),
    ("tag_forward_land", "detour_failure", 90., False),
    ("tag_return_turn", "return_blocked", 90., False),
    ("tag_forward_land", "depth_loss", None, False),
    ("tag_forward_land", "vio_loss", None, False),
    ("tag_takeoff_land", "tag_border_loss", None, True),
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True)
    parser.add_argument("--px4-source", default="/home/waterc/PX4-Autopilot")
    parser.add_argument("--feedback-source",choices=("uorb","mavlink"),default="uorb")
    parser.add_argument("--with-safety", action="store_true")
    parser.add_argument("--allow-simulated-arming", action="store_true")
    parser.add_argument("--compute-load", choices=("spin", "light", "idle"), default="light")
    args = parser.parse_args()
    if not args.allow_simulated_arming:
        parser.error("--allow-simulated-arming required; local SIH only")
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    runner = Path(__file__).with_name("run_tag_flight_sih.py")
    cases = [(task, "nominal", None, False) for task in TASKS]
    if args.with_safety:
        cases.extend(SAFETY_CASES)
    matrix = dict(kind="TAG_TASK_HANDOVER_SIH_MATRIX", result="FAIL", out=str(out),
                  feedback_source=args.feedback_source, cases=[])
    baseline = {}
    for index, (task, scenario, budget, legacy) in enumerate(cases):
        directory = out/(task+"-"+scenario)
        command = [sys.executable, str(runner), "--task", task, "--scenario", scenario,
                   "--out", str(directory), "--px4-source", args.px4_source,
                   "--allow-simulated-arming", "--compute-load", args.compute_load,
                   "--feedback-source", args.feedback_source]
        if budget is not None:
            command.extend(["--flight-budget", str(budget)])
        if legacy:
            command.append("--no-handover")
        print("Running " + task + " / " + scenario, flush=True)
        env = dict(os.environ, ROS_DOMAIN_ID=str(210+index))
        with (out/(directory.name+".log")).open("w") as log:
            process = subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT)
        path = directory/"report.json"
        report = json.loads(path.read_text()) if path.exists() else dict(result="FAIL", error="report_missing")
        # Different tasks/budgets are intentional; shared sources must stay identical.
        hashes = {key: value for key, value in report.get("source_sha256", {}).items()
                  if not key.endswith(".yaml")}
        for name, digest in hashes.items():
            if name in baseline and baseline[name] != digest:
                report = dict(report, result="FAIL", error="source_changed_during_matrix:"+name)
            baseline[name] = digest
        handover = report.get("handover_report", {})
        matrix["cases"].append(dict(task=task, scenario=scenario,
            nominal=scenario in SUCCESS_SCENARIOS, handover=not legacy,
            exit_code=process.returncode, result=report["result"], error=report.get("error"),
            report=str(path), log=str(out/(directory.name+".log")),
            landing_xy_error_m=report.get("landing_xy_error_m"),
            mission_elapsed_s=report.get("mission_elapsed_s"),
            confirmed=handover.get("confirmed"),
            release_verified=handover.get("release_verified"),
            vio_inputs_stopped=handover.get("vio_inputs_stopped"),
            wire_violations=(handover.get("wire") or {}).get("violations"),
            cpu_before=(handover.get("compute") or {}).get("cpu_before_percent"),
            cpu_after=(handover.get("compute") or {}).get("cpu_after_percent"),
            command=" ".join(report.get("command", [])),
            thresholds=report.get("thresholds")))
        (out/"matrix.json").write_text(json.dumps(matrix, indent=2))
        print(report["result"] + " " + str(report.get("error", "")), flush=True)
    nominal = [case for case in matrix["cases"] if case["nominal"]]
    faults = [case for case in matrix["cases"] if not case["nominal"]]
    matrix["pass"] = all(case["exit_code"] == 0 and case["result"] == "PASS" for case in matrix["cases"])
    matrix["result"] = "PASS" if matrix["pass"] else "FAIL"
    matrix["stats"] = dict(nominal_total=len(nominal),
        nominal_pass=sum(case["result"] == "PASS" for case in nominal),
        fault_total=len(faults), fault_pass=sum(case["result"] == "PASS" for case in faults))
    matrix["shared_source_sha256"] = baseline
    (out/"matrix.json").write_text(json.dumps(matrix, indent=2))
    print(matrix["result"] + ": " + str(out/"matrix.json"))
    return 0 if matrix["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
