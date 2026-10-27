#!/usr/bin/env python3
"""Plot recorded SIH truth, handover timeline and matrix summary; starts no ROS/PX4."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SUCCESS_SCENARIOS = ("nominal", "detour", "tag_temporary_loss")


def load(case):
    reportPath = Path(case["report"])
    report = json.loads(reportPath.read_text())
    tracePath = reportPath.with_name("trace.jsonl")
    rows = [json.loads(line) for line in tracePath.read_text().splitlines()] if tracePath.is_file() else []
    return report, rows


def truthRows(rows):
    return [row for row in rows if row.get("kind") == "truth"]


def plotTrajectory(case, outdir):
    report, rows = load(case)
    truth = truthRows(rows)
    if not truth:
        return None
    p = np.array([row["position"] for row in truth])
    home = np.array(report["home_world"])
    board = np.array(report["board_world"])
    figure, axis = plt.subplots(figsize=(7.2, 5.6), constrained_layout=True)
    axis.plot(p[:, 0], p[:, 1], lw=1.7, color="tab:blue", label="flown truth path")
    for obstacle in report.get("obstacles", []):
        low, high = np.array(obstacle["low"]), np.array(obstacle["high"])
        axis.add_patch(plt.Rectangle((low[0], low[1]), high[0]-low[0], high[1]-low[1],
                                     color="tab:orange", alpha=.35, label="injected obstacle"))
    step = max(1, len(truth)//12)
    for row in truth[::step]:
        angle = row["yaw"]
        point = np.array(row["position"])
        axis.arrow(point[0], point[1], .06*np.cos(angle), .06*np.sin(angle),
                   width=.003, color="tab:green", alpha=.7)
    axis.scatter(*home[:2], marker="x", s=90, color="black", label="start / home")
    axis.scatter(*board[:2], marker="*", s=200, color="tab:red", label="Tag landing point")
    axis.scatter(p[-1, 0], p[-1, 1], marker="o", s=70, color="tab:purple", label="final position")
    axis.set(xlabel="x (m, initial-frame truth)", ylabel="y (m)",
             title=case["task"]+" / "+case["scenario"]+" ["+case["result"]+"]", aspect="equal")
    axis.grid(alpha=.25)
    axis.legend(fontsize=8, loc="best")
    path = outdir / (case["task"]+"-"+case["scenario"]+"-trajectory.png")
    figure.savefig(path, dpi=150)
    plt.close(figure)
    return path


def codeSeries(values):
    order = list(dict.fromkeys(values))
    return [order.index(value) for value in values], order


def plotTimeline(case, outdir):
    report, rows = load(case)
    truth = truthRows(rows)
    if not truth:
        return None
    startRows = [row for row in rows if row.get("kind") == "start"]
    start = startRows[0]["now"] if startRows else rows[0]["now"]
    t = np.array([row["t"] for row in truth])
    height = np.array([row["position"][2]-report["home_world"][2] for row in truth])
    vertical = np.array([row["velocity"][2] for row in truth])
    figure, axes = plt.subplots(3, 1, figsize=(9.6, 8.2), sharex=True, constrained_layout=True)
    axis = axes[0]
    states, stateNames = codeSeries([str(row.get("state")) for row in truth])
    axis.step(t, states, where="post", color="tab:blue", label="mission state")
    platform = [str(row.get("platform_state")) for row in truth]
    if any(item != "None" for item in platform):
        codes, names = codeSeries(platform)
        axis.step(t, codes, where="post", color="tab:orange", alpha=.8,
                  label="platform state: " + ", ".join(str(i)+"="+name for i, name in enumerate(names)))
    axis.set_yticks(range(len(stateNames)))
    axis.set_yticklabels(stateNames, fontsize=8)
    axis.set_ylabel("state")
    axis.legend(fontsize=7, loc="upper left")
    axis.grid(alpha=.25)
    for row in rows:
        if row.get("kind") == "vio_release":
            axis.axvline(row["now"]-start, color="tab:red", linestyle="--", lw=1.2, label="VIO released")
    axis = axes[1]
    attitude = [row["now"]-start for row in rows if row.get("kind") == "mavros_attitude"]
    velocity = [row["now"]-start for row in rows if row.get("kind") == "mavros_position"]
    top = max(1., float(t[-1])) + .5
    bins = np.arange(0, top, .25)
    axis.hist(attitude, bins=bins, color="tab:blue", alpha=.6, label="attitude setpoints (nav)")
    axis.hist(velocity, bins=bins, color="tab:orange", alpha=.6, label="velocity setpoints (platform)")
    axis.set(ylabel="setpoints / 0.25 s", title="control output ownership (mutually exclusive)")
    axis.legend(fontsize=8)
    axis.grid(alpha=.25)
    axis = axes[2]
    axis.plot(t, height, color="tab:green", label="height above start pose")
    axis.plot(t, vertical, color="tab:brown", alpha=.7, label="world vz")
    axis.axhline(0., color="grey", lw=.8)
    axis.set(xlabel="time since START (s)", ylabel="m / (m/s)")
    axis.legend(fontsize=8)
    axis.grid(alpha=.25)
    handover = report.get("handover_report", {})
    figure.suptitle(case["task"]+" / "+case["scenario"]+" ["+case["result"]+"]"
                    + "  released=" + str(handover.get("release_verified")))
    path = outdir / (case["task"]+"-"+case["scenario"]+"-timeline.png")
    figure.savefig(path, dpi=150)
    plt.close(figure)
    return path


def plotCombined(matrix, outdir):
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    plan, height, yaw, safety = axes.flat
    for case in matrix["cases"]:
        report, rows = load(case)
        truth = truthRows(rows)
        if not truth:
            continue
        t = np.array([row["t"] for row in truth])
        p = np.array([row["position"] for row in truth])
        home = np.array(report["home_world"])
        label = case["task"].replace("tag_", "")+" ["+case["result"]+"]"
        if case["scenario"] in SUCCESS_SCENARIOS:
            plan.plot(p[:, 0]-home[0], p[:, 1]-home[1], label=label)
            height.plot(t, p[:, 2]-home[2], label=label)
            if case["task"] == "tag_return_turn" and case["scenario"] == "nominal":
                angles = np.unwrap([row["yaw"] for row in truth])
                yaw.plot(t, np.degrees(angles-angles[0]), label=label)
        elif case["scenario"] in ("detour", "detour_failure", "return_blocked"):
            safety.plot(p[:, 0]-home[0], p[:, 1]-home[1],
                        label=case["scenario"]+" ["+case["result"]+"]")
    for axis in (plan, safety):
        axis.set(xlabel="initial x (m)", ylabel="initial y (m)", aspect="equal")
        axis.grid(alpha=.25)
        axis.legend(fontsize=7, loc="upper left")
    plan.scatter([0, 1], [0, 0], marker="x", color="black")
    plan.set_title("Success scenarios")
    height.set(title="Height above start", xlabel="time since START (s)", ylabel="m")
    height.grid(alpha=.25)
    height.legend(fontsize=7)
    yaw.axhline(180, color="grey", linestyle="--")
    yaw.axhline(-180, color="grey", linestyle="--")
    yaw.set(title="Task 2 rotation", xlabel="time since START (s)", ylabel="unwrapped yaw change (deg)")
    yaw.grid(alpha=.25)
    yaw.legend(fontsize=7)
    safety.set_title("Detour / return scenarios")
    figure.suptitle("PX4 SIH: synthetic VIO, raycast depth, rendered AprilTags")
    path = outdir/"trajectories.png"
    figure.savefig(path, dpi=150)
    plt.close(figure)
    return path


def fmt(value):
    if value is None:
        return "-"
    return f"{value:.3f}" if isinstance(value, float) else str(value)


def writeSummary(matrix, outdir):
    nominal = [case for case in matrix["cases"] if case.get("scenario") in SUCCESS_SCENARIOS]
    faults = [case for case in matrix["cases"] if case.get("scenario") not in SUCCESS_SCENARIOS]
    lines = ["# 独立 Tag 任务交接 SIH 矩阵", "",
             "- 矩阵结果：**" + str(matrix["result"]) + "**（正常 "
             + str(sum(c["result"] == "PASS" for c in nominal)) + "/" + str(len(nominal)) + "，故障 "
             + str(sum(c["result"] == "PASS" for c in faults)) + "/" + str(len(faults)) + "）",
             "- 输出目录：" + str(matrix.get("out", "")), ""]
    for title, group in (("正常场景", nominal), ("故障场景", faults)):
        lines += ["## " + title, "",
                  "| 任务 | 场景 | 结果 | 落点误差 m | 交接确认 | VIO 释放 | 计算退出 | 平台速度 | 备注 |",
                  "| --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
        for case in group:
            report, _ = load(case)
            handover = report.get("handover_report", {})
            lines.append("| " + case["task"] + " | " + case["scenario"] + " | " + case["result"] + " | "
                         + fmt(report.get("landing_xy_error_m")) + " | "
                         + str(handover.get("confirmed")) + " | " + str(handover.get("release_verified")) + " | "
                         + str((handover.get("compute") or {}).get("all_stopped")) + " | "
                         + str((handover.get("wire") or {}).get("platform_velocity")) + " | "
                         + str(case.get("error") or "") + " |")
        lines.append("")
    lines += ["## 阈值", "",
              "- 正常任务必须 PX4 接地且 disarmed；落点水平误差 <= 0.12 m；交接确认前高度变化 <= 0.06 m。",
              "- 成功场景必须 release_verified=true、自有计算进程全部退出、无关进程仍存活、平台接管后无姿态 setpoint。",
              "- 故障场景只要求预期的中止/保护行为、无碰撞、最终接地 disarm；不要求任务成功。",
              "- 任务 2 外出距离 0.9-1.15 m，末段航向误差 <= 0.15 rad（相对起飞航向 +180 度）。",
              "", "## 每例证据与命令", ""]
    for case in matrix["cases"]:
        report, _ = load(case)
        handover = report.get("handover_report", {})
        compute = handover.get("compute") or {}
        lines += ["### " + case["task"] + " / " + case["scenario"], "",
                  "- 结果：" + case["result"] + "；错误：" + str(case.get("error") or "-"),
                  "- 命令：" + " ".join(report.get("command", [])),
                  "- 证据：" + case["report"] + "、" + str(Path(case["report"]).with_name("trace.jsonl")),
                  "- 阈值：" + json.dumps(report.get("thresholds", {}), ensure_ascii=False),
                  "- 实测：落点误差 " + fmt(report.get("landing_xy_error_m")) + " m，交接确认 "
                  + str(handover.get("confirmed")) + "，释放 " + str(handover.get("release_verified"))
                  + "，CPU(before/after) " + str(compute.get("cpu_before_percent"))
                  + "/" + str(compute.get("cpu_after_percent")) + "，最终 armed="
                  + str(report.get("final_state", {}).get("armed")) + " landed="
                  + str(report.get("final_state", {}).get("landed_state")) + "。", ""]
    path = outdir/"summary.md"
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("matrix", type=Path)
    args = parser.parse_args()
    matrix = json.loads(args.matrix.read_text())
    outdir = args.matrix.parent/"plots"
    outdir.mkdir(parents=True, exist_ok=True)
    produced = []
    for case in matrix["cases"]:
        produced.append(plotTrajectory(case, outdir))
        produced.append(plotTimeline(case, outdir))
    produced.append(plotCombined(matrix, outdir))
    produced.append(writeSummary(matrix, outdir))
    for path in produced:
        if path is not None:
            print(path)


if __name__ == "__main__":
    main()
