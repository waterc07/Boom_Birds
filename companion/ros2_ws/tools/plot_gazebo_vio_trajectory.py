"""按仿真时间对齐 OpenVINS 原始位置与 ULog 真值并绘图。"""

import argparse
import json
import math
from pathlib import Path


def rosYaw(quaternion):
    x, y, z, w = quaternion
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def plot(run, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from pyulog import ULog

    ulog = ULog(str(next(run.glob("log/*/*.ulg"))), ["vehicle_local_position_groundtruth"])
    truth = ulog.get_dataset("vehicle_local_position_groundtruth").data
    truthTime = truth["timestamp"] / 1.e6
    truthPosition = np.column_stack((truth["y"], truth["x"], -truth["z"]))
    with (run / "vio-trajectory.jsonl").open() as stream:
        samples = [json.loads(line) for line in stream if line.endswith("\n")]
    samples = [s for s in samples if truthTime[0] <= s["stamp"] <= truthTime[-1]]
    if len(samples) < 2:
        raise ValueError("OpenVINS and Gazebo do not yet share a logged time interval")
    t = np.asarray([s["stamp"] for s in samples])
    vio = np.asarray([s["position"] for s in samples])
    matched = np.column_stack([np.interp(t, truthTime, x) for x in truthPosition.T])
    ready = np.asarray([s["last_visual_stamp"] is not None
                        and -.004 <= s["stamp"] - s["last_visual_stamp"] <= .50 for s in samples])
    reference = int(np.flatnonzero(ready)[0]) if np.any(ready) else 0
    yaw = math.pi/2 - float(np.interp(t[reference], truthTime, np.unwrap(truth["heading"])))
    angle = yaw - rosYaw(samples[reference]["quaternion"])
    c, s = math.cos(angle), math.sin(angle)
    fixedRotation = np.array(((c, -s, 0), (s, c, 0), (0, 0, 1)))
    aligned = (vio - vio[reference]) @ fixedRotation.T + matched[reference]
    error = np.linalg.norm(aligned - matched, axis=1)
    validVio = np.where(ready[:, None], aligned, np.nan)
    route = np.asarray(json.loads((run / "arena-route.json").read_text()))
    trace = run / "task3-integrated.jsonl"
    with trace.open() as stream:
        events = [json.loads(line) for line in stream if line.endswith("\n")]
    terminal = next((item for item in reversed(events) if item["stage"] in ("failed", "complete")), None)

    endTime = min(truthTime[-1], terminal["sim_s"]) if terminal else t[-1]
    truthWindow = truthTime <= endTime

    scans = []
    scanStart = None
    for event in events:
        if event["stage"] == "arena_imu_turn_started":
            scanStart = event["sim_s"]
        elif event["stage"] in ("arena_imu_turn_recovered", "failed") and scanStart is not None:
            scans.append((scanStart, event["sim_s"]))
            scanStart = None

    fig = plt.figure(figsize=(13, 9), layout="constrained")
    layout = fig.add_gridspec(4, 2, width_ratios=(1.28, 1))
    mapAxis = fig.add_subplot(layout[:, 0])
    mapAxis.plot(*route.T, color="#a8a8b3", ls="--", lw=1.3, label="Planned route")
    mapAxis.plot(*truthPosition[truthWindow, :2].T, color="#125a8a", lw=2, label="Gazebo truth")
    mapAxis.plot(*aligned[:, :2].T, color="#e88d43", alpha=.45, lw=1.4, ls=":", label="OpenVINS propagated")
    mapAxis.plot(*validVio[:, :2].T, color="#c34810", lw=1.8, label="OpenVINS with fresh visuals")
    mapAxis.scatter(*route[0], marker="o", s=38, color="#20664f", label="Takeoff / landing")
    mapAxis.scatter(-11.35, 0, marker="s", s=45, color="#a24960", label="Red base")
    mapAxis.set(xlabel="World X (m)", ylabel="World Y (m)", title="Top view", aspect="equal")
    mapAxis.legend(loc="lower center", fontsize=8)
    timeAxes = []
    for direction, name in enumerate("XYZ"):
        ax = fig.add_subplot(layout[direction, 1])
        timeAxes.append(ax)
        ax.plot(truthTime[truthWindow], truthPosition[truthWindow, direction],
                color="#125a8a", lw=1.7)
        ax.plot(t, aligned[:, direction], color="#e88d43", lw=1.3, alpha=.55, ls=":")
        ax.plot(t, validVio[:, direction], color="#c34810", lw=1.25)
        if name == "Z":
            initialZ = float(np.median(truthPosition[:50, 2]))
            ax.axhline(initialZ + .30, color="#92939c", ls="--", lw=1, label="Takeoff Z + 0.30 m")
            ax.legend(loc="best", fontsize=8)
        ax.set(xlim=(truthTime[0], endTime), ylabel=f"{name} (m)")
        ax.grid(alpha=.23)
    ax = fig.add_subplot(layout[3, 1])
    timeAxes.append(ax)
    ax.plot(t, error, color="#ae7b97", lw=1.4, ls=":")
    ax.plot(t, np.where(ready, error, np.nan), color="#753a74", lw=1.6)
    ax.set(xlim=(truthTime[0], endTime), xlabel="Gazebo simulation time (s)", ylabel="3D error (m)")
    ax.grid(alpha=.23)
    for axis in timeAxes:
        for start, stop in scans:
            axis.axvspan(start, stop, color="#e6bb52", alpha=.22, lw=0)
    mapAxis.grid(alpha=.23)
    result = terminal["stage"] if terminal else "in progress"
    fig.suptitle(f"RMUC2027 | {run.name} | Gazebo truth and raw OpenVINS ({result})", fontsize=16)
    fig.savefig(output, dpi=170)
    plt.close(fig)
    csv = np.column_stack((t, matched, aligned, error, ready))
    np.savetxt(output.with_suffix(".csv"), csv, delimiter=",",
               header="sim_time_s,truth_x_m,truth_y_m,truth_z_m,vio_x_m,vio_y_m,vio_z_m,error_3d_m,vio_visual_fresh",
               comments="")
    stats = dict(samples=len(t), comparison_interval_s=[float(t[0]), float(t[-1])],
                 reference_time_s=float(t[reference]), fresh_samples=int(ready.sum()),
                 rmse_3d_m=float(np.sqrt(np.mean(error[ready]**2))) if np.any(ready) else None,
                 max_error_3d_m=float(np.max(error[ready])) if np.any(ready) else None,
                 terminal_stage=terminal, inertial_turn_intervals_s=scans, alignment="one initial rigid yaw and translation; no scale or retiming")
    output.with_suffix(".json").write_text(json.dumps(stats, indent=2)+"\n")
    print(json.dumps(stats))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    plot(args.run, args.out)
