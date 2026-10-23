from pathlib import Path
import argparse
import json
import numpy as np
from pyulog import ULog


def ds(ulog, name, multiId=0):
    try:
        return ulog.get_dataset(name, multiId).data
    except (KeyError, IndexError):
        return None


def intervals(t, mask):
    if len(t) == 0:
        return []
    mask = np.asarray(mask, bool)
    edges = np.flatnonzero(np.diff(np.r_[False, mask, False]))
    return [(float(t[a]), float(t[b - 1])) for a, b in edges.reshape(-1, 2)]


def valsIn(d, field, spans):
    if d is None or not spans or field not in d:
        return np.array([])
    t = d["timestamp"] / 1e6
    m = np.zeros(len(t), bool)
    for a, b in spans:
        m |= (t >= a) & (t <= b)
    v = np.asarray(d[field], float)[m]
    return v[np.isfinite(v)]


def stats(v):
    if len(v) == 0:
        return None
    return {
        "n": int(len(v)), "min": float(np.min(v)), "max": float(np.max(v)),
        "mean": float(np.mean(v)), "std": float(np.std(v)),
        "p05": float(np.percentile(v, 5)), "p95": float(np.percentile(v, 95)),
        "ptp": float(np.ptp(v)), "rms": float(np.sqrt(np.mean(v * v))),
    }


def boolSummary(d, field, spans):
    v = valsIn(d, field, spans)
    return None if not len(v) else {"true_fraction": float(np.mean(v > 0.5)), "transitions": int(np.sum(np.diff(v > .5) != 0))}


def analyze(path):
    u = ULog(str(path))
    vcm = ds(u, "vehicle_control_mode")
    vs = ds(u, "vehicle_status")
    if vcm is None:
        spans = []
    else:
        t = vcm["timestamp"] / 1e6
        mask = (vcm["flag_armed"] > 0) & (vcm["flag_control_altitude_enabled"] > 0)
        spans = intervals(t, mask)
    allSpan = [(u.start_timestamp / 1e6, u.last_timestamp / 1e6)]
    armed = intervals(vcm["timestamp"] / 1e6, vcm["flag_armed"] > 0) if vcm is not None else []
    lp = ds(u, "vehicle_local_position")
    lpsp = ds(u, "vehicle_local_position_setpoint")
    rng = ds(u, "distance_sensor")
    flow = ds(u, "sensor_optical_flow")
    batt = ds(u, "battery_status")
    motors = ds(u, "actuator_motors")
    land = ds(u, "vehicle_land_detected")
    flags = ds(u, "estimator_status_flags")
    fail = ds(u, "failsafe_flags")
    rngAid = ds(u, "estimator_aid_src_rng_hgt")
    baroAid = ds(u, "estimator_aid_src_baro_hgt")
    imu0, imu1 = ds(u, "vehicle_imu_status", 0), ds(u, "vehicle_imu_status", 1)
    selector = ds(u, "estimator_selector_status")

    z = valsIn(lp, "z", spans)
    zsp = valsIn(lpsp, "z", spans)
    # Interpolated tracking error only where both finite.
    zerr = np.array([])
    if lp is not None and lpsp is not None and spans:
        tl = lp["timestamp"] / 1e6
        ts = lpsp["timestamp"] / 1e6
        m = np.zeros(len(tl), bool)
        for a, b in spans: m |= (tl >= a) & (tl <= b)
        zi = np.asarray(lp["z"], float)[m]
        tsi = tl[m]
        goodsp = np.isfinite(lpsp["z"])
        if np.sum(goodsp) >= 2:
            zspi = np.interp(tsi, ts[goodsp], np.asarray(lpsp["z"], float)[goodsp])
            good = np.isfinite(zi) & np.isfinite(zspi)
            zerr = zi[good] - zspi[good]

    motorSpread = np.array([])
    motorMax = np.array([])
    if motors is not None and spans:
        tm = motors["timestamp"] / 1e6
        mm = np.zeros(len(tm), bool)
        for a,b in spans: mm |= (tm >= a) & (tm <= b)
        ctrl = np.column_stack([motors[f"control[{i}]"] for i in range(4)])[mm]
        ctrl = ctrl[np.all(np.isfinite(ctrl), axis=1)]
        if len(ctrl):
            motorSpread = np.ptp(ctrl, axis=1)
            motorMax = np.max(ctrl, axis=1)

    def clipping(d):
        if d is None: return None
        return {axis: int(np.max(d[f"accel_clipping[{i}]"])) for i, axis in enumerate("xyz")}

    messages = [m.message for m in u.logged_messages]
    p = u.initial_parameters
    return {
        "file": path.name,
        "duration_s": float((u.last_timestamp-u.start_timestamp)/1e6),
        "armed_intervals_s": armed,
        "altitude_control_intervals_s": spans,
        "params": {k: p.get(k) for k in ["EKF2_HGT_REF","EKF2_RNG_CTRL","EKF2_BARO_CTRL","EKF2_OF_CTRL","EKF2_RNG_A_HMAX","EKF2_RNG_A_VMAX","MPC_THR_HOVER","MPC_THR_MIN","MPC_THR_MAX","MPC_Z_VEL_MAX_UP","MPC_Z_VEL_MAX_DN"]},
        "height_control": {"z_m": stats(z), "z_setpoint_m": stats(zsp), "z_error_m": stats(zerr), "vz_m_s": stats(valsIn(lp,"vz",spans)), "dist_bottom_m": stats(valsIn(lp,"dist_bottom",spans)), "raw_range_m": stats(valsIn(rng,"current_distance",spans))},
        "range": {"quality": stats(valsIn(rng,"signal_quality",spans)), "flow_quality": stats(valsIn(flow,"quality",spans)), "valid": boolSummary(lp,"dist_bottom_valid",spans), "rng_fused": boolSummary(rngAid,"fused",spans), "rng_rejected": boolSummary(rngAid,"innovation_rejected",spans), "rng_test_ratio": stats(valsIn(rngAid,"test_ratio",spans)), "rng_innov_m": stats(valsIn(rngAid,"innovation",spans)), "cs_rng_hgt": boolSummary(flags,"cs_rng_hgt",spans), "cs_rng_fault": boolSummary(flags,"cs_rng_fault",spans), "reject_hagl": boolSummary(flags,"reject_hagl",spans)},
        "baro": {"fused": boolSummary(baroAid,"fused",spans), "rejected": boolSummary(baroAid,"innovation_rejected",spans), "test_ratio": stats(valsIn(baroAid,"test_ratio",spans)), "innovation_m": stats(valsIn(baroAid,"innovation",spans)), "cs_baro_hgt": boolSummary(flags,"cs_baro_hgt",spans)},
        "control": {"motor_spread": stats(motorSpread), "motor_max": stats(motorMax), "battery_v": stats(valsIn(batt,"voltage_v",spans)), "current_a": stats(valsIn(batt,"current_a",spans)), "landed": boolSummary(land,"landed",spans), "local_alt_invalid": boolSummary(fail,"local_altitude_invalid",spans), "local_pos_invalid": boolSummary(fail,"local_position_invalid",spans), "termination": boolSummary(vcm,"flag_control_termination_enabled",allSpan)},
        "imu_clipping_all": {"imu0": clipping(imu0), "imu1": clipping(imu1)},
        "estimator_switching": {
            "primary_instances": sorted(set(map(int, selector["primary_instance"]))) if selector is not None else [],
            "instance_changed_count_max": int(np.max(selector["instance_changed_count"])) if selector is not None else None,
            "z_reset_counter_max": int(np.max(lp["z_reset_counter"])) if lp is not None else None,
            "z_reset_delta_max_m": float(np.max(np.abs(lp["delta_z"]))) if lp is not None else None,
            "dist_bottom_reset_counter_max": int(np.max(lp["dist_bottom_reset_counter"])) if lp is not None else None,
            "accel_fault_detected": bool(np.any(selector["accel_fault_detected"] > 0)) if selector is not None else None,
            "gyro_fault_detected": bool(np.any(selector["gyro_fault_detected"] > 0)) if selector is not None else None,
        },
        "messages": [m for m in messages if any(s in m.lower() for s in ["fail", "clip", "land", "kill", "termination", "height", "range", "optical"])],
        "nav_states": sorted(set(map(int, vs["nav_state"]))) if vs is not None else [],
        "failsafe_any": bool(np.any(vs["failsafe"] > 0)) if vs is not None else None,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="历史高度控制日志分析；输出不是飞行验收。")
    parser.add_argument("logs", type=Path, nargs="+")
    parser.add_argument("--out", type=Path, required=True, help="新的输出目录，不覆盖旧报告")
    args = parser.parse_args()
    for path in args.logs:
        if not path.is_file(): parser.error(f"日志不存在：{path}")
    outdir = args.out
    outdir.mkdir(parents=True, exist_ok=False)
    out = [analyze(path) for path in args.logs]
    target = outdir / "metrics.json"
    target.write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
    summary = []
    for item in out:
        switch = item["estimator_switching"]
        summary.append({
            "log": item["file"].split("_UnknownDate")[0],
            "duration_s": round(item["duration_s"], 2),
            "primary_instances": ",".join(map(str, switch["primary_instances"])),
            "max_z_reset_m": switch["z_reset_delta_max_m"],
            "range_valid_rate": (item["range"]["valid"] or {}).get("true_fraction"),
            "range_fused_rate": (item["range"]["rng_fused"] or {}).get("true_fraction"),
            "imu1_z_clipping": item["imu_clipping_all"]["imu1"]["z"],
            "flow_quality_min": (item["range"]["flow_quality"] or {}).get("min"),
            "flow_quality_mean": (item["range"]["flow_quality"] or {}).get("mean"),
            "failsafe": item["failsafe_any"],
        })
    (outdir / "log_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(target)
    print(json.dumps(out, ensure_ascii=False, indent=2))
