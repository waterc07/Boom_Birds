"""把一次 SIH 场景的证据目录汇总成一行 JSON（矩阵用）。

失败原因与未执行原因分开：`final_state`/`disarmed` 是实测；`error` 表示汇总本身
读不到证据（例如进程被 timeout 杀掉），此时不得当作"通过"。

第 17 轮新增（回答"注入的故障是不是首要原因"）：
1. `latches` / `first_latch`：mission/status 时间线上**首个** latch 的 reason + detail；
2. `handoff_criteria`：从 `handoff_discontinuous:<k=v>` 里解析 4 条判据实测值与门限；
3. `fault_injection`：注入时刻（`fault_injected_at.txt`）与首个 latch 的先后关系 ——
   latch 早于注入即证明"注入故障不是首要原因"；注入的遥测高度样本（低空门限用）；
4. `hold_ready_gate_*`（为什么没进 OFFBOARD）、`depth_stats`（含量程内像素的可推导值）、
   `ego`（ready / "odom or depth lost" 计数）、`task_execution`（实际飞了多远，区分
   "执行任务"与"起飞→悬停→降落"）。
"""

import json
import math
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "boom_birds_control"))
from boom_birds_control.px4_frames import LocalFrameAlignment

#: 注入故障 → 首个 latch 里应当出现的关键词。用于"注入故障是不是首要原因"的判据，
#: 只做关键词匹配，最终判定仍以时间先后 + detail 实测值为准（见报告）。
FAULT_KEYWORDS = {
    "规划器重启": ("session", "restart", "planner"),
    "setpoint中断瞬态": ("session", "restart", "executor"),
    "恢复模式拒绝": ("offboard", "request_rejected"),
    "模式码注入": ("unknown", "manual", "mode"),
    "规划取消": ("cancel", "planning", "planner"),
    "Offboard中断瞬态": ("setpoint", "offboard", "link"),
    "深度挂起": ("depth", "sensor", "map", "planning", "link"),
    "里程计挂起": ("pose", "odom", "sensor"),
    "规划器挂起": ("planning", "planner", "link"),
    "人工取消": ("cancel", "manual", "operator"),
    "深度断流": ("depth", "sensor", "stream", "map"),
    "里程计断流": ("odom", "vio", "pose", "sensor", "stream"),
    "setpoint中断": ("setpoint", "link", "interrupt"),
    "模式确认失败": ("mode", "control", "service", "offboard"),
    "相机断流": ("camera", "sensor", "stream"),
    "飞控重启": ("restart", "session", "epoch", "control"),
    "飞控重启恢复": ("restart", "session", "epoch", "control"),
    "编排器重启": ("restart", "session", "epoch"),
}


def recorded_alignment(rows):
    values = set()
    for row in rows:
        data = (row.get("data") or {}).get("frame_alignment") or {}
        if data.get("invalidated_after_px4_restart"):
            raise ValueError("recorded alignment reset after PX4 restart")
        if not data.get("position_allowed"):
            continue
        origin = data.get("translation_m")
        yaw = data.get("yaw_offset_rad")
        if (not isinstance(origin, (list, tuple)) or len(origin) != 3
                or yaw is None or not all(math.isfinite(v) for v in (*origin, yaw))):
            raise ValueError("recorded alignment is invalid")
        values.add((*origin, yaw))
    if len(values) != 1:
        raise ValueError("recorded alignment missing or changed")
    x, y, z, yaw = values.pop()
    return LocalFrameAlignment(yaw_offset_rad=yaw, translation_m=(x, y, z),
                               source="recorded control status")


def load(path, name):
    f = path / name
    if not f.is_file():
        return []
    rows = []
    for line in f.read_text(errors="replace").splitlines():
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except Exception:  # noqa: BLE001
            pass
    return rows


def parse_handoff(detail):
    """`handoff_discontinuous:pose_age=0.2>0.15,dist_pos=0.7>0.5` → 实测值 + 门限。"""
    if not detail or "handoff_discontinuous" not in detail:
        return None
    body = detail.split(":", 1)[1] if ":" in detail else ""
    values, limits, raw = {}, {}, []
    for part in body.split(","):
        part = part.strip()
        if not part:
            continue
        raw.append(part)
        if "=" not in part:
            continue
        key, val = part.split("=", 1)
        key = key.strip()
        if ">" in val:
            v, lim = val.split(">", 1)
            try:
                values[key] = float(v)
                limits[key] = float(lim)
            except ValueError:
                values[key] = v
        else:
            try:
                values[key] = float(val)
            except ValueError:
                values[key] = val
    return {"values": values, "limits": limits, "raw": raw}


def mission_fault_events(rows, t0=None):
    """保留短暂闭锁；恢复状态不计为闭锁。history 同一事件取第一次记录。"""
    events = []
    seen = set()
    for row in rows:
        data = row.get("data") or {}
        candidates = data.get("history")
        if candidates is None:
            candidates = [dict(t=row.get("t"), state=data.get("state"),
                               reason=data.get("reason"),
                               detail=(data.get("recovery") or {}).get("detail"))]
        for e in candidates:
            if e.get("state") not in ("RECOVERING", "FAULT_LATCHED") or not e.get("reason"):
                continue
            key = (e.get("state"), e.get("reason"), round(e.get("t") or 0, 2))
            if key in seen:
                continue
            seen.add(key)
            events.append(dict(t=e.get("t"),
                               t_rel_s=None if t0 is None or e.get("t") is None else round(e["t"]-t0, 3),
                               state=e["state"], reason=e["reason"], detail=e.get("detail")))
    return sorted(events, key=lambda e: e["t"] or 0)


def minimum_cloud_distance(points, cloud):
    import numpy as np
    points = np.asarray(points, dtype=float)
    cloud = np.asarray(cloud, dtype=float)
    if (points.ndim != 2 or cloud.ndim != 2 or points.shape[1] != 3
            or cloud.shape[1] != 3 or not len(points) or not len(cloud)
            or not np.isfinite(points).all() or not np.isfinite(cloud).all()):
        raise ValueError("invalid position or cloud")
    best = float("inf")
    for i in range(0, len(points), 32):
        for j in range(0, len(cloud), 2048):
            delta = points[i:i+32, None, :] - cloud[None, j:j+2048, :]
            best = min(best, float(np.sqrt(np.sum(delta * delta, axis=2)).min()))
    return best


def main():
    out = {"scenario": sys.argv[2], "fault": sys.argv[3], "obl_action": sys.argv[4]}
    out["depth_max_range_m"] = (float(sys.argv[5]) if len(sys.argv) > 5 and sys.argv[5].strip() else None)
    d = Path(sys.argv[1])
    if not d.is_dir():
        out["error"] = "evidence_dir_missing"
        print(json.dumps(out, ensure_ascii=False))
        return
    args = (d / "run.args")
    out["run_args"] = args.read_text(errors="replace").strip() if args.is_file() else None
    if (d / "run_exit.txt").is_file():
        out["run_exit"] = (d / "run_exit.txt").read_text(errors="replace").strip()

    mission = load(d, "recorder_mission.jsonl")
    execs = load(d, "recorder_execution.jsonl")
    ctrl = load(d, "recorder_control.jsonl")
    topics = load(d, "recorder_topics.jsonl")

    t0 = mission[0].get("t") if mission else None
    out["t0"] = t0

    # ---------------------------------------------------------------- 任务状态
    if mission:
        seq = []
        for r in mission:
            s = (r.get("data") or {}).get("state")
            if not seq or seq[-1] != s:
                seq.append(s)
        latest_history = (mission[-1].get("data") or {}).get("history") or []
        if latest_history:
            seq = (["IDLE"] if seq and seq[0] == "IDLE" else []) + [e["state"] for e in latest_history]
            out["state_history"] = latest_history
        out["state_sequence"] = seq
        last = (mission[-1].get("data") or {})
        out["final_state"] = last.get("state")
        rec = last.get("recovery") or {}
        out["recovery"] = {k: rec.get(k) for k in ("fault", "source", "attempts", "budget", "revoked", "detail")}

        out["goal_reached"] = last.get("goal_reached")
        out["recovery"]["total_attempts"] = rec.get("total_attempts")
        events = mission_fault_events(mission, t0)
        latches = [e for e in events if e["state"] == "FAULT_LATCHED"]
        out["fault_events"] = events
        out["first_fault"] = events[0] if events else None
        out["latches"] = latches
        out["first_latch"] = latches[0] if latches else None
        for entry in latches:
            parsed = parse_handoff(entry.get("detail"))
            if parsed:
                out["handoff_criteria"] = parsed
                out["handoff_latch"] = entry
                break

        # 门控逐项值：最后一次 + HOLD_READY 期间的最后一次（未进 OFFBOARD 的直接原因）
        gate_last = None
        gate_hold = None
        gate_seq = []
        for r in mission:
            data = r.get("data") or {}
            g = data.get("hold_ready_gate")
            if not g:
                continue
            gate_last = g
            if data.get("state") == "HOLD_READY":
                gate_hold = g
                if not gate_seq or gate_seq[-1] != g:
                    gate_seq.append(g)
        out["hold_ready_gate_last"] = gate_last
        out["hold_ready_gate_at_hold"] = gate_hold
        out["hold_ready_gate_seq"] = gate_seq[:6]
        modes = [(r.get("data") or {}).get("mode_detail") for r in mission]
        out["mode_details"] = sorted({m for m in modes if m})

    mode_sequence = []
    for row in execs:
        data = row.get("data") or {}
        mode = data.get("mode_detail")
        if mode and (not mode_sequence or mode_sequence[-1]["mode_detail"] != mode):
            mode_sequence.append({"t": row.get("t"), "mode_detail": mode, "armed": data.get("armed")})
    out["mode_sequence"] = mode_sequence

    # ---------------------------------------------------------------- 执行/任务量
    if execs:
        goal = None
        m = re.search(r"goal=([-\d. ]+)", out.get("run_args") or "")
        if m:
            try:
                goal = tuple(float(v) for v in m.group(1).split())
            except ValueError:
                goal = None
        alignment = None
        try:
            alignment = recorded_alignment(ctrl)
        except ValueError as exc:
            out["goal_distance_error"] = str(exc)
        out["scene_origin_ros_m"] = alignment.translation_m if alignment else None
        out["scene_yaw_offset_rad"] = alignment.yaw_offset_rad if alignment else None
        out["alignment_source"] = "recorded control status"
        pts = []
        for r in execs:
            e = r.get("data") or {}
            if not e.get("position_known"):
                continue
            p = e["position_ned"]
            if alignment is not None:
                pts.append((r.get("t"), tuple(alignment.position_ned_to_ros(p)), e))
        best = None
        if alignment is not None and goal and len(goal) == 3:
            for _, ros, _e in pts:
                dist = math.dist(ros, goal)
                if best is None or dist < best:
                    best = dist
        out["closest_goal_m"] = None if best is None else round(best, 3)
        out["final_armed"] = (execs[-1].get("data") or {}).get("armed")
        out["final_landed_state"] = (execs[-1].get("data") or {}).get("landed_state")
        if pts:
            start = pts[0][1]
            horiz = [math.dist(p[:2], start[:2]) for _, p, _ in pts]
            path = sum(math.dist(pts[i - 1][1], pts[i][1]) for i in range(1, len(pts)))
            out["task_execution"] = {
                "n": len(pts),
                "n_offboard": sum(1 for _, _, e in pts if e.get("mode_detail") == "offboard"),
                "n_armed": sum(1 for _, _, e in pts if e.get("armed")),
                "n_sending": sum(1 for _, _, e in pts if e.get("sending")),
                "start_ros": [round(v, 3) for v in start],
                "last_ros": [round(v, 3) for v in pts[-1][1]],
                "max_horizontal_excursion_m": round(max(horiz), 3),
                "path_length_m": round(path, 3),
                "max_altitude_ros_m": round(max(p[2] for _, p, _ in pts), 3),
                "max_altitude_at_offboard_m": (round(max((p[2] for _, p, e in pts
                                                           if e.get("mode_detail") == "offboard"), default=0.0), 3)),
                "modes": sorted({e.get("mode_detail") for _, _, e in pts if e.get("mode_detail")}),
                "reasons_last": list(((execs[-1].get("data") or {}).get("reasons") or []))[:12],
            }
    cloud_file = d / "scene_cloud.npy"
    cloud_meta = d / "scene_cloud.json"
    if cloud_file.is_file() and cloud_meta.is_file() and execs:
        try:
            import numpy as np
            metadata = json.loads(cloud_meta.read_text())
            if metadata["frame_id"] not in ("world", "global"):
                raise ValueError("unrecognized scene cloud frame")
            alignment = recorded_alignment(ctrl)
            centers = [alignment.position_ned_to_ros(r["data"]["position_ned"])
                       for r in execs if r["data"].get("position_known")
                       and r["data"].get("armed") and r["data"].get("mode_detail") == "offboard"]
            out["center_to_raw_cloud_offboard_m"] = round(minimum_cloud_distance(
                centers, np.load(cloud_file, allow_pickle=False)), 3)
            out["clearance_scope"] = "sampled armed Offboard centers to raw scene cloud; no body envelope"
        except (ValueError, KeyError, OSError) as exc:
            out["clearance_error"] = str(exc)
    if ctrl:
        c = (ctrl[-1].get("data") or {}).get("counters") or {}
        out["setpoints_sent"] = c.get("setpoints_sent")
        out["position_cmd_received"] = c.get("position_cmd_received")

    # ---------------------------------------------------------------- 深度/流统计
    if topics:
        last = topics[-1].get("data") or {}
        ds = dict(last.get("depth_stats") or {})
        rng = out.get("depth_max_range_m")
        if rng is not None and ds.get("max_finite_m") is not None:
            # 只能**推导**：depth_node 若把超量程值裁掉，则 max_finite_m<=量程 时
            # 所有有限像素都在量程内；否则不给结论（None）。
            ds["pixels_within_range_derived"] = (ds.get("pixels_finite_positive")
                                                 if ds["max_finite_m"] <= rng else None)
            ds["within_range_rule"] = f"finite if max_finite_m<={rng} else indeterminate"
        out["depth_stats"] = ds
        out["stream_gaps"] = last.get("stream_gaps")
        out["imu_publisher_count"] = last.get("imu_publisher_count")
        out["imu_publishers"] = last.get("imu_publishers")
        out["depth_messages"] = last.get("depth_messages")
        out["imu_messages"] = last.get("imu_messages")
        out["camera_pose_z"] = last.get("camera_pose_z")
        if last.get("depth_stats", {}).get("decode_error"):
            out["depth_decode_error"] = last["depth_stats"]["decode_error"]

    # ---------------------------------------------------------------- EGO 侧计数
    log = (d / "launch.log")
    if log.is_file():
        lg = log.read_text(errors="replace")
        out["ego"] = {
            "traj_server_ready": lg.count("[Traj server]: ready."),
            "odom_or_depth_lost": lg.count("odom or depth lost"),
            "project_goal": len(re.findall(r"PROJECT_GOAL", lg)),
            "project_cancel": len(re.findall(r"PROJECT_CANCEL", lg)),
            "plan_success": len(re.findall(r"plan_success=1", lg)),
            "traj_success": len(re.findall(r"traj \d+ success\.", lg)),
            "traj_failed": len(re.findall(r"traj \d+ failed\.", lg)),
            "plan_fail": len(re.findall(r"plan_success=0", lg)),
            "lines": len(lg.splitlines()),
        }
        out["ego_events"] = re.findall(
            r"PROJECT_(?:GOAL|CANCEL)|plan_success=\d|traj \d+ (?:success|failed)\.", lg)[-6:]

    # ---------------------------------------------------------------- 故障注入
    inj = {"note": None, "t": None, "telemetry": None}
    ft = (d / "fault.txt")
    if ft.is_file():
        inj["note"] = ft.read_text(errors="replace").strip()
    fts = (d / "fault_injected_at.txt")
    if fts.is_file() and out["fault"] not in (None, "", "none"):
        try:
            inj["t"] = float(fts.read_text().strip())
        except ValueError:
            inj["t"] = None
    tel = (d / "fault_telemetry.json")
    if tel.is_file():
        try:
            inj["telemetry"] = json.loads(tel.read_text())
        except Exception:  # noqa: BLE001
            inj["telemetry"] = "unreadable"
    fl = out.get("first_fault")
    inj["correlation_event"] = "first_fault"
    if inj["t"] is not None and fl is not None and fl.get("t") is not None:
        inj["first_latch_delay_s"] = round(fl["t"] - inj["t"], 3)
        inj["first_latch_after_injection"] = fl["t"] > inj["t"]
        inj["state_before_latch"] = fl.get("state")
    elif inj["t"] is not None:
        inj["first_latch_after_injection"] = None
        inj["no_latch"] = True
    keys = FAULT_KEYWORDS.get(out["fault"] or "", ())
    hay = " ".join(str(fl.get(k) or "") for k in ("reason", "detail")) if fl else ""
    inj["expected_keywords"] = list(keys)
    inj["keyword_hit"] = bool(keys) and any(k in hay.lower() for k in keys)
    if inj.get("first_latch_after_injection") is True:
        inj["fault_is_primary_suspected"] = bool(inj["keyword_hit"])
    elif inj.get("first_latch_after_injection") is False:
        inj["fault_is_primary_suspected"] = False
    else:
        inj["fault_is_primary_suspected"] = None
    out["fault_injection"] = inj

    # ---------------------------------------------------------------- 收尾（PX4 读回）
    fin = (d / "final_state.txt")
    if fin.is_file():
        txt = fin.read_text(errors="replace")
        out["disarmed"] = "disarmed=yes" in txt
        out["disarmed_readback"] = "disarmed=yes" in txt
        out["disarmed_source"] = "px4-commander status（PX4 回读）"
        out["land_detected"] = "landed: True" in txt
        out["final_state_txt"] = txt.strip().splitlines()[:6]
    else:
        out["disarmed_readback"] = None
        out["disarmed_source"] = None

    print(json.dumps(out, ensure_ascii=False))


if __name__ == "__main__":
    main()
