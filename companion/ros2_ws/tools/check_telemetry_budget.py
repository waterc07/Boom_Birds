#!/usr/bin/env python3
"""离线计算 MAVLink 预算，并分析已归档 UART RX；不打开串口或发送消息。"""
import argparse
import json
from pathlib import Path
import statistics
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src/boom_birds_control"))
from boom_birds_control.telemetry_budget import budgetReport


def analyzeRx(path, duration, streams):
    from pymavlink.dialects.v20 import development as mav
    parser = mav.MAVLink(None)
    parser.robust_parsing = True
    counts, stamps = {}, {}
    bad = 0
    with Path(path).open("rb") as handle:
        while chunk := handle.read(65536):
            for message in parser.parse_buffer(chunk) or []:
                if message.get_type() == "BAD_DATA":
                    bad += 1
                    continue
                mid = message.get_msgId()
                counts[mid] = counts.get(mid, 0)+1
                if hasattr(message, "time_boot_ms"):
                    stamp = message.time_boot_ms*.001
                elif hasattr(message, "time_usec"):
                    stamp = message.time_usec*1e-6
                else:
                    continue
                stamps.setdefault(mid, []).append(stamp)
    rows = []
    for stream in streams:
        mid = stream["message_id"]
        times = stamps.get(mid, [])
        gaps = [b-a for a,b in zip(times,times[1:]) if b > a]
        rows.append(dict(message_id=mid, requested_hz=stream["requested_hz"],
            frames=counts.get(mid, 0), received_hz=counts.get(mid, 0)/duration,
            source_gap_median_s=statistics.median(gaps) if gaps else None,
            source_gap_max_s=max(gaps) if gaps else None,
            non_increasing_stamps=sum(b <= a for a,b in zip(times,times[1:]))))
    return dict(status="OBSERVATION_ONLY", duration_s=duration, bad_data_blocks=bad,
                rx_bytes=Path(path).stat().st_size, streams=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baud", type=int, required=True, help="实际 UART 波特率；不修改设备")
    parser.add_argument("--mav-rate", type=float, required=True, help="对应 MAV_X_RATE 实际回读值，bytes/s；0 自动档须先查其实际分配")
    parser.add_argument("--control-mode", choices=("companion_attitude","px4_velocity"), default="companion_attitude")
    parser.add_argument("--platform", action="store_true", help="额外计入平台回读流")
    parser.add_argument("--signed", action="store_true")
    parser.add_argument("--reserve", type=float, default=.2)
    parser.add_argument("--rx-bin", type=Path)
    parser.add_argument("--duration", type=float, help="完整 RX 采集时长，秒")
    args = parser.parse_args()
    from pymavlink.dialects.v20 import development as mav
    sizes = {mid: definition.unpacker.size for mid,definition in mav.mavlink_map.items()}
    try:
        report = budgetReport(args.baud, args.mav_rate, sizes, platform=args.platform,
                              signed=args.signed, reserve=args.reserve, controlMode=args.control_mode)
        if args.rx_bin:
            import math
            if args.duration is None or not math.isfinite(args.duration) or args.duration <= 0:
                parser.error("--rx-bin requires a finite positive --duration")
            report["observation"] = analyzeRx(args.rx_bin, args.duration, report["streams"])
    except ValueError as exc:
        parser.error(str(exc))
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "BUDGET_OK_UNVERIFIED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
