"""MAVROS 请求档与 UART 下行容量核算；不执行设备操作。"""
import math


def observationIntervals(controlMode, *, sih=False, platform=False, ekf=False):
    intervals = ({30:50000., 32:500000., 245:1000000., 436:1000000., 331:1000000.}
                 if controlMode == "companion_attitude" else
                 {30:20000., 32:20000., 245:1000000., 436:100000., 331:100000.})
    if controlMode == "companion_attitude" and sih:
        intervals[30] = intervals[32] = 20000.
    if controlMode == "px4_velocity":
        intervals[32] = 50000.
        intervals[230] = 50000.
    if platform:
        intervals[30] = min(intervals[30], 50000.)
        intervals[32] = min(intervals[32], 50000.)
        intervals.update({85:50000., 230:100000.})
    if ekf:
        intervals[331] = min(intervals[331], 50000.)
        intervals[230] = min(intervals.get(230, 50000.), 50000.)
    return intervals


def budgetReport(baud, mavRate, payloadSizes, *, platform=False, signed=False, reserve=.2, controlMode="companion_attitude"):
    if (not all(math.isfinite(x) and x > 0 for x in (baud, mavRate))
            or not math.isfinite(reserve) or not 0 <= reserve < 1):
        raise ValueError("baud/mavRate must be positive; reserve must be in [0,1)")
    # Includes IMU/range/time and baseline status, in addition to the runtime controller requests.
    intervals = {0:1000000., 1:1000000., 2:1000000., 105:20000., 111:100000.,
                 132:50000., 147:1000000., **observationIntervals(controlMode, platform=platform, ekf=controlMode!="px4_velocity")}
    overhead = 12 + (13 if signed else 0)
    streams = [dict(message_id=mid, requested_hz=1e6/interval,
                    max_frame_bytes=payloadSizes[mid]+overhead,
                    bytes_per_s=(payloadSizes[mid]+overhead)*1e6/interval)
               for mid, interval in sorted(intervals.items())]
    required = sum(row["bytes_per_s"] for row in streams)
    # 8N1 costs ten bits per byte; MAV_X_RATE and physical UART are separate limits.
    usable = min(baud/10., mavRate)*(1.-reserve)
    uplink = None
    if controlMode == "px4_velocity":
        # UART 全双工；Pi 上行不再受飞控 MAV_X_RATE 下行额度限制。
        uplinkStreams = [dict(message_id=mid, requested_hz=hz,
            max_frame_bytes=payloadSizes[mid]+overhead,
            bytes_per_s=(payloadSizes[mid]+overhead)*hz)
            for mid,hz in ((331,50.),(84,50.),(111,20.),(0,1.))]
        uplinkRequired=sum(row["bytes_per_s"] for row in uplinkStreams)
        uplink=dict(required_max_bytes_s=uplinkRequired,usable_bytes_s=baud/10.*(1.-reserve),
                    streams=uplinkStreams)
    fits=required<=usable and (uplink is None or uplink["required_max_bytes_s"]<=uplink["usable_bytes_s"])
    return dict(status="BUDGET_OK_UNVERIFIED" if fits else "BUDGET_INSUFFICIENT",
                baud=baud, mav_rate_bytes_s=mavRate, signed=signed, reserve=reserve,
                required_max_bytes_s=required, usable_bytes_s=usable, streams=streams, uplink=uplink,
                limitations="Full payload upper bound; verify actual frames, all other streams and source timestamps on device.")
