"""MAVROS 请求档与 UART 下行容量核算；不执行设备操作。"""
import math


def observationIntervals(controlMode, *, sih=False, platform=False, ekf=False):
    intervals = ({30:50000., 32:500000., 245:1000000., 436:1000000., 331:1000000.}
                 if controlMode == "companion_attitude" else
                 {30:20000., 32:20000., 245:1000000., 436:100000., 331:100000.})
    if controlMode == "companion_attitude" and sih:
        intervals[30] = intervals[32] = 20000.
    if platform:
        intervals[30] = min(intervals[30], 50000.)
        intervals[32] = min(intervals[32], 50000.)
        intervals.update({85:50000., 230:100000.})
    if ekf:
        intervals[331] = min(intervals[331], 50000.)
        intervals[230] = min(intervals.get(230, 50000.), 50000.)
    return intervals


def budgetReport(baud, mavRate, payloadSizes, *, platform=False, signed=False, reserve=.2):
    if (not all(math.isfinite(x) and x > 0 for x in (baud, mavRate))
            or not math.isfinite(reserve) or not 0 <= reserve < 1):
        raise ValueError("baud/mavRate must be positive; reserve must be in [0,1)")
    # Includes IMU/range/time and baseline status, in addition to the runtime controller requests.
    intervals = {0:1000000., 1:1000000., 2:1000000., 105:20000., 111:100000.,
                 132:50000., 147:1000000., **observationIntervals("companion_attitude", platform=platform, ekf=True)}
    overhead = 12 + (13 if signed else 0)
    streams = [dict(message_id=mid, requested_hz=1e6/interval,
                    max_frame_bytes=payloadSizes[mid]+overhead,
                    bytes_per_s=(payloadSizes[mid]+overhead)*1e6/interval)
               for mid, interval in sorted(intervals.items())]
    required = sum(row["bytes_per_s"] for row in streams)
    # 8N1 costs ten bits per byte; MAV_X_RATE and physical UART are separate limits.
    usable = min(baud/10., mavRate)*(1.-reserve)
    return dict(status="BUDGET_OK_UNVERIFIED" if required <= usable else "BUDGET_INSUFFICIENT",
                baud=baud, mav_rate_bytes_s=mavRate, signed=signed, reserve=reserve,
                required_max_bytes_s=required, usable_bytes_s=usable, streams=streams,
                limitations="Full payload upper bound; verify actual frames, all other streams and source timestamps on device.")
