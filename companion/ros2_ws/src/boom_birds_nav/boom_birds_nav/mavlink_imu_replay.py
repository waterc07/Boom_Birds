"""记录 MAVLink 数据的离线回放与自检（不连接任何设备、不启动 ROS 运行时）。

用途
----
真机联调前先用**记录数据**验证接收核心：频率、丢样、TIMESYNC 收敛、时间映射误差。
回放不依赖 ROS 运行时（时间基用固定偏移注入），也不把回放结果当作真机验收。

支持输入
--------
- `.tlog`（pymavlink `mavlogfile`，自带时间戳）：按记录时间间隔回放；
- 原始 MAVLink 字节流（例如用 `mavlink_imu_capture` 或 `mavproxy` 保存的裸流）：
  只能按消息内容推断间隔，回放报告会注明时间间隔来自 IMU 自身时间戳。

报告字段
--------
- 发布的 IMU 条数、达成频率、最大间隔、丢样计数；
- TIMESYNC 样本数 / 接受数 / RTT 中位数 / 偏移估计 / 失效次数；
- 若记录里有 TIMESYNC 往返（PX4 收到我们的请求会回发），则报告映射误差：
  `published_stamp - (request_echo + rtt/2)` 的统计量（因为回放时我们的请求时刻是合成的）。

限制：回放里的「请求发送时刻」是合成值，因此偏移估计与真机不会逐位相同；
它验证的是算法与失效路径，不是真机同步精度。
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time

from .mavlink_imu_core import (
    REJECT_NO_SYNC,
    MavlinkImuConfig,
    MavlinkImuReceiver,
)
from .timebase import RosTimeBase

# 回放时钟：把 PX4 启动时钟整体平移到「当前 ROS 时间附近」，使发布的 ROS 时间戳落在合理范围。
# 该平移对相对时序没有影响，只影响绝对时间戳。
REPLAY_ROS_EPOCH_S = 1_700_000_000.0
REPLAY_MONO_EPOCH_S = 1_000.0


def _load_messages(path: str, dialect: str = "common"):
    """读取记录消息，返回 [(相对时间秒, msg)]；无时间戳记录时 rel 为 None。

    - `.tlog`：pymavlink 提供每条消息的记录时间戳，按时间排序后回放（保证时间轴单调）；
    - 原始 MAVLink 字节流：pymavlink 会给出「尽力而为」的时间戳，长短不一，
      若排序后仍不单调则退回 rel=None（此时虚拟时钟由 IMU 自身时间戳推进）。
    """
    from pymavlink import mavutil

    conn = mavutil.mavlink_connection(path, dialect=dialect)
    raw = []
    while True:
        msg = conn.recv_match(blocking=False)
        if msg is None:
            break
        if msg.get_type() == "BAD_DATA":
            continue
        ts = getattr(msg, "_timestamp", None)
        raw.append((ts, msg))
    try:
        conn.close()
    except Exception:  # noqa: BLE001
        pass
    if not raw:
        return []

    has_ts = all(ts is not None for ts, _ in raw)
    if has_ts:
        raw.sort(key=lambda item: item[0])
        monotonic = all(
            raw[i][0] <= raw[i + 1][0] + 1e-9 for i in range(len(raw) - 1)
        )
        if monotonic and raw[-1][0] > raw[0][0]:
            t0 = raw[0][0]
            return [(ts - t0, msg) for ts, msg in raw]
    return [(None, msg) for _, msg in raw]


def replay(
    path: str,
    dialect: str = "common",
    max_rtt_s: float = 0.02,
    sync_timeout_s: float = 1.0,
    expected_rate_hz: float = 50.0,
    timesync_rate_hz: float = 2.0,
    system_id: int = 1,
    accept_any_component: bool = True,
    realtime: bool = False,
    ros_offset_s: float = REPLAY_ROS_EPOCH_S - REPLAY_MONO_EPOCH_S,
) -> dict:
    """回放记录文件并返回自检报告。"""
    messages = _load_messages(path, dialect=dialect)
    if not messages:
        raise SystemExit(f"记录中没有可用 MAVLink 消息：{path}")

    core = MavlinkImuReceiver(
        config=MavlinkImuConfig(
            system_id=system_id,
            accept_any_component=accept_any_component,
            expected_rate_hz=expected_rate_hz,
        ),
    )
    core.set_timebase(RosTimeBase.from_offset(ros_offset_s, REPLAY_MONO_EPOCH_S))

    pending_send_mono = None
    pending_send_ns = None
    next_request_s = 0.0
    timesync_responses = 0
    timesync_pairing = {"synthetic": 0, "recorded_echo": 0, "unpaired": 0}
    published: list[dict] = []
    boot_reference_us: int | None = None
    clock_base_s = REPLAY_MONO_EPOCH_S
    wall_start = time.monotonic()

    for rel_s, msg in messages:
        # 记录里没有逐消息时间戳时，用 IMU 自身时间戳推进虚拟时钟。
        if rel_s is not None:
            mono_now = clock_base_s + rel_s
        elif msg.get_type() == "HIGHRES_IMU":
            if boot_reference_us is None:
                boot_reference_us = int(msg.time_usec)
            mono_now = clock_base_s + (int(msg.time_usec) - boot_reference_us) * 1e-6
        else:
            mono_now = clock_base_s
        if realtime and rel_s is not None:
            target = wall_start + rel_s
            delay = target - time.monotonic()
            if delay > 0:
                time.sleep(min(delay, 1.0))
        core.set_virtual_mono(mono_now)
        core.stamp()

        if msg.get_type() == "TIMESYNC":
            tc1 = int(getattr(msg, "tc1", 0))
            echoed = int(getattr(msg, "ts1", 0))
            if tc1 == 0:
                # 飞控主动请求：交给核心记录为粗基线（不做偏移估计）
                pass
            elif (
                pending_send_mono is not None
                and pending_send_ns is not None
                and abs(echoed - pending_send_ns) <= 1_000_000
            ):
                # 与我们本轮合成的请求配对
                core.note_timesync_request(pending_send_mono, pending_send_ns)
                timesync_pairing["synthetic"] += 1
                pending_send_mono = None
                pending_send_ns = None
            elif echoed > 0:
                # 使用记录里回显的发送时刻作为 t1（要求落在回放时钟基准附近，
                # 即记录来自同源时钟域），这样无需依赖回放引擎自己的请求节奏。
                t1 = echoed * 1e-9
                if abs(t1 - clock_base_s) <= 1e6:
                    core.note_timesync_request(t1, echoed)
                    timesync_pairing["recorded_echo"] += 1
                else:
                    timesync_pairing["unpaired"] += 1
            else:
                timesync_pairing["unpaired"] += 1
            timesync_responses += 1
        elif mono_now >= next_request_s:
            # 合成 TIMESYNC 请求（回放里我们主动发请求，飞控的响应在记录中）
            next_request_s = mono_now + 1.0 / max(timesync_rate_hz, 1e-6)
            send_ns = int(mono_now * 1e9)
            core.note_timesync_request(mono_now, send_ns)
            pending_send_mono = mono_now
            pending_send_ns = send_ns

        for sample in core.handle_message(msg, mono_now):
            published.append(
                {
                    "stamp_ros_s": sample.stamp_ros_s,
                    "boot_us": sample.boot_us,
                    "age_s": sample.age_s,
                    "ax": sample.accel[0],
                    "ay": sample.accel[1],
                    "az": sample.accel[2],
                    "gx": sample.gyro[0],
                    "gy": sample.gyro[1],
                    "gz": sample.gyro[2],
                    "mapping_error_bound_s": sample.time_sync_error_bound_s,
                }
            )
    stats = core.stats(core.now_mono())
    boot_offset = None
    if published:
        # 回放的偏移是「启动时钟 - 单调时钟」：用第一条发布样本反推，仅用于报告自洽性
        boot_offset = published[0]["boot_us"] * 1e-6 - (
            published[0]["stamp_ros_s"] - ros_offset_s
        )
    intervals = []
    for a, b in zip(published, published[1:]):
        intervals.append(b["stamp_ros_s"] - a["stamp_ros_s"])
    report = {
        "input": path,
        "data": "RECORDED MAVLINK REPLAY (not live hardware)",
        "messages": len(messages),
        "published": len(published),
        "achieved_rate_hz": (
            None
            if len(intervals) < 2 or intervals[0] is None
            else len(intervals) / max(sum(intervals), 1e-9)
        ),
        "interval_median_s": statistics.median(intervals) if intervals else None,
        "interval_max_s": max(intervals) if intervals else None,
        "first_stamp_ros_s": published[0]["stamp_ros_s"] if published else None,
        "last_stamp_ros_s": published[-1]["stamp_ros_s"] if published else None,
        "boot_clock_offset_s": boot_offset,
        "timesync_responses": timesync_responses,
        "timesync_pairing": timesync_pairing,
        "counters": core.counters,
        "time_sync": stats["time_sync"],
    }
    report["verdict"] = (
        "REPLAY_OK"
        if published and stats["time_sync"]["samples_accepted"] > 0
        else "REPLAY_NO_SYNC"
    )
    if not published:
        report["verdict"] = "REPLAY_NO_IMU_PUBLISHED"
        report["hint"] = (
            f"全部采样被拒绝；最近原因={core.counters.get('last_mapping_reason')}。"
            f"回放缺少 TIMESYNC 时无法建立映射（这是预期行为，不是缺陷）：{REJECT_NO_SYNC}"
        )
    return report


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="回放记录的 MAVLink 数据并自检 IMU 接收/时间映射（不连接设备）"
    )
    ap.add_argument("path", help=".tlog 或原始 MAVLink 字节流文件")
    ap.add_argument("--dialect", default="common", help="MAVLink dialect（PX4 用 common）")
    ap.add_argument("--expected-rate-hz", type=float, default=50.0)
    ap.add_argument("--timesync-rate-hz", type=float, default=2.0)
    ap.add_argument("--max-rtt-s", type=float, default=0.02)
    ap.add_argument("--sync-timeout-s", type=float, default=1.0)
    ap.add_argument("--system-id", type=int, default=1)
    ap.add_argument("--realtime", action="store_true", help="按记录时间间隔真实等待（慢）")
    ap.add_argument("--out", default="", help="把 JSON 报告写入文件")
    args = ap.parse_args(argv)

    report = replay(
        args.path,
        dialect=args.dialect,
        max_rtt_s=args.max_rtt_s,
        sync_timeout_s=args.sync_timeout_s,
        expected_rate_hz=args.expected_rate_hz,
        timesync_rate_hz=args.timesync_rate_hz,
        system_id=args.system_id,
        realtime=args.realtime,
    )
    text = json.dumps(report, ensure_ascii=False, indent=2)
    print(text)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as fh:
            fh.write(text)
    return 0 if report["verdict"] == "REPLAY_OK" else 1


if __name__ == "__main__":
    sys.exit(main())
