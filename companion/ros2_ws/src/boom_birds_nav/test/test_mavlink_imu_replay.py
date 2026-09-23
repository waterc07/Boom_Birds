"""记录 MAVLink 回放入口的脱机测试（构造的记录文件，不是实测数据）。

覆盖：
- 有 TIMESYNC 往返时能建立映射并发布；
- 缺 TIMESYNC 时判定为 REPLAY_NO_SYNC 并给出原因（不静默发布错误时间戳）；
- 报告字段完整（频率、间隔、丢样、RTT、偏移、失效次数）。

记录格式说明：`.tlog` = 12 字节头 + 「8 字节大端微秒时间戳 + 原始 MAVLink 帧」序列。
本测试按该格式直接写出，从而精确控制每条消息的记录时间（pymavlink 的写接口
不做逐消息时间戳，读回的时间戳不可靠）。
"""

from __future__ import annotations

import json
import os
import struct
import subprocess
import sys

import pytest

pymavlink = pytest.importorskip("pymavlink")

from boom_birds_nav.mavlink_imu_replay import main as replay_main, replay  # noqa: E402

NS = 1_000_000_000
MONO_T0 = 1_000.0            # 回放虚拟单调时钟起点（与 replay.REPLAY_MONO_EPOCH_S 一致）
# 飞控启动时钟模型：boot_us = (mono - MONO_T0 + BOOT_REF_S) * 1e6
# 即回放起点处飞控已启动 BOOT_REF_S 秒；boot 与单调时钟同步推进（同一时间轴）。
BOOT_REF_S = 1.0
TLOG_HEADER = b"\x00\x00\x00\x00" + struct.pack(">I", 0) + struct.pack(">I", 0)


def _write_synthetic_tlog(path: str, *, with_timesync: bool, frames: int = 30,
                          rate_hz: float = 50.0, half_trip_s: float = 0.002,
                          timesync_every: int = 5,
                          t0_wall_s: float = 1_700_000_000.0) -> None:
    """构造 .tlog：模拟 PX4 的 HIGHRES_IMU（与 TIMESYNC 应答）。

    timesync_every：每多少帧携带一次 TIMESYNC 应答（默认 5 帧 = 0.1 s，
    使 1 s 的 sync_timeout 在整段记录内都保持有效）。
    """
    from pymavlink.dialects.v20 import common as mavlink2

    builder = mavlink2.MAVLink(None, srcSystem=1, srcComponent=1)
    entries = []
    period = 1.0 / rate_hz
    for i in range(frames):
        rel = i * period
        mono = MONO_T0 + rel
        boot_us = int((mono - MONO_T0 + BOOT_REF_S) * 1e6)
        entries.append((rel, builder.highres_imu_encode(
            boot_us, 0.1, -0.2, 9.81, 0.01, 0.02, -0.03,
            0.0, 0.0, 0.0, 1013.25, 0.0, 0.0, 25.0, 0x3F, 0).pack(builder)))
        if with_timesync and i % timesync_every == 0:
            # 我们（Companion）在第 i 帧时刻发出请求；飞控 half_trip 后应答
            send_ns = int(round(mono * NS))
            tc1 = int(round((mono + half_trip_s - MONO_T0 + BOOT_REF_S) * NS))
            entries.append((rel + half_trip_s,
                            builder.timesync_encode(tc1, send_ns).pack(builder)))
    with open(path, "wb") as fh:
        fh.write(TLOG_HEADER)
        for rel, raw in entries:
            fh.write(struct.pack(">Q", int(round((t0_wall_s + rel) * 1e6))))
            fh.write(raw)


def test_replay_with_timesync_publishes_mapped_timestamps(tmp_path):
    path = str(tmp_path / "with_sync.tlog")
    _write_synthetic_tlog(path, with_timesync=True)
    report = replay(path, timesync_rate_hz=10.0, expected_rate_hz=50.0)

    assert report["published"] > 20, json.dumps(report, ensure_ascii=False)[:2000]
    assert report["time_sync"]["samples_accepted"] > 0
    assert report["time_sync"]["locked"] is True
    assert report["time_sync"]["rtt_median_s"] is not None
    assert report["verdict"] == "REPLAY_OK"
    assert report["achieved_rate_hz"] == pytest.approx(50.0, rel=0.02)
    assert report["interval_median_s"] == pytest.approx(0.02, rel=0.02)
    # 首个 TIMESYNC 之前的样本必然因无映射被拒（预期行为），之后应全部发布
    assert report["counters"]["rejected_no_clock_mapping"] <= report["counters"]["highres_imu_seen"] // 5
    # 偏移 = 飞控启动时钟 − Companion 单调时钟 = BOOT_REF_S − MONO_T0
    assert report["time_sync"]["offset_s"] == pytest.approx(BOOT_REF_S - MONO_T0, abs=5e-3)
    # 时间戳必须落在回放 ROS 时间域，而不是原始 boot 秒（约 1 秒）
    assert report["first_stamp_ros_s"] > 1e9


def test_replay_without_timesync_refuses_to_publish(tmp_path):
    path = str(tmp_path / "no_sync.tlog")
    _write_synthetic_tlog(path, with_timesync=False)
    report = replay(path)

    assert report["published"] == 0, "没有时钟映射时不得发布任何时间戳"
    assert report["verdict"] == "REPLAY_NO_IMU_PUBLISHED"
    assert report["counters"]["rejected_no_clock_mapping"] > 0
    assert "TIMESYNC" in report["hint"]


def test_replay_cli_writes_report(tmp_path):
    path = str(tmp_path / "cli.tlog")
    out = tmp_path / "report.json"
    _write_synthetic_tlog(path, with_timesync=True)
    env = os.environ.copy()
    paths = [p for p in sys.path if p and os.path.isdir(p)]
    env["PYTHONPATH"] = os.pathsep.join(paths + [env.get("PYTHONPATH", "")]).strip(os.pathsep)
    proc = subprocess.run(
        [sys.executable, "-m", "boom_birds_nav.mavlink_imu_replay", str(path),
         "--timesync-rate-hz", "1.0", "--out", str(out)],
        capture_output=True, text=True, env=env, timeout=120,
    )
    assert proc.returncode == 0, f"stdout={proc.stdout[-2000:]}\nstderr={proc.stderr[-2000:]}"
    data = json.loads(out.read_text(encoding="utf-8"))
    assert data["verdict"] == "REPLAY_OK"
    assert data["data"].startswith("RECORDED MAVLINK REPLAY")
