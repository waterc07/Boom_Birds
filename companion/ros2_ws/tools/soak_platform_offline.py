#!/usr/bin/env python3
"""有界缓存虚拟时钟耐久测试及实际墙钟图像过载测试；仅 WSL。"""
import argparse
from platform_evidence import loadProfile
from platform_evidence import fingerprints
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time
import numpy as np
from boom_birds_control.platform_executor import PlatformExecutor
from boom_birds_control.platform_landing import RangeSample, FlightSample, HandoffFeedback
from boom_birds_control.platform_model import BoardObservation
from boom_birds_control.px4_backend import FakePx4Backend, ManualClock
from boom_birds_control.platform_executor import SegmentedTrace
from boom_birds_sensing.frame_worker import LatestFrameWorker
from boom_birds_sensing.platform_observation import BoardDetector
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.sensor_replay.platform_replay import renderBoard


def rssKb():
    for line in Path("/proc/self/status").read_text().splitlines():
        if line.startswith("VmRSS:"):
            return int(line.split()[1])

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--virtual-seconds", type=float, default=3600.)
    ap.add_argument("--overload-seconds", type=float, default=60.)
    args = ap.parse_args()
    if not np.isfinite([args.virtual_seconds, args.overload_seconds]).all() or min(args.virtual_seconds, args.overload_seconds) <= 0:
        raise ValueError("durations")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    p = loadProfile(args.config)
    writer = SegmentedTrace(out/"trace", queueCapacity=256, segmentRecords=1000)
    clock = ManualClock(0.)
    backend = FakePx4Backend(clock=clock)
    backend.connect()
    ex = PlatformExecutor(backend, p, test_only=True, revokeNavigation=lambda: True,
                          release_compute=lambda token: True, nativeLand=lambda why: True, disarm=lambda: True)
    ex.request()
    pose = np.eye(4)
    pose[:3, 3] = [0., 0., -1.5]
    rss = []
    times = []
    unsafe = 0
    start = time.monotonic()
    samples = int(args.virtual_seconds/.02)
    # Fake backend 的调用审计另有无界列表；耐久测试每批保存摘要后清空，
    # 防止把测试替身的记录开销误当生产控制内存。
    for i in range(samples):
        now = i*.02
        clock.set(now)
        backend.feedHeartbeat()
        obs = BoardObservation(now, p["board"]["name"], True, "TEST_ONLY_HELD_INPUT",
                               tuple(map(tuple, pose)), (7,), .1, 5., 100.)
        flight = FlightSample(now, 0., 0., 0., (0., 0., 0.), True, True, True)
        ack = None
        if ex.core.lastSentSequence:
            seq = ex.core.lastSentSequence
            ack = HandoffFeedback(now, ex.core.token, seq, 0, True, False, ex.core.sent_history[seq][1], "TEST_ONLY_FAKE_PX4")
        before = time.monotonic()
        result = ex.tick(now, obs, RangeSample(now, 1.48), flight, ack, navigationReady=True)
        duration = time.monotonic()-before
        if i % 100 == 0:
            times.append(duration)
            writer.put(dict(sample=i, output=asdict(result), tick_s=duration))
        unsafe += int(result.velocity_ned is not None and result.velocity_ned[2] > 0 and not result.descent_permitted)
        if i % 5000 == 0:
            rss.append(dict(sample=i, rss_kb=rssKb(), history=len(ex.history), sent_history=len(ex.core.sent_history)))
        backend.clearCalls()
        backend.setpoints.clear()
    tickElapsed = time.monotonic()-start
    assert writer.close()
    detector = BoardDetector(p, test_only=True)
    overloadPose = pose.copy()
    overloadPose[2, 3] = -1.
    frame = renderBoard(detector, overloadPose)
    if not detector.observe(frame, 1., "TEST_ONLY_OVERLOAD_PREFLIGHT").valid:
        raise ValueError("overload_fixture_invalid")
    count = [0]
    def detect(item):
        obs = detector.observe(item, time.monotonic(), "TEST_ONLY_OVERLOAD")
        assert obs.valid
        count[0] += 1
    worker = LatestFrameWorker(detect)
    started = time.monotonic()
    nextSubmit = started
    while time.monotonic()-started < args.overload_seconds:
        worker.submit(frame)
        nextSubmit += .002  # 500 Hz 输入，刻意高于正常测试负载。
        delay = nextSubmit-time.monotonic()
        if delay > 0:
            time.sleep(delay)
    assert worker.close()
    stats = worker.snapshot()
    durations = stats.pop("durations_s")
    stable = rss[2:]
    report = dict(kind="WSL_SOAK", result="PASS", config=p,
                  virtual_seconds=args.virtual_seconds, virtual_samples=samples,
                  tick_wall_seconds=tickElapsed, overload_wall_seconds=time.monotonic()-started,
                  final_control_state=ex.core.state, confirmed=ex.core.confirmed,
                  history_capacity=ex.history.maxlen, history_evicted=ex.history_evicted,
                  sent_history_capacity=256, unauthorized_descent=unsafe,
                  rss_samples=rss, rss_span_after_warmup_kb=max(x["rss_kb"] for x in stable)-min(x["rss_kb"] for x in stable) if stable else None,
                  tick_p50_p95_max_s=list(map(float, np.percentile(times, [50, 95, 100]))),
                  overload=stats, detect_p50_p95_max_s=list(map(float, np.percentile(durations, [50, 95, 100]))),
                  trace=writer.stats(),
                  limitation="virtual held inputs + wall-clock overload in WSL; no Pi timing or physical flight evidence")
    report["rss_growth_gate_kb"] = 8192
    if report["rss_span_after_warmup_kb"] is None or report["rss_span_after_warmup_kb"] > 8192:
        report["result"] = "FAIL"
    if ex.core.state != "DESCEND" or not ex.core.confirmed or unsafe or stats["errors"] or len(ex.history) > ex.history.maxlen or len(ex.core.sent_history) > 256:
        report["result"] = "FAIL"
    report["source_sha256"] = fingerprints(args.config)
    (out/"report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps({k:v for k,v in report.items() if k not in ("config", "rss_samples")}, indent=2))
    return int(report["result"] != "PASS")

if __name__ == "__main__":
    raise SystemExit(main())
