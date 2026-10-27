#!/usr/bin/env python3
"""TEST-ONLY: read local PX4 SIH uORB control/velocity evidence in a separate process.

控制进程本身不再启动 px4-listener 子进程，避免 SIH 单机反馈轮询拖慢控制/任务周期。
只读取 uORB，不写 PX4 参数、不发 setpoint、不连接硬件。
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"/"boom_birds_control"))
from boom_birds_control.sih_guard import verifySihProcess

TOPICS = ("offboard_control_mode", "trajectory_setpoint", "vehicle_control_mode", "vehicle_local_position")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pid", type=int, required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--interval", type=float, default=.08)
    args = parser.parse_args()
    out = Path(args.out)
    if not verifySihProcess(args.pid):
        raise SystemExit("local SIH pid required")
    binary = Path("/proc")/str(args.pid)/"exe"
    binary = binary.resolve(strict=True)
    root = binary.parent.parent/"rootfs"/"0"
    tmp = out.with_suffix(".tmp")
    def readTopic(topic):
        sampledAt = time.monotonic()
        result = subprocess.run([str(binary.with_name("px4-listener")), topic, "-n", "1"],
                                cwd=root, capture_output=True, text=True, timeout=.4)
        if result.returncode:
            raise ValueError("uorb_listener")
        return topic, result.stdout, sampledAt, time.monotonic()

    with ThreadPoolExecutor(max_workers=len(TOPICS)) as pool:
        while True:
            if not verifySihProcess(args.pid):
                print("sih process changed", flush=True)
                return 2
            try:
                readings = list(pool.map(readTopic, TOPICS))
            except (ValueError, OSError, subprocess.TimeoutExpired):
                time.sleep(args.interval)
                continue
            samples = {topic: text for topic, text, _, _ in readings}
            sampleStamps = {topic: at for topic, _, at, _ in readings}
            sampleEnds = {topic: end for topic, _, _, end in readings}
            tmp.write_text(json.dumps(dict(stamp=time.monotonic(), samples=samples,
                                           sample_stamps=sampleStamps, sample_ends=sampleEnds)))
            tmp.replace(out)
            time.sleep(args.interval)


if __name__ == "__main__":
    raise SystemExit(main())
