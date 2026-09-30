"""Pause and restore SIH setpoints using one connected parameter client."""
import argparse
import json
import time
import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from boom_birds_control.sih_guard import verify_sih_process


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--sih-pid", type=int, required=True)
    p.add_argument("--seconds", type=float, required=True)
    args = p.parse_args()
    if not verify_sih_process(args.sih_pid) or not 0 < args.seconds <= 10:
        raise SystemExit("verified local SIH and bounded pause required")
    rclpy.init()
    node = rclpy.create_node("sih_setpoint_pause_injector")
    client = AsyncParameterClient(node, "/boom_birds_px4_interface")
    def set_pause(value):
        future = client.set_parameters([Parameter("sih_pause_setpoint", value=value)])
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
        if not future.done() or not future.result().results[0].successful:
            raise RuntimeError("SIH pause parameter not accepted")
    started = None
    try:
        if not client.wait_for_services(timeout_sec=5.):
            raise RuntimeError("PX4 interface parameter service unavailable")
        set_pause(True)
        started = time.monotonic()
        stamp = time.time()
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.01)
    finally:
        try:
            if started is not None:
                set_pause(False)
            if started is not None:
                print(json.dumps({"fault": "Offboard中断瞬态", "accepted_at": stamp,
                                  "requested_s": args.seconds, "actual_s": time.monotonic() - started}))
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
