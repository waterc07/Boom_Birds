"""Inject one ordered planner cancellation into the verified local SIH task."""
import argparse
import json
import time
import rclpy
from boom_birds_interfaces.msg import ControlCommand
from boom_birds_control.runtime_config import DEFAULTS
from boom_birds_control.sih_guard import verify_sih_process


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sih-pid", type=int, required=True)
    args = parser.parse_args()
    if not verify_sih_process(args.sih_pid):
        raise SystemExit("authorized SIH process not verified")
    rclpy.init()
    node = rclpy.create_node("sih_planner_cancel_injector")
    latest = []
    pub = node.create_publisher(ControlCommand, DEFAULTS.planner_command_topic, 50)
    node.create_subscription(ControlCommand, DEFAULTS.planner_command_topic, latest.append, 50)
    deadline = time.monotonic() + 5.
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            commands = [m for m in latest if m.command_type == m.EXECUTE]
            if not commands or pub.get_subscription_count() < 2:
                continue
            msg = commands[-1]
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.sequence += 1
            msg.command_type = msg.CANCEL
            pub.publish(msg)
            end = time.monotonic() + .3
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.02)
            print(json.dumps({"command_type": "CANCEL", "session_id": msg.session_id,
                              "trajectory_id": msg.trajectory_id, "sequence": msg.sequence}))
            return
        raise SystemExit("active planner execution not observed; cancellation not injected")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
