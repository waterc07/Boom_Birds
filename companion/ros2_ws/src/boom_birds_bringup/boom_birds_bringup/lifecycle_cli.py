"""任务服务 CLI；不持有 MAVLink 控制连接。"""
from boom_birds_control.runtime_config import DEFAULTS
import argparse
import json
import time
import rclpy
from boom_birds_interfaces.srv import Mission
from std_msgs.msg import String


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("start", "cancel", "land", "status"))
    parser.add_argument("--goal", nargs=3, type=float)
    parser.add_argument("--wait-state")
    parser.add_argument("--timeout", type=float, default=10.)
    options = parser.parse_args(args)
    if options.action == "start" and options.goal is None:
        parser.error("start requires --goal X Y Z in the configured ROS world frame")
    rclpy.init()
    node = rclpy.create_node("boom_birds_mission_cli")
    try:
        deadline = time.monotonic() + options.timeout
        if options.action == "status":
            latest = []
            node.create_subscription(String, DEFAULTS.mission_status_topic, lambda msg: latest.append(json.loads(msg.data)), 10)
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                if latest and (not options.wait_state or latest[-1]["state"] == options.wait_state):
                    print(json.dumps(latest[-1], ensure_ascii=False))
                    return
                if latest and latest[-1]["state"] == "FAULT_LATCHED":
                    raise SystemExit(json.dumps(latest[-1]))
        else:
            client = node.create_client(Mission, DEFAULTS.mission_service)
            if not client.wait_for_service(timeout_sec=options.timeout):
                raise SystemExit("mission service unavailable")
            request = Mission.Request(action={"start":1, "cancel":2, "land":3}[options.action])
            if options.goal: request.goal.x, request.goal.y, request.goal.z = options.goal
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=max(0., deadline-time.monotonic()))
            if future.done():
                result = future.result()
                print(json.dumps(dict(accepted=result.accepted, session=result.session_id, state=result.state, reason=result.reason)))
                if not result.accepted: raise SystemExit(1)
                return
        raise SystemExit("mission operation timed out")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
