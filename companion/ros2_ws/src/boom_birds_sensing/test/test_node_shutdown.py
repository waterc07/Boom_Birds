"""用实际 ROS 订阅负载检查节点的 SIGINT/SIGTERM 收尾；不连接设备。"""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from mavros_msgs.msg import Mavlink


@pytest.mark.parametrize("module", ["pose_adapter", "mavros_range_node"])
@pytest.mark.parametrize("stop_signal", [signal.SIGINT, signal.SIGTERM])
def test_signal_stops_active_subscription_without_context_race(tmp_path, module, stop_signal):
    token = uuid.uuid4().hex
    topic = "/shutdown_" + token
    args = []
    if module == "pose_adapter":
        extrinsics = tmp_path / "extrinsics.yaml"
        extrinsics.write_text("T_I_C0: [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]\n"
                              "T_I_B: [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]\n"
                              "source: TEST_ONLY_signal_shutdown\n")
        calibration = Path(__file__).resolve().parents[2] / "stereo_depth/calibration/upright_20261022/candidate.npz"
        assert calibration.is_file()
        args = ["-p", "extrinsics_file:=" + str(extrinsics),
                "-p", "calibration_file:=" + str(calibration),
                "-p", "use_depth_gate:=false", "-p", "odom_imu_topic:=" + topic]
        kind = Odometry
    else:
        args = ["-p", "raw_topic:=" + topic, "-p", "stream_rate_hz:=0.0"]
        kind = Mavlink
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join(sys.path)
    code = "from boom_birds_sensing." + module + " import main; main()"
    context = Context()
    rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
    node = Node("shutdown_probe_" + token, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    publisher = node.create_publisher(kind, topic, 20)
    child = None
    output = tmp_path / "child.log"
    try:
        with output.open("w") as log:
            child = subprocess.Popen([sys.executable, "-c", code, "--ros-args", *args],
                                     env=env, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 12
            while node.count_subscribers(topic) == 0:
                assert child.poll() is None, output.read_text()
                assert time.monotonic() < deadline, output.read_text()
                executor.spin_once(timeout_sec=.02)
            deadline = time.monotonic() + .6
            while time.monotonic() < deadline:
                msg = kind()
                if kind is Odometry:
                    msg.header.stamp = node.get_clock().now().to_msg()
                    msg.header.frame_id = "global"
                    msg.child_frame_id = "imu"
                    msg.pose.pose.orientation.w = 1.
                publisher.publish(msg)
                executor.spin_once(timeout_sec=.001)
            child.send_signal(stop_signal)
            assert child.wait(timeout=8) == 0, output.read_text()
        assert "Traceback" not in output.read_text()
    finally:
        if child is not None and child.poll() is None:
            child.kill()
            child.wait(timeout=3)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
