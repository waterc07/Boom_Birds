"""位姿适配节点行为测试：用真实 launch 启动节点（子进程），验证发布语义与失效行为。

不在进程内直接构造节点：配置参数在 configure 阶段读取，进程内构造无法注入测试参数；
用真实 launch + CLI 参数更接近实际运行方式，且避免为测试改生产代码。
"""

import math
import os
import time

import numpy as np
import pytest
import rclpy

from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402


def _spin_until(executor, predicate, timeout=6.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


_STAMP = {"t": 0.0}


def _next_stamp(owner):
    """单调递增的时间戳：适配器对同一输入时间戳只发布一次（避免重复 stamp 干扰地图同步），
    因此测试必须模拟真实传感器的时间推进。"""
    now = owner.get_clock().now().nanoseconds * 1e-9
    _STAMP["t"] = max(_STAMP["t"] + 0.02, now)
    msg = owner.get_clock().now().to_msg()
    msg.sec = int(_STAMP["t"])
    msg.nanosec = int((_STAMP["t"] - int(_STAMP["t"])) * 1e9)
    return msg


def _wait_inputs(executor, pub_odom, pub_depth, timeout=8.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if pub_odom.get_subscription_count() > 0 and pub_depth.get_subscription_count() > 0:
            return True
        executor.spin_once(timeout_sec=0.05)
    return False


def _depth_msg(owner):
    m = Image()
    m.header.stamp = _next_stamp(owner)
    m.height, m.width = 1, 1
    m.encoding = "32FC1"
    m.step = 4
    m.data = np.array([1.0], dtype=np.float32).tobytes()
    return m


@pytest.fixture()
def adapter(tmp_path):
    """启动真实 pose_adapter 节点进程（ros2 run），退出时终止。

    用子进程而非进程内构造：节点参数在 configure 阶段读取，必须通过命令行注入；
    输出重定向到文件而不是管道，避免受限环境下的管道捕获问题。
    """
    import subprocess
    import sys

    from boom_birds_nav.synthetic import write_synth_calibration

    calib = tmp_path / "c.npz"
    write_synth_calibration(str(calib))
    extr = tmp_path / "e.yaml"
    extr.write_text(
        'source: "TEST-ONLY pytest"\n'
        "assume_origin_coincident: true\n"
        "T_I_C0:\n"
        "  - [1.0, 0.0, 0.0, 0.0]\n"
        "  - [0.0, 1.0, 0.0, 0.0]\n"
        "  - [0.0, 0.0, 1.0, 0.0]\n"
        "  - [0.0, 0.0, 0.0, 1.0]\n",
        encoding="utf-8",
    )
    log_path = tmp_path / "adapter.log"
    args = [
        sys.executable, "-m", "boom_birds_nav.pose_adapter", "--ros-args",
        "-r", "__node:=boom_birds_pose_adapter_test",
        "-p", f"calibration_file:={calib}",
        "-p", f"extrinsics_file:={extr}",
        "-p", "rate_hz:=50.0",
        "-p", "pose_timeout_s:=0.2",
        "-p", "depth_timeout_s:=1.0",
    ]
    env = os.environ.copy()
    paths = [p for p in sys.path if p and os.path.isdir(p)]
    env["PYTHONPATH"] = os.pathsep.join(paths + [env.get("PYTHONPATH", "")]).strip(os.pathsep)
    with log_path.open("w") as logf:
        proc = subprocess.Popen(args, stdout=logf, stderr=subprocess.STDOUT, env=env)
        time.sleep(2.5)
        try:
            yield str(log_path)
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()


def test_publishes_expected_values(adapter):
    log_path = adapter
    if not rclpy.ok():
        rclpy.init()
    probe = rclpy.create_node("probe_adapter")
    got = {"cam": [], "body": [], "ego": []}
    probe.create_subscription(PoseStamped, "/boom_birds/vio/camera_pose", lambda m: got["cam"].append(m), 10)
    probe.create_subscription(Odometry, "/boom_birds/vio/odom_body", lambda m: got["body"].append(m), 10)
    probe.create_subscription(Odometry, "/boom_birds/vio/odom_ego", lambda m: got["ego"].append(m), 10)
    pub_odom = probe.create_publisher(Odometry, "/boom_birds/ov/odomimu", 10)
    pub_depth = probe.create_publisher(Image, "/boom_birds/depth/image", 10)
    ex = SingleThreadedExecutor()
    ex.add_node(probe)
    assert _wait_inputs(ex, pub_odom, pub_depth), "DDS 输入订阅未匹配"
    pub_depth.publish(_depth_msg(probe))
    time.sleep(0.2)
    ex.spin_once(timeout_sec=0.1)

    msg = Odometry()
    msg.header.frame_id = "global"
    msg.child_frame_id = "imu"
    msg.header.stamp = _next_stamp(probe)
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = 1.0, 2.0, 3.0
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.y = 0.5
    ok = False
    for _ in range(40):
        msg.header.stamp = _next_stamp(probe)
        pub_odom.publish(msg)
        pub_depth.publish(_depth_msg(probe))
        if _spin_until(ex, lambda: len(got["cam"]) > 1 and len(got["body"]) > 1, timeout=0.5):
            ok = True
            break
    if not ok:
        log = open(adapter).read() if adapter else ""
        raise AssertionError(f"未收到发布：{ {k: len(v) for k, v in got.items()} }\n--- adapter log ---\n{log[-2000:]}")
    cam = got["cam"][-1]
    assert abs(cam.pose.position.x - 1.0) < 1e-6 and abs(cam.pose.position.z - 3.0) < 1e-6
    assert abs(cam.pose.orientation.w - 1.0) < 1e-6
    assert abs(got["body"][-1].twist.twist.linear.y - 0.5) < 1e-6
    assert abs(got["ego"][-1].twist.twist.linear.y - 0.5) < 1e-6
    assert got["body"][-1].child_frame_id == "body"
    ex.remove_node(probe)
    probe.destroy_node()


def test_stops_publishing_when_pose_stale(adapter):
    log_path = adapter
    if not rclpy.ok():
        rclpy.init()
    probe = rclpy.create_node("probe_stale")
    count = {"n": 0}
    probe.create_subscription(Odometry, "/boom_birds/vio/odom_body", lambda m: count.__setitem__("n", count["n"] + 1), 10)
    pub_odom = probe.create_publisher(Odometry, "/boom_birds/ov/odomimu", 10)
    pub_depth = probe.create_publisher(Image, "/boom_birds/depth/image", 10)
    ex = SingleThreadedExecutor()
    ex.add_node(probe)
    assert _wait_inputs(ex, pub_odom, pub_depth), "DDS 输入订阅未匹配"

    msg = Odometry()
    msg.pose.pose.orientation.w = 1.0
    for _ in range(20):
        msg.header.stamp = _next_stamp(probe)
        pub_odom.publish(msg)
        pub_depth.publish(_depth_msg(probe))
        ex.spin_once(timeout_sec=0.05)
        if count["n"] > 0:
            break
    if count["n"] == 0:
        log = open(adapter).read() if adapter else ""
        raise AssertionError(f"初始未发布\n--- adapter log ---\n{log[-2000:]}")
    time.sleep(0.8)                     # 超过 pose_timeout_s=0.2
    for _ in range(10):
        ex.spin_once(timeout_sec=0.05)
    n_after_stale = count["n"]
    time.sleep(0.5)
    for _ in range(10):
        ex.spin_once(timeout_sec=0.05)
    assert count["n"] == n_after_stale, "位姿过期后仍在发布，违反契约"
    ex.remove_node(probe)
    probe.destroy_node()
