"""速度参考系测试：姿态非单位时，机体里程计与 EGO 输出的速度参考系必须不同。"""

import math
import os
import subprocess
import sys
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402


def test_body_and_ego_velocity_frames_differ_under_rotation(tmp_path):
    """R_W_I = Rz(90°)，输入为 IMU 局部系速度 (1,0,0)：

    期望  v_W_I = R_W_I · v_I = (0,1,0)   → EGO 输出（世界系速度）
          v_B   = Rᵀ_W_B · v_W_I = (1,0,0) → 标准机体里程计（机体系 = IMU 系，R 同 R_W_I）

    说明：Rᵀ · (0,1,0) = (1,0,0)。两者在姿态非单位时数值不同，若相等即说明存在
    "直接复制字段、不做换系"的参考系错误（单位姿态下两家相同，掩盖该错误）。

    测试方式：单线程自旋 + 定时器稳定发布，回调记录最后一个样本（避免一次性
    突发发布导致的队列竞争）。
    """
    from boom_birds_nav.synthetic import write_synth_calibration

    calib = tmp_path / "c.npz"
    write_synth_calibration(str(calib))
    extr = tmp_path / "e.yaml"
    extr.write_text(
        'source: "TEST-ONLY velocity frame test"\n'
        "assume_origin_coincident: true\n"
        "T_I_C0:\n"
        "  - [1.0, 0.0, 0.0, 0.0]\n"
        "  - [0.0, 1.0, 0.0, 0.0]\n"
        "  - [0.0, 0.0, 1.0, 0.0]\n"
        "  - [0.0, 0.0, 0.0, 1.0]\n",
        encoding="utf-8",
    )
    args = [
        sys.executable, "-m", "boom_birds_nav.pose_adapter", "--ros-args",
        "-r", "__node:=bb_velocity_frame_test",
        "-p", f"calibration_file:={calib}",
        "-p", f"extrinsics_file:={extr}",
        "-p", "pose_timeout_s:=1.0",
        "-p", "depth_timeout_s:=2.0",
    ]
    env = os.environ.copy()
    paths = [p for p in sys.path if p and os.path.isdir(p)]
    env["PYTHONPATH"] = os.pathsep.join(paths + [env.get("PYTHONPATH", "")]).strip(os.pathsep)

    if not rclpy.ok():
        rclpy.init()
    probe = rclpy.create_node("bb_velocity_probe")
    latest = {"body": None, "ego": None}
    probe.create_subscription(
        Odometry, "/boom_birds/vio/odom_body", lambda m: latest.__setitem__("body", m), 10
    )
    probe.create_subscription(
        Odometry, "/boom_birds/vio/odom_ego", lambda m: latest.__setitem__("ego", m), 10
    )
    pub_odom = probe.create_publisher(Odometry, "/boom_birds/ov/odomimu", 10)
    pub_depth = probe.create_publisher(Image, "/boom_birds/depth/image", 10)

    yaw = math.pi / 2.0
    qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    state = {"t": 0.0}

    def publish_once():
        state["t"] = max(state["t"] + 0.05, time.time())
        stamp = probe.get_clock().now().to_msg()
        stamp.sec = int(state["t"])
        stamp.nanosec = int((state["t"] - int(state["t"])) * 1e9)
        d = Image()
        d.header.stamp = stamp
        d.header.frame_id = "cam0_rect"
        d.height, d.width = 1, 1
        d.encoding = "32FC1"
        d.step = 4
        d.data = np.array([1.5], dtype=np.float32).tobytes()
        pub_depth.publish(d)
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = "global"
        o.child_frame_id = "imu"
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x = 1.0        # IMU 局部系速度（OpenVINS 语义）
        pub_odom.publish(o)

    with open(tmp_path / "adapter.log", "w") as logf:
        proc = subprocess.Popen(args, stdout=logf, stderr=subprocess.STDOUT, env=env)
        timer = probe.create_timer(0.05, publish_once)
        deadline = time.time() + 12.0
        try:
            while time.time() < deadline and (latest["body"] is None or latest["ego"] is None):
                rclpy.spin_once(probe, timeout_sec=0.05)
            # 再多转一会儿，确保取到的是稳定后的样本
            end = time.time() + 1.0
            while time.time() < end:
                rclpy.spin_once(probe, timeout_sec=0.05)
        finally:
            probe.destroy_timer(timer)
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()

    if latest["body"] is None or latest["ego"] is None:
        log = (tmp_path / "adapter.log").read_text(encoding="utf-8", errors="replace")
        raise AssertionError(f"未收到里程计\n--- adapter log ---\n{log[-1500:]}")

    v_world = np.array(
        [latest["ego"].twist.twist.linear.x, latest["ego"].twist.twist.linear.y, latest["ego"].twist.twist.linear.z]
    )
    v_body = np.array(
        [latest["body"].twist.twist.linear.x, latest["body"].twist.twist.linear.y, latest["body"].twist.twist.linear.z]
    )
    assert np.allclose(v_world, [0.0, 1.0, 0.0], atol=1e-6), v_world
    assert np.allclose(v_body, [1.0, 0.0, 0.0], atol=1e-6), v_body
    assert not np.allclose(v_world, v_body), "两种输出的速度参考系必须不同（姿态非单位时）"

    probe.destroy_node()
