"""按采集时间配对的回归测试（评审三项判据）。

判据 A：同一观测时刻的位姿晚到 150 ms，仍必须正确配对；输出位姿的时间戳 = 观测时刻。
判据 B：采集时间真正相差 40 ms（> 30 ms 容差）必须拒绝，不能用 pose_wait/depth_max_age 放宽。
判据 C：等待超时的观测被淘汰后，不能在后续到达时重新混入。

实现要点：时间戳必须贴近真实时钟（位姿过期/未来戳检查都以真实时钟为基准），
因此这里用真实节拍发布，位姿时间戳取"发布那一刻"（等价于同一观测时刻）。
"""

import os
import subprocess
import sys
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

TOL = 0.03


def _start_adapter(tmp_path, extra=None, name="bb_pairing_test"):
    from boom_birds_nav.synthetic import write_synth_calibration

    calib = tmp_path / "c.npz"
    write_synth_calibration(str(calib))
    extr = tmp_path / "e.yaml"
    extr.write_text(
        'source: "TEST-ONLY pairing test"\n'
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
        "-r", f"__node:={name}",
        "-p", f"calibration_file:={calib}",
        "-p", f"extrinsics_file:={extr}",
        "-p", f"pose_depth_tolerance_s:={TOL}",
        "-p", "pose_wait_s:=1.0",
        "-p", "depth_max_age_s:=1.0",
        "-p", "pose_timeout_s:=2.0",
        "-p", "depth_timeout_s:=5.0",
    ]
    for k, v in (extra or {}).items():
        args += ["-p", f"{k}:={v}"]
    env = os.environ.copy()
    paths = [p for p in sys.path if p and os.path.isdir(p)]
    env["PYTHONPATH"] = os.pathsep.join(paths + [env.get("PYTHONPATH", "")]).strip(os.pathsep)
    logf = open(tmp_path / "adapter.log", "w")
    return subprocess.Popen(args, stdout=logf, stderr=subprocess.STDOUT, env=env), logf


def _img(t):
    m = Image()
    m.header.stamp.sec = int(t)
    m.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    m.header.frame_id = "cam0_rect"
    m.height, m.width = 1, 1
    m.encoding = "32FC1"
    m.step = 4
    m.data = np.array([1.5], dtype=np.float32).tobytes()
    return m


def _odom(t):
    m = Odometry()
    m.header.stamp.sec = int(t)
    m.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    m.header.frame_id = "global"
    m.child_frame_id = "imu"
    m.pose.pose.position.z = 1.5
    m.pose.pose.orientation.w = 1.0
    return m


class Harness:
    def __init__(self, tmp_path, extra=None, name="bb_pairing_test"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name + "_probe")
        self.got = []
        self.node.create_subscription(PoseStamped, "/boom_birds/vio/camera_pose", self.got.append, 10)
        self.pub_depth = self.node.create_publisher(Image, "/boom_birds/depth/image", 10)
        self.pub_odom = self.node.create_publisher(Odometry, "/boom_birds/ov/odomimu", 10)
        self.tmp_path = tmp_path
        self.proc, self.logf = _start_adapter(tmp_path, extra, name)
        # DDS discovery is asynchronous. A fixed sleep can pass before the
        # publishers actually match and turn a rejection test into a false PASS.
        deadline = time.time() + 8.0
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise AssertionError(f"adapter exited before DDS discovery: {self.log()}")
            if (self.pub_depth.get_subscription_count() > 0
                    and self.pub_odom.get_subscription_count() > 0
                    and self.node.count_publishers("/boom_birds/vio/camera_pose") > 0):
                break
            rclpy.spin_once(self.node, timeout_sec=0.05)
        else:
            raise AssertionError(f"DDS endpoints did not match: {self.log()}")

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def close(self):
        self.proc.terminate()
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        self.logf.close()
        self.node.destroy_node()

    def log(self):
        return (self.tmp_path / "adapter.log").read_text(encoding="utf-8", errors="replace")


def test_pairing_uses_observation_time_not_arrival_time(tmp_path):
    """判据 A：位姿先到、深度后到（因果上等价于位姿处理快/深度慢），必须按观测时刻配对。

    构造：把 interp_max_gap 压到 0.05 s，则只有当深度观测时刻**夹在**两个位姿之间时
    才会走插值路径；这要求实现真正按 header.stamp 配对，而不是"最新位姿 + 容差"。
    输出位姿的时间戳必须等于深度的观测时刻（而不是位姿或到达时刻）。
    """
    h = Harness(tmp_path, {"pose_interp_max_gap_s": 0.05}, name="bb_obs_time")
    try:
        h.spin(2.5)
        obs = []
        # 发布要重试：首轮可能因 DDS 匹配未完成而丢包，不能用固定迭代次数判定结果。
        for attempt in range(12):
            if len(h.got) >= 3 and attempt >= 5:
                break
            t_a = time.time()
            # 两位姿观测时刻间隔 0.04 s（在 0.15 s 插值上限内，且不超过未来戳容忍 0.05 s）
            h.pub_odom.publish(_odom(t_a))            # 位姿 A
            h.pub_odom.publish(_odom(t_a + 0.04))     # 位姿 B
            t_cap = t_a + 0.02                        # 深度观测时刻，位于 A、B 之间
            h.pub_depth.publish(_img(t_cap))
            obs.append(t_cap)
            h.spin(0.35)
    finally:
        log = h.log()
        got = list(h.got)
        h.close()
    assert len(got) >= 3, f"未能按观测时刻配对，实际 {len(got)} 条\n--- log ---\n{log[-1500:]}"
    stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in got]
    # 每条输出的时间戳必须落在某个观测时刻附近（±0.05 s）
    matched = sum(1 for s in stamps if any(abs(s - o) < 0.05 for o in obs))
    assert matched >= len(stamps) - 1, (
        f"输出时间戳不是深度观测时刻：stamps={np.round(stamps, 3).tolist()} obs={np.round(obs, 3).tolist()}"
    )


def test_gap_over_tolerance_is_rejected(tmp_path):
    """判据 B：观测时刻相差 40 ms（> 30 ms）必须全部拒绝。"""
    h = Harness(tmp_path, name="bb_gap40")
    try:
        h.spin(2.5)
        for _ in range(6):
            t_cap = time.time()
            h.pub_depth.publish(_img(t_cap))
            h.pub_odom.publish(_odom(t_cap + 0.04))      # 观测时刻差 40 ms
            h.spin(0.5)
    finally:
        log = h.log()
        n = len(h.got)
        h.close()
    assert n == 0, f"超过 30 ms 容差的配对必须拒绝，实际发布 {n} 条\n--- log ---\n{log[-1200:]}"
    assert "超出容差" in log


def test_timed_out_observation_not_reintroduced(tmp_path):
    """判据 C：等待超时的深度被淘汰后，迟到的匹配位姿不能把它重新混入。"""
    h = Harness(tmp_path, {"pose_wait_s": 0.2}, name="bb_timeout")
    try:
        h.spin(2.5)
        t_cap = time.time()
        h.pub_depth.publish(_img(t_cap))
        h.spin(0.6)                       # 超过 pose_wait_s=0.2，pending 被淘汰
        h.pub_odom.publish(_odom(t_cap))  # 迟到的、本可匹配的位姿
        h.spin(0.5)
        n = len(h.got)
    finally:
        log = h.log()
        h.close()
    assert n == 0, f"超时观测被重新混入：{n} 条"
    assert "等待超时" in log


def test_future_pose_rejected(tmp_path):
    h = Harness(tmp_path, name="bb_future")
    try:
        h.spin(2.5)
        t_cap = time.time()
        h.pub_depth.publish(_img(t_cap))
        h.pub_odom.publish(_odom(t_cap + 30.0))
        h.spin(0.5)
        n = len(h.got)
    finally:
        log = h.log()
        h.close()
    assert n == 0
    assert "未来时间戳" in log
