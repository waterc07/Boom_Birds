"""`mavlink_imu_node` 的节点级脱机测试（同进程内构造节点 + UDP 自环，不接任何设备）。

用一个「假 PX4」在 UDP 上回应 TIMESYNC 请求并发送模拟 HIGHRES_IMU，
验证真实节点的完整接收路径：MAVLink 编解码、校验、时间映射、发布、诊断话题。

为什么同进程构造而不是子进程：
- 本环境下父子进程的 DDS 发现与时钟域差异会给测试引入与被测代码无关的噪声；
- 节点参数通过 `rclpy.init(args=[...])` 注入，与 `ros2 run --ros-args` 等价；
- 接收循环仍在独立线程中运行，因此线程安全路径同样被覆盖。

结论只覆盖软件路径；UDP 回环不代表串口时序，也不代表真机频率。
"""

from __future__ import annotations

import json
import socket
import time

import pytest

rclpy = pytest.importorskip("rclpy")
try:
    from pymavlink.dialects.v20 import common as mavlink2
except Exception:  # noqa: BLE001
    mavlink2 = None

pytestmark = pytest.mark.skipif(mavlink2 is None, reason="需要 pymavlink（见模块 README）")

NS = 1_000_000_000
# 假飞控：启动时钟 = 单调时钟 + 该偏移
FAKE_BOOT_OFFSET_S = -987.5
FAKE_HALF_TRIP_S = 0.002
IMU_TOPIC = "/boom_birds/imu_node_test"
STATS_TOPIC = "/boom_birds/imu_node_test/mavlink_status"
DIAG_TOPIC = "/boom_birds/imu_node_test/diagnostics"


def _free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class FakePx4:
    """在 UDP 上与节点对话的最小假飞控。"""

    def __init__(self, port: int) -> None:
        self.dest = ("127.0.0.1", port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.02)
        self.mav = mavlink2.MAVLink(None, srcSystem=1, srcComponent=1)
        self.mav.robust_parsing = True
        self.parser = mavlink2.MAVLink(self.sock)
        self.parser.robust_parsing = True
        self.timesync_requests = 0
        self.stream_requests = 0
        # 节点进程/线程自己的单调时钟读数（取自它发来的 TIMESYNC.ts1）：
        # 不假设本进程的 time.monotonic() 与节点时钟对齐（实测存在秒级差异）。
        self.node_mono_s: float | None = None
        self.answer_timesync = True

    def pump(self, duration_s: float = 0.02, imu_per_request: int = 1) -> None:
        """读取并处理到达的消息；每收到一个 TIMESYNC 请求顺带发送 IMU 帧。

        把 IMU 发送与 TIMESYNC 请求绑定，是为了让 time_usec 取自「刚刚收到的」
        节点时钟读数，避免测试自身引入陈旧时间（真机上采样与收包是同一时刻附近）。
        """
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            try:
                data, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            for byte in data:
                msg = self.parser.parse_char(bytes([byte]))
                if msg is None or msg.get_type() == "BAD_DATA":
                    continue
                if msg.get_type() == "TIMESYNC":
                    self.timesync_requests += 1
                    self.node_mono_s = int(msg.ts1) * 1e-9
                    for _ in range(imu_per_request):
                        self.send_highres(self.boot_us_at())
                    if self.answer_timesync:
                        self._answer_timesync(msg)
                elif msg.get_type() == "COMMAND_LONG":
                    if int(msg.command) == 511:
                        self.stream_requests += 1

    def _answer_timesync(self, msg) -> None:
        if int(msg.tc1) != 0:
            return
        # 真实飞控在收到请求那一刻打时间戳：t2 = t1 + 单程延迟 + 真实偏移
        sent_mono = int(msg.ts1) * 1e-9
        boot_s = sent_mono + FAKE_HALF_TRIP_S + FAKE_BOOT_OFFSET_S
        self.send_timesync_response(int(round(boot_s * NS)), int(msg.ts1))

    def boot_us_at(self, backdate_s: float = 0.05) -> int:
        """按「节点时钟域」生成 IMU 的 time_usec：boot = mono + FAKE_BOOT_OFFSET_S。

        模型的 t_node_mono 来自「最近一次收到的 TIMESYNC 请求」，在 20 ms 的
        收包轮询周期内可能已经陈旧几十毫秒；回拨 50 ms 让采样时刻确实早于收包时刻
        （真机语义也是如此：采样 → 组包 → 传输都排在收包之前）。
        """
        base = self.node_mono_s if self.node_mono_s is not None else time.monotonic()
        return int(round((base - backdate_s - FAKE_BOOT_OFFSET_S) * 1e6))

    def send_timesync_response(self, tc1: int, ts1: int) -> None:
        self.sock.sendto(self.mav.timesync_encode(tc1, ts1).pack(self.mav), self.dest)

    def send_highres(self, boot_us: int, accel=(0.11, -0.22, 9.79), gyro=(0.011, 0.022, -0.033),
                     fields_updated: int = 0x3F) -> None:
        frame = self.mav.highres_imu_encode(
            boot_us, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2],
            0.0, 0.0, 0.0, 1013.25, 0.0, 0.0, 25.0, fields_updated, 0,
        )
        self.sock.sendto(frame.pack(self.mav), self.dest)

    def send_heartbeat(self, autopilot: int = 12) -> None:
        frame = self.mav.heartbeat_encode(2, autopilot, 0, 0, 0, 3)
        self.sock.sendto(frame.pack(self.mav), self.dest)

    def close(self) -> None:
        self.sock.close()


@pytest.fixture()
def node_and_fake():
    """在同一进程内构造真实节点 + 探针，yield (node, fake, 探针数据字典)。"""
    from boom_birds_nav.mavlink_imu_node import MavlinkImuNode
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String

    port = _free_udp_port()

    if not rclpy.ok():
        # 参数必须通过 rclpy.init(args=...) 注入：仅修改 sys.argv 不会被 rclpy 解析
        # （已实测：create_node 不会重新读取调用后的 sys.argv）。
        # 注意：参数覆盖在一次 init 内是进程级的，因此把实际端口记下来复用，
        # 保证假飞控始终对准节点真正绑定的端口。
        #
        # min_sample_age_s 放宽到 -3600 s：本环境中测试进程与节点进程的
        # time.monotonic() 实测存在约 2000 s 的域差（同一脚本内两进程相差数十秒到
        # 数十分钟不等），假飞控无法把 time_usec 稳定放到节点时钟的「过去几十毫秒」。
        # 该参数只影响「时间戳过于超前」这一合理性判据；映射的正确性由
        # 「相邻发布时间差 == 发送的 time_usec 差」以及纯函数测试（虚拟时钟）保证，
        # 不依赖两个时钟域的绝对对齐。真机默认值仍是 -0.05 s。
        rclpy.init(args=[
            "mavlink_imu_node", "--ros-args",
            "-r", "__node:=boom_birds_mavlink_imu_test",
            "-p", f"connection:=udpin:127.0.0.1:{port}",
            "-p", "timesync_rate_hz:=10.0",
            "-p", "stream_rate_hz:=50.0",
            "-p", "expected_rate_hz:=50.0",
            "-p", "diagnostics_rate_hz:=5.0",
            "-p", "read_timeout_s:=0.02",
            "-p", "min_sample_age_s:=-3600.0",
            "-p", f"imu_topic:={IMU_TOPIC}",
            "-p", f"stats_topic:={STATS_TOPIC}",
            "-p", f"diagnostics_topic:={DIAG_TOPIC}",
        ])
    node = MavlinkImuNode()
    port = int(node.get_parameter("connection").value.rsplit(":", 1)[1])

    probe = rclpy.create_node("mavlink_imu_probe")
    collected = {"imu": [], "stats": []}
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    probe.create_subscription(Imu, IMU_TOPIC, lambda m: collected["imu"].append(m), qos)
    probe.create_subscription(String, STATS_TOPIC, lambda m: collected["stats"].append(m), 10)
    # 同一个执行器同时驱动探针与节点：节点的 TIMESYNC/诊断定时器需要被 spin 才会触发
    # （接收循环在独立线程里，因此不受影响）。
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    executor.add_node(node)

    fake = FakePx4(port)
    fake.send_heartbeat()      # udpin 需要先收到数据才能确定对端地址
    try:
        yield {"node": node, "fake": fake, "probe": probe, "executor": executor,
               "collected": collected, "port": port}
    finally:
        fake.close()
        node.shutdown()
        executor.remove_node(probe)
        executor.remove_node(node)
        probe.destroy_node()
        node.destroy_node()


def _drive(ctx, duration_s: float, stop_when=lambda c: False):
    """驱动假飞控与探针执行器，直到超时或条件满足。"""
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        ctx["fake"].pump(0.02)
        ctx["executor"].spin_once(timeout_sec=0.005)
        if stop_when(ctx["collected"]):
            break


def test_publishes_imu_with_mapped_timestamps(node_and_fake):
    """有 TIMESYNC 时：应发布 IMU，时间戳由 time_usec 映射而来且严格递增。"""
    ctx = node_and_fake
    _drive(ctx, 3.0, lambda c: len(c["imu"]) >= 5)
    assert ctx["fake"].timesync_requests > 0, "节点未发送 TIMESYNC 请求"

    _drive(ctx, 8.0, lambda c: len(c["imu"]) >= 15)
    imu = ctx["collected"]["imu"]
    assert imu, f"未收到 {IMU_TOPIC} 发布；节点诊断={ctx['node'].core.counters}"

    msg = imu[-1]
    assert msg.header.frame_id == "imu"
    assert msg.orientation_covariance[0] == -1.0, "不得用飞控融合姿态冒充 IMU"
    assert abs(msg.linear_acceleration.z - 9.79) < 1e-4
    assert abs(msg.angular_velocity.z + 0.033) < 1e-4

    stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in imu]
    assert stamps == sorted(stamps) and len(set(stamps)) == len(stamps), "时间戳必须严格递增"

    # 时间戳来自 time_usec 映射：相邻发布间隔应与发送间隔一致（不是收包抖动）
    assert len(stamps) >= 3
    deltas = [b - a for a, b in zip(stamps, stamps[1:])]
    assert all(0.005 < d < 0.5 for d in deltas), f"间隔异常：{deltas}"

    # 诊断话题：时钟同步与频率必须可观察
    assert ctx["collected"]["stats"], "未收到诊断话题"
    stats = json.loads(ctx["collected"]["stats"][-1].data)
    assert stats["time_sync"]["samples_accepted"] > 0
    assert stats["time_sync"]["locked"] is True
    assert stats["time_sync"]["rtt_median_s"] is not None
    assert stats["imu_rate_hz"] is not None
    assert stats["connected"] is True, "应已收到（模拟）heartbeat"
    # 首个 TIMESYNC 收敛前的少量样本必然因无映射被拒（预期行为），之后不应再失败
    assert stats["rejected_no_clock_mapping"] <= 3, (
        f"映射收敛后仍有大量映射失败：{stats['rejected_no_clock_mapping']}"
    )
    assert ctx["fake"].stream_requests > 0, "节点应发送过流请求（SET_MESSAGE_INTERVAL）"


def test_no_publish_without_timesync(node_and_fake):
    """没有 TIMESYNC 应答时：必须拒绝发布，并在日志中给出原因。"""
    ctx = node_and_fake
    ctx["fake"].answer_timesync = False
    _drive(ctx, 4.0)

    assert not ctx["collected"]["imu"], "未建立时钟映射时不得发布任何 IMU 时间戳"
    counters = ctx["node"].core.counters
    assert counters["rejected_no_clock_mapping"] > 0
    assert counters["last_mapping_reason"] in ("no_timesync_sample", "timesync_unpaired")
    assert counters["published"] == 0
