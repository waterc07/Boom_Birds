"""Companion 端 PX4 MAVLink IMU 接收节点（真实链路；不经 mavros）。

本节点是 `/boom_birds/imu` 的**唯一真实来源**：订阅 PX4 `HIGHRES_IMU`，
校验后按 `mavlink_imu_core` 的时间映射发布 `sensor_msgs/Imu`。
现有的 `vio_source` 只是 TEST-ONLY 合成源，两者不得同时运行在同一话题上。

设计要点
--------
- 串口/连接串、波特率、目标 sysid/compid、请求的流频率、话题都由参数提供，
  不在代码里写死历史设备值（`/dev/ttyAMA0`、115200 只作为文档中的示例）。
- 时间戳只来自 `HIGHRES_IMU.time_usec` 经 TIMESYNC 偏移与 ROS 时钟映射的结果；
  **绝不使用串口收包时刻**。映射不可用时拒绝发布并计数（见诊断话题）。
- 只接收上行；不发送控制指令、不做 Offboard/解锁、不回传外部视觉。
  唯一发出的消息是 `TIMESYNC` 请求与可选的 `MAV_CMD_SET_MESSAGE_INTERVAL` 流请求。
- 依赖 `pymavlink` + `pyserial`；缺失时节点显式报错退出，不做静默降级。

运行（真机前必须先确认接线与参数；参数示例见 config/mavlink_imu.yaml）：

    ros2 run boom_birds_nav mavlink_imu_node --ros-args \
        -p connection:=serial:/dev/ttyAMA0 -p baud:=115200

离线联调（不接飞控）：用 `mavlink_imu_replay` 回放记录文件，或用 UDP 自环
（`connection:=udpin:127.0.0.1:14555`）配合测试脚本发送模拟 MAVLink 消息。
"""

from __future__ import annotations

import json
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from .mavlink_clock import ClockMapperConfig
from .mavlink_imu_core import (
    CONTRACT_IMU_TOPIC,
    MavlinkImuConfig,
    MavlinkImuReceiver,
)

# 契约：/boom_birds/imu 为 best_effort / depth 5（与 vio_source 的 sensor data QoS 一致）
QOS_IMU = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)

# MAV_CMD_SET_MESSAGE_INTERVAL
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAVLINK_MSG_ID_HIGHRES_IMU = 105

# 时间戳同步请求内容：tc1=0 表示「远端发起」；ts1 由发送时刻的单调时钟给出（纳秒）
_REASON_NONE = None


class MavlinkImuNode(Node):
    def __init__(self) -> None:
        super().__init__("boom_birds_mavlink_imu")

        # ---- 链路参数（真机值必须由用户显式提供；默认只是可编辑的起点）----
        self.declare_parameter("connection", "serial:/dev/ttyAMA0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("dialect", "common")          # PX4 用 MAVLink common
        self.declare_parameter("read_timeout_s", 0.2)

        # ---- 目标系统 ----
        self.declare_parameter("target_system", 1)
        self.declare_parameter("target_component", 1)
        self.declare_parameter("accept_any_component", True)
        self.declare_parameter("device_id", -1)              # -1 = 不校验 HIGHRES_IMU.id

        # ---- 流请求（不预设 115200 一定支持目标频率）----
        self.declare_parameter("auto_request_stream", True)
        self.declare_parameter("stream_rate_hz", 50.0)
        self.declare_parameter("stream_request_retry_s", 5.0)

        # ---- 时间同步 ----
        self.declare_parameter("timesync_rate_hz", 2.0)
        self.declare_parameter("max_rtt_s", 0.02)
        self.declare_parameter("sync_timeout_s", 1.0)
        self.declare_parameter("max_offset_deviation_s", 0.05)
        self.declare_parameter("jump_confirm_samples", 3)
        self.declare_parameter("converge_samples", 5)
        # 相机—IMU 时间偏移：offset = t_cam_ros - t_imu_ros（见契约 timing）
        self.declare_parameter("camera_imu_offset_s", 0.0)
        self.declare_parameter("apply_camera_imu_offset", False)
        # TIMESYNC 配对：本地请求的等待窗口，以及是否校验回包来源。
        # PX4 自己也会主动发 TIMESYNC(tc1=0)，不能因为插进来一条就把在等的请求作废。
        self.declare_parameter("pending_timeout_s", 0.25)
        self.declare_parameter("validate_timesync_source", True)

        # ---- 数据与诊断 ----
        self.declare_parameter("imu_topic", CONTRACT_IMU_TOPIC)
        self.declare_parameter("expected_rate_hz", 50.0)
        self.declare_parameter("max_sample_age_s", 1.0)
        # 允许的最大「未来」量（负年龄）：TIMESYNC 中点估计自身有 RTT/2 量级不确定度，
        # 采样时刻略微超前于收包时刻是正常的；真正错乱的时钟会远超该值。
        self.declare_parameter("min_sample_age_s", -0.05)
        self.declare_parameter("ros_offset_stale_s", 1.0)
        self.declare_parameter("timebase_uncertainty_limit_s", 0.005)
        self.declare_parameter("heartbeat_timeout_s", 3.0)
        self.declare_parameter("stats_topic", "/boom_birds/imu/mavlink_status")
        self.declare_parameter("diagnostics_topic", "/boom_birds/imu/diagnostics")
        self.declare_parameter("diagnostics_rate_hz", 1.0)

        self.imu_topic = str(self.get_parameter("imu_topic").value)
        if self.imu_topic != CONTRACT_IMU_TOPIC:
            self.get_logger().warn(
                f"imu_topic={self.imu_topic} 不是契约话题 {CONTRACT_IMU_TOPIC}；"
                "OpenVINS/契约链路只消费契约话题"
            )

        device_id = int(self.get_parameter("device_id").value)
        self.core = MavlinkImuReceiver(
            config=MavlinkImuConfig(
                system_id=int(self.get_parameter("target_system").value),
                component_id=int(self.get_parameter("target_component").value),
                accept_any_component=bool(self.get_parameter("accept_any_component").value),
                device_id=None if device_id < 0 else device_id,
                expected_rate_hz=float(self.get_parameter("expected_rate_hz").value),
                camera_imu_offset_s=float(self.get_parameter("camera_imu_offset_s").value),
                apply_camera_imu_offset=bool(self.get_parameter("apply_camera_imu_offset").value),
                max_sample_age_s=float(self.get_parameter("max_sample_age_s").value),
                min_sample_age_s=float(self.get_parameter("min_sample_age_s").value),
                ros_offset_stale_s=float(self.get_parameter("ros_offset_stale_s").value),
                timebase_uncertainty_limit_s=float(
                    self.get_parameter("timebase_uncertainty_limit_s").value
                ),
                heartbeat_timeout_s=float(self.get_parameter("heartbeat_timeout_s").value),
                pending_timeout_s=float(self.get_parameter("pending_timeout_s").value),
                validate_timesync_source=bool(
                    self.get_parameter("validate_timesync_source").value
                ),
            ),
            clock_config=ClockMapperConfig(
                max_rtt_s=float(self.get_parameter("max_rtt_s").value),
                sync_timeout_s=float(self.get_parameter("sync_timeout_s").value),
                max_deviation_s=float(self.get_parameter("max_offset_deviation_s").value),
                jump_confirm_samples=int(self.get_parameter("jump_confirm_samples").value),
                converge_samples=int(self.get_parameter("converge_samples").value),
            ),
        )
        self.core.set_ros_clock(self.get_clock())

        # ---- 链路 ----
        self._conn = None
        self._writer = None
        self._stop = threading.Event()
        self._last_error: str | None = None
        self._send_blocked_logged = False
        self._connect()
        if self._writer is None:
            # udpin 在对端出现前不可回发；对端地址会在收到第一条数据时确定。
            self.get_logger().info(
                "当前为被动监听模式（udpin）：收到对端数据后才会发送 TIMESYNC 请求"
            )

        # ---- 发布 ----
        self.pub_imu = self.create_publisher(Imu, self.imu_topic, QOS_IMU)
        self.pub_stats = self.create_publisher(
            String, str(self.get_parameter("stats_topic").value), 10
        )
        self.pub_diag = self.create_publisher(
            DiagnosticArray, str(self.get_parameter("diagnostics_topic").value), 10
        )

        self._last_report_reason = _REASON_NONE
        self._last_stream_request_mono = 0.0
        self._diag_interval = 1.0 / max(float(self.get_parameter("diagnostics_rate_hz").value), 1e-3)

        # ---- 线程与定时器 ----
        self._thread = threading.Thread(target=self._receive_loop, name="mavlink_rx", daemon=True)
        self._thread.start()
        self.create_timer(1.0 / max(float(self.get_parameter("timesync_rate_hz").value), 1e-3),
                          self._on_timesync_timer)
        self.create_timer(self._diag_interval, self._on_diagnostics_timer)

        self.get_logger().info(
            f"MAVLink IMU 接收已启动：connection={self._connection_str} "
            f"target={self.core.config.system_id}/{self.core.config.component_id} "
            f"stream={self.get_parameter('stream_rate_hz').value}Hz "
            f"topic={self.imu_topic}"
        )
        if self.core.config.apply_camera_imu_offset:
            self.get_logger().warn(
                f"已启用相机—IMU 时间偏移归算：offset={self.core.config.camera_imu_offset_s}s "
                "(offset = t_cam_ros - t_imu_ros)；未标定前应保持关闭"
            )
        else:
            self.get_logger().warn(
                "相机—IMU 时间偏移未标定（apply_camera_imu_offset=false）："
                "IMU 时间戳只做时钟域映射，不含相机触发/曝光偏移"
            )

    # ---------------------------------------------------------------- 链路管理

    @property
    def _connection_str(self) -> str:
        return str(self.get_parameter("connection").value)

    def _connect(self) -> None:
        """建立链路。serial: 前缀显式携带设备与波特率；其余交给 pymavlink。"""
        try:
            from pymavlink import mavutil
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                "未安装 pymavlink：真实 MAVLink 链路需要它（离线单测不需要）。"
                "安装方式见模块 README"
            ) from exc

        conn_str = self._connection_str
        dialect = str(self.get_parameter("dialect").value)
        if conn_str.startswith("serial:"):
            target = conn_str[len("serial:"):]
            self._conn = mavutil.mavlink_connection(
                target,
                baud=int(self.get_parameter("baud").value),
                dialect=dialect,
                source_system=255,
                source_component=190,
                autoreconnect=True,
            )
            self._writer = self._conn
        elif conn_str.startswith("udpin:") or conn_str == "udpin":
            self._conn = mavutil.mavlink_connection(
                conn_str, dialect=dialect, source_system=255, source_component=190,
                input=True,
            )
            # 收到第一个对端地址后即可回发（TIMESYNC 请求）
            self._writer = None
        else:
            self._conn = mavutil.mavlink_connection(
                conn_str, dialect=dialect, source_system=255, source_component=190,
                input=True,
            )
            self._writer = self._conn

        if self._conn is None:
            raise RuntimeError(f"无法建立 MAVLink 链路：{conn_str}")
        self.get_logger().info(f"MAVLink 链路已建立：{conn_str}")

    def _send(self, msg) -> bool:
        """发送一条 MAVLink 消息（仅 TIMESYNC 与流请求）。返回是否成功。"""
        if self._writer is None:
            # udpin：等第一个对端地址出现后再回发；此前只记录一次，避免刷屏。
            if not self._send_blocked_logged:
                self._send_blocked_logged = True
                self.get_logger().info("尚未收到对端数据，暂不能发送（udpin 模式）")
            return False
        try:
            self._writer.write(msg.pack(self._writer.mav))
            return True
        except Exception as exc:  # noqa: BLE001
            self._last_error = f"send_failed: {exc}"
            return False

    def _receive_loop(self) -> None:
        """后台接收循环：解码 → 校验 → 发布。绝不使用收包时刻做数据时间戳。"""
        empty_reads = 0
        while not self._stop.is_set():
            try:
                self.core.stamp()               # 每周期刷新 ROS 时间基采样
                msg = self._conn.recv_match(blocking=True, timeout=self._read_timeout)
            except Exception as exc:  # noqa: BLE001
                self._last_error = f"recv_failed: {exc}"
                time.sleep(0.2)
                continue
            if msg is None:
                empty_reads += 1
                if empty_reads % 50 == 0:
                    self._note_quiet()
                continue
            empty_reads = 0
            if self._writer is None and self._conn is not None:
                # udpin 收到数据后才具备可回发地址
                self._writer = self._conn
            self._dispatch(msg)

    def _dispatch(self, msg) -> None:
        mono_now = time.monotonic()
        try:
            samples = self.core.handle_message(msg, mono_now)
        except Exception as exc:  # noqa: BLE001
            self._last_error = f"handle_failed: {exc}"
            self.get_logger().error(f"处理 MAVLink 消息失败：{exc}")
            return
        for sample in samples:
            self.pub_imu.publish(self.core.to_ros_imu(sample))
            self.core.note_published()
        if samples:
            self._last_report_reason = _REASON_NONE
        else:
            self._report_blocked()

    def _report_blocked(self) -> None:
        """映射/校验失败时给出可见诊断，但只在原因变化时打印一次，避免刷屏。

        注意：TIMESYNC 配对失败（`timesync_*`）也会写进 `last_mapping_reason`，
        但那时**并没有**丢弃 IMU 采样（配对失败只损失一次时钟样本）。两种情况要分开说，
        否则日志会误报「已丢弃 IMU 采样」。
        """
        reason = self.core.counters.get("last_mapping_reason")
        if reason is None or reason == self._last_report_reason:
            return
        self._last_report_reason = reason
        if reason.startswith("timesync_"):
            timesync_reason = self.core.counters.get("last_timesync_reason") or reason
            self.get_logger().warn(
                f"TIMESYNC 配对未完成（{timesync_reason}）：本次不产生时钟样本，继续等待"
            )
        elif reason.startswith("no_clock_mapping") or reason in ("no_timesync_sample", "timesync_stale"):
            self.get_logger().warn(
                f"时钟映射未建立（{reason}）：拒绝发布 IMU 时间戳，等待 TIMESYNC 收敛"
            )
        else:
            self.get_logger().warn(f"已丢弃 HIGHRES_IMU 采样（{reason}）")

    def _note_quiet(self) -> None:
        stats = self.core.stats(time.monotonic())
        if not stats["connected"]:
            self.get_logger().warn("尚未收到飞控 heartbeat：核对串口设备、波特率、接线与端口配置")

    # ---------------------------------------------------------------- 定时任务

    @property
    def _read_timeout(self) -> float:
        return max(float(self.get_parameter("read_timeout_s").value), 1e-3)

    def _on_timesync_timer(self) -> None:
        send_mono = time.monotonic()
        send_ns = int(send_mono * 1e9)
        try:
            msg = self._conn.mav.timesync_encode(0, send_ns)
        except Exception as exc:  # noqa: BLE001
            self._last_error = f"timesync_encode_failed: {exc}"
            return
        if self._send(msg):
            self.core.note_timesync_request(send_mono, send_ns)
        self._maybe_request_stream(send_mono)

    def _maybe_request_stream(self, now_mono: float) -> None:
        """请求 HIGHRES_IMU 流频率。

        频率由参数给出，不假定 115200 一定支持目标值：实际达成频率由
        `/boom_birds/imu/mavlink_status` 的 `imu_rate_hz` 观测，必要时下调。
        """
        if not bool(self.get_parameter("auto_request_stream").value):
            return
        retry = max(float(self.get_parameter("stream_request_retry_s").value), 0.1)
        if now_mono - self._last_stream_request_mono < retry:
            return
        self._last_stream_request_mono = now_mono
        rate = float(self.get_parameter("stream_rate_hz").value)
        try:
            msg = self._conn.mav.command_long_encode(
                self.core.config.system_id,
                self.core.config.component_id,
                MAV_CMD_SET_MESSAGE_INTERVAL,
                0,                                  # confirmation
                MAVLINK_MSG_ID_HIGHRES_IMU,
                1e6 / max(rate, 1e-3),              # interval (us)；-1 表示恢复默认
                0, 0, 0, 0, 0,
            )
        except Exception as exc:  # noqa: BLE001
            self._last_error = f"stream_request_encode_failed: {exc}"
            return
        self._send(msg)

    def _on_diagnostics_timer(self) -> None:
        mono_now = time.monotonic()
        stats = self.core.stats(mono_now)
        stats["link"] = {
            "connection": self._connection_str,
            "last_error": self._last_error,
            "response_rate_hz": float(self.get_parameter("stream_rate_hz").value),
        }
        out = String()
        out.data = json.dumps(stats, ensure_ascii=False, sort_keys=True)
        self.pub_stats.publish(out)
        self.pub_diag.publish(self._diagnostic_array(stats, mono_now))

    def _diagnostic_array(self, stats: dict, mono_now: float) -> DiagnosticArray:
        sync = stats.get("time_sync") or {}
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus()
        ok = bool(sync.get("locked")) and bool(stats.get("connected"))
        st.level = DiagnosticStatus.OK if ok else DiagnosticStatus.WARN
        if not stats.get("connected"):
            st.message = "无飞控 heartbeat"
        elif not sync.get("locked"):
            st.message = f"时钟映射不可用（{sync.get('invalid_reason')}）"
        else:
            st.message = "IMU 时间映射正常"
        st.name = "boom_birds/mavlink_imu"
        st.hardware_id = self._connection_str
        keys = [
            ("connected", stats.get("connected")),
            ("imu_rate_hz", stats.get("imu_rate_hz")),
            ("imu_rate_expected_hz", stats.get("imu_rate_expected_hz")),
            ("interval_max_s", stats.get("interval_max_s")),
            ("gaps", stats.get("gaps")),
            ("published", stats.get("published")),
            ("rejected_total", self._rejected_total(stats)),
            ("time_sync_locked", sync.get("locked")),
            ("time_sync_offset_s", sync.get("offset_s")),
            ("time_sync_rtt_last_s", sync.get("rtt_last_s")),
            ("time_sync_rtt_median_s", sync.get("rtt_median_s")),
            ("time_sync_error_bound_s", sync.get("error_bound_s")),
            ("time_sync_sample_age_s", sync.get("sample_age_s")),
            ("time_sync_resets", sync.get("filter_resets")),
            ("timesync_paired", stats.get("timesync_paired")),
            ("timesync_rejected_source", stats.get("rejected_timesync_source")),
            ("timesync_rejected_echo_mismatch", stats.get("rejected_timesync_echo_mismatch")),
            ("timesync_rejected_timeout", stats.get("rejected_timesync_timeout")),
            ("time_sync_last_reason", stats.get("last_timesync_reason")),
            ("px4_restarts", sync.get("px4_restarts")),
            ("ros_timebase", stats.get("ros_timebase")),
            ("sample_to_publish_delay_s", stats.get("sample_to_publish_delay_s")),
            ("camera_imu_offset_s", stats.get("camera_imu_offset_s")),
            ("camera_imu_offset_applied", stats.get("camera_imu_offset_applied")),
        ]
        for key, value in keys:
            if value is None:
                continue
            st.values.append(KeyValue(key=key, value=str(value)))
        arr.status.append(st)
        return arr

    @staticmethod
    def _rejected_total(stats: dict) -> int:
        return sum(
            int(stats.get(k) or 0)
            for k in (
                "rejected_source", "rejected_fields", "rejected_id", "rejected_nonfinite",
                "rejected_epoch_time", "rejected_boot_backwards", "rejected_ros_backwards",
                "rejected_duplicate", "rejected_future", "rejected_stale",
                "rejected_no_clock_mapping",
            )
        )

    # ---------------------------------------------------------------- 收尾

    def shutdown(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:  # noqa: BLE001
                pass


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = None
    try:
        node = MavlinkImuNode()
        rclpy.spin(node)
    except Exception as exc:  # noqa: BLE001
        print(f"[boom_birds_nav] mavlink_imu_node 启动失败：{exc}")
        raise
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
