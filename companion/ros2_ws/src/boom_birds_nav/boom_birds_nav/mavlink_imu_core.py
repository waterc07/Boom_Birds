"""PX4 `HIGHRES_IMU` → `sensor_msgs/Imu` 的纯 Python 接收核心（不含 rclpy）。

职责边界
--------
本模块只做「MAVLink 帧已经解码成对象」之后的事情：
1. 来源校验：sysid / compid /（可选）IMU 实例 id；
2. 字段校验：`fields_updated` 必须包含加速度与角速度位，数值必须有限；
3. 时间校验：`time_usec` 必须是 **PX4 启动时钟**（不是 UNIX epoch），且单调；
4. 时间戳映射：`time_usec` → PX4 启动时钟秒 → Companion 单调时钟 → ROS 时间，
   **绝不使用串口收包时刻**；
5. 诊断：实际频率、消息间隔、丢样、时间戳年龄、时间回退、拒绝原因计数。

时间戳映射的配对前提
--------------------
`HIGHRES_IMU` 的时间戳依赖 TIMESYNC 往返样本，而 PX4 自己也会以 10 Hz 主动广播
`TIMESYNC(tc1=0, ts1=启动时钟纳秒)`（`src/modules/mavlink/streams/TIMESYNC.hpp`）。
这类广播**不是**我们请求的响应：它只能作为「链路活着 + 飞控启动时钟可用」的粗基线，
绝不能让本地「待配对请求」作废——否则插在「请求」与「响应」之间的那条广播
会把紧随其后的响应变成孤儿，偏移估计静默丢样本，映射会间歇性失效。
配对规则（含超时与来源校验）见 `MavlinkImuReceiver._handle_timesync`。

不依赖 rclpy 是有意的：时钟映射、校验与失效行为可以在无 ROS 运行时逐条单测，
节点层只负责收发与话题。

物理约定（与契约一致）
----------------------
- `linear_acceleration`：m/s²，机体系 FRD（与 PX4 `vehicle_imu` → `HIGHRES_IMU` 一致），
  **包含重力反作用**：静止水平时 z ≈ +9.81 m/s²。
- `angular_velocity`：rad/s，机体系 FRD。
- `orientation`：**不发布**飞控融合姿态。按 ROS 约定用 `w=1` 的四元数并把
  `orientation_covariance[0]` 置 `-1` 表示「无姿态信息」，不用 EKF 姿态冒充 IMU 测量。
- PX4 在 `HIGHRES_IMU.hpp` 里已扣除估计器给出的加速度/角速度零偏；本模块不再二次扣除，
  也不做任何旋转、单位换算或标定。
"""

from __future__ import annotations

import math
import statistics
import time
from collections import deque
from dataclasses import dataclass

from .mavlink_clock import (
    REASON_BACKWARDS,
    REASON_NONPOSITIVE,
    ClockMapper,
    ClockMapperConfig,
)
from .timebase import RosTimeBase, TimeBaseSample, msg_from_ros_seconds

# MAVLink 消息 ID（common dialect）
MAVLINK_MSG_ID_HEARTBEAT = 0
MAVLINK_MSG_ID_HIGHRES_IMU = 105
MAVLINK_MSG_ID_TIMESYNC = 111

# 注意：pymavlink 解码出的消息对象**没有 `msgid` 属性**（只有 `get_msgId()`/`get_type()`），
# 因此按类型名分派，避免静默地把所有消息都当成未知类型。
_TYPE_TO_MSGID = {
    "HEARTBEAT": MAVLINK_MSG_ID_HEARTBEAT,
    "HIGHRES_IMU": MAVLINK_MSG_ID_HIGHRES_IMU,
    "TIMESYNC": MAVLINK_MSG_ID_TIMESYNC,
}


def message_id(msg) -> int:
    """取得 MAVLink 消息 ID（兼容 pymavlink 对象与自带 msgid 的替身）。"""
    value = getattr(msg, "msgid", None)
    if value is not None:
        return int(value)
    getter = getattr(msg, "get_msgId", None)
    if callable(getter):
        return int(getter())
    return _TYPE_TO_MSGID.get(msg.get_type(), -1)


def message_sysid(msg) -> int:
    """取得源系统 ID。

    注意：pymavlink 解码对象**没有 `sysid` 属性**（只有 `get_srcSystem()`），
    因此不能写成 `getattr(msg, "sysid", -1)`，否则所有真实消息都会被当成来源不符。
    """
    value = getattr(msg, "sysid", None)
    if value is not None:
        return int(value)
    getter = getattr(msg, "get_srcSystem", None)
    if callable(getter):
        return int(getter())
    return -1


def message_compid(msg) -> int:
    """取得源组件 ID（同上：pymavlink 用 `get_srcComponent()`）。"""
    value = getattr(msg, "compid", None)
    if value is not None:
        return int(value)
    getter = getattr(msg, "get_srcComponent", None)
    if callable(getter):
        return int(getter())
    return -1

# HIGHRES_IMU.fields_updated 位定义（MAVLink common.xml，bit 0 = xacc … bit 12 = temperature）
ACCEL_BITS = (1 << 0) | (1 << 1) | (1 << 2)
GYRO_BITS = (1 << 3) | (1 << 4) | (1 << 5)
MAG_BITS = (1 << 6) | (1 << 7) | (1 << 8)
VALID_FIELD_BITS = (1 << 13) - 1

# boot 时钟量级阈值（微秒）：PX4 启动时钟远小于该值，UNIX epoch 远大于该值。
BOOT_TIME_MAX_US = 10**14

# 输出话题/坐标系固定按契约；节点参数只允许用前缀派生诊断话题。
CONTRACT_IMU_TOPIC = "/boom_birds/imu"
CONTRACT_IMU_FRAME = "imu"

# 拒绝原因码
REJECT_FIELDS = "fields_updated_missing_accel_or_gyro"
REJECT_ID = "device_id_mismatch"
REJECT_NONFINITE = "nonfinite_value"
REJECT_EPOCH_TIME = "time_usec_not_boot_time"
REJECT_BOOT_BACKWARDS = "boot_time_backwards"
REJECT_ROSBACKWARDS = "ros_time_backwards"
REJECT_DUPLICATE = "duplicate_timestamp"
REJECT_FUTURE = "sample_in_future"
REJECT_STALE = "sample_age_too_large"
REJECT_NO_SYNC = "no_clock_mapping"
REJECT_TIMEBASE = "ros_timebase_unstable"

# TIMESYNC 配对拒绝原因码（写入 last_timesync_reason，并同步写 last_mapping_reason）。
# "timesync_unpaired" 与 ClockMapper 的 REASON_UNPAIRED 用同一字面量：同一种失效在
# 诊断面上不应出现两个名字。
REJECT_TIMESYNC_UNPAIRED = "timesync_unpaired"
REJECT_TIMESYNC_ECHO = "timesync_echo_mismatch"
REJECT_TIMESYNC_TIMEOUT = "timesync_pending_timeout"
REJECT_TIMESYNC_SOURCE = "timesync_source_mismatch"

_REJECT_COUNTER = {
    REJECT_FIELDS: "rejected_fields",
    REJECT_ID: "rejected_id",
    REJECT_NONFINITE: "rejected_nonfinite",
    REJECT_EPOCH_TIME: "rejected_epoch_time",
    REJECT_BOOT_BACKWARDS: "rejected_boot_backwards",
}


@dataclass
class HighresValidation:
    """单条 HIGHRES_IMU 的校验结果。"""

    ok: bool
    reason: str | None = None
    time_usec: int = 0
    accel: tuple[float, float, float] | None = None
    gyro: tuple[float, float, float] | None = None
    fields_updated: int = 0
    device_id: int | None = None


@dataclass
class ImuSample:
    """一条可发布的 IMU 采样（时间戳已落在 ROS 时间域）。"""

    stamp_ros_s: float
    boot_us: int
    age_s: float                              # 采样时刻到收包时刻（含链路与映射）
    accel: tuple[float, float, float]
    gyro: tuple[float, float, float]
    fields_updated: int
    device_id: int | None
    time_sync_error_bound_s: float
    time_sync_state: str
    camera_offset_applied_s: float            # 若做过相机时基归算，记录所用偏移


@dataclass
class MavlinkImuConfig:
    """接收核心配置（全部可通过节点参数 / YAML 覆盖，无历史设备硬编码）。"""

    system_id: int = 1
    component_id: int = 1
    accept_any_component: bool = True     # 真机自动选主组件：只校验 sysid
    device_id: int | None = None          # HIGHRES_IMU.id（多 IMU 时选择）；None = 不校验
    expected_rate_hz: float = 50.0        # 仅用于丢样判定，不假定串口一定支持
    rate_window: int = 200
    camera_imu_offset_s: float = 0.0      # offset = t_cam_ros - t_imu_ros
    apply_camera_imu_offset: bool = False
    ros_offset_stale_s: float = 1.0       # ROS 时钟映射样本多久算过期
    max_sample_age_s: float = 1.0         # 采样→收包的最大允许年龄
    min_sample_age_s: float = -0.05       # 允许的最大「未来」量（负年龄）
    heartbeat_timeout_s: float = 3.0
    timebase_uncertainty_limit_s: float = 0.005
    # TIMESYNC 配对：本地请求的等待窗口（秒）。超过它仍未配对的请求按「响应已丢失」
    # 丢弃——MAVLink 不重传，一直挂着只会让之后的响应全部被误判成无配对。
    # 该窗口只影响配对，不影响已锁定偏移的失效判定（那是 ClockMapper.sync_timeout_s）。
    pending_timeout_s: float = 0.25
    # 是否校验 TIMESYNC 来源（判据镜像 `_source_ok`：sysid 必查，compid 仅在
    # `accept_any_component=False` 时查）。多链路/编队转发时，别人的 TIMESYNC 会把
    # 错误时钟带进偏移估计；真机上若飞控 sysid/compid 与配置不符，可临时关掉先跑通链路。
    validate_timesync_source: bool = True


class MavlinkImuReceiver:
    """接收核心：`handle_message()` 返回本次应当发布的采样列表（0 或 1 条）。"""

    def __init__(
        self,
        config: MavlinkImuConfig | None = None,
        clock: ClockMapper | None = None,
        clock_config: ClockMapperConfig | None = None,
    ) -> None:
        self.config = config or MavlinkImuConfig()
        self.clock = clock if clock is not None else ClockMapper(clock_config or ClockMapperConfig())
        self._timebase: RosTimeBase | None = None
        self._last_timebase: TimeBaseSample | None = None
        self._virtual_mono_s: float | None = None      # 离线回放：外部提供的单调时刻
        self._last_stamp_ros_s: float | None = None
        self._last_boot_us: int | None = None
        self._last_recv_mono_s: float | None = None
        self._intervals: deque = deque(maxlen=max(int(self.config.rate_window), 2))
        self._heartbeat_last_mono_s: float | None = None
        self._heartbeat_autopilot: int | None = None
        # 未配对的 TIMESYNC 响应无法定位 t1；只有配对信息完整时才提交样本。
        # _pending_send_mono_s 是发出请求的单调时刻（算 RTT 与超时），
        # _pending_send_ns 是随请求发出的 ts1（飞控原样回抄，用它做逐位配对）。
        self._pending_send_mono_s: float | None = None
        self._pending_send_ns: int | None = None
        self._reset_epoch_seen: int = 0
        self._counters: dict = {
            "messages_total": 0,
            "messages_other": 0,
            "highres_imu_seen": 0,
            "heartbeats": 0,
            "published": 0,
            "rejected_source": 0,
            "rejected_fields": 0,
            "rejected_id": 0,
            "rejected_nonfinite": 0,
            "rejected_epoch_time": 0,
            "rejected_boot_backwards": 0,
            "rejected_ros_backwards": 0,
            "rejected_duplicate": 0,
            "rejected_future": 0,
            "rejected_stale": 0,
            "rejected_no_clock_mapping": 0,
            "rejected_unpaired_timesync": 0,
            "rejected_timesync_echo_mismatch": 0,
            "rejected_timesync_timeout": 0,
            "rejected_timesync_source": 0,
            "px4_timesync_requests": 0,
            "timesync_paired": 0,
            "timesync_requests_superseded": 0,
            "last_timesync_reason": None,
            "gaps": 0,
            "mapping_failures": 0,
            "timeline_resets": 0,
            "last_mapping_reason": None,
        }

    # ------------------------------------------------------------------ 依赖注入

    def set_ros_clock(self, clock, max_bracket_s: float = 2e-3) -> None:
        """注入 rclpy Clock，建立「单调时钟 → ROS 时钟」映射。"""
        self._timebase = RosTimeBase(clock, max_bracket_s=max_bracket_s)

    def set_timebase(self, timebase: RosTimeBase) -> None:
        """注入任意时间基（离线回放 / 单测用固定偏移或自定义时钟）。"""
        self._timebase = timebase

    def set_virtual_mono(self, mono_s: float | None) -> None:
        """离线回放：用外部提供的单调时刻替代 `time.monotonic()`。None 表示恢复真实时钟。"""
        self._virtual_mono_s = None if mono_s is None else float(mono_s)

    def now_mono(self) -> float:
        """当前单调时刻（虚拟模式下由外部注入）。"""
        return time.monotonic() if self._virtual_mono_s is None else self._virtual_mono_s

    def stamp(self) -> TimeBaseSample:
        """采样当前时间基（接收循环每周期调用一次，使映射紧跟当前时刻）。"""
        if self._timebase is None:
            raise RuntimeError("未设置 ROS 时钟：请先调用 set_ros_clock() 或 set_timebase()")
        sample = self._timebase.sample()
        self._last_timebase = sample
        return sample

    # ------------------------------------------------------------------ 消息分发

    def handle_message(self, msg, mono_now_s: float | None = None) -> list:
        """处理一条已解码的 MAVLink 消息，返回本次应发布的 `ImuSample` 列表。

        msg 需具备 pymavlink 风格属性（`msgid`/`sysid`/`compid` + 字段）。
        """
        if mono_now_s is None:
            mono_now_s = self.now_mono()
        self._counters["messages_total"] += 1

        msgid = message_id(msg)
        if msgid == MAVLINK_MSG_ID_HEARTBEAT:
            return self._handle_heartbeat(msg, mono_now_s)
        if msgid == MAVLINK_MSG_ID_TIMESYNC:
            self._handle_timesync(msg, mono_now_s)
            return []
        if msgid == MAVLINK_MSG_ID_HIGHRES_IMU:
            self._counters["highres_imu_seen"] += 1
            sample = self._handle_highres(msg, mono_now_s)
            return [sample] if sample is not None else []
        self._counters["messages_other"] += 1
        return []

    def _source_ok(self, msg) -> bool:
        if self.config.system_id is not None and message_sysid(msg) != self.config.system_id:
            return False
        if not self.config.accept_any_component and message_compid(msg) != self.config.component_id:
            return False
        return True

    def _handle_heartbeat(self, msg, mono_now_s: float) -> list:
        self._counters["heartbeats"] += 1
        if self._source_ok(msg):
            self._heartbeat_last_mono_s = mono_now_s
            self._heartbeat_autopilot = int(getattr(msg, "autopilot", -1))
        return []

    # ------------------------------------------------------------------ TIMESYNC

    def note_timesync_request(self, send_mono_s: float, send_ns: int) -> None:
        """记录一次 TIMESYNC 请求的发送时刻（单调时钟秒）与线上 ts1（纳秒）。

        只保留最新一次请求：MAVLink 是无连接协议，迟到的旧响应必须被丢弃，
        否则会把陈旧样本当成当前偏移。

        覆盖旧请求前先看它是否已超过 `pending_timeout_s`：**超时**（响应丢了）与
        **被覆盖**（响应比请求周期还慢）分开计数，真机排查时含义不同。
        """
        send_mono_s = float(send_mono_s)
        if not self._expire_pending_request(send_mono_s) and self._pending_send_ns is not None:
            self._counters["timesync_requests_superseded"] += 1
        self._pending_send_mono_s = send_mono_s
        self._pending_send_ns = int(send_ns)
        self.clock.note_outbound()

    def _clear_pending_request(self) -> None:
        self._pending_send_mono_s = None
        self._pending_send_ns = None

    def _expire_pending_request(self, mono_now_s: float) -> bool:
        """丢弃超过 `pending_timeout_s` 仍未配对的本地请求，返回是否真的丢弃。

        为什么必须超时：MAVLink 没有重传。响应一丢，pending 就会一直挂着，之后每条
        响应都被当成「无配对」，而新请求又把它覆盖，配对永远错位。这里只丢配对状态，
        不动已锁定的偏移估计——配对失败只是少一个样本，映射还能不能用由
        `ClockMapper.mapping_valid()`（`sync_timeout_s`）单独判定。
        """
        if self._pending_send_mono_s is None:
            return False
        if mono_now_s - self._pending_send_mono_s <= float(self.config.pending_timeout_s):
            return False
        self._clear_pending_request()
        self._counters["rejected_timesync_timeout"] += 1
        return True

    def _timesync_source_ok(self, msg) -> bool:
        """TIMESYNC 来源校验（可用 `validate_timesync_source` 关闭）。

        判据刻意与 `_source_ok` 完全一致（sysid 必查，compid 只在
        `accept_any_component=False` 时查），避免同一份配置在 IMU 与 TIMESYNC 两条
        路径上给出不同结论。校验对所有 TIMESYNC 生效（含飞控主动请求）：别人的启动
        时钟连「粗基线」都不该进我们的诊断。
        """
        if not self.config.validate_timesync_source:
            return True
        return self._source_ok(msg)

    def _note_timesync_reason(self, reason: str) -> None:
        """记录最近一次 TIMESYNC 配对**失败**原因（成功时不写，保留上一次失败线索）。

        与既有 `last_mapping_reason` 的语义一致：它是「最近一次拒绝原因」，
        不是「当前状态」；当前状态看 `clock.stats()` 的 locked/invalid_reason。

        - `last_timesync_reason`：本模块自己的配对诊断，只由 TIMESYNC 路径写；
        - `last_mapping_reason`：既有键（节点与回放报告都读它）。配对失败就是
          「时钟映射少了一个样本」，同步写进去才能让现有诊断面看得见这次丢样。
        """
        self._counters["last_timesync_reason"] = reason
        self._counters["last_mapping_reason"] = reason

    def _handle_timesync(self, msg, mono_now_s: float) -> None:
        """处理一条 TIMESYNC：来源校验 → 分「飞控主动请求」与「对我们的响应」。

        配对规则（顺序即语义，改动前先读模块头「时间戳映射的配对前提」）：
        1. 来源不符（且开启校验）：整条消息丢弃，不碰 pending；
        2. `tc1 == 0`：PX4 主动广播，只交给 ClockMapper 当粗基线，**绝不清 pending**；
        3. 无 pending 可配对的响应（从没发过请求，或请求已被超时清掉）：计
           `rejected_unpaired_timesync`，不产生偏移样本；
        4. 回显 `ts1` 与记录的 `_pending_send_ns` 不逐位相等：这是被覆盖的旧请求的
           迟到响应，丢弃但**保留**当前 pending 等它自己的响应；
        5. 逐位相等：消费该 pending 并提交往返样本。

        为什么按 `ts1` 逐位配对，而不是「收到响应就用最后一个请求」：飞控回抄的是
        我们发出去的原值（PX4 `mavlink_timesync.cpp`：`rsync.ts1 = tsync.ts1`），
        因此等值比较就是「这条响应属于哪次请求」的可靠判据；只按到达顺序配对，
        一旦有响应迟到或乱序，就会用错误的 t1 算出错误偏移。
        """
        tc1 = int(getattr(msg, "tc1", 0))
        ts1 = int(getattr(msg, "ts1", 0))

        if not self._timesync_source_ok(msg):
            self._counters["rejected_timesync_source"] += 1
            self._note_timesync_reason(REJECT_TIMESYNC_SOURCE)
            return

        if tc1 == 0:
            # PX4 10 Hz 广播（`streams/TIMESYNC.hpp`：tc1=0、ts1=启动时钟纳秒）。
            # 它证明链路与飞控启动时钟可用，可以给 ClockMapper 当基线；但它与我们的
            # 请求没有任何配对关系——在这里清 pending 正是「响应变孤儿」的根因。
            self._counters["px4_timesync_requests"] += 1
            self.clock.on_timesync(tc1=0, ts1=ts1, t1_mono_s=mono_now_s, t3_mono_s=mono_now_s)
            return

        # 响应（tc1 != 0）：先看有没有仍在等待窗口内的本地请求。
        timed_out = False
        if self._pending_send_mono_s is not None:
            timed_out = self._expire_pending_request(mono_now_s)
        if self._pending_send_ns is None or self._pending_send_mono_s is None:
            # 没有可配对的请求：迟到响应的 t1 不可信（RTT 无上界），不产生偏移样本。
            # 统一用 t1 = t3 = 收包时刻交给 ClockMapper：它会以 `t3 <= t1` 拒绝，
            # 只留下计数，不会把「零 RTT 的假样本」写进估计。
            self._counters["rejected_unpaired_timesync"] += 1
            self._note_timesync_reason(
                REJECT_TIMESYNC_TIMEOUT if timed_out else REJECT_TIMESYNC_UNPAIRED
            )
            self.clock.on_timesync(tc1=tc1, ts1=ts1, t1_mono_s=mono_now_s, t3_mono_s=mono_now_s)
            return

        t1 = self._pending_send_mono_s
        sent_ns = self._pending_send_ns
        if ts1 != sent_ns:
            self._counters["rejected_timesync_echo_mismatch"] += 1
            self._note_timesync_reason(REJECT_TIMESYNC_ECHO)
            # 仍交给 ClockMapper（带 sent_ns_raw）：由它在时钟层再挡一次并计数，
            # 两条路径的计数口径保持一致。
            self.clock.on_timesync(
                tc1=tc1, ts1=ts1, t1_mono_s=t1, t3_mono_s=mono_now_s, sent_ns_raw=sent_ns
            )
            return

        # 配对成功：一次请求只允许被消费一次，否则重复响应会重复计入同一次往返。
        self._clear_pending_request()
        self._counters["timesync_paired"] += 1
        self.clock.on_timesync(
            tc1=tc1, ts1=ts1, t1_mono_s=t1, t3_mono_s=mono_now_s, sent_ns_raw=sent_ns
        )

    # ------------------------------------------------------------------ IMU 校验

    def validate_highres(self, msg) -> HighresValidation:
        """来源 / 字段 / 时间量级校验（不含时钟映射）。"""
        fields_updated = int(getattr(msg, "fields_updated", 0)) & VALID_FIELD_BITS
        accel = (
            float(getattr(msg, "xacc", float("nan"))),
            float(getattr(msg, "yacc", float("nan"))),
            float(getattr(msg, "zacc", float("nan"))),
        )
        gyro = (
            float(getattr(msg, "xgyro", float("nan"))),
            float(getattr(msg, "ygyro", float("nan"))),
            float(getattr(msg, "zgyro", float("nan"))),
        )
        time_usec = int(getattr(msg, "time_usec", 0))
        raw_id = getattr(msg, "id", None)
        device_id = None if raw_id is None else int(raw_id)

        if (fields_updated & (ACCEL_BITS | GYRO_BITS)) != (ACCEL_BITS | GYRO_BITS):
            return HighresValidation(False, REJECT_FIELDS, time_usec, accel, gyro,
                                     fields_updated, device_id)
        if self.config.device_id is not None and device_id != self.config.device_id:
            return HighresValidation(False, REJECT_ID, time_usec, accel, gyro,
                                     fields_updated, device_id)
        if not all(math.isfinite(v) for v in accel + gyro):
            return HighresValidation(False, REJECT_NONFINITE, time_usec, accel, gyro,
                                     fields_updated, device_id)
        if time_usec <= 0 or time_usec >= BOOT_TIME_MAX_US:
            return HighresValidation(False, REJECT_EPOCH_TIME, time_usec, accel, gyro,
                                     fields_updated, device_id)
        if self._last_boot_us is not None and time_usec < self._last_boot_us:
            return HighresValidation(False, REJECT_BOOT_BACKWARDS, time_usec, accel, gyro,
                                     fields_updated, device_id)
        return HighresValidation(True, None, time_usec, accel, gyro, fields_updated, device_id)

    def note_timeline_reset(self, reason: str) -> None:
        """通知接收核心：时钟时间线已重置（飞控重启 / 偏移跳变）。

        清空与旧时间线绑定的状态：已发布的最后一个时间戳、上一个 boot 时间、
        间隔统计。否则新时间线的第一条样本会被旧时间线的单调性判据误拒。
        """
        self.clock.reset_epoch += 1
        self._reset_epoch_seen = self.clock.reset_epoch
        self._last_stamp_ros_s = None
        self._last_boot_us = None
        self._intervals.clear()
        self._counters["timeline_resets"] += 1
        self._counters["last_timeline_reset_reason"] = reason

    def _handle_highres(self, msg, mono_now_s: float) -> ImuSample | None:
        if not self._source_ok(msg):
            self._counters["rejected_source"] += 1
            return None

        v = self.validate_highres(msg)
        if not v.ok:
            counter = _REJECT_COUNTER.get(v.reason)
            if counter:
                self._counters[counter] += 1
            self._counters["last_mapping_reason"] = v.reason
            return None

        # 时钟映射被重置（飞控重启 / 偏移跳变）意味着时间线不连续：
        # 清空单调性状态与间隔统计，避免用旧时间线的最后一个时间戳拒绝新时间线的第一条。
        if self.clock.reset_epoch != self._reset_epoch_seen:
            self._reset_epoch_seen = self.clock.reset_epoch
            self._last_stamp_ros_s = None
            self._last_boot_us = None
            self._intervals.clear()
            self._counters["timeline_resets"] += 1

        # 1) PX4 启动时钟 → Companion 单调时钟（MAVLink TIMESYNC 偏移）
        t_mono_s, reason = self.clock.map_boot_to_mono(v.time_usec, mono_now_s)
        if t_mono_s is None:
            self._counters["rejected_no_clock_mapping"] += 1
            self._counters["mapping_failures"] += 1
            self._counters["last_mapping_reason"] = reason
            return None

        # 2) Companion 单调时钟 → ROS 时间域
        if self._last_timebase is None:
            self._counters["rejected_no_clock_mapping"] += 1
            self._counters["last_mapping_reason"] = "no_ros_timebase"
            return None
        if self._last_timebase.uncertainty_s > self.config.timebase_uncertainty_limit_s:
            self._counters["rejected_no_clock_mapping"] += 1
            self._counters["last_mapping_reason"] = REJECT_TIMEBASE
            return None
        t_ros_s = t_mono_s + self._last_timebase.offset_s

        # 3) 可选：把 IMU 归算到相机时基（相机—IMU 时间偏移标定入口，默认关闭）
        shift = 0.0
        if self.config.apply_camera_imu_offset:
            shift = float(self.config.camera_imu_offset_s)
            t_ros_s += shift

        if t_ros_s <= 0.0:
            self._counters["rejected_stale"] += 1
            self._counters["last_mapping_reason"] = "nonpositive_ros_time"
            return None
        if self._last_stamp_ros_s is not None and t_ros_s < self._last_stamp_ros_s:
            self._counters["rejected_ros_backwards"] += 1
            self._counters["last_mapping_reason"] = REJECT_ROSBACKWARDS
            return None
        if self._last_stamp_ros_s is not None and t_ros_s == self._last_stamp_ros_s:
            self._counters["rejected_duplicate"] += 1
            self._counters["last_mapping_reason"] = REJECT_DUPLICATE
            return None

        age = mono_now_s - t_mono_s
        if age > self.config.max_sample_age_s:
            self._counters["rejected_stale"] += 1
            self._counters["last_mapping_reason"] = REJECT_STALE
            return None
        # 「未来」判据：允许的最大超前量为 max(|min_sample_age_s|, 当前映射误差上界 + 5 ms)。
        # 理由：TIMESYNC 中点估计自身有 RTT/2 量级的不确定度，若 RTT 较大，
        # 采样时刻合理地可能略微超前于收包时刻；用固定阈值会误杀正常样本，
        # 而完全不设阈值又会放过真正的时钟错误。
        # 注意 min_sample_age_s 是**负值**（阈值语义），比较时必须取绝对值。
        allowed_future_s = max(
            abs(self.config.min_sample_age_s), self.clock.error_bound_s() + 0.005
        )
        if age < -allowed_future_s:
            self._counters["rejected_future"] += 1
            self._counters["last_mapping_reason"] = REJECT_FUTURE
            return None

        # 4) 间隔统计使用采样时刻（不是收包时刻）
        if self._last_boot_us is not None:
            dt = (v.time_usec - self._last_boot_us) * 1e-6
            if dt > 0:
                self._intervals.append(dt)
                nominal = 1.0 / max(self.config.expected_rate_hz, 1e-6)
                if dt > 2.5 * nominal:
                    self._counters["gaps"] += 1
        self._last_boot_us = v.time_usec
        self._last_recv_mono_s = mono_now_s
        self._last_stamp_ros_s = t_ros_s

        return ImuSample(
            stamp_ros_s=t_ros_s,
            boot_us=v.time_usec,
            age_s=age,
            accel=v.accel,
            gyro=v.gyro,
            fields_updated=v.fields_updated,
            device_id=v.device_id,
            time_sync_error_bound_s=self.clock.error_bound_s(),
            time_sync_state="LOCKED",
            camera_offset_applied_s=shift,
        )

    # ------------------------------------------------------------------ 消息构造

    def to_ros_imu(self, sample: ImuSample):
        """`ImuSample` → `sensor_msgs/Imu`（m/s²、rad/s；orientation 标为不可用）。"""
        from sensor_msgs.msg import Imu

        msg = Imu()
        msg.header.stamp = msg_from_ros_seconds(sample.stamp_ros_s)
        msg.header.frame_id = CONTRACT_IMU_FRAME
        msg.linear_acceleration.x = float(sample.accel[0])
        msg.linear_acceleration.y = float(sample.accel[1])
        msg.linear_acceleration.z = float(sample.accel[2])
        msg.angular_velocity.x = float(sample.gyro[0])
        msg.angular_velocity.y = float(sample.gyro[1])
        msg.angular_velocity.z = float(sample.gyro[2])
        # 不用飞控融合姿态：按 ROS 约定把 orientation 标为不可用。
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation_covariance[0] = -1.0
        # 未估计不确定度：协方差填 0，而不是伪造数值。
        for i in range(9):
            msg.angular_velocity_covariance[i] = 0.0
            msg.linear_acceleration_covariance[i] = 0.0
        return msg

    # ------------------------------------------------------------------ 诊断

    def stats(self, mono_now_s: float | None = None) -> dict:
        """可观察诊断数据（由节点序列化为 JSON 发布）。"""
        if mono_now_s is None:
            mono_now_s = self.now_mono()
        intervals = list(self._intervals)
        hb_age = (
            None if self._heartbeat_last_mono_s is None else mono_now_s - self._heartbeat_last_mono_s
        )
        out = {
            "connected": hb_age is not None and hb_age <= self.config.heartbeat_timeout_s,
            "heartbeat_age_s": hb_age,
            "heartbeat_autopilot": self._heartbeat_autopilot,
            "imu_rate_hz": (len(intervals) / sum(intervals)) if intervals and sum(intervals) > 0 else None,
            "imu_rate_expected_hz": self.config.expected_rate_hz,
            "interval_mean_s": statistics.fmean(intervals) if intervals else None,
            "interval_median_s": statistics.median(intervals) if intervals else None,
            "interval_max_s": max(intervals) if intervals else None,
            "interval_samples": len(intervals),
            "last_boot_us": self._last_boot_us,
            "last_stamp_ros_s": self._last_stamp_ros_s,
            "sample_age_s": (
                None if self._last_recv_mono_s is None else mono_now_s - self._last_recv_mono_s
            ),
            "sample_to_publish_delay_s": (
                None
                if (self._last_stamp_ros_s is None or self._last_recv_mono_s is None)
                else self._last_recv_mono_s - self._last_stamp_ros_s
            ),
            "camera_imu_offset_s": self.config.camera_imu_offset_s,
            "camera_imu_offset_applied": self.config.apply_camera_imu_offset,
            "time_sync": self.clock.stats(mono_now_s),
            "ros_timebase": (
                None
                if self._timebase is None
                else self._timebase.stability(self.config.ros_offset_stale_s)
            ),
            "time_sync_error_bound_s": self.clock.error_bound_s(),
            # 配对状态：真机排查「响应到底回没回来、有没有回显错」先看这两个键。
            "pending_timesync_ns": self._pending_send_ns,
            "pending_timesync_age_s": (
                None
                if self._pending_send_mono_s is None
                else mono_now_s - self._pending_send_mono_s
            ),
        }
        out.update(self._counters)
        return out

    def note_published(self) -> None:
        """由节点在真正发出话题消息后调用（核心本身不接触 ROS 发布器）。"""
        self._counters["published"] += 1

    # 便于测试/诊断读取的只读视图
    @property
    def published_count(self) -> int:
        return self._counters["published"]

    @property
    def counters(self) -> dict:
        return dict(self._counters)

    @property
    def pending_timesync(self) -> tuple[float, int] | None:
        """当前待配对的本地 TIMESYNC 请求 `(发送单调时刻, 发出时的 ts1)`。

        None 表示没有在等的请求（从未发出，或已被超时丢弃）。诊断与单测都读它，
        避免外部去碰私有字段。
        """
        if self._pending_send_mono_s is None or self._pending_send_ns is None:
            return None
        return (self._pending_send_mono_s, self._pending_send_ns)
