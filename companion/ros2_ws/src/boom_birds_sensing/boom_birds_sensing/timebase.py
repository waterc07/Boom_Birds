"""时间基：单调时钟 ↔ ROS 时间域的显式映射。

为什么需要这一层
----------------
飞控 `HIGHRES_IMU.time_usec` 与 MAVLink `TIMESYNC` 都在 **PX4 启动时钟**（boot time）域；
V4L2 帧时间戳在 **Companion 单调时钟**（`CLOCK_MONOTONIC`）域；
ROS 消息 `header.stamp` 在 **ROS 时钟**域（默认系统时间，`use_sim_time=true` 时为仿真时间）。

三者不能互相冒充。本模块只负责最后一跳：把「Companion 单调时钟秒」映射到「ROS 时间秒」，
并给出可观测的残差上界。PX4 启动时钟 → Companion 单调时钟由 `mavlink_clock.ClockMapper`
负责（MAVLink TIMESYNC）。两个偏移的符号定义不同，不要混用：

    t_mono = t_px4_boot - clock_offset_s        # MAVLink 时钟偏移（mavlink_clock）
    t_ros  = t_mono + ros_minus_mono_s          # ROS 时钟与单调时钟之差（本模块）

`camera_imu_offset_s` 是**物理量**而不是时钟量：同一物理时刻在两个传感器上留下的
时间标记之差，符号定义为 `offset = t_cam_ros - t_imu_ros`（见契约 timing 节）。

限制
----
本模块不做 NTP/PTP 同步，也不修正 ROS 时钟自身被外部步进；它只做「同一时刻采样两个时钟」
并报告二者差值的漂移。若 ROS 时钟被步进（NTP step / 手动改时间），`offset_s` 会跳变，
调用方必须用 `stability()` 判定映射是否仍可信。
"""

from __future__ import annotations

import time
from dataclasses import dataclass

# 采样两个时钟时允许的最大区间宽度：超过则本次采样不够紧密。
DEFAULT_MAX_BRACKET_S = 2e-3
# 判定 ROS 时钟被步进的阈值（秒）：远大于正常 ppm 级漂移。
DEFAULT_JUMP_THRESHOLD_S = 0.1


@dataclass(frozen=True)
class TimeBaseSample:
    """一次「同一时刻采样两个时钟」的结果。"""

    mono_s: float            # 归算到区间中点的 time.monotonic()
    ros_s: float             # rclpy 时钟（秒）
    offset_s: float          # ros_s - mono_s
    uncertainty_s: float     # 采样区间半宽与相邻样本漂移的保守上界
    bracket_s: float         # 两次单调时钟读数之差


class RosTimeBase:
    """单调时钟 → ROS 时钟的映射，可重复采样并检查稳定性。

    参数
    ----
    clock: 具有 `now()` 的 rclpy Clock。
    """

    def __init__(self, clock, max_bracket_s: float = DEFAULT_MAX_BRACKET_S) -> None:
        self._clock = clock
        self.max_bracket_s = float(max_bracket_s)
        self._last: TimeBaseSample | None = None
        self._last_change_mono: float | None = None
        self.offset_jump_count = 0
        self._max_observed_bracket = 0.0
        self._override: TimeBaseSample | None = None

    @classmethod
    def from_offset(
        cls, offset_s: float, mono_s: float, uncertainty_s: float = 0.0
    ) -> "RosTimeBase":
        """构造一个固定偏移的时间基（离线回放与单测用；不读取真实时钟）。"""
        base = cls(clock=None)
        base._override = TimeBaseSample(
            mono_s=float(mono_s),
            ros_s=float(mono_s) + float(offset_s),
            offset_s=float(offset_s),
            uncertainty_s=float(uncertainty_s),
            bracket_s=0.0,
        )
        return base

    def sample(self) -> TimeBaseSample:
        """紧邻采样 ROS 时钟与单调时钟，返回映射样本。

        顺序：单调时钟 t0 → ROS 时钟 → 单调时钟 t1。ROS 读数被认为发生在 [t0, t1] 内，
        区间中点作为归算时刻，区间半宽作为该样本自身的时延不确定度。
        """
        if self._override is not None:
            self._last = self._override
            return self._override
        t0 = time.monotonic()
        ros_s = self._clock.now().nanoseconds * 1e-9
        t1 = time.monotonic()
        bracket = t1 - t0
        mid = 0.5 * (t0 + t1)
        offset = ros_s - mid
        uncertainty = 0.5 * bracket
        if self._last is not None:
            drift = abs(offset - self._last.offset_s)
            uncertainty = max(uncertainty, 0.5 * drift)
            if drift > DEFAULT_JUMP_THRESHOLD_S:
                self.offset_jump_count += 1
                self._last_change_mono = t1
        self._max_observed_bracket = max(self._max_observed_bracket, bracket)
        sample = TimeBaseSample(
            mono_s=mid, ros_s=ros_s, offset_s=offset, uncertainty_s=uncertainty, bracket_s=bracket
        )
        self._last = sample
        return sample

    def to_ros_s(self, mono_s: float) -> tuple[float, float]:
        """把单调时钟秒映射到 ROS 秒，返回 (t_ros_s, 不确定度上界)。

        使用最近一次采样。调用方应让采样时刻尽量接近使用时刻（本节点每周期重采一次）。
        """
        if self._last is None:
            raise RuntimeError("RosTimeBase 尚未采样：请先调用 sample()")
        return mono_s + self._last.offset_s, self._last.uncertainty_s

    def stability(self, stale_s: float) -> dict:
        """返回映射稳定性诊断（可直接进入统计 JSON）。"""
        out = {
            "sampled": self._last is not None,
            "offset_s": None,
            "uncertainty_s": None,
            "bracket_s": None,
            "max_bracket_s": self._max_observed_bracket,
            "offset_jump_count": self.offset_jump_count,
            "age_s": None,
            "stale": True,
        }
        if self._last is None:
            return out
        # 固定偏移模式（离线回放/单测）没有「真实时钟」可比；使用样本自身时刻，
        # 使稳定性判断保持确定性。
        now_mono = self._last.mono_s if self._override is not None else time.monotonic()
        age = now_mono - self._last.mono_s
        out.update(
            offset_s=self._last.offset_s,
            uncertainty_s=self._last.uncertainty_s,
            bracket_s=self._last.bracket_s,
            age_s=age,
            stale=age > stale_s,
        )
        return out


def ros_seconds_from_msg(stamp) -> float:
    """builtin_interfaces/Time → 秒。"""
    return stamp.sec + stamp.nanosec * 1e-9


def msg_from_ros_seconds(seconds: float):
    """秒 → rosidl 时间消息（保留纳秒；负值不修正，调用方应先行拒绝）。

    先尝试 `builtin_interfaces/Time`；不可用时回退 `std_msgs/Header` 的 stamp 类型，
    它是同一类型（`std_msgs/Header.stamp` 就是 `builtin_interfaces/Time`）。
    """
    msg_type = None
    try:
        from builtin_interfaces.msg import Time as msg_type  # type: ignore[no-redef]
    except ImportError:  # pragma: no cover - 仅在缺少 builtin_interfaces 时发生
        from std_msgs.msg import Header  # type: ignore[no-redef]

        msg_type = type(Header().stamp)

    sec = int(seconds // 1)
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:      # 取整溢出：进位到秒
        sec += 1
        nanosec -= 1_000_000_000
    msg = msg_type()
    msg.sec = sec
    msg.nanosec = nanosec
    return msg
