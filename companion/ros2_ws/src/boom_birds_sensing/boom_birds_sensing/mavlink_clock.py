"""MAVLink TIMESYNC → PX4 启动时钟与 Companion 单调时钟的偏移估计。

协议语义（依据本机 PX4 源码核验，只读）
--------------------------------------
`/home/waterc/PX4-Autopilot/src/modules/mavlink/mavlink_timesync.cpp`
- PX4 收到 `TIMESYNC(tc1=0, ts1=X)`（远端发起）时回发
  `TIMESYNC(tc1=now_boot_ns, ts1=X)`，其中 `now = hrt_absolute_time()`。
- `TIMESYNC(ts1=0, tc1=now_boot_ns)` 是 PX4 主动发起的请求（`streams/TIMESYNC.hpp`），
  **不能**当作往返样本使用。
- `/home/waterc/PX4-Autopilot/src/modules/mavlink/streams/HIGHRES_IMU.hpp` 里
  `msg.time_usec = imu.timestamp_sample`，即 **PX4 启动时钟微秒**。
  两者同域，所以只需一个偏移即可把 IMU 采样时刻落到 Companion 单调时钟。

偏移定义（与「相机—IMU 时间偏移」严格区分）
------------------------------------------
    clock_offset_s = t2 - (t1 + t3) / 2 = (2·t2 - t1 - t3) / 2
    t1 = Companion 发出请求的单调时刻（TIMESYNC.ts1，随请求发出）
    t2 = PX4 收到请求时的启动时钟读数（PX4 回包里的 tc1 / 1e9）
    t3 = Companion 收到响应时的单调时刻（与 t1 同一时钟，故 RTT 可测）
    t_companion_mono_s = t_px4_boot_s - clock_offset_s

推导：设单程延迟 d、真实时钟差 O（t_px4 = t_mono + O），则
    t2 = t1 + d + O， t3 = t1 + 2d
    ⇒ t2 - (t1+t3)/2 = t1 + d + O - (t1 + d) = O
所以本偏移就是「飞控启动时钟 − Companion 单调时钟」；PX4 启动时钟起点（0）远早于
Companion 的上电时间，因此真机上该偏移为**负值**。RTT/2 = d 是同步误差量级的上界。

与 PX4 侧的一致性：`lib/timesync/Timesync.cpp` 里
    offset_us = (originate_ns/1000 + now_us - remote_ns/1000*2) / 2
其中 PX4 的 `originate` 是它自己发起的 ts1、`remote` 是 rc1 携带的远端时间，
`sync_stamp(usec) = usec + offset`。代入本场景（`originate`=t3、`remote`=t1、
`now`=t2）得到 PX4 的 `offset = (t3 + t2 - 2·t1)/2 = -clock_offset_s`，
即**符号相反**：PX4 记的是「远端 − 本地」，本模块记的是「本地(飞控) − 远端(Companion)」。
对照飞控日志时不要直接比数值。

它与「相机—IMU 时间偏移」（`offset_s = t_cam_ros - t_imu_ros`）更是两回事，不能互相套用。

`t1` 与 `t3` 用同一个 Companion 单调时钟，因此该偏移**不依赖** t2/t3 之间的时钟同步；
t2 只需与 t3 落在同一个 PX4 启动时钟域。

失效行为
--------
- 未取到样本、样本超时（`sync_timeout_s`）、连续异常后重置滤波器期间：`mapping_valid()` 为 False，
  调用方必须**拒绝发布**带时间戳的数据，不得回退到收包时刻。
- PX4 重启：boot 时间显著倒退 → 立即失效并重建。
- 时间倒退 / 非正时间戳 / 采样时钟异常：拒绝该样本并计数。
- 偏移跳变：连续 `jump_confirm_samples` 次偏离当前估计超过 `max_deviation_s` 后重置，
  期间保持失效，避免把跳变后的错误偏移当成有效映射。
"""

from __future__ import annotations

import statistics
from collections import deque
from dataclasses import dataclass, field

# 拒绝/失效原因码（进入诊断输出，便于真机排查）
REASON_NO_SAMPLE = "no_timesync_sample"
REASON_STALE = "timesync_stale"
REASON_HIGH_RTT = "rtt_too_high"
REASON_BACKWARDS = "companion_time_backwards"
REASON_BOOT_BACKWARDS = "px4_boot_time_backwards"
REASON_NONPOSITIVE = "nonpositive_timestamp"
REASON_DEVIATION = "offset_deviation"
REASON_JUMP_RESET = "offset_jump_reset"
REASON_UNPAIRED = "timesync_unpaired"


@dataclass
class ClockEstimate:
    """当前偏移估计与质量指标。"""

    offset_s: float
    rtt_s: float
    deviation_s: float
    sample_mono_s: float


@dataclass
class ClockMapperConfig:
    max_rtt_s: float = 0.02               # 超过即拒绝该样本（PX4 自身用 10 ms）
    sync_timeout_s: float = 1.0           # 距最后一个可用样本超过该值即失效
    max_deviation_s: float = 0.05         # 偏离当前估计的上限
    jump_confirm_samples: int = 3         # 连续偏离次数达到该值才判定跳变并重置
    restart_deviation_s: float = 1.0      # 超过该偏离量视为飞控重启/时钟重置，立即失效
    converge_samples: int = 5             # 少于该样本数视为尚未收敛（不确定度更大）
    ewma_alpha: float = 0.2               # 偏移平滑增益
    rtt_window: int = 50                  # RTT 统计窗口


@dataclass
class ClockMapper:
    """TIMESYNC 往返样本 → 偏移估计（含异常、超时、重启处理）。"""

    config: ClockMapperConfig = field(default_factory=ClockMapperConfig)

    # 内部状态
    _estimate: ClockEstimate | None = None
    _samples_accepted: int = 0
    _last_boot_ns: int | None = None
    _high_deviation_streak: int = 0
    _last_reset_mono_s: float | None = None
    _rtts: deque = field(default_factory=lambda: deque(maxlen=50))
    _last_reason: str | None = None
    _counters: dict = field(default_factory=dict)
    _baseline_ns: int | None = None
    _baseline_mono_s: float | None = None
    reset_epoch: int = 0

    def __post_init__(self) -> None:
        self._rtts = deque(maxlen=max(int(self.config.rtt_window), 2))
        self._counters = {
            "samples_received": 0,
            "samples_accepted": 0,
            "rejected_high_rtt": 0,
            "rejected_deviation": 0,
            "rejected_backwards": 0,
            "rejected_nonpositive": 0,
            "rejected_echo_mismatch": 0,
            "outbound_requests": 0,
            "filter_resets": 0,
            "immediate_resets": 0,
            "px4_restarts": 0,
            "ignored_px4_requests": 0,
        }

    # ---------------------------------------------------------------- 采样更新

    def on_timesync(
        self,
        tc1: int,
        ts1: int,
        t1_mono_s: float,
        t3_mono_s: float,
        sent_ns_raw: int | None = None,
    ) -> str | None:
        """处理一条来自飞控的 TIMESYNC，返回接受时的 None 或拒绝原因码。

        tc1 == 0：飞控主动发起的请求（PX4 `streams/TIMESYNC.hpp` 恒发 tc1=0）。它仍是
            第一层「基线」样本：能证明链路与飞控启动时钟可用，但没有往返配对，
            因此只用于诊断与初始基线，精度不足以给 IMU 打时间戳。
        tc1 != 0：对我们请求的响应（`ts1` 应等于我们发出的值），用于正式偏移估计。

        注意：tc1 是 PX4 启动时钟纳秒，**不要求为正**：当 Companion 单调时钟的
        起点（例如 1.7e9 s 的 ROS epoch 量级）远大于飞控启动时间时，tc1 完全可能是
        负值。判据是「是否为零」，不是正负。
        """
        self._counters["samples_received"] += 1
        if tc1 == 0:
            # 飞控发起的请求：记录为基线，不产生偏移样本。
            self._counters["ignored_px4_requests"] += 1
            if self._baseline_ns is None:
                self._baseline_ns = ts1
                self._baseline_mono_s = t3_mono_s
            self._last_reason = REASON_NO_SAMPLE
            return REASON_NO_SAMPLE
        if ts1 <= 0:
            self._last_reason = REASON_NONPOSITIVE
            self._counters["rejected_nonpositive"] += 1
            return REASON_NONPOSITIVE
        if sent_ns_raw is not None and ts1 != sent_ns_raw:
            # 回显不匹配：可能是迟到的、属于更早请求的响应。t1 不可信 → 拒绝。
            self._counters["rejected_echo_mismatch"] += 1
            self._last_reason = "echo_mismatch"
            return "echo_mismatch"

        echo_s = ts1 * 1e-9
        if abs(echo_s - t1_mono_s) > 0.5:
            # 回显时间与记录的发送时刻相差过大：请求配对错误。
            self._counters["rejected_echo_mismatch"] += 1
            self._last_reason = "echo_mismatch"
            return "echo_mismatch"

        if t3_mono_s <= t1_mono_s:
            self._counters["rejected_backwards"] += 1
            self._last_reason = REASON_BACKWARDS
            return REASON_BACKWARDS

        boot_s = tc1 * 1e-9
        rtt = t3_mono_s - t1_mono_s
        if rtt > self.config.max_rtt_s:
            self._counters["rejected_high_rtt"] += 1
            self._last_reason = REASON_HIGH_RTT
            return REASON_HIGH_RTT

        offset = boot_s - (t1_mono_s + t3_mono_s) / 2.0
        old = self._estimate.offset_s if self._estimate is not None else None
        deviation = 0.0 if old is None else abs(offset - old)

        # 顺序很重要：**先**判偏移跳变，**再**判 boot 时间倒退。
        # 飞控重启时偏移会先出现巨大偏离：
        # - 偏离 ≥ restart_deviation_s（默认 1 s）：视为重启/时钟重置，立即失效，
        #   不能等确认次数，更不能沿用一个已经过期的偏移继续给数据打时间戳；
        # - 较小的偏离（可能是调度抖动）按 jump_confirm_samples 确认后重置。
        if old is not None and deviation > self.config.max_deviation_s:
            self._high_deviation_streak += 1
            if deviation >= self.config.restart_deviation_s:
                self._counters["immediate_resets"] += 1
                self._reset("offset_jump_immediate", t3_mono_s)
                # 时间线已判定不连续：从本样本重新建立 boot 时间基准，
                # 否则「倒退」判据会拿旧时间线的值去比，语义不清。
                self._last_boot_ns = tc1
            elif self._high_deviation_streak >= max(int(self.config.jump_confirm_samples), 1):
                self._reset(REASON_JUMP_RESET, t3_mono_s)
            self._counters["rejected_deviation"] += 1
            self._last_reason = REASON_DEVIATION
            return REASON_DEVIATION

        # boot 时间显著倒退（允许小的乱序，故要求明显负跳变）→ 飞控重启
        if self._last_boot_ns is not None and tc1 + 1_000_000 < self._last_boot_ns:
            self._counters["px4_restarts"] += 1
            self._reset("px4_restart", t3_mono_s)
            self._last_boot_ns = tc1
            self._last_reason = REASON_BOOT_BACKWARDS
            return REASON_BOOT_BACKWARDS
        self._last_boot_ns = tc1

        self._high_deviation_streak = 0
        smoothed = offset if old is None else old + self.config.ewma_alpha * (offset - old)
        self._estimate = ClockEstimate(
            offset_s=smoothed, rtt_s=rtt, deviation_s=deviation, sample_mono_s=t3_mono_s
        )
        self._samples_accepted += 1
        self._counters["samples_accepted"] += 1
        self._rtts.append(rtt)
        self._last_reason = None
        return None

    def _reset(self, reason: str, now_mono_s: float) -> None:
        self._estimate = None
        self._samples_accepted = 0
        self._last_reset_mono_s = now_mono_s
        self._high_deviation_streak = 0
        self.reset_epoch += 1        # 下游据此清空自己与旧时间线相关的状态
        self._counters["filter_resets"] += 1
        self._counters["last_reset_reason"] = reason

    # ---------------------------------------------------------------- 查询/映射

    @property
    def offset_s(self) -> float | None:
        return None if self._estimate is None else self._estimate.offset_s

    def mapping_valid(self, now_mono_s: float) -> tuple[bool, str | None]:
        """当前是否可以用本映射给数据打时间戳。

        只有「往返配对」样本才算锁定：
        - 完全没有样本 → no_timesync_sample；
        - 只有飞控主动请求（无配对）→ timesync_unpaired（不得用于给 IMU 打时间戳）；
        - 距最近一个配对样本超时 → timesync_stale。
        """
        if self._estimate is None:
            if self._last_reset_mono_s is not None:
                return False, self._counters.get("last_reset_reason", REASON_NO_SAMPLE)
            if self._baseline_ns is not None:
                return False, REASON_UNPAIRED
            return False, REASON_NO_SAMPLE
        age = now_mono_s - self._estimate.sample_mono_s
        if age > self.config.sync_timeout_s:
            return False, REASON_STALE
        return True, None

    def map_boot_to_mono(self, boot_us: int, now_mono_s: float) -> tuple[float | None, str | None]:
        """PX4 启动时钟微秒 → Companion 单调时钟秒。

        返回 (t_mono_s, None) 或 (None, 原因码)。绝不返回「近似有效」的值。
        """
        ok, reason = self.mapping_valid(now_mono_s)
        if not ok:
            self._last_reason = reason
            return None, reason
        return boot_us * 1e-6 - self._estimate.offset_s, None

    def error_bound_s(self) -> float:
        """当前映射的误差上界估计（秒）：RTT/2 + 尚未收敛的额外不确定度。

        TIMESYNC 只能给出 RTT/2 量级的上界；这里不把它当作实测同步精度。
        """
        if self._estimate is None:
            return float("inf")
        bound = 0.5 * self._estimate.rtt_s
        if self._samples_accepted < max(int(self.config.converge_samples), 1):
            bound += self.config.max_deviation_s
        return bound

    def stats(self, now_mono_s: float) -> dict:
        """可观察诊断数据（序列化为 JSON 后原样发布）。"""
        ok, reason = self.mapping_valid(now_mono_s)
        out = {
            "locked": ok,
            "state": "LOCKED" if ok else ("INIT" if self._estimate is None else "STALE"),
            "invalid_reason": reason,
            "offset_s": self.offset_s,
            "error_bound_s": None if self._estimate is None else self.error_bound_s(),
            "rtt_last_s": None if self._estimate is None else self._estimate.rtt_s,
            "rtt_median_s": statistics.median(self._rtts) if self._rtts else None,
            "rtt_max_s": max(self._rtts) if self._rtts else None,
            "sample_age_s": (
                None if self._estimate is None else now_mono_s - self._estimate.sample_mono_s
            ),
            "samples_accepted": self._samples_accepted,
            "converged": self._samples_accepted >= max(int(self.config.converge_samples), 1),
            "last_reason": self._last_reason,
            "reset_epoch": self.reset_epoch,
            "baseline": (
                None
                if self._baseline_ns is None
                else {
                    "px4_boot_s": self._baseline_ns * 1e-9,
                    "companion_mono_s": self._baseline_mono_s,
                    "offset_s": self._baseline_ns * 1e-9 - (self._baseline_mono_s or 0.0),
                    "note": "PX4 主动请求给出的粗基线；无往返配对，不用于给 IMU 打时间戳",
                }
            ),
        }
        out.update(self._counters)
        return out

    def note_outbound(self) -> None:
        self._counters["outbound_requests"] += 1
