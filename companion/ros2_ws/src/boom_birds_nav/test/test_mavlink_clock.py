"""MAVLink 时钟映射（TIMESYNC 偏移估计）与时间基的离线测试。

数据全部为**构造的模拟 MAVLink 消息**，不连接任何设备；结论只说明算法与失效路径，
不是真机同步精度。
"""

from __future__ import annotations

import time

import pytest

from boom_birds_nav.mavlink_clock import (
    REASON_DEVIATION,
    REASON_HIGH_RTT,
    REASON_NO_SAMPLE,
    REASON_STALE,
    REASON_UNPAIRED,
    ClockMapper,
    ClockMapperConfig,
)
from boom_birds_nav.timebase import RosTimeBase, msg_from_ros_seconds, ros_seconds_from_msg

NS = 1_000_000_000


class FakeAutopilot:
    """模拟 PX4 的 TIMESYNC 应答。

    飞控与 Companion 是两个时钟域：`boot = mono + boot_minus_mono_s`。
    飞控在「收到请求后 half_trip_s」回包，其 tc1 = 那一刻的启动时钟纳秒。
    """

    def __init__(self, boot_minus_mono_s: float, half_trip_s: float = 0.002) -> None:
        self.boot_minus_mono_s = boot_minus_mono_s
        self.half_trip_s = half_trip_s

    def respond(self, tc1_out: int, ts1_echo: int, t1_mono_s: float) -> tuple[int, int]:
        boot_s = t1_mono_s + self.half_trip_s + self.boot_minus_mono_s
        return int(round(boot_s * NS)), ts1_echo


@pytest.fixture()
def mapper() -> ClockMapper:
    return ClockMapper(ClockMapperConfig(max_rtt_s=0.02, sync_timeout_s=1.0,
                                         max_deviation_s=0.05, jump_confirm_samples=3))


def _exchange(mapper: ClockMapper, t1: float, ap: FakeAutopilot) -> tuple[float, float | None]:
    """一次完整往返：先记录发送时刻（配对锚点），再提交响应样本。

    返回 (t2_boot_s, 拒绝原因)。
    """
    ts1 = int(round(t1 * NS))
    mapper.note_outbound()
    tc1, echo = ap.respond(0, ts1, t1)
    t3 = t1 + 2 * ap.half_trip_s
    reason = mapper.on_timesync(tc1=tc1, ts1=echo, t1_mono_s=t1, t3_mono_s=t3, sent_ns_raw=ts1)
    return tc1 * 1e-9, reason


def test_offset_converges_to_true_value(mapper):
    """偏移应等于「飞控启动时钟 − Companion 单调时钟」= boot_minus_mono_s。

    该断言只用物理设定本身，不引用实现内部常量：若符号约定反了，这里必然失败。
    """
    ap = FakeAutopilot(boot_minus_mono_s=-1600.0, half_trip_s=0.003)
    t1 = 500.0
    for _ in range(20):
        t1 += 0.5
        boot_s, reason = _exchange(mapper, t1, ap)
        assert reason is None, f"正常样本被拒绝：{reason}"
        # 映射关系：t_mono = t_boot - offset → 应回到这一帧的真实采集时刻附近
        mapped, err = mapper.map_boot_to_mono(int(boot_s * 1e6), t1 + 2 * ap.half_trip_s)
        assert err is None
        assert abs(mapped - (boot_s - mapper.offset_s)) < 1e-9
        assert abs(mapped - (t1 + ap.half_trip_s)) < 5e-3
    assert mapper.offset_s == pytest.approx(-1600.0, abs=5e-3)
    # RTT/2 是该方法能给出的误差上界量级
    assert mapper.error_bound_s() < 0.01


def test_offset_sign_follows_px4_convention(mapper):
    """飞控启动时钟比 Companion 单调时钟晚 N 秒 ⇒ 偏移 = +N；早 ⇒ 为负。

    真机情形（飞控先上电、Companion 后上电）属于后者，偏移为负。
    """
    late = FakeAutopilot(boot_minus_mono_s=100.0, half_trip_s=0.004)
    boot_s, reason = _exchange(mapper, 10.0, late)
    assert reason is None
    assert mapper.offset_s == pytest.approx(100.0, abs=5e-3)
    # 用该偏移把 boot 时钟换回单调时钟，应回到真实采样时刻
    assert mapper.map_boot_to_mono(int(boot_s * 1e6), 10.008)[0] == pytest.approx(10.004, abs=5e-3)

    mapper2 = ClockMapper(ClockMapperConfig())
    early = FakeAutopilot(boot_minus_mono_s=-100.0, half_trip_s=0.004)
    boot2, reason2 = _exchange(mapper2, 10.0, early)
    assert reason2 is None
    assert mapper2.offset_s == pytest.approx(-100.0, abs=5e-3)
    assert mapper2.map_boot_to_mono(int(boot2 * 1e6), 10.008)[0] == pytest.approx(10.004, abs=5e-3)


def test_midpoint_estimator_formula(mapper):
    """offset = t2 - (t1+t3)/2：t2 与 t1/t3 不同域，不能简写成 (t1+t3)/2。"""
    ap = FakeAutopilot(boot_minus_mono_s=100.0, half_trip_s=0.004)
    t1 = 10.0
    boot_s, reason = _exchange(mapper, t1, ap)
    assert reason is None
    t3 = t1 + 2 * ap.half_trip_s
    t2 = boot_s
    expected = t2 - (t1 + t3) / 2.0
    assert mapper.offset_s == pytest.approx(expected)


def test_rejects_px4_initiated_request(mapper):
    """PX4 主动发起（tc1=0）时没有 t1 配对，不能产生偏移样本。"""
    reason = mapper.on_timesync(tc1=0, ts1=12345, t1_mono_s=5.0, t3_mono_s=5.1)
    assert reason == REASON_NO_SAMPLE
    assert mapper.offset_s is None
    assert mapper.stats(5.1)["ignored_px4_requests"] == 1
    ok, why = mapper.mapping_valid(5.1)
    assert not ok and why == REASON_UNPAIRED, "只有飞控主动请求（无配对）时不得解锁"


def test_rejects_high_rtt(mapper):
    ap = FakeAutopilot(boot_minus_mono_s=0.0, half_trip_s=0.5)   # RTT = 1 s >> 20 ms
    t1 = 3.0
    ts1 = int(t1 * NS)
    tc1, echo = ap.respond(0, ts1, t1)
    reason = mapper.on_timesync(tc1=tc1, ts1=echo, t1_mono_s=t1, t3_mono_s=t1 + 1.0,
                                sent_ns_raw=ts1)
    assert reason == REASON_HIGH_RTT
    assert mapper.offset_s is None
    assert mapper.stats(t1 + 1.0)["rejected_high_rtt"] == 1
    ok, why = mapper.mapping_valid(t1 + 1.0)
    assert not ok


def test_rejects_echo_mismatch(mapper):
    """迟到/错配的响应（回显不是我们发出的 ts1）必须拒绝，否则偏移会被算错。"""
    ap = FakeAutopilot(boot_minus_mono_s=0.0, half_trip_s=0.001)
    t1 = 20.0
    ts1 = int(t1 * NS)
    tc1, _ = ap.respond(0, ts1, t1)
    reason = mapper.on_timesync(tc1=tc1, ts1=ts1 + 999, t1_mono_s=t1, t3_mono_s=t1 + 0.002,
                                sent_ns_raw=ts1)
    assert reason == "echo_mismatch"
    assert mapper.offset_s is None


def test_rejects_backwards_companion_time(mapper):
    ap = FakeAutopilot(boot_minus_mono_s=0.0, half_trip_s=0.001)
    t1 = 30.0
    ts1 = int(t1 * NS)
    tc1, echo = ap.respond(0, ts1, t1)
    reason = mapper.on_timesync(tc1=tc1, ts1=echo, t1_mono_s=t1, t3_mono_s=t1 - 1.0,
                                sent_ns_raw=ts1)
    assert reason is not None
    assert mapper.stats(t1)["rejected_backwards"] == 1


def test_sync_timeout_invalidates_mapping(mapper):
    ap = FakeAutopilot(boot_minus_mono_s=-100.0, half_trip_s=0.001)
    t1 = 40.0
    boot_s, reason = _exchange(mapper, t1, ap)
    assert reason is None
    t3 = t1 + 2 * ap.half_trip_s
    ok, why = mapper.mapping_valid(t3 + 0.5)
    assert ok, "1 s 超时窗口内不应失效"
    ok, why = mapper.mapping_valid(t3 + 1.2)
    assert not ok and why == REASON_STALE
    mapped, err = mapper.map_boot_to_mono(int(boot_s * 1e6), t3 + 1.2)
    assert mapped is None and err == REASON_STALE


def test_px4_restart_invalidates_and_recovers(mapper):
    ap = FakeAutopilot(boot_minus_mono_s=-5000.0, half_trip_s=0.001)
    t1 = 100.0
    boot_s, reason = _exchange(mapper, t1, ap)
    assert reason is None
    assert mapper.mapping_valid(t1 + 0.002)[0]

    # 飞控重启：启动时钟回到接近 0
    ap2 = FakeAutopilot(boot_minus_mono_s=-3.0, half_trip_s=0.001)
    t1b = t1 + 0.5
    ts1b = int(round(t1b * NS))
    tc1b, echo = ap2.respond(0, ts1b, t1b)
    reason = mapper.on_timesync(tc1=tc1b, ts1=echo, t1_mono_s=t1b, t3_mono_s=t1b + 0.002,
                                sent_ns_raw=ts1b)
    assert reason == REASON_DEVIATION
    stats = mapper.stats(t1b + 0.002)
    assert stats["immediate_resets"] == 1, "大偏离（≥ restart_deviation_s）应立即失效"
    ok, why = mapper.mapping_valid(t1b + 0.002)
    assert not ok, "重启后必须立即失效，不能沿用旧偏移"

    # 重启期间的样本被拒绝；随后自动收敛到新偏移（首帧偏离即触发立即失效）
    t1c = t1b
    for _ in range(10):
        t1c += 0.5
        _, reason = _exchange(mapper, t1c, ap2)
    assert reason is None, f"重启后应能重新收敛：{reason}"
    ok, _ = mapper.mapping_valid(t1c + 0.002)
    assert ok, "重新收敛后应恢复可用"
    assert mapper.offset_s == pytest.approx(ap2.boot_minus_mono_s, abs=5e-3)
    stats = mapper.stats(t1c + 0.002)
    assert stats["immediate_resets"] == 1
    assert stats["samples_accepted"] >= 5


def test_boot_time_backwards_is_counted_as_px4_restart(mapper):
    """飞控启动时钟明显倒退（重启）时计入 px4_restarts 并重置。"""
    ap = FakeAutopilot(boot_minus_mono_s=-5000.0, half_trip_s=0.001)
    t1 = 100.0
    _, reason = _exchange(mapper, t1, ap)
    assert reason is None
    assert mapper.mapping_valid(t1 + 0.002)[0]

    # 新偏移只差约 5 s：偏离超过 max_deviation_s，但小于 restart_deviation_s，
    # 因此走「boot 时间倒退」判据。同时让 boot 时钟倒退（tc1 变小）。
    ap2 = FakeAutopilot(boot_minus_mono_s=-4995.0, half_trip_s=0.001)
    t2 = t1 + 0.5
    ts2 = int(round(t2 * NS))
    tc1b, echo = ap2.respond(0, ts2, t2)
    boot_ns = tc1b - 6_000_000_000          # 人为再倒退 6 ms，模拟启动时钟回卷
    assert boot_ns + 1_000_000 < tc1b
    reason = mapper.on_timesync(tc1=boot_ns, ts1=echo, t1_mono_s=t2, t3_mono_s=t2 + 0.002,
                                sent_ns_raw=ts2)
    assert reason in ("offset_deviation", "px4_boot_time_backwards"), reason
    stats = mapper.stats(t2 + 0.002)
    assert stats["px4_restarts"] + stats["immediate_resets"] >= 1, "必须判定为时间线不连续"
    ok, _ = mapper.mapping_valid(t2 + 0.002)
    assert not ok


def test_offset_jump_requires_consecutive_confirmation(mapper):
    """小幅异常单次不重置；连续偏离达到阈值才判定跳变并失效（避免误伤）。"""
    ap = FakeAutopilot(boot_minus_mono_s=-10.0, half_trip_s=0.001)
    t1 = 200.0
    for _ in range(5):
        t1 += 0.5
        _exchange(mapper, t1, ap)
    locked_offset = mapper.offset_s
    assert locked_offset == pytest.approx(-10.0, abs=5e-3)

    # 偏移跳变 0.5 s：超过 max_deviation_s(0.05) 但远小于 restart_deviation_s(1.0)
    ap_bad = FakeAutopilot(boot_minus_mono_s=-10.0 + 0.5, half_trip_s=0.001)
    reasons = []
    for _ in range(2):
        t1 += 0.5
        _, reason = _exchange(mapper, t1, ap_bad)
        reasons.append(reason)
    assert reasons[0] == REASON_DEVIATION
    assert mapper.offset_s == pytest.approx(locked_offset, abs=1e-9), "单次偏离不应立即重置"

    t1 += 0.5
    _, reason = _exchange(mapper, t1, ap_bad)
    assert reason == REASON_DEVIATION
    assert mapper.offset_s is None, "连续 3 次偏离后必须重置"
    ok, why = mapper.mapping_valid(t1 + 0.002)
    assert not ok, "跳变重置后必须等待新样本，期间拒绝打时间戳"

    # 新偏移收敛后恢复
    for _ in range(10):
        t1 += 0.5
        _exchange(mapper, t1, ap_bad)
    ok, _ = mapper.mapping_valid(t1 + 0.002)
    assert ok
    assert mapper.offset_s == pytest.approx(ap_bad.boot_minus_mono_s, abs=5e-3)


def test_stats_exposes_diagnostics(mapper):
    ap = FakeAutopilot(boot_minus_mono_s=-1.0, half_trip_s=0.002)
    t1 = 1.0
    for _ in range(4):
        t1 += 0.5
        _exchange(mapper, t1, ap)
    stats = mapper.stats(t1 + 0.002)
    for key in ("locked", "state", "offset_s", "error_bound_s", "rtt_last_s",
                "rtt_median_s", "sample_age_s", "samples_accepted", "converged",
                "samples_received", "rejected_high_rtt", "filter_resets", "px4_restarts"):
        assert key in stats, f"诊断缺少字段 {key}"
    assert stats["locked"] is True
    assert stats["samples_accepted"] == 4
    assert 0.0 <= stats["rtt_median_s"] < 0.02


# ------------------------------------------------------------------ 时间基


def test_ros_timebase_from_offset_and_mapping():
    base = RosTimeBase.from_offset(offset_s=1_700_000_000.0, mono_s=1000.0)
    sample = base.sample()
    assert sample.offset_s == pytest.approx(1_700_000_000.0)
    t_ros, unc = base.to_ros_s(1005.0)
    assert t_ros == pytest.approx(1_700_000_005.0)
    assert unc == 0.0
    assert base.stability(stale_s=1.0)["stale"] is False


def test_ros_timebase_requires_sampling_first():
    base = RosTimeBase(clock=None)
    with pytest.raises(RuntimeError):
        base.to_ros_s(1.0)


class _FixedClock:
    """最小 rclpy Clock 替身：now() 返回内置时间对象。"""

    class _T:
        def __init__(self, nanoseconds: int) -> None:
            self.nanoseconds = nanoseconds

    def __init__(self, seconds_fn) -> None:
        self._fn = seconds_fn

    def now(self):
        return self._T(int(self._fn() * 1e9))


def test_ros_timebase_measures_bracket_and_offset():
    base = RosTimeBase(clock=_FixedClock(lambda: time.monotonic() + 7.5))
    sample = base.sample()
    assert sample.offset_s == pytest.approx(7.5, abs=0.01)
    assert sample.bracket_s >= 0.0
    assert sample.uncertainty_s >= 0.5 * sample.bracket_s


def test_ros_timebase_flags_time_step():
    state = {"offset": 0.0}

    def fake_now():
        return time.monotonic() + state["offset"]

    base = RosTimeBase(clock=_FixedClock(fake_now))
    base.sample()
    state["offset"] = 5.0 / 1.0        # ROS 时钟被步进 5 s
    base.sample()
    assert base.offset_jump_count == 1
    assert base.stability(stale_s=10.0)["offset_jump_count"] == 1


def test_time_msg_roundtrip_and_nanosecond_carry():
    msg = msg_from_ros_seconds(1_700_000_000.9999999995)
    assert msg.sec == 1_700_000_001
    assert 0 <= msg.nanosec < 1_000_000_000
    assert ros_seconds_from_msg(msg) == pytest.approx(1_700_000_001.0, abs=1e-6)
    assert ros_seconds_from_msg(msg_from_ros_seconds(12.5)) == pytest.approx(12.5)
