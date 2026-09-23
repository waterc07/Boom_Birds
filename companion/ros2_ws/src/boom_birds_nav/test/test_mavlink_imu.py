"""MAVLink IMU 接收核心的脱机测试：校验、时间戳映射、单位/坐标约定、诊断。

所有 MAVLink 消息都是构造的模拟对象（`FakeMav`），**不是实测数据**。

测试遵循与真实接收循环相同的顺序：每处理一条消息前先刷新 ROS 时间基（`sample_once`），
这一点很关键——核心就是靠最近一次时间基采样把单调时钟映射到 ROS 时间域。
"""

from __future__ import annotations

import time

import pytest

from boom_birds_nav.mavlink_clock import ClockMapperConfig
from boom_birds_nav.mavlink_imu_core import (
    ACCEL_BITS,
    GYRO_BITS,
    MAG_BITS,
    REJECT_DUPLICATE,
    REJECT_EPOCH_TIME,
    REJECT_FIELDS,
    REJECT_ID,
    REJECT_NONFINITE,
    REJECT_ROSBACKWARDS,
    REJECT_TIMESYNC_ECHO,
    REJECT_TIMESYNC_SOURCE,
    REJECT_TIMESYNC_TIMEOUT,
    REJECT_TIMESYNC_UNPAIRED,
    MavlinkImuConfig,
    MavlinkImuReceiver,
)
from boom_birds_nav.timebase import RosTimeBase

NS = 1_000_000_000

# 模拟时钟模型（与真机同构的两个域）：
#   飞控先启动（boot=0），Companion 在 1234.5 s 后才上电：
#       boot_us(mono) = (mono - BOOT_REF_S) * 1e6
#   ⇒ 映射偏移（boot − mono）= -1234.5 s（负值，符合「飞控先上电」）。
# 同步（TIMESYNC）与 IMU 采样必须用同一个模型，否则测的就不是同一件事。
BOOT_REF_S = 1234.5
MONO_T0 = BOOT_REF_S + 1.0  # 虚拟单调时钟：飞控启动后 1235.5 s（boot 时间有效且 > 0）
ROS_OFFSET_S = 1_700_000_000.0
# 真实时钟场景（时间基用一个很小的固定偏移，使采样时刻本身接近当前单调时钟）
REAL_ROS_OFFSET_S = 0.5


class FakeMav:
    """最小 MAVLink 消息替身（pymavlink 消息对象只有属性 + msgid）。"""

    def __init__(self, msgid: int, **fields) -> None:
        self.msgid = msgid
        self.sysid = fields.pop("sysid", 1)
        self.compid = fields.pop("compid", 1)
        for key, value in fields.items():
            setattr(self, key, value)

    def get_type(self) -> str:
        return {0: "HEARTBEAT", 105: "HIGHRES_IMU", 111: "TIMESYNC"}.get(self.msgid, "UNKNOWN")


def boot_us_at(mono_s: float) -> int:
    """给定 Companion 单调时刻，返回对应的 PX4 启动时钟微秒。"""
    return int(round((mono_s - BOOT_REF_S) * 1e6))


def mono_from_boot_us(boot_us: int) -> float:
    """PX4 启动时钟微秒 → Companion 单调时刻（模拟时钟模型的逆）。"""
    return boot_us * 1e-6 + BOOT_REF_S


def highres(boot_us: int, *, fields_updated: int = ACCEL_BITS | GYRO_BITS,
            accel=(0.1, -0.2, 9.81), gyro=(0.01, 0.02, -0.03), device_id=None,
            sysid: int = 1, compid: int = 1) -> FakeMav:
    fields = dict(
        time_usec=boot_us,
        xacc=accel[0], yacc=accel[1], zacc=accel[2],
        xgyro=gyro[0], ygyro=gyro[1], zgyro=gyro[2],
        xmag=0.2, ymag=-0.1, zmag=0.4,
        abs_pressure=1013.25, diff_pressure=0.0, pressure_alt=0.0, temperature=25.0,
        fields_updated=fields_updated,
    )
    if device_id is not None:
        fields["id"] = device_id
    return FakeMav(105, sysid=sysid, compid=compid, **fields)


def heartbeat(autopilot: int = 12, sysid: int = 1, compid: int = 1) -> FakeMav:
    return FakeMav(0, sysid=sysid, compid=compid, autopilot=autopilot, type=2)


def timesync_response(mono_s: float, half_trip_s: float = 0.002) -> FakeMav:
    """构造一条 TIMESYNC 响应（tc1 = 飞控启动时钟纳秒，ts1 = 我们的发送时刻纳秒）。

    飞控在「收到请求后 half_trip_s」应答 → 它那一刻的启动时钟取
    mono_from_boot_us(boot_us_at(mono_s + half_trip_s))。
    """
    boot_us = boot_us_at(mono_s + half_trip_s)
    tc1 = int(round((boot_us * 1e-6) * NS))
    return FakeMav(111, tc1=tc1, ts1=int(round(mono_s * NS)))


def make_receiver(**cfg) -> MavlinkImuReceiver:
    """虚拟时间接收器：单调时钟由测试显式给出，时间基为固定偏移。"""
    config = MavlinkImuConfig(expected_rate_hz=50.0, **cfg)
    core = MavlinkImuReceiver(
        config=config,
        clock_config=ClockMapperConfig(max_rtt_s=0.02, sync_timeout_s=1.0,
                                       max_deviation_s=0.05, jump_confirm_samples=3),
    )
    core.set_timebase(RosTimeBase.from_offset(ROS_OFFSET_S, MONO_T0))
    core.set_virtual_mono(MONO_T0)
    core.stamp()
    return core


def make_real_clock_receiver(**cfg) -> MavlinkImuReceiver:
    """真实单调时钟接收器：时间基偏移很小，使「采样时刻」本身就接近当前单调时钟。

    用于验证基于真实时间的判据（时间戳年龄、未来时间戳）。
    """
    config = MavlinkImuConfig(expected_rate_hz=50.0, **cfg)
    core = MavlinkImuReceiver(
        config=config,
        clock_config=ClockMapperConfig(max_rtt_s=0.02, sync_timeout_s=10.0,
                                       max_deviation_s=0.05, jump_confirm_samples=3),
    )
    core.set_timebase(RosTimeBase.from_offset(REAL_ROS_OFFSET_S, time.monotonic()))
    core.stamp()
    return core


def sync_once(core: MavlinkImuReceiver, mono_s: float, half_trip_s: float = 0.002) -> None:
    """模拟一次成功的 TIMESYNC 往返（请求 → 飞控应答 → 节点收到）。"""
    send_ns = int(round(mono_s * NS))
    core.note_timesync_request(mono_s, send_ns)
    core.handle_message(timesync_response(mono_s, half_trip_s), mono_s + 2 * half_trip_s)


def lock_clock(core: MavlinkImuReceiver, mono_s: float = MONO_T0, n: int = 8) -> float:
    """连续同步直到偏移收敛，返回最后一次同步的时刻。"""
    t = mono_s
    for _ in range(n):
        sync_once(core, t)
        t += 0.5
    return t - 0.5


def lock_real_clock(core: MavlinkImuReceiver, n: int = 8) -> float:
    """用真实单调时钟锁定映射，返回最后一次同步的时刻。"""
    t = time.monotonic()
    for _ in range(n):
        sync_once(core, t)
        t += 0.005
    return t - 0.005


def sample_once(core: MavlinkImuReceiver, mono_s: float, msg):
    """按真实接收循环的顺序处理一条消息：先刷新 ROS 时间基，再处理消息。"""
    core.set_virtual_mono(mono_s)
    core.stamp()
    return core.handle_message(msg, mono_s)


# ------------------------------------------------------------------ 无同步时拒绝发布


def test_rejects_publication_without_clock_mapping():
    core = make_receiver()
    out = sample_once(core, MONO_T0, highres(boot_us_at(MONO_T0)))
    assert out == [], "未建立 TIMESYNC 映射时不得发布任何时间戳"
    assert core.counters["published"] == 0
    assert core.counters["rejected_no_clock_mapping"] == 1
    assert core.counters["last_mapping_reason"] == "no_timesync_sample"
    assert core.stats(MONO_T0)["time_sync"]["locked"] is False


def test_px4_initiated_timesync_alone_is_not_enough():
    """只收到飞控主动 TIMESYNC（tc1=0）时没有往返配对，仍不得给 IMU 打时间戳。"""
    core = make_receiver()
    core.set_virtual_mono(MONO_T0)
    core.stamp()
    core.handle_message(FakeMav(111, tc1=0, ts1=int(MONO_T0 * NS)), MONO_T0)
    out = sample_once(core, MONO_T0 + 0.02, highres(boot_us_at(MONO_T0 + 0.02)))
    assert out == []
    assert core.counters["last_mapping_reason"] == "timesync_unpaired"
    baseline = core.stats(MONO_T0)["time_sync"]["baseline"]
    assert baseline is not None and baseline["px4_boot_s"] > 0


def test_rejects_publication_after_sync_timeout():
    core = make_receiver()
    t = lock_clock(core)
    # 锁定期内：采样时刻与收包时刻一致 → 可以发布
    out = sample_once(core, t + 0.002, highres(boot_us_at(t + 0.002)))
    assert out, "锁定期内应能发布"
    # 之后 1.5 s 没有任何新的 TIMESYNC 样本（> sync_timeout_s=1.0）
    late = t + 1.5
    out = sample_once(core, late, highres(boot_us_at(late)))
    assert out == [], "同步样本超时后必须拒绝发布"
    assert core.counters["last_mapping_reason"] == "timesync_stale"


# ------------------------------------------------------------------ 时间戳来源


def test_timestamp_comes_from_time_usec_not_receive_time():
    """IMU 时间戳必须由 time_usec 映射得到，而不是收包时刻。

    人为让收包时刻比采样时刻晚 100 ms：若实现偷用收包时刻，断言会失败。
    """
    core = make_receiver()
    lock_clock(core)
    sample_boot_us = boot_us_at(MONO_T0 + 0.02)
    receive_mono = MONO_T0 + 0.12          # 链路延迟 100 ms
    out = sample_once(core, receive_mono, highres(sample_boot_us))
    assert len(out) == 1
    sample = out[0]
    expected_stamp = sample_boot_us * 1e-6 - core.clock.offset_s + ROS_OFFSET_S
    receive_stamp = receive_mono + ROS_OFFSET_S
    assert sample.stamp_ros_s == pytest.approx(expected_stamp, abs=1e-6)
    assert abs(sample.stamp_ros_s - receive_stamp) > 0.05, "时间戳不得等于收包时刻"
    assert sample.age_s == pytest.approx(0.1, abs=2e-3)


def test_sample_in_future_is_rejected():
    core = make_real_clock_receiver()
    lock_real_clock(core)
    # time_usec 比当前时刻晚 200 ms → 映射出错或时钟不一致
    out = sample_once(core, time.monotonic(), highres(boot_us_at(time.monotonic() + 0.2)))
    assert out == []
    assert core.counters["rejected_future"] == 1


def test_sample_age_too_large_is_rejected():
    core = make_real_clock_receiver()
    lock_real_clock(core)
    # 采样时刻比收包时刻早 5 s（> max_sample_age_s=1.0）
    out = sample_once(core, time.monotonic(), highres(boot_us_at(time.monotonic() - 5.0)))
    assert out == []
    assert core.counters["rejected_stale"] == 1


# ------------------------------------------------------------------ 校验


def test_rejects_missing_accel_or_gyro_bits():
    core = make_receiver()
    lock_clock(core)
    out = sample_once(core, MONO_T0,
                      highres(boot_us_at(MONO_T0), fields_updated=ACCEL_BITS | MAG_BITS))
    assert out == []
    assert core.counters["rejected_fields"] == 1
    assert core.counters["last_mapping_reason"] == REJECT_FIELDS


def test_rejects_nonfinite_values():
    core = make_receiver()
    lock_clock(core)
    out = sample_once(core, MONO_T0,
                      highres(boot_us_at(MONO_T0), accel=(float("nan"), 0.0, 9.81)))
    assert out == []
    assert core.counters["rejected_nonfinite"] == 1
    assert core.counters["last_mapping_reason"] == REJECT_NONFINITE


def test_rejects_unix_epoch_timestamp():
    """HIGHRES_IMU.time_usec 必须是启动时钟；UNIX epoch 量级说明语义不符。"""
    core = make_receiver()
    lock_clock(core)
    out = sample_once(core, MONO_T0, highres(1_700_000_000_000_000))
    assert out == []
    assert core.counters["rejected_epoch_time"] == 1
    assert core.counters["last_mapping_reason"] == REJECT_EPOCH_TIME


def test_rejects_wrong_device_id():
    core = make_receiver(device_id=1)
    lock_clock(core)
    out = sample_once(core, MONO_T0, highres(boot_us_at(MONO_T0), device_id=0))
    assert out == []
    assert core.counters["rejected_id"] == 1
    assert core.counters["last_mapping_reason"] == REJECT_ID


def test_rejects_wrong_system_id():
    core = make_receiver(system_id=1, component_id=1, accept_any_component=False)
    lock_clock(core)
    out = sample_once(core, MONO_T0, highres(boot_us_at(MONO_T0), sysid=2, compid=1))
    assert out == []
    assert core.counters["rejected_source"] == 1


def test_rejects_boot_time_backwards_and_recovers_after_restart():
    """飞控重启：旧的 IMU 采样被拒绝；重新收敛到新时间线后恢复发布。"""
    core = make_receiver()
    lock_clock(core)
    b1 = boot_us_at(MONO_T0)
    assert sample_once(core, MONO_T0, highres(b1))
    assert sample_once(core, MONO_T0 + 0.02, highres(b1 + 20_000))

    # 飞控重启：time_usec 回到很小的值（旧时间线约 1e6 µs）→ 必须拒绝（不能沿用旧偏移）
    restarted = 300_000
    out = sample_once(core, MONO_T0 + 0.04, highres(restarted))
    assert out == []
    assert core.counters["rejected_boot_backwards"] == 1

    # 重启后重新收敛：新启动时钟基准下，boot_us 应映射回当前时刻。
    # 用节点自身提供的时间线重置入口（真实链路由时钟映射器在检测到重启后触发）。
    from boom_birds_nav.mavlink_clock import ClockEstimate
    new_verify_mono = MONO_T0 + 0.7
    core.clock._estimate = ClockEstimate(
        offset_s=(restarted + 700_000) * 1e-6 - new_verify_mono,
        rtt_s=0.004, deviation_s=0.0, sample_mono_s=MONO_T0 + 0.6,
    )
    core.clock._last_boot_ns = None
    core.note_timeline_reset("px4_restart")
    assert sample_once(core, new_verify_mono, highres(restarted + 700_000))
    assert core.counters["timeline_resets"] == 1


def test_rejects_duplicate_timestamp():
    core = make_receiver()
    lock_clock(core)
    boot_us = boot_us_at(MONO_T0)
    assert sample_once(core, MONO_T0, highres(boot_us))
    assert sample_once(core, MONO_T0 + 0.001, highres(boot_us)) == []
    assert core.counters["rejected_duplicate"] == 1
    assert core.counters["last_mapping_reason"] == REJECT_DUPLICATE
    assert core.counters["rejected_ros_backwards"] == 0


def test_backwards_mapping_rejected_when_clock_jumps():
    """TIMESYNC 偏移在两次 IMU 之间变化（未达重置阈值）时映射后时间倒退：必须拒绝。"""
    core = make_receiver()
    lock_clock(core)
    boot_us = boot_us_at(MONO_T0)
    assert sample_once(core, MONO_T0, highres(boot_us))
    # 把偏移**变大** 0.5 s（等价于映射后的 t_mono 变小 0.5 s）：下一帧会落到已发布时刻之前
    from boom_birds_nav.mavlink_clock import ClockEstimate
    est = core.clock._estimate
    core.clock._estimate = ClockEstimate(
        offset_s=est.offset_s + 0.5, rtt_s=est.rtt_s,
        deviation_s=est.deviation_s, sample_mono_s=est.sample_mono_s,
    )
    out = sample_once(core, MONO_T0 + 0.02, highres(boot_us + 20_000))
    assert out == []
    assert core.counters["rejected_ros_backwards"] == 1
    assert core.counters["last_mapping_reason"] == REJECT_ROSBACKWARDS


# ------------------------------------------------------------------ 频率/丢样


def test_measures_rate_and_counts_gaps():
    core = make_receiver()
    lock_clock(core)
    for i in range(41):                     # 41 条 50 Hz 正常样本
        mono = MONO_T0 + i * 0.02
        sample_once(core, mono, highres(boot_us_at(mono)))
    stats = core.stats(MONO_T0 + 0.8)
    assert stats["imu_rate_hz"] == pytest.approx(50.0, rel=1e-3)
    assert stats["interval_median_s"] == pytest.approx(0.02, rel=1e-3)
    assert stats["gaps"] == 0
    assert stats["published"] == 0        # 核心不计数，节点发布后才计数

    mono = MONO_T0 + 1.0                    # 丢 4 帧 → 间隔 0.20 s > 2.5 × 0.02 s
    sample_once(core, mono, highres(boot_us_at(mono)))
    stats = core.stats(mono)
    assert stats["gaps"] == 1
    assert stats["interval_max_s"] == pytest.approx(0.2, rel=1e-3)


def test_heartbeat_tracking():
    core = make_receiver()
    assert core.stats(MONO_T0)["connected"] is False
    sample_once(core, MONO_T0, heartbeat(autopilot=12))
    assert core.stats(MONO_T0 + 0.5)["connected"] is True
    assert core.stats(MONO_T0 + 4.0)["connected"] is False   # heartbeat_timeout_s=3
    assert core.stats(MONO_T0 + 4.0)["heartbeat_autopilot"] == 12


# ------------------------------------------------------------------ ROS 消息约定


def test_ros_message_units_frames_and_orientation():
    core = make_receiver()
    lock_clock(core)
    out = sample_once(core, MONO_T0, highres(
        boot_us_at(MONO_T0), accel=(0.0, 0.0, 9.81), gyro=(0.1, 0.2, -0.3)
    ))
    assert len(out) == 1
    msg = core.to_ros_imu(out[0])
    assert msg.header.frame_id == "imu"                 # 契约坐标系
    # 单位约定：直接透传 SI 值、不做换算；静止水平时 z 为正重力反作用
    assert msg.linear_acceleration.z == pytest.approx(9.81)
    assert msg.angular_velocity.z == pytest.approx(-0.3)
    # orientation 按 ROS 约定标为不可用（不是飞控融合姿态）
    assert msg.orientation_covariance[0] == -1.0
    assert (msg.orientation.x, msg.orientation.y, msg.orientation.z,
            msg.orientation.w) == (0.0, 0.0, 0.0, 1.0)
    assert all(v == 0.0 for v in msg.linear_acceleration_covariance)
    assert all(v == 0.0 for v in msg.angular_velocity_covariance)


def test_camera_imu_offset_is_opt_in_and_signed():
    """offset = t_cam_ros - t_imu_ros；默认不启用，启用后时间戳按符号平移。"""
    core = make_receiver()
    lock_clock(core)
    boot_us = boot_us_at(MONO_T0)
    base = sample_once(core, MONO_T0, highres(boot_us))[0]

    core2 = make_receiver(camera_imu_offset_s=0.0123, apply_camera_imu_offset=True)
    lock_clock(core2)
    shifted = sample_once(core2, MONO_T0, highres(boot_us))[0]
    # boot_us 是整数微秒，量化误差 1 µs；比较时给出 5 µs 容差
    assert shifted.stamp_ros_s - base.stamp_ros_s == pytest.approx(0.0123, abs=5e-6)
    assert shifted.camera_offset_applied_s == pytest.approx(0.0123)
    assert base.camera_offset_applied_s == 0.0


def test_stats_exposes_required_diagnostics():
    core = make_receiver()
    lock_clock(core)
    sample_once(core, MONO_T0, highres(boot_us_at(MONO_T0)))
    stats = core.stats(MONO_T0)
    for key in (
        "connected", "imu_rate_hz", "imu_rate_expected_hz", "interval_mean_s",
        "interval_median_s", "interval_max_s", "gaps", "sample_age_s",
        "sample_to_publish_delay_s", "last_stamp_ros_s", "time_sync",
        "ros_timebase", "time_sync_error_bound_s", "camera_imu_offset_s",
        "camera_imu_offset_applied", "rejected_no_clock_mapping",
        "rejected_boot_backwards", "rejected_ros_backwards", "rejected_fields",
        "rejected_nonfinite", "rejected_epoch_time", "mapping_failures",
        "rejected_duplicate", "rejected_future", "rejected_stale", "timeline_resets",
    ):
        assert key in stats, f"诊断缺少字段 {key}"
    assert stats["time_sync"]["locked"] is True
    assert stats["ros_timebase"]["sampled"] is True
    assert stats["interval_samples"] == 0        # 只有一条样本时还没有间隔


# ------------------------------------------------- TIMESYNC 往返配对（回归用例）
# 背景：PX4 自己以 10 Hz 广播 `TIMESYNC(tc1=0, ts1=启动时钟纳秒)`
# （`src/modules/mavlink/streams/TIMESYNC.hpp`），它插在「我们发出请求」与
# 「飞控响应」之间时，旧实现会把待配对请求清掉，使紧随其后的响应变成孤儿，
# 偏移估计静默丢样本（映射随之间歇性失效）。以下用例锁定修好后的配对规则：
# 飞控主动请求不作废 pending；响应必须逐位回显 pending 的 ts1；pending 会超时；
# 来源必须与配置一致。


def test_px4_timesync_between_request_and_response_does_not_drop_sample():
    """用户报告的场景：PX4 主动 TIMESYNC 插在请求与响应之间。

    旧代码在此必然失败（pending 被清 → 响应被判无配对 → 样本丢失）。
    """
    core = make_receiver()
    lock_clock(core)
    accepted_before = core.clock.stats(MONO_T0)["samples_accepted"]
    paired_before = core.counters["timesync_paired"]     # lock_clock 已经配对过多次

    t = MONO_T0 + 4.0                      # 仍在 sync_timeout_s=1.0 的锁定期内
    send_ns = int(round(t * NS))
    core.note_timesync_request(t, send_ns)

    # 飞控自己的广播：ts1 是它的启动时钟纳秒，与我们发出的 send_ns 毫无关系
    px4_boot_ns = int(round(boot_us_at(t + 0.001) * 1e3))
    core.handle_message(FakeMav(111, tc1=0, ts1=px4_boot_ns), t + 0.001)

    assert core.pending_timesync == (t, send_ns), "PX4 主动请求不得清掉本地待配对请求"
    assert core.counters["px4_timesync_requests"] == 1
    assert core.counters["rejected_unpaired_timesync"] == 0, "主动请求不是「无配对响应」"

    # 飞控对本地请求的响应随后到达：必须仍能与请求配对
    core.handle_message(timesync_response(t, half_trip_s=0.002), t + 0.004)
    stats = core.clock.stats(t + 0.004)
    assert stats["samples_accepted"] == accepted_before + 1, "该样本被静默丢弃了"
    assert stats["locked"] is True and stats["invalid_reason"] is None
    assert stats["ignored_px4_requests"] == 1, "飞控主动请求仍应作为粗基线交给 ClockMapper"
    assert core.counters["timesync_paired"] == paired_before + 1
    assert core.counters["rejected_unpaired_timesync"] == 0
    assert core.pending_timesync is None, "配对成功后请求应被消费"
    assert core.stats(t + 0.004)["pending_timesync_ns"] is None


def test_only_response_for_latest_request_is_used():
    """两个请求、两个响应（旧的先到）：过期的回显必须丢弃，最新的必须能用。"""
    core = make_receiver()
    lock_clock(core)
    accepted_before = core.clock.stats(MONO_T0)["samples_accepted"]
    paired_before = core.counters["timesync_paired"]

    t_a = MONO_T0 + 4.0
    t_b = t_a + 0.02
    ns_a = int(round(t_a * NS))
    ns_b = int(round(t_b * NS))
    core.note_timesync_request(t_a, ns_a)
    core.note_timesync_request(t_b, ns_b)          # 覆盖：只保留最新一次
    assert core.stats(t_b)["timesync_requests_superseded"] == 1

    # 旧请求的响应（回显 ns_a）先到：t1 已不是当前 pending 的发送时刻 → 不得使用
    core.handle_message(timesync_response(t_a, 0.003), t_a + 0.006)
    assert core.counters["rejected_timesync_echo_mismatch"] == 1
    assert core.counters["last_timesync_reason"] == REJECT_TIMESYNC_ECHO
    assert core.clock.stats(t_a + 0.006)["samples_accepted"] == accepted_before
    assert core.pending_timesync == (t_b, ns_b), "旧响应不得消耗最新请求"

    # 最新请求的响应（回显 ns_b）到达：必须被接受
    core.handle_message(timesync_response(t_b, 0.002), t_b + 0.004)
    assert core.clock.stats(t_b + 0.004)["samples_accepted"] == accepted_before + 1
    assert core.counters["timesync_paired"] == paired_before + 1
    assert core.counters["rejected_unpaired_timesync"] == 0


def test_pending_request_expires_and_frees_the_slot():
    """响应丢了以后 pending 必须超时释放，否则之后每条响应都被误判成无配对。"""
    core = make_receiver(pending_timeout_s=0.25)
    lock_clock(core)
    accepted_before = core.clock.stats(MONO_T0)["samples_accepted"]

    t = MONO_T0 + 4.0
    send_ns = int(round(t * NS))
    core.note_timesync_request(t, send_ns)
    assert core.stats(t)["pending_timesync_age_s"] == pytest.approx(0.0)

    late = t + 0.30                        # > pending_timeout_s=0.25
    core.handle_message(timesync_response(t, 0.002), late)
    assert core.counters["rejected_timesync_timeout"] == 1
    assert core.counters["rejected_unpaired_timesync"] == 1, "迟到响应仍是「无配对响应」"
    assert core.counters["last_timesync_reason"] == REJECT_TIMESYNC_TIMEOUT
    assert core.clock.stats(late)["samples_accepted"] == accepted_before, "迟到响应不得成样本"
    assert core.pending_timesync is None, "超时后槽位必须释放"

    # 槽位已释放：新请求可以正常配对
    t2 = late + 0.01
    core.note_timesync_request(t2, int(round(t2 * NS)))
    core.handle_message(timesync_response(t2, 0.002), t2 + 0.004)
    assert core.clock.stats(t2 + 0.004)["samples_accepted"] == accepted_before + 1
    assert core.counters["rejected_timesync_timeout"] == 1, "不应重复计数"


def test_response_without_any_request_is_counted_and_not_used():
    """从未发出请求就收到响应（例如节点重启后飞控仍在回抄旧值）：只计数。"""
    core = make_receiver()
    lock_clock(core)
    accepted_before = core.clock.stats(MONO_T0)["samples_accepted"]

    t = MONO_T0 + 4.0
    core.handle_message(timesync_response(t, 0.002), t + 0.002)
    assert core.counters["rejected_unpaired_timesync"] == 1
    assert core.counters["last_timesync_reason"] == REJECT_TIMESYNC_UNPAIRED
    assert core.clock.stats(t + 0.002)["samples_accepted"] == accepted_before


def test_duplicate_response_is_not_consumed_twice():
    """同一条响应重放：请求已被消费，不得再次产生偏移样本。"""
    core = make_receiver()
    lock_clock(core)
    accepted_before = core.clock.stats(MONO_T0)["samples_accepted"]

    t = MONO_T0 + 4.0
    core.note_timesync_request(t, int(round(t * NS)))
    response = timesync_response(t, 0.002)
    core.handle_message(response, t + 0.004)
    assert core.clock.stats(t + 0.004)["samples_accepted"] == accepted_before + 1

    core.handle_message(response, t + 0.006)
    assert core.counters["rejected_unpaired_timesync"] == 1
    assert core.clock.stats(t + 0.006)["samples_accepted"] == accepted_before + 1


def test_timesync_response_from_wrong_source_is_rejected():
    """validate_timesync_source=True（默认）：只认目标 sysid 的响应。"""
    core = make_receiver(validate_timesync_source=True, system_id=1)
    lock_clock(core)
    accepted_before = core.clock.stats(MONO_T0)["samples_accepted"]

    t = MONO_T0 + 4.0
    send_ns = int(round(t * NS))
    core.note_timesync_request(t, send_ns)

    ghost = timesync_response(t, 0.002)
    ghost.sysid = 2                        # 同一链路/转发里另一套系统
    core.handle_message(ghost, t + 0.004)
    assert core.counters["rejected_timesync_source"] == 1
    assert core.counters["last_timesync_reason"] == REJECT_TIMESYNC_SOURCE
    assert core.clock.stats(t + 0.004)["samples_accepted"] == accepted_before
    assert core.pending_timesync == (t, send_ns), "来源不符不得消耗待配对请求"

    # 正确来源的响应仍能配对（证明只是挡掉了外来源，没把 pending 弄坏）
    core.handle_message(timesync_response(t, 0.002), t + 0.006)
    assert core.clock.stats(t + 0.006)["samples_accepted"] == accepted_before + 1


def test_timesync_source_validation_can_be_disabled():
    """validate_timesync_source=False：来源不符的响应也照常配对（真机应急开关）。"""
    core = make_receiver(validate_timesync_source=False, system_id=1)
    lock_clock(core)
    accepted_before = core.clock.stats(MONO_T0)["samples_accepted"]

    t = MONO_T0 + 4.0
    core.note_timesync_request(t, int(round(t * NS)))
    other = timesync_response(t, 0.002)
    other.sysid = 2
    core.handle_message(other, t + 0.004)
    assert core.counters["rejected_timesync_source"] == 0
    assert core.clock.stats(t + 0.004)["samples_accepted"] == accepted_before + 1


def test_timesync_source_check_mirrors_accept_any_component():
    """accept_any_component=False 时才校验 compid（与 IMU 的 _source_ok 判据一致）。"""
    core = make_receiver(validate_timesync_source=True, accept_any_component=False,
                         system_id=1, component_id=1)
    lock_clock(core)
    accepted_before = core.clock.stats(MONO_T0)["samples_accepted"]

    t = MONO_T0 + 4.0
    core.note_timesync_request(t, int(round(t * NS)))
    wrong_component = timesync_response(t, 0.002)
    wrong_component.compid = 3
    core.handle_message(wrong_component, t + 0.004)
    assert core.counters["rejected_timesync_source"] == 1

    core.handle_message(timesync_response(t, 0.002), t + 0.006)
    assert core.clock.stats(t + 0.006)["samples_accepted"] == accepted_before + 1


def test_stats_exposes_timesync_pairing_diagnostics():
    """新增配对计数与最近配对原因必须出现在 stats() 里（节点直接序列化它）。"""
    core = make_receiver()
    stats = core.stats(MONO_T0)
    for key in (
        "rejected_unpaired_timesync", "rejected_timesync_echo_mismatch",
        "rejected_timesync_timeout", "rejected_timesync_source",
        "px4_timesync_requests", "timesync_paired", "timesync_requests_superseded",
        "last_timesync_reason", "pending_timesync_ns", "pending_timesync_age_s",
    ):
        assert key in stats, f"诊断缺少字段 {key}"
    assert stats["rejected_unpaired_timesync"] == 0
    assert stats["timesync_paired"] == 0
    assert stats["last_timesync_reason"] is None
    assert stats["pending_timesync_ns"] is None

    # 一次无配对响应 + 一次正常往返：计数与原因都要动起来
    t = MONO_T0
    core.handle_message(timesync_response(t, 0.002), t + 0.002)
    assert core.stats(t + 0.002)["last_timesync_reason"] == REJECT_TIMESYNC_UNPAIRED
    sync_once(core, t + 1.0)
    stats = core.stats(t + 1.0)
    assert stats["rejected_unpaired_timesync"] == 1
    assert stats["timesync_paired"] == 1
    assert stats["time_sync"]["locked"] is True


def test_expired_request_and_superseded_request_are_counted_separately():
    """响应「太慢」与响应「丢了」要能分开：前者是被新请求覆盖，后者是超时。"""
    core = make_receiver(pending_timeout_s=0.25)

    t0 = MONO_T0 + 4.0
    core.note_timesync_request(t0, int(round(t0 * NS)))
    t1 = t0 + 0.1                      # 窗口内再次请求 → 旧请求只是被覆盖
    core.note_timesync_request(t1, int(round(t1 * NS)))
    assert core.counters["timesync_requests_superseded"] == 1
    assert core.counters["rejected_timesync_timeout"] == 0

    t2 = t1 + 0.5                      # 超过 pending_timeout_s → 旧请求算「超时」
    core.note_timesync_request(t2, int(round(t2 * NS)))
    assert core.counters["rejected_timesync_timeout"] == 1
    assert core.counters["timesync_requests_superseded"] == 1
    assert core.pending_timesync == (t2, int(round(t2 * NS)))

    # 超时释放后，新请求照常能配对
    core.handle_message(timesync_response(t2, 0.002), t2 + 0.004)
    assert core.counters["timesync_paired"] == 1


def test_source_validation_also_covers_px4_initiated_requests():
    """来源校验对飞控主动请求同样生效：别人的启动时钟不得进我们的基线/诊断。"""
    core = make_receiver(validate_timesync_source=True, system_id=1)
    foreign_boot_ns = int(round(boot_us_at(MONO_T0) * 1e3))
    core.handle_message(FakeMav(111, tc1=0, ts1=foreign_boot_ns, sysid=7), MONO_T0)
    assert core.counters["rejected_timesync_source"] == 1
    assert core.counters["px4_timesync_requests"] == 0
    assert core.stats(MONO_T0)["time_sync"]["baseline"] is None

    # 关掉校验后：同一个来源会被当作粗基线收下（真机应急开关语义）
    loose = make_receiver(validate_timesync_source=False, system_id=1)
    loose.handle_message(FakeMav(111, tc1=0, ts1=foreign_boot_ns, sysid=7), MONO_T0)
    assert loose.counters["px4_timesync_requests"] == 1
    baseline = loose.stats(MONO_T0)["time_sync"]["baseline"]
    assert baseline is not None and baseline["px4_boot_s"] > 0
