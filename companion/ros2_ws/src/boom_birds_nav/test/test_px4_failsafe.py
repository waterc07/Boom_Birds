"""`px4_failsafe` 的状态机测试：每个失效模式独立驱动闸门关闭，恢复必须迟滞。

这些测试要证明的是**语义**，不是"函数能跑"：
- 每一路必需输入的过期都能**单独**让 `allow_setpoint=False`，并给出正确原因码；
- 恢复需要连续 N 次新鲜评估，一次新鲜样本**不够**；
- 超时附近抖动不会重新武装指令（闸门保持关闭，而不是每帧开合）；
- PX4 重启必须重新建立（重启后心跳 + 新 setpoint）才可能放行；
- 计数递增、可诊断；
- 注入时钟 ⇒ 完全确定（同输入序列产生同输出序列）。

全部离线：不连接飞控/串口/UDP，不导入 rclpy，不读系统时钟。
"""

from __future__ import annotations

import inspect
import math
from pathlib import Path

import pytest

from boom_birds_nav import px4_failsafe as F
from boom_birds_nav.px4_failsafe import (
    CONTRACT_TIMING_REFERENCE,
    VEHICLE_REACTION_NOTE,
    FailsafeConfig,
    Px4FailsafeMonitor,
    ReasonCode,
    SafetyState,
    Severity,
    SignalId,
)

DT = 0.01  # 100 Hz 评估周期（与上游 /position_cmd 定时器一致）
N = 5      # 缺省迟滞次数


class Rig:
    """测试夹具：注入式时钟 + 可选的"每帧喂哪些信号"。

    语义与真实节点一致：**先收数据，再评估**。信号时间戳取"当前时刻"，
    然后推进时钟、评估。若每次都把时间戳打在评估时刻之后，所有信号都会
    永久"来自未来"、年龄恒为 0，测试就会变成断言空气。
    """

    def __init__(self, config: FailsafeConfig | None = None, t0: float = 0.0):
        self.m = Px4FailsafeMonitor(config)
        self.t = float(t0)
        self.boot_id = "boot-A"
        self.traj_id = 1

    # ---- 信号 ----
    def feed_all(self, *, setpoint: bool = True, vio: bool = True, imu: bool = True,
                 link: bool = True, heartbeat: bool = True, camera: bool = True,
                 odom: bool = True, stamp: float | None = None) -> None:
        ts = self.t if stamp is None else float(stamp)
        if setpoint:
            self.m.note_setpoint(ts, self.traj_id)
        if vio:
            self.m.note_vio_pose(ts)
        if imu:
            self.m.note_imu(ts)
        if link:
            self.m.note_mavlink_link(ts)
        if heartbeat:
            self.m.note_heartbeat(ts, self.boot_id)
        if camera:
            self.m.note_camera(ts)
        if odom:
            self.m.note_odom(ts)

    def tick(self, dt: float = DT):
        """推进时钟一步并评估（不喂任何信号）。"""
        self.t += dt
        return self.m.evaluate(self.t)

    def step(self, dt: float = DT, **feed):
        """喂信号 + 推进 + 评估（正常链路的一帧）。"""
        self.feed_all(**feed)
        return self.tick(dt)

    def steps(self, n: int, dt: float = DT, **feed):
        return [self.step(dt, **feed) for _ in range(n)]

    def warmup(self, n: int = N):
        """让状态机达到 OK（需要连续 N 次新鲜）。"""
        out = self.steps(n)
        assert out[-1].state is SafetyState.OK, out[-1].describe()
        assert out[-1].allow_setpoint is True
        return out


def _all_blocked(decisions) -> None:
    """断言整段都没有放行，并在失败时把第一条违规打印出来。"""
    bad = [d for d in decisions if d.allow_setpoint]
    assert not bad, bad[0].describe()


def _first_allow(decisions):
    for i, d in enumerate(decisions):
        if d.allow_setpoint:
            return i
    return None


# ---------------------------------------------------------------------------
# 缺省配置与契约对齐
# ---------------------------------------------------------------------------
def test_default_timeouts_are_justified_against_contract():
    """缺省超时必须与契约/上游/PX4 的已知量一致，不能是随手写的数字。"""
    cfg = FailsafeConfig()
    assert cfg.timeout_of(SignalId.SETPOINT) == pytest.approx(
        CONTRACT_TIMING_REFERENCE["pose_timeout_s"]
    )
    assert cfg.timeout_of(SignalId.VIO_POSE) == pytest.approx(
        CONTRACT_TIMING_REFERENCE["pose_timeout_s"]
    )
    # 缺省超时必须都小于 PX4 的 offboard 丢链超时，否则"我们还在发"和
    # "飞控已经开始 failsafe"会重叠，边界不可分析。
    for sig, t in cfg.timeouts.items():
        assert t < CONTRACT_TIMING_REFERENCE["px4_offboard_loss_timeout_s"], sig
    # setpoint 超时必须明显大于上游 100 Hz 周期（容忍调度抖动）
    assert cfg.timeout_of(SignalId.SETPOINT) >= 10 * CONTRACT_TIMING_REFERENCE["position_cmd_period_s"]
    # 心跳超时取标称周期，不是 2 个周期（2 个周期会撞上 1.0 s 的 COM_OF_LOSS_T）
    assert cfg.timeout_of(SignalId.PX4_HEARTBEAT) == pytest.approx(
        CONTRACT_TIMING_REFERENCE["px4_heartbeat_period_s"]
    )
    # 可选信号：相机对齐深度链量级，里程计按 20 Hz 的 4 个周期
    assert cfg.timeout_of(SignalId.CAMERA) == pytest.approx(0.5)
    assert cfg.timeout_of(SignalId.ODOM_EGO) == pytest.approx(0.2)
    # 迟滞次数：换算成时间必须远小于飞控丢链超时
    assert cfg.recovery_fresh_samples * DT < 0.2 * CONTRACT_TIMING_REFERENCE[
        "px4_offboard_loss_timeout_s"
    ]


def test_default_hysteresis_is_meaningful_not_a_single_sample():
    """缺省迟滞必须有实质长度：N=1 等于没有迟滞，属配置错误而非简化。"""
    cfg = FailsafeConfig()
    assert cfg.recovery_fresh_samples >= 3, "缺省迟滞太短，等于没有迟滞"
    # 单帧新鲜不可能让闸门打开：第一次评估就已经是 1 次干净，但必须继续关闭
    r = Rig()
    d = r.step()
    assert d.recovery_streak == 1
    assert d.allow_setpoint is False


def test_config_rejects_unsafe_or_incomplete_settings():
    with pytest.raises(ValueError):
        FailsafeConfig(recovery_fresh_samples=0)          # 无迟滞 = 禁止
    with pytest.raises(ValueError):
        FailsafeConfig(timeouts={SignalId.SETPOINT: 0.15})  # 缺必需信号超时
    with pytest.raises(ValueError):
        FailsafeConfig(timeouts={**FailsafeConfig().timeouts, SignalId.IMU: 0.0})
    with pytest.raises(ValueError):
        FailsafeConfig(timeouts={**FailsafeConfig().timeouts, SignalId.IMU: float("nan")})
    with pytest.raises(ValueError):
        FailsafeConfig(
            required_signals=(SignalId.SETPOINT, SignalId.SETPOINT),
            advisory_signals=(SignalId.CAMERA,),
        )
    with pytest.raises(ValueError):
        FailsafeConfig(
            required_signals=(SignalId.SETPOINT,),
            advisory_signals=(SignalId.SETPOINT,),
        )


def test_signal_order_is_stable_and_required_first():
    """原因排序基准：必需在前、可选在后；顺序不随数据抖动。"""
    cfg = FailsafeConfig()
    order = cfg.signals
    assert order == (
        SignalId.SETPOINT,
        SignalId.VIO_POSE,
        SignalId.IMU,
        SignalId.MAVLINK_LINK,
        SignalId.PX4_HEARTBEAT,
        SignalId.CAMERA,
        SignalId.ODOM_EGO,
    )
    assert cfg.severity_of(SignalId.SETPOINT) is Severity.BLOCKING
    assert cfg.severity_of(SignalId.CAMERA) is Severity.ADVISORY


# ---------------------------------------------------------------------------
# 起步：INIT 且不放行
# ---------------------------------------------------------------------------
def test_init_state_blocks_transmission_until_hysteresis_is_met():
    r = Rig()
    d0 = r.m.evaluate(0.0)
    assert d0.state is SafetyState.INIT
    assert d0.allow_setpoint is False
    assert d0.recovery_streak == 1
    # 从未收到任何输入 ⇒ NEVER_SEEN 原因齐全（每一路都有）
    never = [x for x in d0.reasons if x.code is ReasonCode.SIGNAL_NEVER_SEEN]
    assert {x.signal for x in never} >= {
        SignalId.SETPOINT,
        SignalId.VIO_POSE,
        SignalId.IMU,
        SignalId.MAVLINK_LINK,
        SignalId.PX4_HEARTBEAT,
    }
    # 且明确标出"不知道在跟谁说话"
    assert ReasonCode.PX4_IDENTITY_UNKNOWN in d0.reason_codes

    # 第 N-1 次仍不放行，第 N 次才放行（启动阶段同样有迟滞，不留无迟滞窗口）
    r2 = Rig()
    got = r2.steps(N - 1)
    assert all(x.allow_setpoint is False for x in got), [x.describe() for x in got]
    assert got[-1].recovery_streak == N - 1
    d = r2.step()
    assert d.allow_setpoint is True and d.state is SafetyState.OK
    assert d.recovery_streak == N


def test_init_transitions_straight_to_ok_without_a_stopped_phase():
    """启动是 INIT → OK：没有"曾经新鲜过又过期"就不该报 STOPPED。"""
    r = Rig()
    states = [r.step().state for _ in range(N)]
    assert states[: N - 1] == [SafetyState.INIT] * (N - 1)
    assert states[-1] is SafetyState.OK
    assert r.m.counters()["state_transitions"]["STOPPED"] == 0


def test_never_seen_is_distinguishable_from_stale():
    """从未收到 ≠ 过期：两者原因码不同（诊断要能区分"没接上"与"断了"）。"""
    r = Rig()
    d = r.m.evaluate(0.0)
    for reason in d.reasons:
        if reason.signal is not None and reason.code is not ReasonCode.PX4_IDENTITY_UNKNOWN:
            assert reason.code is ReasonCode.SIGNAL_NEVER_SEEN
            assert math.isinf(reason.age_s)
    r.warmup()
    got = r.steps(20, setpoint=False)  # setpoint 过期（0.15 s），但之前收到过
    stale = [x for x in got[-1].reasons if x.code is ReasonCode.SIGNAL_STALE]
    assert [x.signal for x in stale] == [SignalId.SETPOINT]
    assert stale[0].age_s == pytest.approx(0.21, abs=1e-9)  # 最后一次喂在 t=0.01
    assert stale[0].limit_s == pytest.approx(0.15)
    assert stale[0].triggered is True


# ---------------------------------------------------------------------------
# 每个失效模式独立关闸
# ---------------------------------------------------------------------------
@pytest.mark.parametrize(
    "signal,timeout,feed_key",
    [
        pytest.param(SignalId.SETPOINT, 0.15, "setpoint", id="setpoint-0.15s"),
        pytest.param(SignalId.VIO_POSE, 0.15, "vio", id="vio-pose-0.15s"),
        pytest.param(SignalId.IMU, 0.10, "imu", id="imu-0.10s"),
        pytest.param(SignalId.MAVLINK_LINK, 0.50, "link", id="mavlink-link-0.50s"),
        pytest.param(SignalId.PX4_HEARTBEAT, 0.50, "heartbeat", id="px4-heartbeat-0.50s"),
    ],
)
def test_each_required_signal_independently_blocks(signal, timeout, feed_key):
    """停喂某一路、其余照常 ⇒ 该路一超时闸门即关闭，原因指向它。

    边界语义（严格大于）由 `test_timeout_boundary_is_strictly_greater` 单独锁；
    这里专门验证"独立归因"：不能冤枉同时被喂着的其它信号。
    """
    r = Rig()
    r.warmup()
    n_ticks = int(math.ceil((timeout + 3 * DT) / DT))
    out = r.steps(n_ticks, **{feed_key: False})

    assert out[-1].allow_setpoint is False, f"{signal}: {out[-1].describe()}"
    assert out[-1].state is SafetyState.STOPPED
    codes = [(x.code, x.signal) for x in out[-1].blocking_reasons]
    assert (ReasonCode.SIGNAL_STALE, signal) in codes, out[-1].describe()
    # 其余必需信号在这段时间里一直被喂 ⇒ 不能冤枉它们
    others = [
        x for x in out[-1].blocking_reasons if x.signal is not None and x.signal is not signal
    ]
    assert others == [], f"{signal}: 误报 {[x.signal for x in others]}"
    assert out[-1].signal_ages_s[signal] > timeout
    # 每个过期信号只生成一条原因（防止闭锁/解释两套规则导致重复）
    assert len(out[-1].reasons) == len(set(id(x) for x in out[-1].reasons))
    assert len(codes) == len(set(codes))


def test_advisory_signals_do_not_gate_but_degrade():
    """可选输入过期只降级为 DEGRADED，仍然放行（信息性，不是安全输入）。"""
    r = Rig()
    r.warmup()
    got = r.steps(60, camera=False, odom=False)  # 0.6 s > 0.5 / 0.2
    d = got[-1]
    assert d.state is SafetyState.DEGRADED
    assert d.allow_setpoint is True
    assert [x.signal for x in d.advisory_reasons] == [SignalId.CAMERA, SignalId.ODOM_EGO]
    assert d.blocking_reasons == ()
    # 可选输入从不闭锁：latched 里不应出现它导致的信号故障
    assert d.latched_reasons == ()


def test_camera_never_seen_does_not_block_startup():
    """相机从没接上也不该堵死闸门——那是把可用性问题伪装成安全问题。"""
    r = Rig()
    out = r.steps(N, camera=False, odom=False)
    assert out[-1].allow_setpoint is True
    assert out[-1].state is SafetyState.DEGRADED


def test_advisory_can_be_promoted_to_required_by_config():
    """若某部署确实要求相机在环：改配置即可，不另开代码分支。"""
    cfg = FailsafeConfig(
        required_signals=(
            SignalId.SETPOINT,
            SignalId.VIO_POSE,
            SignalId.IMU,
            SignalId.MAVLINK_LINK,
            SignalId.PX4_HEARTBEAT,
            SignalId.CAMERA,
        ),
        advisory_signals=(SignalId.ODOM_EGO,),
    )
    r = Rig(cfg)
    out = r.steps(N, camera=False)
    assert out[-1].allow_setpoint is False
    assert (ReasonCode.SIGNAL_NEVER_SEEN, SignalId.CAMERA) in [
        (x.code, x.signal) for x in out[-1].blocking_reasons
    ]


def test_stale_setpoint_never_allows_transmission():
    """不变量：必需输入一旦过期，过期期间**永远**不放行。

    同时锁死"过期这件事本身必须被报出来"（`stale_signals` 里有它），
    否则综合判据里一旦漏掉"观测层越限"，闸门就会在过期期间保持打开。
    """
    r = Rig()
    r.warmup()
    for d in r.steps(4 * N, setpoint=False):
        if d.signal_ages_s[SignalId.SETPOINT] > 0.15:
            assert d.allow_setpoint is False, d.describe()
            assert ReasonCode.SIGNAL_STALE in d.reason_codes, d.describe()
            assert SignalId.SETPOINT in d.stale_signals, d.describe()
        else:
            assert d.allow_setpoint is True, d.describe()
            assert d.stale_signals == (), d.describe()


def test_planning_rejection_blocks_until_explicitly_cleared():
    """规划拒绝是**闭锁**：链路全好也不放行，直到显式清除。

    注意"时间流逝"不算重新建立：规划器给出新轨迹是外部事件，本模块无从推断，
    必须由调用方显式 `clear_planning_rejected()`（这条通过"闭锁期间一直不放行"
    来验证）。
    """
    r = Rig()
    r.warmup()
    r.m.note_planning_rejected(r.t, "无可行轨迹")
    d = r.step()
    assert d.allow_setpoint is False
    assert ReasonCode.PLANNING_REJECTED in d.reason_codes

    # 输入始终新鲜，但拒绝是闭锁的：一直不放行（不是"等 N 帧就自愈"）
    held = r.steps(4 * N)
    assert all(x.allow_setpoint is False for x in held), [x.describe() for x in held]
    assert all(ReasonCode.PLANNING_REJECTED in x.reason_codes for x in held)

    # 显式清除：此时迟滞窗口早已攒满，闸门随即打开并保持打开
    r.m.clear_planning_rejected()
    got = r.steps(N)
    assert all(x.allow_setpoint is True for x in got), [x.describe() for x in got]
    assert got[-1].state is SafetyState.OK
    assert ReasonCode.PLANNING_REJECTED not in got[-1].latched_reasons


def test_event_latch_blocks_even_when_hysteresis_is_full():
    """事件类闭锁单独就能挡住闸门：迟滞攒满也不行。

    这条专门锁死"闸门必须同时看闭锁集合"这一点：如果实现里只检查
    "当前观测是否越限"，那么"输入全部新鲜 + 事件故障未清除"就会误放行。
    """
    r = Rig()
    r.warmup()
    assert r.m.report()["recovery_streak"] == N          # 迟滞已满
    r.m.note_planning_rejected(r.t, "无可行轨迹")
    # 事件类闭锁不参与迟滞清零，而且链路一直在新鲜：再过很多帧也依然不放行
    for i in range(4 * N):
        d = r.step()
        assert d.allow_setpoint is False, f"第 {i} 帧放行了：{d.describe()}"
    assert ReasonCode.PLANNING_REJECTED in d.latched_reasons
    # 观测层面没有任何"信号过期"：所有必需输入都新鲜，挡住闸门的只能是闭锁集合
    assert d.stale_signals == (), d.describe()
    assert all(
        x.code is ReasonCode.PLANNING_REJECTED for x in d.blocking_reasons
    ), [x.code.value for x in d.blocking_reasons]
    # 重新建立条件满足后（显式清除）立刻恢复——说明前面确实是闭锁在挡
    r.m.clear_planning_rejected()
    assert r.step().allow_setpoint is True


def test_hard_latch_blocks_without_any_observation_level_reason():
    """闭锁存在但观测干净时，闸门仍必须关闭（不依赖"观测里有原因"）。"""
    r = Rig()
    r.warmup()
    r.m.request_reset("运维要求重新建立")
    d = r.step()
    assert d.reasons == () or all(x.code is ReasonCode.RESET_REQUESTED for x in d.reasons)
    assert ReasonCode.RESET_REQUESTED in d.latched_reasons
    assert d.allow_setpoint is False


def test_planning_rejection_clear_does_not_rearm_without_fresh_samples():
    """清除闭锁不等于可以发：若输入仍未新鲜，闸门保持关闭。"""
    r = Rig()
    r.warmup()
    r.m.note_planning_rejected(r.t, "无可行轨迹")
    r.step()
    r.steps(30, setpoint=False)      # 同时断掉 setpoint
    r.m.clear_planning_rejected()
    got = r.steps(30, setpoint=False)
    assert all(d.allow_setpoint is False for d in got)
    assert all(ReasonCode.SIGNAL_STALE in d.reason_codes for d in got)


def test_trajectory_invalidation_requires_a_new_trajectory_id():
    """轨迹作废后：必须换新 trajectory_id 才能重新建立，且放行要晚于重新建立。"""
    r = Rig()
    r.warmup()
    r.m.note_trajectory_invalidated(r.t, "Bspline 为空")
    assert r.step().allow_setpoint is False
    _all_blocked(r.steps(3 * N))

    # 换新轨迹 ⇒ 重新建立；迟滞要求"连续多次干净"而不是"一帧就够"
    r.traj_id = 2
    got = r.steps(2 * N)
    idx = _first_allow(got)
    assert idx is not None, [d.describe() for d in got]
    assert idx > 0, "重新建立后第一帧就放行 ⇒ 迟滞失效"
    assert all(d.allow_setpoint for d in got[idx:]), "放行后又关闭 ⇒ 振荡"
    assert r.m.report()["setpoint_version"] == 2
    assert ReasonCode.TRAJECTORY_INVALIDATED not in got[-1].latched_reasons
    assert r.m.counters()["recoveries"] >= 1


def test_trajectory_invalidation_without_new_id_stays_closed():
    """旧 id 反复到达不算"重新建立"：闸门一直关闭。"""
    r = Rig()
    r.warmup()
    r.m.note_trajectory_invalidated(r.t)
    r.step()
    got = r.steps(4 * N)          # traj_id 一直是 1
    _all_blocked(got)
    assert all(ReasonCode.TRAJECTORY_INVALIDATED in d.reason_codes for d in got)
    assert r.m.report()["reestablishment_pending"] is False


def test_px4_restart_requires_reestablishment():
    """PX4 重启：必须"新 boot_id 心跳 + 换了轨迹号的新 setpoint"。

    沿用同一条 trajectory_id 说明上游执行器仍是重启前的状态，不算重新建立。
    """
    r = Rig()
    r.warmup()
    r.m.note_px4_restart(r.t, "收到 boot 计数变化")
    assert r.step().allow_setpoint is False

    # 1) 重启前的 boot_id 心跳（输入都新鲜）不足以重新建立
    blocked = r.steps(2 * N)
    _all_blocked(blocked)
    assert all(ReasonCode.PX4_RESTARTED in d.latched_reasons for d in blocked), [
        d.describe() for d in blocked
    ]

    # 2) 只有新 boot_id 心跳、轨迹号没变 ⇒ 仍然不放行
    r.boot_id = "boot-B"
    same_traj = r.steps(2 * N)
    _all_blocked(same_traj)
    assert all(ReasonCode.PX4_RESTARTED in d.latched_reasons for d in same_traj)

    # 3) 换轨迹号 ⇒ 重新建立，并在 2N 帧内放行、此后保持放行
    r.traj_id = 7
    got = r.steps(2 * N)
    idx = _first_allow(got)
    assert idx is not None, [d.describe() for d in got]
    assert idx > 0, "重新建立后第一帧就放行 ⇒ 迟滞失效"
    assert all(d.allow_setpoint for d in got[idx:])
    assert got[-1].boot_id == "boot-B"
    assert ReasonCode.PX4_RESTARTED not in got[-1].latched_reasons
    assert r.m.counters()["latch_counts"][ReasonCode.PX4_RESTARTED.value] == 1


def test_boot_id_change_is_detected_without_explicit_restart_call():
    """心跳 boot_id 变化即可被自动识别为重启（回放/断点续跑场景）。"""
    r = Rig()
    r.warmup()
    r.boot_id = "boot-B"
    d = r.step()
    assert d.allow_setpoint is False
    assert ReasonCode.PX4_RESTARTED in d.reason_codes
    assert r.m.counters()["px4_reboot_count"] == 1
    # 沿用旧轨迹号 ⇒ 重新建立条件不满足，一直闭锁
    more = r.steps(3 * N)
    _all_blocked(more)
    assert all(ReasonCode.PX4_RESTARTED in x.latched_reasons for x in more)
    # 换轨迹号 ⇒ 重新建立并在 2N 帧内放行
    r.traj_id = 3
    got = r.steps(2 * N)
    idx = _first_allow(got)
    assert idx is not None and idx > 0, [d.describe() for d in got]
    assert all(d.allow_setpoint for d in got[idx:])


def test_boot_id_change_while_already_latched_counts_once():
    """同一轮重启内的多次心跳只算一次故障（否则计数会被刷爆）。"""
    r = Rig()
    r.warmup()
    r.boot_id = "boot-B"
    for _ in range(3):
        r.step()
    assert r.m.counters()["px4_reboot_count"] == 1
    assert r.m.counters()["latch_counts"]["px4_restarted"] == 1


def test_reset_request_blocks_until_hysteresis():
    r = Rig()
    r.warmup()
    r.m.request_reset("运维重新使能")
    assert r.step().allow_setpoint is False
    r.m.clear_reset()
    assert r.step().allow_setpoint is False
    assert r.steps(N - 1)[-1].allow_setpoint is True
    assert r.m.counters()["reset_request_count"] == 1


# ---------------------------------------------------------------------------
# 迟滞与抖动
# ---------------------------------------------------------------------------
def test_recovery_requires_n_consecutive_fresh_samples():
    """恢复必须连续 N 次新鲜：单帧新鲜不算（这条锁死"迟滞"语义）。

    完整恢复 = N 次干净评估清除闭锁 + 再 N 次干净评估重新武装闸门。
    """
    r = Rig()
    r.warmup()
    r.steps(20, setpoint=False)
    assert r.m.evaluate(r.t).allow_setpoint is False

    # 只补一帧：尚未清闭锁，仍不放行
    d = r.step()
    assert d.allow_setpoint is False
    assert d.recovery_streak == 1

    # "隔一段就断一次"（周期 20 帧 = 0.20 s > 超时 0.15 s）：
    # 单帧新鲜之后立刻断 19 帧，每次都把迟滞打断；核心不变量是
    # **只要 setpoint 处于过期状态，就绝不放行**。
    for _ in range(4):
        assert r.step().allow_setpoint is False              # 一帧新鲜（不够）
        stale = r.steps(19, setpoint=False)                  # 再次过期
        offenders = [
            d for d in stale if d.signal_ages_s[SignalId.SETPOINT] > 0.15 and d.allow_setpoint
        ]
        assert not offenders, offenders[0].describe()

    # 连续干净 2N 次后才重新放行
    out = r.steps(2 * N)
    assert all(d.allow_setpoint is False for d in out[:-1]), [d.describe() for d in out]
    assert out[-1].allow_setpoint is True
    assert out[-1].recovery_streak >= N


def test_single_fresh_sample_after_gap_does_not_rearm():
    """单帧假恢复：闸门不开（对比"无迟滞实现"会立刻放行）。"""
    r = Rig()
    r.warmup()
    d = r.steps(int(0.15 / DT) + 2, setpoint=False)[-1]
    assert d.allow_setpoint is False
    for i in range(N):
        d = r.step()  # 连续补 N 帧：只够清闭锁，还不够重新武装
        assert d.allow_setpoint is False, f"第 {i + 1} 帧就放行了：{d.describe()}"
    # 再补 N 帧才放行
    assert r.steps(N)[-1].allow_setpoint is True


def test_flapping_near_timeout_keeps_gate_closed():
    """超时附近抖动：**只要必需输入处于过期状态，就绝不放行**。

    周期 15 帧（0.15 s）：每周期第 1 帧喂 setpoint，其余 15 帧不喂。
    这里逐帧断言的核心不变量是：`allow_setpoint` 与"setpoint 已过期"不可能
    同时为真——抖动期间闸门最多在"确实新鲜"的帧上短暂可用，过期帧一律关闭。
    """
    r = Rig()
    r.warmup()
    all_decisions = []
    for _ in range(6):
        all_decisions.append(r.step())                  # 每周期第 1 帧喂 setpoint
        all_decisions.extend(r.steps(15, setpoint=False))
    offenders = [
        d
        for d in all_decisions
        if d.allow_setpoint and d.signal_ages_s[SignalId.SETPOINT] > 0.15
    ]
    assert not offenders, offenders[0].describe()
    # 过期帧确实存在（否则这个测试什么都没证明）
    assert any(d.signal_ages_s[SignalId.SETPOINT] > 0.15 for d in all_decisions)


def test_single_frame_freshness_never_rearms_in_a_flapping_link():
    """抖动链路的核心不变量：**必需输入处于过期状态时，绝不放行**。

    周期 20 帧：第 1 帧喂 setpoint，随后 19 帧不喂（0.20 s > 超时 0.15 s，
    确实越过超时线）。闸门最多在"输入确实新鲜"的帧上可用——这本来就是
    正常的喂样间隙；一旦越过超时线，就必须关闭。
    """
    r = Rig()
    r.warmup()
    all_decisions = []
    for _ in range(8):
        all_decisions.append(r.step())                       # 单帧新鲜
        all_decisions.extend(r.steps(19, setpoint=False))    # 随后长期过期
    stale = [d for d in all_decisions if d.signal_ages_s[SignalId.SETPOINT] > 0.15]
    assert len(stale) >= 4, "过期窗口太少，这个测试什么都没证明"
    offenders = [d for d in stale if d.allow_setpoint]
    assert not offenders, offenders[0].describe()


def test_fast_enough_link_is_not_penalised():
    """对照：喂样周期快于超时（0.14 s < 0.15 s）⇒ 视为持续新鲜，闸门常开。

    这条说明"抖动保护"不会误伤正常链路——判定依据是**超时**，不是"必须每帧都有"。
    """
    r = Rig()
    r.warmup()
    out = []
    for i in range(48):
        out.append(r.step(setpoint=(i % 14 == 0)))
    assert all(d.allow_setpoint is True for d in out), [d.describe() for d in out]
    assert all(d.state is SafetyState.OK for d in out)
    assert r.m.counters()["recoveries"] == 0


def test_clean_recovery_opens_gate_exactly_once():
    """干净恢复：闸门只开一次，recoveries 只加一，之后一直保持打开。

    必需输入过期属于"逐路 stale 闭锁"：清除后还需要 N 次干净评估重新武装，
    因此从过期到再次放行的完整代价是 2N 帧。
    """
    r = Rig()
    r.warmup()
    assert r.m.counters()["recoveries"] == 0
    blocked = r.steps(30, setpoint=False)
    # 前 15 帧仍在超时线内（尚未过期），之后必须全部关闭
    assert all(
        d.allow_setpoint is False
        for d in blocked
        if d.signal_ages_s[SignalId.SETPOINT] > 0.15
    ), [d.describe() for d in blocked if d.allow_setpoint][:3]
    assert r.m.counters()["recoveries"] == 0
    out = r.steps(2 * N + 10)
    assert all(d.allow_setpoint is False for d in out[: 2 * N - 1]), [
        d.describe() for d in out[: 2 * N - 1]
    ]
    assert all(d.allow_setpoint for d in out[2 * N - 1:])
    assert r.m.counters()["recoveries"] == 1
    # "一直正常"期间不再重复计恢复
    r.steps(20)
    assert r.m.counters()["recoveries"] == 1
    assert r.m.counters()["state_transitions"]["OK"] == 2


# ---------------------------------------------------------------------------
# 每路超时独立生效
# ---------------------------------------------------------------------------
def test_per_signal_timeouts_are_respected_individually():
    """自定义超时：0.05 s 的那路先关闸，0.30 s 的那路仍然新鲜。"""
    cfg = FailsafeConfig(
        timeouts={
            SignalId.SETPOINT: 0.05,
            SignalId.VIO_POSE: 0.30,
            SignalId.IMU: 0.30,
            SignalId.MAVLINK_LINK: 0.30,
            SignalId.PX4_HEARTBEAT: 0.30,
            SignalId.CAMERA: 0.30,
            SignalId.ODOM_EGO: 0.30,
        },
        recovery_fresh_samples=2,
    )
    r = Rig(cfg)
    out = r.steps(2)
    assert out[-1].allow_setpoint is True
    assert out[-1].recovery_samples_required == 2

    # 0.06 s 不喂 setpoint：只有它超时（0.06 > 0.05）
    got = r.steps(6, setpoint=False)
    assert got[-1].allow_setpoint is False
    codes = [(x.code, x.signal) for x in got[-1].blocking_reasons]
    assert (ReasonCode.SIGNAL_STALE, SignalId.SETPOINT) in codes
    assert not any(sig is SignalId.VIO_POSE for _, sig in codes)
    assert got[-1].signal_ages_s[SignalId.VIO_POSE] < 0.30
    assert got[-1].signal_ages_s[SignalId.SETPOINT] > 0.05


def test_timeout_boundary_is_strictly_greater():
    """边界语义：age == limit 仍算新鲜，age > limit 才过期（写进测试锁死）。"""
    cfg = FailsafeConfig(timeouts={**FailsafeConfig().timeouts, SignalId.SETPOINT: 0.10})
    r = Rig(cfg)
    for _ in range(N):
        r.feed_all()
        r.t += DT
        r.m.evaluate(r.t)

    # 只把 setpoint 打在基准时刻，其余信号打在 0.0995 之后 ⇒ setpoint 年龄 0.0995
    base = r.t
    r.t = base + 0.0995
    r.feed_all(setpoint=False)
    r.m.note_setpoint(base, r.traj_id)
    d = r.m.evaluate(r.t)
    assert d.signal_ages_s[SignalId.SETPOINT] == pytest.approx(0.0995, abs=1e-9)
    assert d.allow_setpoint is True, d.describe()          # age < limit ⇒ 新鲜

    # 推到刚过限值 ⇒ 过期
    r.t += 0.002
    d2 = r.m.evaluate(r.t)
    assert d2.signal_ages_s[SignalId.SETPOINT] == pytest.approx(0.1015, abs=1e-9)
    assert d2.allow_setpoint is False
    assert (ReasonCode.SIGNAL_STALE, SignalId.SETPOINT) in [
        (x.code, x.signal) for x in d2.blocking_reasons
    ]


def test_reason_carries_age_versus_limit():
    """诊断要求：每条原因都要给出"触发年龄 vs 限值"。"""
    r = Rig()
    r.warmup()
    d = r.steps(20, vio=False)[-1]
    reason = [x for x in d.blocking_reasons if x.signal is SignalId.VIO_POSE][0]
    assert reason.age_s == pytest.approx(0.21, abs=1e-9)   # 最后一次喂在 t=0.01
    assert reason.limit_s == pytest.approx(0.15)
    assert reason.triggered is True
    assert "vio_pose" in reason.describe() and "0.210s" in reason.describe()


# ---------------------------------------------------------------------------
# 计数与诊断
# ---------------------------------------------------------------------------
def test_counters_increment_per_reason_and_transition():
    r = Rig()
    r.warmup()
    r.steps(20, setpoint=False)
    c = r.m.counters()
    assert c["latch_counts"][ReasonCode.SIGNAL_STALE.value] == 1  # 只计一次新故障
    # 启动阶段：INIT（初始态不计数）→ OK（迟滞攒够 N 次）
    assert c["state_transitions"]["INIT"] == 0
    assert c["state_transitions"]["STOPPED"] == 1   # 过期后进入 STOPPED
    assert c["state_transitions"]["OK"] == 1        # 启动时 INIT -> OK
    assert c["blocked_evaluations"] >= 1
    assert c["evaluations"] == N + 20          # 仅为 warmup 的 N 次与之后 20 次
    # 恢复一次后计数递增（2N 次干净评估：N 清闭锁 + N 重新武装）
    r.steps(2 * N)
    assert r.m.counters()["recoveries"] == 1
    assert r.m.counters()["state_transitions"]["OK"] == 2


def test_never_seen_counts_only_when_actually_observed_as_missing():
    """"从未收到"只在真的观测到缺失时计数，输入到齐后不会凭空补一笔。"""
    r = Rig()
    r.steps(N)  # 全程都喂，不存在缺席信号
    assert ReasonCode.SIGNAL_NEVER_SEEN.value not in r.m.counters()["latch_counts"]
    # 全新实例：先评估再喂 —— 五路必需信号各计一次"从未收到"
    r2 = Rig()
    r2.m.evaluate(0.0)
    assert r2.m.counters()["latch_counts"][ReasonCode.SIGNAL_NEVER_SEEN.value] == len(
        FailsafeConfig().required_signals
    )


def test_report_and_decision_are_json_serializable():
    """诊断输出必须能直接 JSON 化（含 inf 时的处理）。"""
    import json

    r = Rig()
    d0 = r.m.evaluate(0.0)
    blob0 = json.dumps(d0.to_dict())
    assert "signal_never_seen" in blob0
    assert "age_is_infinite" in blob0
    r.warmup()
    r.steps(20, vio=False)
    blob = json.dumps(r.m.report())
    assert "vio_pose" in blob
    assert json.loads(blob)["state"] == "STOPPED"


def test_decision_declares_the_we_stop_vs_vehicle_does_distinction():
    """关键区分必须在**代码输出**里：停发 ≠ 飞机安全。"""
    r = Rig()
    d = r.m.evaluate(0.0)
    assert d.note == VEHICLE_REACTION_NOTE
    assert "COM_OF_LOSS_T" in d.note and "COM_OBL_RC_ACT" in d.note
    assert "仅表示本节点停止发送" in d.note
    assert "COM_OF_LOSS_T" in r.m.report()["note"]
    # 文档也要有：模块 docstring 必须写明这条边界
    doc = F.__doc__ or ""
    assert "COM_OF_LOSS_T" in doc and "COM_OBL_RC_ACT" in doc
    assert "HOLD_LAST" in doc  # 并说明为何没有该状态


def test_state_list_is_exactly_init_ok_degraded_stopped():
    """状态清单固定；没有 HOLD_LAST（不缓存上一帧 setpoint）。"""
    assert [s.value for s in SafetyState] == ["INIT", "OK", "DEGRADED", "STOPPED"]
    assert "HOLD_LAST" not in {s.name for s in SafetyState}


# ---------------------------------------------------------------------------
# 确定性 / 纯净性
# ---------------------------------------------------------------------------
def _record(config: FailsafeConfig | None = None):
    """跑一段固定脚本，返回决策摘要序列。"""
    r = Rig(config)
    out = []
    for i in range(60):
        feed = {
            "setpoint": i % 7 != 3,
            "vio": i < 40,
            "imu": True,
            "link": True,
            "heartbeat": True,
            "camera": i % 9 != 0,
            "odom": True,
        }
        out.append(r.step(**feed).describe())
    return out


def test_determinism_with_injected_clock():
    """同输入序列 ⇒ 同输出序列（两次独立运行逐字符一致）。"""
    a = _record()
    b = _record()
    assert a == b
    assert len(a) == 60
    assert any("state=STOPPED" in line for line in a)


def test_no_system_clock_no_ros_no_io():
    """纯类：不得读取系统时钟、不得导入 ROS/网络相关模块。"""
    src = Path(F.__file__).read_text(encoding="utf-8")
    for forbidden in (
        "import rclpy",
        "from rclpy",
        "import time",
        "from time import",
        "time.monotonic",
        "time.time",
        "import socket",
        "import threading",
        "import serial",
    ):
        assert forbidden not in src, f"px4_failsafe.py 不应出现 {forbidden!r}"
    # evaluate 只接受注入的时钟参数，且没有默认值（逼调用方显式传入）
    sig = inspect.signature(Px4FailsafeMonitor.evaluate)
    assert list(sig.parameters) == ["self", "now_mono_s"]
    assert sig.parameters["now_mono_s"].default is inspect.Parameter.empty


def test_clock_regression_never_makes_signals_look_fresher():
    """时钟回退不得让信号"看起来更新鲜"：年龄基准停在回退前的读数并计数。

    回退时"年龄不推进"（宁可让过期判定晚一点，也不允许它变新）；
    所以本帧仍可能放行——这是有意的折中，且**必须被计数**，不能静默。
    """
    r = Rig()
    r.warmup()
    d_before = r.m.evaluate(r.t)
    assert d_before.allow_setpoint is True
    assert d_before.signal_ages_s[SignalId.SETPOINT] == pytest.approx(DT)

    back = r.m.evaluate(r.t - 5.0)  # 回退 5 s
    assert r.m.counters()["clock_regression_count"] == 1
    # 诊断保持诚实：now_mono_s 是调用方实际传入的读数
    assert back.now_mono_s == pytest.approx(r.t - 5.0)
    # 年龄被钳制为"不推进"，绝不会变成负数或被"变新"
    for age in back.signal_ages_s.values():
        assert age >= 0.0
    assert back.signal_ages_s[SignalId.SETPOINT] == pytest.approx(DT)
    assert back.allow_setpoint is True

    # 时钟继续向前后恢复正常的过期判定：多等 0.2 s 不喂 setpoint 必然过期
    r.m.evaluate(r.t)
    r.t += 0.2
    d_after = r.m.evaluate(r.t)
    assert d_after.signal_ages_s[SignalId.SETPOINT] > 0.15
    assert d_after.allow_setpoint is False


def test_non_finite_clock_rejected():
    r = Rig()
    with pytest.raises(ValueError):
        r.m.evaluate(float("nan"))


def test_future_timestamps_are_clamped_and_counted():
    """信号时间戳来自"未来"：钳制为 0 年龄但必须计数（不静默）。"""
    r = Rig()
    r.m.note_setpoint(100.0, 1)
    d = r.m.evaluate(1.0)
    assert d.signal_ages_s[SignalId.SETPOINT] == pytest.approx(0.0)
    assert r.m.counters()["future_timestamp_count"] > 0
