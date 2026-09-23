"""PX4 offboard setpoint 发送闸门：纯状态机，无 ROS、无 I/O、无系统时钟。

这个模块回答一个问题：**"这一帧允许发出去吗？"**
不回答"飞机应该怎么飞"——那是规划器与 PX4 的事。

它为什么必须独立存在
--------------------
判据散落在节点里会出三类事故：
1) 每个超时各自实现一遍，某个分支忘了判，链路断了还在发；
2) 恢复靠"下一帧到了就继续发"，链路抖动时就变成"断续发指令"；
3) 把"我们停发"误当成"飞机已经安全"。
本模块把这三条变成显式、可单测的语义：**闭锁 + 迟滞恢复 + 明确的职责边界**。

职责边界（**必须写进代码的区分**）
--------------------------------
``SafetyDecision.allow_setpoint = False`` 的含义**仅仅是本节点不再发送 setpoint**。
它不是对飞行器的命令，也不代表飞行器已经进入任何安全状态：

* PX4 侧的 offboard 丢链行为由飞控参数决定，本仓库无法从代码推断其实际生效值：
  ``COM_OF_LOSS_T``（Offboard connection loss timeout，缺省 1.0 s）与
  ``COM_OBL_RC_ACT``（缺省 0 = Position 模式）——见 PX4
  ``src/modules/commander/commander_params.yaml:185-230``（只读引用）。
* 这些参数是**飞控上的配置**，不是本仓库的常量；是否匹配本模块的超时、以及
  实际触发的是哪一种动作，只能在 SITL/实机上验证。本模块不做任何此类声明。
* 因此"停发"只保证"Companion 不再给出错误指令"，不保证"飞行器安全"。

本模块不做的事：不发 MAVLink、不读话题、不看时钟、不做坐标换算、不决定飞行模式。

状态机
------
::

    INIT ──(必需输入全部新鲜，连续 N 次评估)──▶ OK
      ▲                                        │
      │                         任一必需输入过期 / 规划拒绝 /
      │                         轨迹作废 / PX4 重启
      │                                        ▼
      └────────(恢复条件满足)──────────── STOPPED（allow_setpoint=False）
      OK ──(仅可选输入过期)──▶ DEGRADED（仍允许发送）

* ``INIT``：从未到达过 OK。启动阶段同样要求连续 N 次新鲜评估，不因为
  "刚开机所以先放行"而留下无迟滞的窗口。
* ``OK``：允许发送。
* ``DEGRADED``：允许发送，但已知有**可选**输入缺失（信息性，不闭锁，绝不作为
  "部分放行"的借口：任何必需输入过期都直接 STOPPED）。
* ``STOPPED``：闭锁，``allow_setpoint=False``。
* 没有 HOLD_LAST 状态：本模块**不缓存上一帧 setpoint**。上游 EGO 的
  `traj_server` 在轨迹作废时直接停止发布（``receive_traj_ = false`` 时
  ``cmdCallback`` 立即 return），"继续发上一帧的位置"会变成一条没有时间基准的
  指令；而本仓库的立场是"定位失效时不能默认仍可悬停，不能持续盲冲"（README）。
  要不要"原地悬停"是 PX4 侧 failsafe 的职责（见上）。

恢复必须是显式的（迟滞）——精确语义
----------------------------------
一次新鲜样本**不足以**重新放行。闸门条件是::

    allow = 没有"实质越限"              # 某路曾经新鲜却已过期，或有事件类故障
            且 没有未清除的阻塞性闭锁
            且 迟滞计数已满 N 次
            且 所有必需信号都确实收到过

迟滞计数只统计"输入确实新鲜"的评估，**只有** ``signal_stale``（某路曾经新鲜却
已过期）会把它清零。由此得到两条确定的行为：

* **故障恢复需要 2N 次干净评估**：N 次用来清除闭锁，清除本身把计数清零，
  再 N 次才重新武装。因此"单帧恢复"永远无法重新发送指令。
* **抖动链路无法无限重新武装**：喂样周期只要让某路反复越过超时线，计数就反复
  清零。以缺省 N=5、超时 0.15 s 为例：喂样周期 0.20 s（新鲜段 19 帧）仍会
  重新武装，而周期 0.15 s（新鲜段 9 帧 < 2N）不会——这条边界在测试里有断言。

``planning_rejected`` / ``trajectory_invalidated`` / ``px4_restarted`` /
``reset_requested`` 是**事件类**闭锁，不参与计数清零，但各自的重新建立条件必须
满足才可能被清除（见 ``_retry_satisfied``）：时间流逝本身不算重新建立。

上游恢复 100 Hz 之后若某一路只剩 7 Hz（每 0.143 s 擦一次 0.15 s 的线），每次
擦线都会把计数清零，于是闸门保持关闭而不是每帧开合——这是刻意选择的方向：
宁可停发，也不要断续指令。恢复延迟上界约 ``2N / 评估频率``（50 Hz 下 0.2 s），
仍远小于 ``COM_OF_LOSS_T``（1.0 s）。

超时缺省值与依据（**不静默使用研究代码里的数字**）
--------------------------------------------------
每路输入独立配置，缺省值分两类。

必需输入（过期即闭锁）：

===========================  =========  ==========================================
信号                          缺省(s)    依据
===========================  =========  ==========================================
``setpoint``                 0.15       与契约 ``timing.pose_timeout_s`` 同值：
                                        上游 `/position_cmd` 是 100 Hz 定时器
                                        （`traj_server.cpp` 10 ms），
                                        0.15 s ≈ 15 个周期，足以容忍调度抖动，
                                        又远小于 PX4 的 1.0 s 丢链超时。
``vio_pose``                 0.15       契约 ``pose_timeout_s = 0.15``，原值照抄，
                                        不另立数字。VIO 失效 ⇒ 世界系位姿无意义
                                        ⇒ 轨迹坐标系无意义。
``imu``                      0.10       飞控 IMU 是 VIO 前端且速率最高
                                        （HIGHRES_IMU，目标 100 Hz 量级）；
                                        取一个标称周期，过期即说明前端已断。
``mavlink_link``             0.50       MAVLink 链路活动指示；取 2 Hz 心跳的一个
                                        周期作为"链路仍在"的宽松判据。
``px4_heartbeat``            0.50       PX4 心跳标称 2 Hz（0.5 s 周期）：取 2 个
                                        周期会正好撞上 1.0 s 的 ``COM_OF_LOSS_T``，
                                        1 个周期太紧，故取 1 个周期。注意判据不是
                                        "飞机是否已进入 failsafe"（那由 1.0 s 决定），
                                        而是"我们还知不知道在跟谁说话"。
===========================  =========  ==========================================

可选输入（过期只降级为 DEGRADED，不影响发送）：

===========================  =========  ==========================================
``camera``                   0.50       深度/相机链缺数据（与契约
                                        ``depth_timeout_s = 1.0`` 同向），但
                                        `/position_cmd` 仍可能有效；是否把相机
                                        列为必需由调用方按任务决定。
``odom_ego``                 0.20       诊断用里程计（约 20 Hz）的 4 个周期。
===========================  =========  ==========================================

``recovery_fresh_samples`` 缺省 **5**：50 Hz 后端循环里约 0.1 s、100 Hz 里约
0.05 s，远小于 ``COM_OF_LOSS_T``（1.0 s）——否则恢复期的空档本身就会触发飞控
failsafe；同时足够长以拒绝"单帧新鲜"的假恢复。

用法（调用方负责的接线）
------------------------
1) 每收到一条 `/position_cmd`：``monitor.note_setpoint(now, trajectory_id=...)``。
   ``trajectory_id`` 必须来自消息本体（EGO 的 id 从 1 开始，0 表示无轨迹），
   本模块用它判断"轨迹作废之后是否换了新轨迹"。
2) 每收到心跳：``monitor.note_heartbeat(now, boot_id=...)``；``boot_id`` 是飞控
   重启标识（如 boot 计数，具体取值由后端决定），**同一进程内只要变化即视为
   PX4 重启**（重启会清空 PX4 的 offboard 状态机）。
3) 规划器拒绝新目标：``monitor.note_planning_rejected(now, detail=...)``；
   规划器恢复后必须显式 ``clear_planning_rejected()``。
4) 收到"轨迹作废"（``Bspline`` 空消息 / `traj_server` 不再发布）：
   ``monitor.note_trajectory_invalidated(now)``。
5) 每个控制周期：``decision = monitor.evaluate(now)``；只在
   ``decision.allow_setpoint`` 为真时调用后端发送。

无 ROS、无 I/O、无系统时钟：``now_mono_s`` 全部由调用方传入（单调时钟读数），
因此测试完全确定（见 ``test/test_px4_failsafe.py``）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Set, Tuple

__all__ = [
    "SignalId",
    "Severity",
    "ReasonCode",
    "SafetyState",
    "SafetyReason",
    "SafetyDecision",
    "FailsafeConfig",
    "Px4FailsafeMonitor",
    "VEHICLE_REACTION_NOTE",
    "CONTRACT_TIMING_REFERENCE",
]

#: `allow_setpoint=False` 的边界说明。放在常量里，让节点可以直接写进日志/诊断
#: 话题，避免"代码里是一套、运维看到的是另一套"。
VEHICLE_REACTION_NOTE = (
    "allow_setpoint=False 仅表示本节点停止发送 setpoint；飞行器侧的反应取决于 "
    "PX4 飞控参数（COM_OF_LOSS_T 缺省 1.0 s、COM_OBL_RC_ACT 缺省 0=Position），"
    "这些参数在本仓库之外，未经 SITL/实机验证不得据此推断飞行器行为。"
)

#: 与契约/上游对齐的时间量参考值（文档与测试断言用，不参与运行时判断）。
CONTRACT_TIMING_REFERENCE = {
    "pose_timeout_s": 0.15,              # config/contract.yaml timing.pose_timeout_s
    "depth_timeout_s": 1.0,              # config/contract.yaml timing.depth_timeout_s
    "position_cmd_period_s": 0.01,       # traj_server.cpp: 100 Hz 定时器
    "px4_offboard_loss_timeout_s": 1.0,  # PX4 COM_OF_LOSS_T 缺省
    "px4_heartbeat_period_s": 0.5,       # MAVLink 心跳标称 2 Hz
}


class SignalId(str, Enum):
    """所有输入信号。字符串枚举：日志与 JSON 诊断里直接可读。"""

    SETPOINT = "setpoint"
    VIO_POSE = "vio_pose"
    IMU = "imu"
    MAVLINK_LINK = "mavlink_link"
    PX4_HEARTBEAT = "px4_heartbeat"
    CAMERA = "camera"
    ODOM_EGO = "odom_ego"


class Severity(str, Enum):
    """原因等级。

    ``BLOCKING``：禁止发送（fail-closed）。
    ``ADVISORY``：只降级为 DEGRADED，仍然发送。
    """

    BLOCKING = "blocking"
    ADVISORY = "advisory"


class ReasonCode(str, Enum):
    """结构化原因码（诊断与计数的主键）。"""

    SIGNAL_NEVER_SEEN = "signal_never_seen"
    SIGNAL_STALE = "signal_stale"
    PX4_IDENTITY_UNKNOWN = "px4_identity_unknown"
    PLANNING_REJECTED = "planning_rejected"
    TRAJECTORY_INVALIDATED = "trajectory_invalidated"
    PX4_RESTARTED = "px4_restarted"
    RESET_REQUESTED = "reset_requested"


class SafetyState(str, Enum):
    """状态。无 HOLD_LAST：理由见模块 docstring。"""

    INIT = "INIT"
    OK = "OK"
    DEGRADED = "DEGRADED"
    STOPPED = "STOPPED"


#: 原因排序基准（同等级内按此顺序，不随数据抖动）。
_CODE_ORDER: Tuple[ReasonCode, ...] = (
    ReasonCode.SIGNAL_NEVER_SEEN,
    ReasonCode.SIGNAL_STALE,
    ReasonCode.PX4_IDENTITY_UNKNOWN,
    ReasonCode.PLANNING_REJECTED,
    ReasonCode.TRAJECTORY_INVALIDATED,
    ReasonCode.PX4_RESTARTED,
    ReasonCode.RESET_REQUESTED,
)
_CODE_RANK = {c: i for i, c in enumerate(_CODE_ORDER)}


@dataclass(frozen=True)
class SafetyReason:
    """一条触发原因：带**触发年龄 vs 限值**，便于判断"是刚擦线还是彻底断了"。"""

    code: ReasonCode
    severity: Severity
    signal: Optional[SignalId]
    age_s: float
    limit_s: Optional[float]
    detail: str = ""

    @property
    def triggered(self) -> bool:
        """是否真的越限（事件类原因 limit 为 None，恒为 True）。"""
        if self.limit_s is None:
            return True
        return float(self.age_s) > float(self.limit_s)

    def describe(self) -> str:
        tag = "BLOCK" if self.severity is Severity.BLOCKING else "ADVISORY"
        sig = self.signal.value if self.signal is not None else "-"
        age = "inf" if math.isinf(self.age_s) else f"{self.age_s:.3f}s"
        lim = "none" if self.limit_s is None else f"{self.limit_s:.3f}s"
        text = f"[{tag}] {self.code.value} signal={sig} age={age} limit={lim}"
        if self.detail:
            text += f" ({self.detail})"
        return text


@dataclass(frozen=True)
class SafetyDecision:
    """一次评估的结果。

    字段语义：
        state: 见 `SafetyState`。
        allow_setpoint: **唯一的发送许可**。False 时后端必须不发任何 setpoint。
            它只描述本节点的行为，不描述飞行器状态（见 `VEHICLE_REACTION_NOTE`）。
        reasons: 触发原因；BLOCKING 在前，同级按 `_CODE_ORDER`、再按
            `FailsafeConfig.signals` 顺序（刻意不按"最新发生"排序：那会随数据
            变化抖动，诊断日志失去可比性）。
        blocking_reasons / advisory_reasons: 分级视图。
        signal_ages_s / stale_signals: 本次评估看到的年龄与过期集合。
        recovery_streak / recovery_samples_required: 迟滞进度。
        latched_reasons: 尚未清除的故障码（含事件类），用于诊断"是否在等重新建立"。
        boot_id: 当前认定的飞控标识（None = 未知，此时不放行）。
        setpoint_version: 最近一次新鲜 setpoint 的轨迹号（None = 尚无）。
        counters: 只增不减的计数（状态迁移、每原因闭锁次数、恢复次数、累计阻塞
            评估次数、时钟异常计数）。
        note: 固定为 `VEHICLE_REACTION_NOTE`，让诊断话题直接带出边界说明。
        now_mono_s: 调用方实际传入的时钟读数（原值，便于对照）。年龄基准在时钟
            回退时会被钳制，因此不要用本字段反推"刚好经过了多少秒"。
    """

    now_mono_s: float
    state: SafetyState
    allow_setpoint: bool
    reasons: Tuple[SafetyReason, ...] = ()
    signal_ages_s: Dict[SignalId, float] = field(default_factory=dict)
    stale_signals: Tuple[SignalId, ...] = ()
    recovery_streak: int = 0
    recovery_samples_required: int = 0
    latched_reasons: Tuple[ReasonCode, ...] = ()
    signal_latched: Dict[SignalId, ReasonCode] = field(default_factory=dict)
    boot_id: Optional[str] = None
    setpoint_version: Optional[int] = None
    counters: Dict[str, object] = field(default_factory=dict)
    note: str = VEHICLE_REACTION_NOTE

    # ---- 便利只读视图 ----
    @property
    def blocking_reasons(self) -> Tuple[SafetyReason, ...]:
        return tuple(r for r in self.reasons if r.severity is Severity.BLOCKING)

    @property
    def advisory_reasons(self) -> Tuple[SafetyReason, ...]:
        return tuple(r for r in self.reasons if r.severity is Severity.ADVISORY)

    @property
    def reason_codes(self) -> Tuple[ReasonCode, ...]:
        return tuple(r.code for r in self.reasons)

    @property
    def transmit_gate_open(self) -> bool:
        """`allow_setpoint` 的同义只读视图（名字更贴近"闸门"语义）。"""
        return bool(self.allow_setpoint)

    @property
    def age_of(self) -> Dict[SignalId, float]:
        """按信号取年龄（`signal_ages_s` 的别名，语义更明确）。"""
        return dict(self.signal_ages_s)

    def describe(self) -> str:
        """单行摘要（日志用）。"""
        head = (
            f"state={self.state.value} allow_setpoint={self.allow_setpoint} "
            f"streak={self.recovery_streak}/{self.recovery_samples_required}"
        )
        if not self.reasons:
            return head + " reasons=[]"
        return head + " | " + " | ".join(r.describe() for r in self.reasons)

    def to_dict(self) -> Dict[str, object]:
        """JSON 可序列化（诊断话题用）。"""
        return {
            "now_mono_s": float(self.now_mono_s),
            "state": self.state.value,
            "allow_setpoint": bool(self.allow_setpoint),
            "transmit_gate_open": bool(self.allow_setpoint),
            "reasons": [
                {
                    "code": r.code.value,
                    "severity": r.severity.value,
                    "signal": None if r.signal is None else r.signal.value,
                    "age_s": None if math.isinf(r.age_s) else float(r.age_s),
                    "age_is_infinite": bool(math.isinf(r.age_s)),
                    "limit_s": None if r.limit_s is None else float(r.limit_s),
                    "triggered": bool(r.triggered),
                    "detail": r.detail,
                }
                for r in self.reasons
            ],
            "signal_ages_s": {
                k.value: (None if math.isinf(v) else float(v))
                for k, v in self.signal_ages_s.items()
            },
            "stale_signals": [s.value for s in self.stale_signals],
            "recovery_streak": int(self.recovery_streak),
            "recovery_samples_required": int(self.recovery_samples_required),
            "latched_reasons": [c.value for c in self.latched_reasons],
            "signal_latched": {s.value: c.value for s, c in self.signal_latched.items()},
            "boot_id": self.boot_id,
            "setpoint_version": self.setpoint_version,
            "counters": self.counters,
            "note": self.note,
        }


@dataclass(frozen=True)
class FailsafeConfig:
    """每路信号独立超时 + 迟滞参数。

    构造即校验（避免"配错了但悄悄按缺省跑"）：
        - 除 ``advisory_signals`` 之外的每个 `SignalId` 都必须有超时；
        - 必需/可选信号不得重复、不得重叠；
        - 所有超时必须是有限正数；``recovery_fresh_samples >= 1``。
    """

    timeouts: Dict[SignalId, float] = field(
        default_factory=lambda: {
            SignalId.SETPOINT: 0.15,
            SignalId.VIO_POSE: 0.15,
            SignalId.IMU: 0.10,
            SignalId.MAVLINK_LINK: 0.50,
            SignalId.PX4_HEARTBEAT: 0.50,
            SignalId.CAMERA: 0.50,
            SignalId.ODOM_EGO: 0.20,
        }
    )
    #: 过期即闭锁（fail-closed）的信号。
    required_signals: Tuple[SignalId, ...] = (
        SignalId.SETPOINT,
        SignalId.VIO_POSE,
        SignalId.IMU,
        SignalId.MAVLINK_LINK,
        SignalId.PX4_HEARTBEAT,
    )
    #: 过期只降级为 DEGRADED 的信号（可按部署需要提升为必需）。
    advisory_signals: Tuple[SignalId, ...] = (
        SignalId.CAMERA,
        SignalId.ODOM_EGO,
    )
    #: 恢复所需的连续新鲜评估次数（迟滞）。
    recovery_fresh_samples: int = 5
    #: 是否要求先看到 PX4 身份（带 boot_id 的心跳）才允许发送。
    #: 缺省 True：不知道在对谁说话时不应下指令（fail-closed）。
    require_px4_identity: bool = True

    def __post_init__(self) -> None:
        timeouts = {SignalId(k): float(v) for k, v in self.timeouts.items()}
        object.__setattr__(self, "timeouts", timeouts)
        req = tuple(SignalId(s) for s in self.required_signals)
        adv = tuple(SignalId(s) for s in self.advisory_signals)
        object.__setattr__(self, "required_signals", req)
        object.__setattr__(self, "advisory_signals", adv)

        if len(set(req)) != len(req):
            raise ValueError("required_signals 含重复项")
        if len(set(adv)) != len(adv):
            raise ValueError("advisory_signals 含重复项")
        overlap = set(req) & set(adv)
        if overlap:
            raise ValueError(f"信号不能同时是必需与可选：{sorted(s.value for s in overlap)}")
        for sig in SignalId:
            if sig in set(adv):
                continue
            if sig not in timeouts:
                raise ValueError(f"必需信号缺少超时配置：{sig.value}")
        for sig, t in timeouts.items():
            if not math.isfinite(t) or t <= 0.0:
                raise ValueError(f"timeouts[{sig.value}] 必须是有限正数，实得 {t!r}")
        if int(self.recovery_fresh_samples) < 1:
            raise ValueError("recovery_fresh_samples 至少为 1（0 意味着无迟滞，禁止）")
        object.__setattr__(self, "recovery_fresh_samples", int(self.recovery_fresh_samples))

    @property
    def signals(self) -> Tuple[SignalId, ...]:
        """原因排序基准：必需在前、可选在后，各自按 `SignalId` 定义顺序。"""
        adv = set(self.advisory_signals)
        required = tuple(s for s in SignalId if s not in adv)
        advisory = tuple(s for s in SignalId if s in adv)
        return required + advisory

    def timeout_of(self, sig: SignalId) -> float:
        return float(self.timeouts[SignalId(sig)])

    def severity_of(self, sig: SignalId) -> Severity:
        if SignalId(sig) in set(self.advisory_signals):
            return Severity.ADVISORY
        return Severity.BLOCKING


class Px4FailsafeMonitor:
    """纯状态机：每次 ``evaluate(now_mono_s)`` 输出一个 `SafetyDecision`。

    无内部线程、无 sleep、无系统时钟读取（``now_mono_s`` 由调用方传入），
    因此同样的输入序列必然产生同样的输出序列（测试锁死这一点）。
    """

    def __init__(self, config: Optional[FailsafeConfig] = None):
        self.config = config or FailsafeConfig()
        self._last_seen: Dict[SignalId, Optional[float]] = {s: None for s in SignalId}
        self._latched: Dict[ReasonCode, bool] = {}
        #: 逐路闭锁：哪一路过期/从未收到。与 `_latched` 分开维护，因为它的清除
        #: 条件是"该路重新新鲜"（不是"等连续 N 次评估"），两者语义不同。
        self._signal_latch: Dict[SignalId, ReasonCode] = {}
        self._latch_detail: Dict[ReasonCode, str] = {}
        self._reestablishment_pending = False
        self._boot_id: Optional[str] = None
        self._expected_boot_id: Optional[str] = None
        self._awaiting_new_boot_id = False
        self._restart_reestablished = False
        self._planning_rejected_active = False
        self._boot_id_when_restart_detected: Optional[str] = None
        self._restart_requires_new_traj_id: Optional[int] = None
        self._restart_new_trajectory_seen = False
        self._restart_reestablished = False
        self._latched_setpoint_version: Optional[int] = None
        self._setpoint_version: Optional[int] = None
        self._reboot_count = 0
        self._state = SafetyState.INIT
        self._ever_allowed = False
        self._streak = 0
        self._last_now: Optional[float] = None
        self._counters: Dict[str, object] = {
            "state_transitions": {s.value: 0 for s in SafetyState},
            "latch_counts": {},
            "recoveries": 0,
            "blocked_evaluations": 0,
            "evaluations": 0,
            "clock_regression_count": 0,
            "future_timestamp_count": 0,
            "reset_request_count": 0,
            "px4_reboot_count": 0,
        }

    # ------------------------------------------------------------------
    # 输入：信号新鲜度
    # ------------------------------------------------------------------
    def note_setpoint(self, now_mono_s: float, trajectory_id: int) -> None:
        """报告一条新鲜 `/position_cmd` 到达。

        ``trajectory_id`` 取消息本体（EGO 的 id 从 1 开始，0 表示无轨迹）。
        轨迹作废闭锁之后，只有**不同**的 id 才算"重新建立起一条可执行的轨迹"。
        """
        now = float(now_mono_s)
        self._last_seen[SignalId.SETPOINT] = now
        prev = self._setpoint_version
        self._setpoint_version = int(trajectory_id)
        if self._latched.get(ReasonCode.TRAJECTORY_INVALIDATED, False):
            if prev is None or self._setpoint_version != prev:
                self._reestablishment_pending = True
        if self._latched.get(ReasonCode.PX4_RESTARTED, False):
            # 重启后的 setpoint **必须换一条轨迹**：沿用同一条 id 说明上游执行器
            # 还是旧状态，"序列重新建立"并未发生。基准取闭锁时的轨迹号。
            if self._setpoint_version != self._restart_requires_new_traj_id:
                self._restart_new_trajectory_seen = True

    def note_heartbeat(self, now_mono_s: float, boot_id: str) -> None:
        """报告 PX4 心跳到达，并带上飞控重启标识。

        ``boot_id`` **变化**即视为飞控重启：即使调用方漏调 `note_px4_restart`，
        回放/断点续跑也能自动发现重启（重启会清空 PX4 的 offboard 状态机，
        所以必须重走恢复迟滞并重新建立轨迹）。
        """
        now = float(now_mono_s)
        self._last_seen[SignalId.PX4_HEARTBEAT] = now
        bid = str(boot_id)
        if self._boot_id is not None and bid != self._boot_id:
            self._reboot_count += 1
            self._counters["px4_reboot_count"] = self._reboot_count
            # 只有"本次重启尚未被闭锁"时才置位，避免重复计一次故障。
            if not self._latched.get(ReasonCode.PX4_RESTARTED, False):
                self._latch(ReasonCode.PX4_RESTARTED, f"boot_id {self._boot_id} → {bid}")
                self._boot_id_when_restart_detected = self._boot_id
                self._awaiting_new_boot_id = True
                self._restart_requires_new_traj_id = self._setpoint_version
                self._restart_new_trajectory_seen = False
            self._streak = 0
        self._boot_id = bid
        if self._awaiting_new_boot_id and bid != self._boot_id_when_restart_detected:
            # 重启后的第一帧心跳：序列的另一半（新 setpoint）另行判定。
            self._awaiting_new_boot_id = False
        if self._expected_boot_id is None:
            self._expected_boot_id = bid

    def note_vio_pose(self, now_mono_s: float) -> None:
        """报告 VIO 位姿到达（契约 `pose_timeout_s` 的判据对齐这里）。"""
        self._last_seen[SignalId.VIO_POSE] = float(now_mono_s)

    def note_imu(self, now_mono_s: float) -> None:
        """报告飞控 IMU 到达。"""
        self._last_seen[SignalId.IMU] = float(now_mono_s)

    def note_mavlink_link(self, now_mono_s: float) -> None:
        """报告 MAVLink 链路活动（粒度由后端决定：收到任意有效帧即可）。"""
        self._last_seen[SignalId.MAVLINK_LINK] = float(now_mono_s)

    def note_camera(self, now_mono_s: float) -> None:
        """报告可选相机链（配对成功的相机位姿）到达。"""
        self._last_seen[SignalId.CAMERA] = float(now_mono_s)

    def note_odom(self, now_mono_s: float) -> None:
        """报告可选里程计（`/boom_birds/vio/odom_ego`）到达。"""
        self._last_seen[SignalId.ODOM_EGO] = float(now_mono_s)

    # ------------------------------------------------------------------
    # 输入：离散事件（全部闭锁，需重新建立）
    # ------------------------------------------------------------------
    def note_planning_rejected(self, now_mono_s: float, detail: str = "") -> None:
        """规划器拒绝了当前目标（无可行轨迹）。**闭锁**，不会自愈。

        为什么闭锁："没有可执行的轨迹"不会因为时间流逝而变好；必须由调用方在
        规划器给出新轨迹后显式 ``clear_planning_rejected()``，再走连续新鲜迟滞。
        """
        self._latch(ReasonCode.PLANNING_REJECTED, detail or "规划器拒绝当前目标")
        # 规划拒绝是**外部事件**，"时间流逝"本身不构成重新建立：必须由调用方在
        # 规划器确实给出新轨迹后显式 `clear_planning_rejected()`。用这个标志把
        # 它和"信号类自动清除"区分开，避免闸门在几十帧后悄悄自己打开。
        self._planning_rejected_active = True
        self._streak = 0

    def clear_planning_rejected(self) -> None:
        """规划器重新给出可行轨迹后显式清除该故障。

        清除后仍需连续 N 次干净评估才会重新放行（`_clear_recovered()` 会清零
        迟滞计数），所以这里不需要再额外做别的。
        """
        self._planning_rejected_active = False
        self._latched.pop(ReasonCode.PLANNING_REJECTED, None)
        self._latch_detail.pop(ReasonCode.PLANNING_REJECTED, None)

    def note_trajectory_invalidated(self, now_mono_s: float, detail: str = "") -> None:
        """轨迹作废（上游 `traj_server` 停止发布 / Bspline 空消息）。**闭锁**。

        清除条件：``note_setpoint`` 带**不同**的 ``trajectory_id`` 到达之后，
        再满足连续新鲜迟滞。
        """
        self._latch(ReasonCode.TRAJECTORY_INVALIDATED, detail or "轨迹作废")
        self._latched_setpoint_version = self._setpoint_version
        self._reestablishment_pending = False
        self._streak = 0

    def note_px4_restart(self, now_mono_s: float, detail: str = "") -> None:
        """观察到 PX4 重启（显式告知；心跳 boot_id 变化也会自动触发）。**闭锁**。

        清除条件（"序列重新建立"）：**变了**的 boot_id 心跳到达，**且**收到新的
        setpoint。只要求"有一个 boot_id"是不够的——重启前的那个 boot_id 在重启后
        仍然留在状态里，会让闭锁在下一帧就被误判为已重新建立。
        """
        if not self._latched.get(ReasonCode.PX4_RESTARTED, False):
            self._reboot_count += 1
            self._counters["px4_reboot_count"] = self._reboot_count
        self._latch(ReasonCode.PX4_RESTARTED, detail or "PX4 重启")
        # 记住"重启前"的 boot_id，并要求看到一个**不同**的 boot_id：
        # 只要求"存在一个 boot_id"是不够的——重启前的那个仍然留在状态里，
        # 会让闭锁在下一帧就被误判为已重新建立。
        self._boot_id_when_restart_detected = self._boot_id
        self._awaiting_new_boot_id = True
        self._restart_requires_new_traj_id = self._setpoint_version
        self._restart_new_trajectory_seen = False
        self._restart_reestablished = False
        self._expected_boot_id = None
        self._reestablishment_pending = False
        self._streak = 0

    def request_reset(self, detail: str = "调用方要求重新建立") -> None:
        """显式要求重新建立（例如运维重新使能）：闭锁并清零迟滞进度。"""
        self._latch(ReasonCode.RESET_REQUESTED, detail)
        self._streak = 0
        self._counters["reset_request_count"] = int(self._counters["reset_request_count"]) + 1

    def clear_reset(self) -> None:
        """清除"重新建立"要求（仍需连续新鲜迟滞才会放行）。"""
        self._latched.pop(ReasonCode.RESET_REQUESTED, None)

    # ------------------------------------------------------------------
    # 评估
    # ------------------------------------------------------------------
    def evaluate(self, now_mono_s: float) -> SafetyDecision:
        """在时刻 ``now_mono_s`` 评估一次，返回 `SafetyDecision`。"""
        raw_now = float(now_mono_s)
        basis = self._clamp_clock(raw_now)
        self._counters["evaluations"] = int(self._counters["evaluations"]) + 1

        ages = self._ages(basis)
        stale = self._stale(ages)

        # 1) 先按当前新鲜度逐路闭锁（只置位，不清除）。
        self._latch_stale(stale, ages)

        # 2) 事件类闭锁的重新建立条件（不满足则**保留**闭锁）。
        self._try_reestablish()

        # 3) 原因（含 PX4_IDENTITY_UNKNOWN）。
        info_codes = {ReasonCode.SIGNAL_NEVER_SEEN, ReasonCode.PX4_IDENTITY_UNKNOWN}
        reasons = self._reasons(ages, stale)
        # 分三类：
        #   obs_hard  —— 真正的越限（某路**曾经新鲜但已过期**、事件类故障）；
        #   obs_info  —— "还没有数据 / 还不知道在跟谁说话"：不是越限，但同样不许发；
        #   advisory  —— 可选输入缺失，只降级为 DEGRADED。
        # 这个区分是**语义核心**：信息类阻塞不参与迟滞计数，否则
        # "攒够 N 次才清 never_seen" 与 "never_seen 压住进度" 会互相锁死。
        obs_stale = [
            r for r in reasons if r.severity is Severity.BLOCKING and r.code is ReasonCode.SIGNAL_STALE
        ]
        obs_event = [
            r
            for r in reasons
            if r.severity is Severity.BLOCKING
            and r.code
            in (
                ReasonCode.PLANNING_REJECTED,
                ReasonCode.TRAJECTORY_INVALIDATED,
                ReasonCode.PX4_RESTARTED,
                ReasonCode.RESET_REQUESTED,
            )
        ]
        obs_info = [
            r
            for r in reasons
            if r.severity is Severity.BLOCKING
            and r.code in (ReasonCode.SIGNAL_NEVER_SEEN, ReasonCode.PX4_IDENTITY_UNKNOWN)
        ]
        obs_blocking = obs_stale + obs_event
        advisory = [r for r in reasons if r.severity is Severity.ADVISORY]
        latched_now = self._latched_set()
        hard_latched = bool(latched_now - {ReasonCode.SIGNAL_NEVER_SEEN, ReasonCode.PX4_IDENTITY_UNKNOWN})
        never_seen_required = any(self._last_seen[s] is None for s in self.config.required_signals)

        # 4) 迟滞计数：连续"输入确实新鲜"的评估次数。**只有**某路"曾经新鲜却已过期"
        #    才会清零。语义要点：
        #      * 攒够 N 次 ⇒ 允许清除已置位的闭锁（清除会清零，需再攒一次）；
        #      * 清除之后重新从 0 攒够 N 次 ⇒ 才允许**重新武装**闸门。
        #    为什么事件类闭锁（规划拒绝/轨迹作废/PX4 重启）不清零：它们是持久状态，
        #    若压住进度就会形成"闭锁不清 -> 进度不涨 -> 闭锁不清"的死锁（PX4 重启后
        #    永远无法恢复）。放行由第 6 步的 not hard_latched 把关，闭锁期间即使
        #    进度攒满也不会发送任何 setpoint。
        #    为什么信息类闭锁（从未收到 / 不知道在跟谁说话）不清零：否则
        #    "攒够 N 次才清它"与"它压住进度"同样会互相锁死。
        if obs_stale:
            self._streak = 0
        elif self._streak < self.config.recovery_fresh_samples:
            self._streak += 1

        # 5) 清除：连续 N 次观测干净之后，才允许清除阻塞性闭锁。
        #    注意这里**不**要求 ：事件类闭锁（规划拒绝、轨迹作废、
        #    PX4 重启）是持久状态，把它算进迟滞条件会造成死锁
        #    （"闭锁压住进度 → 进度不达标 → 闭锁不清"）。放行本身由第 6 步的
        #     把关，所以事件闭锁期间绝不会发送。
        just_cleared = False
        if self._streak >= self.config.recovery_fresh_samples:
            if not self._retry_satisfied():
                # 重新建立条件未满足（例如 PX4 重启后还没换轨迹号）：
                # 保持闭锁，进度停在阈值。
                self._streak = self.config.recovery_fresh_samples
            elif hard_latched:
                # 只有**阻塞性**闭锁才需要走"清除 + 重新武装"：信息类闭锁
                # （从未收到 / 不知道在跟谁说话）不属于故障，直接失效即可。
                self._clear_recovered()
                just_cleared = True
        # 5b) "信息类"闭锁（从未收到 / 不知道在跟谁说话）在数据到齐后立即失效：
        #     它描述的是"还没有数据"，不该被当成需要重新建立的故障，也**不**
        #     消耗迟滞计数（它不参与 `_streak` 的清零，只受 `allow` 约束）。
        if not obs_info and not never_seen_required:
            for code in info_codes:
                self._drop_latch(code)
        if just_cleared:
            # 清掉了一个**阻塞性**闭锁：刚攒的 N 次干净评估被消耗掉，闸门要
            # **再**连续 N 次干净才打开。这就是"单帧新鲜不足以重新武装"的硬
            # 保证——抖动链路里每次擦线都会把计数清零，于是闸门在抖动期间
            # 始终保持关闭。
            self._streak = 0

        # 6) 闸门条件（本模块的核心语义）：
        #    allow = 没有实质越限
        #           且 没有实质闭锁
        #           且 已攒够 N 次连续的干净评估
        #           且 所有必需信号都确实收到过（不能靠"没数据"蒙混过关）
        #    任何一次新的实质越限都会清零计数（第 4 步），因此抖动链路永远
        #    攒不满 N 次；单帧恢复也永远不足以重新武装指令。
        allow = (
            (not obs_blocking)
            and (not hard_latched)
            and (not never_seen_required)
            and self._streak >= self.config.recovery_fresh_samples
        )

        if allow:
            self._ever_allowed = True
            state = SafetyState.DEGRADED if advisory else SafetyState.OK
        elif self._has_live_violation(obs_blocking) or self._ever_allowed:
            # 有实质越限（某路曾经新鲜但已过期、事件类故障），或到达过 OK
            # 之后的任何等待 ⇒ 运行中失联/恢复中，都记 STOPPED。
            state = SafetyState.STOPPED
        else:
            # 从未成功过、当前也没有越限（只是还没收到数据）⇒ 启动阶段。
            state = SafetyState.INIT
        self._transition(state)

        if not allow:
            self._counters["blocked_evaluations"] = (
                int(self._counters["blocked_evaluations"]) + 1
            )

        return SafetyDecision(
            now_mono_s=raw_now,
            state=state,
            allow_setpoint=allow,
            reasons=tuple(reasons),
            signal_ages_s={s: ages[s] for s in self.config.signals},
            stale_signals=tuple(s for s in self.config.signals if s in stale),
            recovery_streak=int(self._streak),
            recovery_samples_required=self.config.recovery_fresh_samples,
            latched_reasons=tuple(c for c in _CODE_ORDER if c in self._latched_set()),
            signal_latched={s: c for s, c in self._signal_latch.items()},
            boot_id=self._boot_id,
            setpoint_version=self._setpoint_version,
            counters=self._snapshot_counters(),
        )

    # ------------------------------------------------------------------
    # 诊断
    # ------------------------------------------------------------------
    def counters(self) -> Dict[str, object]:
        """计数快照（JSON 可序列化）。"""
        return self._snapshot_counters()

    def report(self) -> Dict[str, object]:
        """完整诊断摘要（状态、闭锁、迟滞进度、计数、超时表）。"""
        return {
            "state": self._state.value,
            "latched_reasons": [c.value for c in _CODE_ORDER if c in self._latched_set()],
            "signal_latched": {s.value: c.value for s, c in self._signal_latch.items()},
            "recovery_streak": int(self._streak),
            "recovery_samples_required": self.config.recovery_fresh_samples,
            "boot_id": self._boot_id,
            "expected_boot_id": self._expected_boot_id,
            "setpoint_version": self._setpoint_version,
            "latched_setpoint_version": self._latched_setpoint_version,
            "reestablishment_pending": bool(self._reestablishment_pending),
            "last_seen": {
                s.value: (None if t is None else float(t)) for s, t in self._last_seen.items()
            },
            "required_signals": [s.value for s in self.config.required_signals],
            "advisory_signals": [s.value for s in self.config.advisory_signals],
            "timeouts": {s.value: float(t) for s, t in self.config.timeouts.items()},
            "counters": self._snapshot_counters(),
            "note": VEHICLE_REACTION_NOTE,
        }

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------
    def _clamp_clock(self, now: float) -> float:
        """时钟回退钳制：非单调时钟不得导致"年龄为负 = 看起来更新鲜"。

        为什么不由调用方保证：ROS 时钟在 use_sim_time 切换或 bag 回放时可能回跳；
        把回跳静默当成"新鲜"是 fail-open，必须避免。

        实现：年龄基准保持在 ``max(上次读数, 本次读数)``，即回退期间**不推进**年龄
        （宁可让"过期判定"晚一点，也不允许它变新），同时计数。
        ``decision.now_mono_s`` 仍是调用方实际传入的读数，诊断不会因此说谎。
        """
        if not math.isfinite(now):
            raise ValueError(f"now_mono_s 必须有限，实得 {now!r}")
        basis = now
        if self._last_now is not None and now < self._last_now:
            self._counters["clock_regression_count"] = (
                int(self._counters["clock_regression_count"]) + 1
            )
            basis = self._last_now
        else:
            self._last_now = now
        return basis

    def _ages(self, now: float) -> Dict[SignalId, float]:
        out: Dict[SignalId, float] = {}
        for sig in SignalId:
            t = self._last_seen[sig]
            if t is None:
                out[sig] = math.inf
                continue
            age = now - t
            if age < 0.0:
                # 时间戳来自"未来"：通常是时钟问题，不是真的更新鲜。
                # 钳制为 0（乐观）但计数，避免静默。
                self._counters["future_timestamp_count"] = (
                    int(self._counters["future_timestamp_count"]) + 1
                )
                age = 0.0
            out[sig] = age
        return out

    def _stale(self, ages: Dict[SignalId, float]) -> Set[SignalId]:
        stale: Set[SignalId] = set()
        for sig in SignalId:
            if self._last_seen[sig] is None or ages[sig] > self.config.timeout_of(sig):
                stale.add(sig)
        return stale

    def _latch_stale(self, stale: Set[SignalId], ages: Dict[SignalId, float]) -> None:
        """闭锁过期的**必需**信号。

        可选信号（camera/odom_ego）**从不闭锁**：它们不参与发送许可，只让状态降到
        DEGRADED。否则"相机从没接上"会永久堵死闸门——那是把信息性输入误当安全
        输入，属于把可用性 bug 伪装成安全设计。若某部署确实要求相机在环，正确做法
        是在 `FailsafeConfig` 里把它放进 ``required_signals``（同一套语义，不另开分支）。
        """
        advisory = set(self.config.advisory_signals)
        for sig in self.config.signals:
            if sig in advisory or sig not in stale:
                continue
            if self._last_seen[sig] is None:
                code = ReasonCode.SIGNAL_NEVER_SEEN
                detail = f"{sig.value} 从未收到"
            else:
                code = ReasonCode.SIGNAL_STALE
                detail = (
                    f"{sig.value} age={ages[sig]:.3f}s > "
                    f"{self.config.timeout_of(sig):.3f}s"
                )
            # 只有"该路从未闭锁 → 闭锁"或"闭锁原因变化"才算一次新故障。
            # 持续断开时每帧都会重新观测到同一状态，不应反复计数。
            if self._signal_latch.get(sig) is not code:
                latch_counts = self._counters["latch_counts"]
                assert isinstance(latch_counts, dict)
                latch_counts[code.value] = int(latch_counts.get(code.value, 0)) + 1
            self._signal_latch[sig] = code
            self._mark_latched(code, detail)

    def _reasons(
        self, ages: Dict[SignalId, float], stale: Set[SignalId]
    ) -> List[SafetyReason]:
        """按 `_CODE_ORDER` + `FailsafeConfig.signals` 顺序生成原因列表。

        每个过期信号只生成**一条**原因（原因码由 `_latch_stale` 的判定规则决定，
        这里复用同一规则，避免"闭锁用一套、解释用另一套"）。
        """
        out: List[SafetyReason] = []
        advisory = set(self.config.advisory_signals)
        for sig in self.config.signals:
            if sig not in stale:
                continue
            never = self._last_seen[sig] is None
            if sig in advisory:
                code = ReasonCode.SIGNAL_NEVER_SEEN if never else ReasonCode.SIGNAL_STALE
            else:
                code = self._signal_latch.get(sig) or (
                    ReasonCode.SIGNAL_NEVER_SEEN if never else ReasonCode.SIGNAL_STALE
                )
            out.append(
                SafetyReason(
                    code=code,
                    severity=self.config.severity_of(sig),
                    signal=sig,
                    age_s=ages[sig],
                    limit_s=self.config.timeout_of(sig),
                    detail="从未收到该输入" if never else "",
                )
            )
        if self.config.require_px4_identity and self._boot_id is None:
            out.append(
                SafetyReason(
                    code=ReasonCode.PX4_IDENTITY_UNKNOWN,
                    severity=Severity.BLOCKING,
                    signal=SignalId.PX4_HEARTBEAT,
                    age_s=ages[SignalId.PX4_HEARTBEAT],
                    limit_s=None,
                    detail="尚无带 boot_id 的心跳：不知道在跟哪台飞控说话",
                )
            )
        for code in _CODE_ORDER:
            if code in (
                ReasonCode.SIGNAL_STALE,
                ReasonCode.SIGNAL_NEVER_SEEN,
                ReasonCode.PX4_IDENTITY_UNKNOWN,
            ):
                continue
            if self._latched.get(code, False):
                out.append(
                    SafetyReason(
                        code=code,
                        severity=Severity.BLOCKING,
                        signal=None,
                        age_s=0.0,
                        limit_s=None,
                        detail=self._latch_detail.get(code, ""),
                    )
                )

        def rank(reason: SafetyReason) -> Tuple[int, int, int]:
            severity_rank = 0 if reason.severity is Severity.BLOCKING else 1
            code_rank = _CODE_RANK[reason.code]
            if reason.signal is None:
                return (severity_rank, code_rank, -1)
            return (severity_rank, code_rank, self.config.signals.index(reason.signal))

        return sorted(out, key=rank)

    def _latch(self, code: ReasonCode, detail: str) -> None:
        """置位一个**事件类**故障并计一次"新故障"。"""
        self._mark_latched(code, detail)
        latch_counts = self._counters["latch_counts"]
        assert isinstance(latch_counts, dict)
        latch_counts[code.value] = int(latch_counts.get(code.value, 0)) + 1

    def _mark_latched(self, code: ReasonCode, detail: str) -> None:
        """只更新闭锁标记与说明，**不**计数（信号类故障每帧都会被重新观测到）。

        为什么要分开：`signal_stale` 在链路持续断开时每帧都会命中，若都计数，
        计数就退化成"评估次数"，失去诊断价值。
        """
        self._latched[code] = True
        self._latch_detail[code] = str(detail)

    def _latched_set(self) -> Set[ReasonCode]:
        """当前**有效**的闭锁集合。

        ``signal_never_seen`` / ``signal_stale`` 是逐路状态：只要还有任一路处于
        该状态，码就有效；全部恢复新鲜后，``_latched`` 里残留的标记不再算有效
        （残留历史用于诊断计数，不参与放行判断）。
        """
        codes = {c for c, on in self._latched.items() if on}
        if not self._signal_latch:
            # 逐路闭锁已全部清除：`_latched` 里残留的信号类标记不再算有效
            # （残留只用于诊断计数，不参与放行判断）。
            codes -= {ReasonCode.SIGNAL_STALE, ReasonCode.SIGNAL_NEVER_SEEN}
        return codes

    def _try_reestablish(self) -> None:
        """事件类故障的"重新建立"条件；满足时清除对应闭锁。

        语义：``signal_stale`` / ``signal_never_seen`` 是**逐路**闭锁（哪路重新
        新鲜就清哪路），其余（规划拒绝 / 轨迹作废 / PX4 重启 / 重新建立要求）
        必须显式重新建立——时间流逝本身不算重新建立。
        """
        if self._latched.get(ReasonCode.PX4_RESTARTED, False):
            # 判据统一放在 `_retry_satisfied()`：需要看到**变了**的 boot_id 心跳
            # 与一条**换了轨迹号**的新 setpoint，才认为 offboard 序列重新建立。
            # 并且**必须迟滞达标**才真正清除：重新建立是必要条件，不是放行许可；
            # 少了这道闸，重启后第一条新轨迹就会立刻开闸（"单帧重新武装"）。
            if self._restart_reestablished:
                if self._streak >= self.config.recovery_fresh_samples:
                    self._clear_event(ReasonCode.PX4_RESTARTED)
                    self._restart_reestablished = False
                    self._restart_new_trajectory_seen = False
            else:
                self._restart_reestablished = False
        if self._latched.get(ReasonCode.TRAJECTORY_INVALIDATED, False):
            if self._reestablishment_pending:
                self._clear_event(ReasonCode.TRAJECTORY_INVALIDATED)
                self._reestablishment_pending = False

    def _clear_event(self, code: ReasonCode) -> None:
        """清除一个**阻塞性**事件闭锁，并计一次"从闭锁恢复"。

        统一在这里计数（而不是在 `_clear_recovered` 里按集合推断），
        这样 `recoveries` 的含义始终是"从阻塞性故障回到可发送状态"的次数，
        与清除路径无关。
        """
        if self._latched.pop(code, None) is None:
            return
        self._latch_detail.pop(code, None)
        # 清除消耗掉迟滞进度：闸门要重新攒够 N 次干净评估才会打开。
        self._streak = 0
        self._counters["recoveries"] = int(self._counters["recoveries"]) + 1

    def _drop_latch(self, code: ReasonCode) -> None:
        """无条件清除一个闭锁标记（含逐路记录）。"""
        self._latched.pop(code, None)
        self._latch_detail.pop(code, None)
        if code in (ReasonCode.SIGNAL_STALE, ReasonCode.SIGNAL_NEVER_SEEN):
            for sig in [s for s, c in self._signal_latch.items() if c is code]:
                self._signal_latch.pop(sig, None)

    def _retry_satisfied(self) -> bool:
        """事件类故障的"重新建立"条件是否满足（不满足则闭锁不可被迟滞清除）。

        * PX4 重启：需要重启后的心跳（boot_id 与闭锁后记录的一致）+ 新的 setpoint；
        * 轨迹作废：需要一条 **不同** ``trajectory_id`` 的新轨迹。
        """
        if self._latched.get(ReasonCode.PLANNING_REJECTED, False) or self._planning_rejected_active:
            # 只有显式 clear 才算重新建立（规划器给出新轨迹是外部事件，
            # 本模块无法从输入新鲜度推断它）。
            if self._planning_rejected_active:
                return False
        if self._latched.get(ReasonCode.PX4_RESTARTED, False):
            if self._awaiting_new_boot_id or not self._restart_new_trajectory_seen:
                return False
            # 条件已满足：由 `_try_reestablish()` 在**迟滞达标后**消费这个标志。
            # 不要在这里直接清除，否则重启后第一条新轨迹就会立刻开闸。
            self._restart_reestablished = True
        if self._latched.get(ReasonCode.TRAJECTORY_INVALIDATED, False):
            # 判据是"换了一条轨迹"，直接比较 id，而不只依赖 note_setpoint 里设的
            # 一次性标志：标志可能被上一轮的清除顺手抹掉，那样旧 id 就会被误放行。
            if self._setpoint_version is None:
                return False
            if self._setpoint_version == self._latched_setpoint_version:
                return False
        return True

    def _clear_recovered(self) -> None:
        """连续 N 次新鲜后清除**全部**闭锁并计一次恢复。

        ``recoveries`` 只在**确实清除过闭锁**时递增：否则"一直正常"的运行会被
        每次评估都记一次恢复，诊断计数就失去意义。
        """
        latched = self._latched_set()
        if not latched:
            return
        blocking = latched - {ReasonCode.SIGNAL_NEVER_SEEN, ReasonCode.PX4_IDENTITY_UNKNOWN}
        for code in list(latched):
            self._drop_latch(code)
        if blocking:
            self._counters["recoveries"] = int(self._counters["recoveries"]) + 1
        self._signal_latch.clear()
        self._reestablishment_pending = False
        self._planning_rejected_active = False
        self._restart_reestablished = False
        self._restart_new_trajectory_seen = False
        self._latched_setpoint_version = self._setpoint_version
        self._expected_boot_id = self._boot_id
        # 清除闭锁会消耗掉刚攒够的 N 次干净评估：闸门要**再**连续 N 次干净才打开。
        # 这是"单帧新鲜不足以重新武装"的落点——抖动链路每次擦线都会清掉进度。
        self._streak = 0

    def _has_live_violation(self, blocking: List[SafetyReason]) -> bool:
        """是否存在"实质越限"，用于区分 STOPPED 与 INIT。

        规则：
        * 事件类故障（规划拒绝/轨迹作废/PX4 重启/重新建立要求）⇒ 实质越限；
        * 某路**曾经新鲜但已过期**（``signal_stale``）⇒ 实质越限（运行中失联）；
        * 某路**从未收到**（``signal_never_seen``）⇒ 不算越限，属于"还没接上"，
          在从未放行过时应该是 INIT 而不是 STOPPED。
        """
        for reason in blocking:
            if reason.code in (ReasonCode.SIGNAL_NEVER_SEEN, ReasonCode.PX4_IDENTITY_UNKNOWN):
                continue
            return True
        return False

    def _transition(self, new_state: SafetyState) -> None:
        if new_state is self._state:
            return
        self._state = new_state
        transitions = self._counters["state_transitions"]
        assert isinstance(transitions, dict)
        transitions[new_state.value] = int(transitions.get(new_state.value, 0)) + 1

    def _snapshot_counters(self) -> Dict[str, object]:
        out: Dict[str, object] = {}
        for k, v in self._counters.items():
            out[k] = dict(v) if isinstance(v, dict) else v
        return out
