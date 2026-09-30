"""任务状态机；输入已观测状态，输出交给 PX4 接口的动作。

职责边界（**写进代码的区分**）
==============================

* 本模块**只编排**：读一份 ``Observation``，输出一组动作名。动作由
  ``lifecycle_node`` 执行，且只通过 PX4 接口服务（``VehicleAction``）请求
  解锁/起飞/模式，或向规划器发 ``PlannerRequest``。编排器**绝不**自己发模式
  命令、不碰 MAVLink、不输出 PWM/DShot。
* 失效的**最终**判断不在这里：``px4_failsafe`` 决定"这一帧允不允许发
  setpoint"，``px4_interface_node`` 把结果作为 ``ExecutionStatus`` 回读。
  本模块只**消费**那份回读（``sending`` / ``sensors_ready`` / ``offboard`` /
  ``mode_detail``），不另写一套互相矛盾的失效判断。
* 无 ROS、无 I/O、无系统时钟：``now`` 与 ``Observation`` 由调用方传入，
  因此测试完全确定。

状态机
======

::

    IDLE ─start()─▶ PRECHECK ─▶ TAKEOFF ─▶ HOLD_READY ─▶ OFFBOARD_PENDING
                                                              │ 模式回读确认
                                                              ▼
        COMPLETE ◀── LANDING ◀── land() ── EXECUTING ──故障──▶ RECOVERING
                                            ▲                    │
                                            └──回读确认 + 重新规划─┘
    任意阶段 ──故障/取消/重启/坐标系重置──▶ FAULT_LATCHED（终态，不自动解锁）

``RECOVERING`` 缺省是**不可达状态**：``RuntimeConfig.recovery_enabled``
缺省 ``False``（真实设备配置默认禁用），只有 SIH 任务入口显式打开才可能进入。

锁点（HOLD_READY 之前的 hold 定位）
==================================

位姿新鲜 + **达到起飞高度**（AGL，基准是 PRECHECK 记录的地面 z）+ 连续
``stable_duration_s`` 内速度 ≤ ``stable_speed_m_s``、位置波动 ≤
``stable_radius_m``。“达到高度”只是开始累积窗口的前提：**不得用首次越过高度的
样本**，整段窗口都要满足速度与波动约束。

自动恢复（B）——语义要点
========================

1. 只有 ``recovery_enabled=True`` 且故障属于 ``RECOVERABLE_FAULTS`` 才可能进入
   ``RECOVERING``；故障码与来源（``mode_detail`` 等）逐条保留。
2. 进入前必须同时满足：未人工取消、仍解锁、**明确在空中**
   （MAV_LANDED_STATE = air/takeoff）。观测缺失或过期即拒绝（fail-closed）。
3. 进入后按故障瞬间位置、速度生成加速度有界的制动段，
   并且只通过既有执行许可（``Observation.sending``，即 px4_failsafe 的放行结果）
   才请求 OFFBOARD；请求后必须看到 ``mode_detail`` **回读确认**，命令被接受不算
   确认。只中断 PX4 自主进入的 ``auto:land`` / ``auto:rtl``，不泛化为任意模式争抢。
4. 定位/IMU/深度/地图/坐标对齐必须**连续** ``recovery_valid_duration_s`` 有效；
   相对起飞地面高度 ≥ ``recovery_min_altitude_agl_m``、速度 ≤
   ``recovery_max_speed_m_s``。
5. 每次故障最多 ``recovery_attempts`` 次尝试，每次尝试 ``mode_timeout_s`` 内必须走完
   「条件就绪 → 请求 OFFBOARD → 模式回读确认」；耗尽后闭锁，**不循环争抢、不自动
   重新解锁**。恢复后连续有效 1 s 才关闭该故障事件，随后新故障使用新预算。
6. 回读确认后重新规划剩余目标：动作给出 ``replan``（旧轨迹退役，同一
   ``trajectory_id`` 不再可能续播）与 ``enable_planner``，**禁止续播旧轨迹**。
7. 人工取消/人工模式干预/未知故障/落地/重启/坐标系重置：立即闭锁，并永久撤销
   本次任务的恢复资格（``recovery_revoked``）。
"""
from collections import deque
from dataclasses import dataclass
from enum import Enum
import math

from boom_birds_control.runtime_config import DEFAULTS


class State(str, Enum):
    IDLE = "IDLE"
    PRECHECK = "PRECHECK"
    TAKEOFF = "TAKEOFF"
    HOLD_READY = "HOLD_READY"
    OFFBOARD_PENDING = "OFFBOARD_PENDING"
    EXECUTING = "EXECUTING"
    RECOVERING = "RECOVERING"
    LANDING = "LANDING"
    COMPLETE = "COMPLETE"
    FAULT_LATCHED = "FAULT_LATCHED"


#: 允许自动恢复的**已识别故障类别**。其余故障一律闭锁（fail-closed）。
RECOVERABLE_FAULTS = (
    "sensor_link",      # 定位/IMU/深度/里程计断流（px4_failsafe 输入状态提取）
    "planning_link",    # 规划/地图链路断流
    "setpoint_link",    # setpoint 流停发
    "offboard_lost",    # 显式报告的 Offboard 链路故障
    "px4_auto_land",    # PX4 自主 AUTO Land（可被**有界**中断）
    "px4_auto_return",  # PX4 自主 AUTO Return/RTL（可被**有界**中断）
)

#: 永久撤销本次恢复资格的路径：人工取消 / 人工模式干预 / 未知故障 / 落地 /
#: 重启 / 坐标系重置。命中任意一条 ⇒ 闭锁，且不再有恢复机会。
REVOKING_FAULTS = (
    "manual_cancel", "manual_mode", "unknown", "landed_or_disarmed",
    "flight_controller_restart", "session_changed", "frame_reset",
    "geometry_changed",
)

#: PX4 自主进入且允许被**有界**中断的 AUTO 模式。只识别这两个：其余任何已观测到
#: 的非 offboard 模式都算人工/未知干预，不得泛化为“任意模式争抢”。
INTERRUPTIBLE_AUTO_MODES = {
    "auto:land": "px4_auto_land",
    "auto:rtl": "px4_auto_return",
}

#: 模式名缺失或无法识别时，不得确认 Offboard。
UNKNOWN_MODE_DETAILS = ("", "unknown")

#: 期望的 OFFBOARD 模式名（完整模式名，见 px4_backend.px4_custom_sub_mode_name）。
OFFBOARD_MODE_DETAIL = "offboard"

#: ``OFFBOARD_PENDING`` 阶段 PX4 可能仍停留的、由本任务自己请求过的模式。
PENDING_EXPECTED_MODES = ("auto:takeoff", "auto:loiter")

#: 明确在空中的 MAV_LANDED_STATE 取值：2=air、3=takeoff(上升中)。
AIRBORNE_LANDED_STATES = (2, 3, 4)


def normalize_mode_detail(value) -> str:
    """模式名归一化（大小写/空白不敏感）。"""
    return str(value or "").strip().lower()


def mode_detail_known(value) -> bool:
    """模式回读是否给出了一个**已知模式名**（``unknown``/空串不算）。"""
    return normalize_mode_detail(value) not in UNKNOWN_MODE_DETAILS


def offboard_observed(observation) -> bool:
    """OFFBOARD 是否被**实际观测**确认。

    完整模式名来自 PX4 HEARTBEAT。``offboard`` 布尔另受心跳年龄门限约束，
    可能在模式名仍为 offboard 时短暂变假；此处只按完整模式名确认。
    """
    detail = normalize_mode_detail(observation.mode_detail)
    return detail == OFFBOARD_MODE_DETAIL


@dataclass(frozen=True)
class Observation:
    session: str = ""
    connected: bool = False
    armed: bool = False
    landed: int | None = None  # MAV_LANDED_STATE: 1 ground, 2 air, 3 takeoff, 4 landing
    status_age: float = math.inf
    pose_age: float = math.inf
    position: tuple = (math.nan, math.nan, math.nan)
    velocity: tuple = (math.nan, math.nan, math.nan)
    offboard: bool = False
    mode: str = "unknown"
    #: 实际观测到的完整模式名（AUTO 下带子模式）：auto:land / auto:rtl / offboard /
    #: auto:loiter / position / manual / unknown。只报主模式 ``mode`` 区分不了
    #: AUTO Land 与 AUTO Return，恢复判定与验收都按这一项。
    mode_detail: str = ""
    current_mode_detail: str = ""
    intended_mode_detail: str = ""
    current_mode_age_s: float = math.inf
    px4_safety_mode: str = ""
    px4_failsafe_cause: str = ""
    px4_safety_age_s: float = math.inf
    frame_reset_known: bool = False
    frame_reset_age_s: float = math.inf
    boot_epoch: int = 0
    alignment: bool = False
    map_ready: bool = False
    sensors_ready: bool = False  # 从 px4_failsafe 的输入状态提取，不重新判断传感器失效
    sending: bool = False
    fault: str = ""


class StableWindow:
    """连续稳定性窗口：位姿新鲜 + 达到起飞高度 + 速度/位置波动受限。

    * **不得用首次越过高度的样本**：到达高度只是开始累积窗口的前提，必须整段
      ``stable_duration_s`` 都满足速度与位置波动约束才返回 True。
    * 高度基准是 PRECHECK 记录的**地面 z**（AGL），不是 0，也不是首次观测高度。
    * 高度下限取 ``takeoff_altitude_agl_m``（任务起飞高度，必须达到）与
      ``hold_lock_min_altitude_agl_m``（锁定起点设定点的下限）中更严的一个：
      两个字段都被引用，且都不放宽。
    """

    def __init__(self, config=DEFAULTS):
        self.config = config
        self.samples = deque()

    @property
    def min_altitude_agl_m(self) -> float:
        return max(self.config.takeoff_altitude_agl_m, self.config.hold_lock_min_altitude_agl_m)

    def clear(self):
        self.samples.clear()

    def update(self, now, observation, ground_z):
        c = self.config
        p, v = observation.position, observation.velocity
        valid = (math.isfinite(now) and math.isfinite(ground_z) and
                 0.0 <= observation.pose_age <= c.pose_timeout_s and
                 all(math.isfinite(x) for x in (*p, *v)) and
                 p[2] - ground_z >= self.min_altitude_agl_m and
                 math.hypot(*v) <= c.stable_speed_m_s)
        if not valid:
            self.clear()
            return False
        if self.samples and (now <= self.samples[-1][0] or now - self.samples[-1][0] > c.pose_timeout_s):
            self.clear()
        # 点间最大距离约束覆盖整个窗口，不能只比较最后一个位置。
        if any(math.dist(p, old) > c.stable_radius_m for _, old in self.samples):
            self.clear()
        self.samples.append((now, p))
        ready = now - self.samples[0][0] >= c.stable_duration_s - 1e-9
        while len(self.samples) > 1 and now - self.samples[1][0] >= c.stable_duration_s:
            self.samples.popleft()
        return ready


class Lifecycle:
    def __init__(self, config=DEFAULTS):
        self.config = config
        self.state = State.IDLE
        self.session = ""
        self.goal = None
        self.hold = None
        self.ground_z = None
        self.boot_epoch = None
        self.entered = 0.
        self.reason = ""
        self.cancelled = False
        self.stable = StableWindow(config)
        self.takeoff_requested = False
        self.goal_since = None
        self.goal_reached = False
        self.planner_started = False
        # ---- 自动恢复（B）诊断/预算状态 ----
        self.recovery_fault = ""          # 触发恢复的故障码
        self.recovery_source = ""         # 故障来源（status.fault / mode_detail=...）
        self.recovery_attempts = 0        # 当前故障事件的尝试次数
        self.recovery_total_attempts = 0
        self.recovery_healthy_since = None
        self.events = deque(maxlen=128)
        self.recovery_revoked = False     # 恢复资格是否被永久撤销
        self.recovery_detail = ""         # waiting:*/offboard_requested/confirmed/exhausted/revoked:*
        self.recovery_hold = None         # 制动起点
        self.recovery_brake_velocity = (0., 0., 0.)
        self.recovery_brake_started = None
        self.recovery_valid_since = None  # 连续有效性窗口起点
        self.recovery_window_reset = ""   # 最近一次窗口清零的原因（诊断用）
        self.recovery_attempt_started = None
        self.recovery_request_at = None   # 本次尝试请求 OFFBOARD 的时刻

    # ------------------------------------------------------------------
    # 会话与终态
    # ------------------------------------------------------------------
    def transition(self, state, now, reason=""):
        self.state, self.entered, self.reason = state, now, reason
        self.events.append(dict(state=state.value, at=now, reason=reason, detail=self.recovery_detail))
        self.stable.clear()

    def start(self, session, goal, now):
        if self.state not in (State.IDLE, State.COMPLETE) or not session or len(goal) != 3 or not all(math.isfinite(x) for x in (*goal, now)):
            return False
        self.session, self.goal = session, tuple(goal)
        self.cancelled = False
        self.ground_z = self.boot_epoch = None
        self.takeoff_requested = self.planner_started = False
        self.hold = self.goal_since = None
        self.goal_reached = False
        self._reset_recovery()
        self.events.clear()
        self.transition(State.PRECHECK, now)
        return True

    def hold_here(self, observation):
        """把保持/制动点设到**当前观测位置**。

        用途：规划流停止（轨迹走完或规划器收工）后，编排器必须自己维持一个
        受约束的保持段。否则 setpoint 会断流，先触发链路故障判定，把本来已经
        到点的任务闭锁降落（SIH 实测：飞到目标半径附近后触发 setpoint_link）。
        与恢复期的保持段同一语义：位置=当前观测位置，速度/加速度由发布端置 0。
        """
        p = tuple(observation.position)
        if len(p) != 3 or not all(math.isfinite(v) for v in p):
            return False
        self.hold = p
        return True

    def hold_at(self, point):
        """把保持/制动点设到**指定点**（要求有限）。

        用于"规划流已结束、但离目标只差一点"的收尾：把保持点放在目标上，
        飞控会把最后一段收敛掉；否则保持点在当前位置，飞行器会永久停在
        离目标一步之遥的地方（SIH 实测：最近 0.379 m 而容差 0.3 m）。
        """
        p = tuple(point)
        if len(p) != 3 or not all(math.isfinite(v) for v in p):
            return False
        self.hold = p
        return True

    def _reset_recovery(self):
        self.recovery_fault = self.recovery_source = self.recovery_detail = ""
        self.recovery_attempts = self.recovery_total_attempts = 0
        self.recovery_healthy_since = None
        self.recovery_revoked = False
        self.recovery_hold = None
        self.recovery_brake_velocity = (0., 0., 0.)
        self.recovery_brake_started = None
        self.recovery_valid_since = None
        self.recovery_attempt_started = None
        self.recovery_request_at = None

    def _revoke(self, reason):
        """永久撤销本次任务的恢复资格（人工/未知/落地/重启/坐标系重置…）。"""
        self.recovery_revoked = True
        self.recovery_detail = "revoked:" + reason if reason else "revoked"
        self.recovery_valid_since = None
        self.recovery_request_at = None

    def latch(self, now, reason, detail=None):
        """闭锁并**永久撤销**恢复资格；``detail`` 可覆盖撤销说明（诊断用）。"""
        self._revoke(reason)
        if detail is not None:
            self.recovery_detail = detail
        self.transition(State.FAULT_LATCHED, now, reason)
        return ["disable_planner", "cancel"]

    def cancel(self, now):
        self.cancelled = True
        return self.latch(now, "manual_cancel")

    def land(self, now):
        self.cancelled = True
        self._revoke("landed")
        self.transition(State.LANDING, now)
        return ["disable_planner", "cancel", "land"]

    # ------------------------------------------------------------------
    # 自动恢复：资格、前置条件、尝试预算
    # ------------------------------------------------------------------
    @property
    def recovery_exhausted(self) -> bool:
        """尝试预算是否已耗尽（耗尽即闭锁，不自动解锁）。"""
        return self.recovery_attempts >= self.config.recovery_attempts

    def recovery_available(self, fault) -> bool:
        """该故障是否有资格走自动恢复（开关/撤销/类别/预算四道门）。"""
        return (bool(self.config.recovery_enabled) and not self.recovery_revoked and
                fault in RECOVERABLE_FAULTS and not self.recovery_exhausted)

    def recovery_preconditions(self, observation):
        """进入恢复**之前**必须同时满足的条件（观测缺失/过期即拒绝）。"""
        c = self.config
        if self.cancelled:
            return False, "cancelled"
        if not observation.connected:
            return False, "disconnected"
        if not observation.frame_reset_known or not 0 <= observation.frame_reset_age_s <= c.status_timeout_s:
            return False, "reset_observation_missing"
        if not observation.armed:
            return False, "disarmed"
        if observation.landed not in AIRBORNE_LANDED_STATES:
            return False, "not_airborne"
        if not 0.0 <= observation.status_age <= c.status_timeout_s:
            return False, "status_stale"
        if not 0.0 <= observation.pose_age <= c.pose_timeout_s:
            return False, "pose_stale"
        if not all(math.isfinite(x) for x in (*observation.position, *observation.velocity)):
            return False, "position_unknown"
        return True, ""

    def _begin_recovery_attempt(self, now):
        self.recovery_attempts += 1
        self.recovery_total_attempts += 1
        self.recovery_healthy_since = None
        self.recovery_attempt_started = now
        self.recovery_request_at = None
        self.recovery_detail = "attempt_started"

    def _recovery_window_expired(self, now) -> bool:
        """本次尝试是否超时（未请求时是“等条件”，请求后是“等模式回读”）。"""
        c = self.config
        if self.recovery_request_at is not None:
            return now - self.recovery_request_at >= c.mode_timeout_s
        if self.recovery_attempt_started is None:
            return False
        return now - self.recovery_attempt_started >= c.mode_timeout_s

    def _expire_recovery_attempt(self, now):
        """一次尝试用尽：预算还有就重试，没有就闭锁。返回动作或 None（已重试）。"""
        if self.recovery_exhausted:
            return self.latch(now, self.recovery_fault, detail="exhausted")
        self._begin_recovery_attempt(now)
        self.recovery_detail = "attempt_expired"
        return None

    def _observations_fresh(self, observation) -> bool:
        c = self.config
        return (0.0 <= observation.status_age <= c.status_timeout_s and
                0.0 <= observation.pose_age <= c.pose_timeout_s)

    def _recovery_input_blockers(self, observation) -> list:
        """逐项说明"连续有效"窗口为什么没成立（诊断用，不改变判定）。"""
        c = self.config
        why = []
        if not observation.sensors_ready:
            why.append("sensors_not_ready")
        if not observation.map_ready:
            why.append("map_not_ready")
        if not observation.alignment:
            why.append("alignment_invalid")
        if not (0.0 <= observation.status_age <= c.status_timeout_s):
            why.append(f"status_age={observation.status_age:.2f}")
        if not (0.0 <= observation.pose_age <= c.pose_timeout_s):
            why.append(f"pose_age={observation.pose_age:.2f}")
        if not all(math.isfinite(x) for x in (*observation.position, *observation.velocity)):
            why.append("position_not_finite")
        return why

    def _recovery_inputs_valid(self, observation) -> bool:
        """定位/IMU/深度/地图/坐标对齐是否**当前**全部有效（连续性由调用方累积）。"""
        return not self._recovery_input_blockers(observation)

    def _recovery_ready(self, now, observation):
        """恢复接管条件是否全部满足；返回 (是否就绪, 阻塞项列表)。"""
        c = self.config
        blockers = []
        ok, why = self.recovery_preconditions(observation)
        if not ok:
            blockers.append(why)
        input_why = self._recovery_input_blockers(observation)
        if input_why:
            blockers.extend(input_why)
        if (self.recovery_valid_since is None or
                now - self.recovery_valid_since + 1e-9 < c.recovery_valid_duration_s):
            blockers.append("valid_duration_not_met")
            if self.recovery_window_reset:
                blockers.append("window_reset_by:" + self.recovery_window_reset)
        if not observation.sending:
            blockers.append("execution_gate_closed")
        if self.recovery_brake_started is not None:
            speed_at_fault = math.sqrt(sum(v * v for v in self.recovery_brake_velocity))
            if now - self.recovery_brake_started < speed_at_fault / c.recovery_brake_accel_m_s2:
                blockers.append("braking")
        if self.ground_z is None:
            blockers.append("no_ground_reference")
        elif all(math.isfinite(x) for x in (*observation.position, *observation.velocity)):
            if observation.position[2] - self.ground_z < c.recovery_min_altitude_agl_m:
                blockers.append("below_min_altitude")
            if math.hypot(*observation.velocity) > c.recovery_max_speed_m_s:
                blockers.append("above_max_speed")
        else:
            blockers.append("position_unknown")
        if observation.mode_detail in INTERRUPTIBLE_AUTO_MODES:
            if not (0 <= observation.px4_safety_age_s <= c.status_timeout_s
                    and observation.px4_safety_mode == observation.mode_detail
                    and observation.px4_failsafe_cause == "offboard_link"):
                blockers.append("px4_failsafe_cause_not_verified")
        return (not blockers), blockers

    def automatic_mode_authorized(self, observation):
        return (0 <= observation.current_mode_age_s <= self.config.status_timeout_s
                and observation.current_mode_detail == observation.mode_detail
                and observation.intended_mode_detail == OFFBOARD_MODE_DETAIL)

    def inflight_fault(self, observation) -> str:
        """模式失效优先；AUTO Land/Return 必须伴随已识别的链路故障。"""
        detail = normalize_mode_detail(observation.mode_detail)
        if detail == "unknown":
            return "unknown"
        if (self.state == State.EXECUTING and observation.px4_safety_mode == detail
                and 0 <= observation.px4_safety_age_s <= self.config.status_timeout_s
                and observation.px4_failsafe_cause == "unknown"):
            return "unknown"
        if detail in INTERRUPTIBLE_AUTO_MODES:
            if not self.automatic_mode_authorized(observation):
                return "manual_mode"
            if observation.fault in ("sensor_link", "planning_link", "setpoint_link"):
                return INTERRUPTIBLE_AUTO_MODES[detail]
            return "manual_mode"
        if detail and detail != OFFBOARD_MODE_DETAIL:
            if self.state == State.OFFBOARD_PENDING and detail in PENDING_EXPECTED_MODES:
                return ""
            return "manual_mode"
        if observation.fault:
            return observation.fault
        if not offboard_observed(observation):
            if self.state == State.OFFBOARD_PENDING:
                return ""
            return "unknown" if not detail else "offboard_lost"
        return ""

    @staticmethod
    def _fault_source(observation, fault) -> str:
        if observation.fault == fault:
            return "status.fault"
        detail = normalize_mode_detail(observation.mode_detail)
        return f"status.mode_detail={detail}" if detail else "status.offboard"

    def _start_recovery_or_latch(self, now, observation, fault):
        """已识别故障：满足全部条件才进入 RECOVERING，否则立即闭锁（fail-closed）。"""
        if not self.recovery_available(fault):
            if self.recovery_exhausted and self.config.recovery_enabled and fault in RECOVERABLE_FAULTS:
                return self.latch(now, fault, detail="exhausted")
            return self.latch(now, fault)
        ok, why = self.recovery_preconditions(observation)
        wait_for_sensor = (why == "pose_stale" and observation.fault == "sensor_link"
                           and normalize_mode_detail(observation.mode_detail) == OFFBOARD_MODE_DETAIL
                           and math.isfinite(observation.pose_age) and observation.pose_age >= 0
                           and all(math.isfinite(x) for x in (*observation.position, *observation.velocity)))
        if not ok and not wait_for_sensor:
            return self.latch(now, fault, detail=f"rejected:{why}")
        return self.begin_recovery(now, observation, fault)

    def inflight_fault_actions(self, now, observation, fault):
        """外部观测到的执行期故障（例如规划器作废轨迹）→ 动作列表。

        与 ``inflight_fault`` 同源：走同一套"可恢复类别 / 撤销 / 预算"判定，
        不另写第二套失效判断。规划类故障（``planning_link``）属于提示词 B2 明确
        允许恢复的类别，因此这里不会立即永久闭锁，而是先尝试有界恢复。
        """
        return self._start_recovery_or_latch(now, observation, fault)

    def begin_recovery(self, now, observation, fault):
        """进入 RECOVERING：保留故障来源和制动初始状态，旧轨迹退役。"""
        self.recovery_fault = fault
        self.recovery_source = self._fault_source(observation, fault)
        self.recovery_hold = tuple(observation.position)
        self.recovery_brake_velocity = tuple(observation.velocity)
        self.recovery_brake_started = now if self._observations_fresh(observation) else None
        self.recovery_valid_since = None
        self.recovery_request_at = None
        self._begin_recovery_attempt(now)
        self.transition(State.RECOVERING, now, fault)
        # "recover"：旧轨迹立即退役（同 trajectory_id 不再可能续播）+ 停发规划流。
        return ["disable_planner", "recover"] + (["hold_setpoint"] if self._observations_fresh(observation) else [])

    def _revoke_reason(self, observation) -> str:
        """恢复期间任一永久撤销条件；空串表示没有命中。"""
        if self.cancelled:
            return "manual_cancel"
        if observation.fault in REVOKING_FAULTS:
            return observation.fault
        if not observation.armed or observation.landed == 1:
            return "landed_or_disarmed"
        detail = normalize_mode_detail(observation.mode_detail)
        if detail == "unknown":
            return "unknown"
        if (0 <= observation.px4_safety_age_s <= self.config.status_timeout_s
                and observation.px4_safety_mode == detail
                and observation.px4_failsafe_cause not in ("", "offboard_link")):
            return "unknown"
        if mode_detail_known(detail) and detail != OFFBOARD_MODE_DETAIL:
            if (detail not in INTERRUPTIBLE_AUTO_MODES
                    or not self.automatic_mode_authorized(observation)
                    or self.recovery_fault not in RECOVERABLE_FAULTS):
                return "manual_mode"
        return ""

    def _confirm_recovery(self, now, observation):
        """模式回读确认：重新规划剩余目标，禁止续播旧轨迹。"""
        self.recovery_detail = "confirmed"
        self.goal_since = None
        self.planner_started = False
        # 恢复确认后，保持点取本次观测位置。
        self.hold = tuple(observation.position)
        self.transition(State.EXECUTING, now, self.recovery_fault)
        # "replan"：旧轨迹退役；"enable_planner"：按剩余目标重新规划。
        return ["hold_setpoint", "replan", "enable_planner"]

    def _tick_recovering(self, now, observation):
        revoke = self._revoke_reason(observation)
        if revoke:
            return self.latch(now, revoke)
        if self._recovery_window_expired(now):
            expired = self._expire_recovery_attempt(now)
            if expired is not None:
                return expired
        # 连续有效性窗口：任一输入失效即清零重来。
        input_why = self._recovery_input_blockers(observation)
        if not input_why:
            if self.recovery_valid_since is None:
                self.recovery_valid_since = now
        else:
            # 窗口被清零：记下是哪一路在这一刻不成立，否则只会看到
            # "valid_duration_not_met" 而查不出是谁在抖动。
            self.recovery_valid_since = None
            self.recovery_window_reset = ",".join(input_why)
        fresh = self._observations_fresh(observation)
        if fresh and self.recovery_brake_started is None:
            self.recovery_hold = tuple(observation.position)
            self.recovery_brake_velocity = tuple(observation.velocity)
            self.recovery_brake_started = now
        actions = ["hold_setpoint"] if fresh else []
        ready, blockers = self._recovery_ready(now, observation)
        if self.recovery_request_at is not None and offboard_observed(observation):
            if not ready:
                return self.latch(now, "recovery_conditions_lost", detail=",".join(blockers))
            return self._confirm_recovery(now, observation)
        if not ready:
            self.recovery_detail = "waiting:" + ",".join(blockers)
            return actions
        if self.recovery_request_at is None:
            self.recovery_request_at = now
            self.recovery_detail = "offboard_requested"
            return actions + ["offboard"]
        self.recovery_detail = "awaiting_mode_readback"
        return actions

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------
    def tick(self, now, o):
        c = self.config
        if self.state in (State.IDLE, State.COMPLETE, State.FAULT_LATCHED):
            return []
        if not math.isfinite(now) or now < self.entered:
            return self.latch(now, "clock_reset")
        if o.status_age < 0 or o.status_age > c.mode_timeout_s:
            return self.latch(now, "status_missing")
        if self.boot_epoch is not None and o.boot_epoch != self.boot_epoch:
            return self.latch(now, "flight_controller_restart")
        if o.session != self.session:
            return self.latch(now, "session_changed")
        if self.state == State.LANDING:
            if o.connected and o.status_age <= c.status_timeout_s and not o.armed and o.landed == 1:
                self.transition(State.COMPLETE, now)
            return []
        if o.fault in REVOKING_FAULTS:
            return self.latch(now, o.fault)
        if self.state == State.PRECHECK:
            ready = (o.connected and o.status_age <= c.status_timeout_s and not o.armed and o.landed == 1 and
                     o.pose_age <= c.pose_timeout_s and o.alignment and all(math.isfinite(x) for x in o.position))
            if ready:
                self.ground_z = o.position[2]
                self.boot_epoch = o.boot_epoch
                self.transition(State.TAKEOFF, now)
                return ["arm"]
            if now - self.entered > c.precheck_timeout_s:
                return self.latch(now, "precheck_timeout")
            return []
        if self.state == State.TAKEOFF:
            if not o.armed:
                if now - self.entered > c.mode_timeout_s:
                    return self.latch(now, "arming_not_confirmed")
                return []
            if not self.takeoff_requested:
                self.takeoff_requested = True
                return ["takeoff"]
            if now - self.entered > c.takeoff_timeout_s:
                return self.latch(now, "takeoff_timeout")
            if o.landed == 2 and self.stable.update(now, o, self.ground_z):
                self.hold = tuple(o.position)
                self.transition(State.HOLD_READY, now)
                return ["hold_setpoint"]
            return []
        if not o.armed or o.landed == 1:
            return self.latch(now, "landed_or_disarmed")
        if self.state == State.HOLD_READY:
            if not o.alignment:
                return self.latch(now, "frame_reset")
            actions = ["hold_setpoint"]
            if o.sending and o.sensors_ready and o.map_ready and now - self.entered >= c.stable_duration_s:
                self.transition(State.OFFBOARD_PENDING, now)
                actions.append("offboard")
            elif now - self.entered > c.hold_ready_timeout_s:
                return self.latch(now, "map_or_stream_not_ready")
            return actions
        if self.state == State.OFFBOARD_PENDING:
            if offboard_observed(o):
                self.transition(State.EXECUTING, now)
                self.planner_started = True
                return ["hold_setpoint", "enable_planner"]
            fault = self.inflight_fault(o)
            if fault:
                return self._start_recovery_or_latch(now, o, fault)
            if now - self.entered >= c.mode_timeout_s:
                return self.latch(now, "offboard_not_confirmed")
            return ["hold_setpoint"]
        if self.state == State.EXECUTING:
            if not o.alignment:
                # 坐标系重置：永久撤销恢复资格，不尝试恢复。
                return self.latch(now, "frame_reset")
            fault = self.inflight_fault(o)
            healthy = not fault and self._recovery_inputs_valid(o) and o.sending and offboard_observed(o)
            if self.recovery_detail == "confirmed" and healthy:
                if self.recovery_healthy_since is None:
                    self.recovery_healthy_since = now
                elif now - self.recovery_healthy_since >= c.recovery_valid_duration_s:
                    self.recovery_attempts = 0
                    self.recovery_fault = self.recovery_source = ""
                    self.recovery_detail = "incident_closed"
            else:
                self.recovery_healthy_since = None
            if fault:
                return self._start_recovery_or_latch(now, o, fault)
            if (o.pose_age <= c.pose_timeout_s and math.dist(o.position, self.goal) <= c.goal_tolerance_m
                    and math.hypot(*o.velocity) <= c.stable_speed_m_s):
                if self.goal_since is None:
                    self.goal_since = now
                if now - self.goal_since >= c.stable_duration_s:
                    self.goal_reached = True
                    return self.land(now)
            else:
                self.goal_since = None
            return []
        if self.state == State.RECOVERING:
            return self._tick_recovering(now, o)
        return []

    def hold_setpoint(self, now):
        """从故障瞬间速度按加速度上限减速；返回位置、速度、加速度。"""
        if self.state == State.RECOVERING and self.recovery_brake_started is not None:
            velocity = self.recovery_brake_velocity
            speed = math.sqrt(sum(v * v for v in velocity))
            limit = self.config.recovery_brake_accel_m_s2
            elapsed = max(0., now - self.recovery_brake_started)
            duration = speed / limit
            t = min(elapsed, duration)
            if speed > 0.:
                direction = tuple(v / speed for v in velocity)
                travel = speed * t - .5 * limit * t * t
                position = tuple(p + d * travel for p, d in zip(self.recovery_hold, direction))
                remaining = 0. if elapsed >= duration - 1e-9 else max(0., speed - limit * t)
                return (position, tuple(d * remaining for d in direction),
                        tuple(-d * limit for d in direction) if remaining > 0. else (0., 0., 0.))
        point = self.hold_point
        return (point, (0., 0., 0.), (0., 0., 0.))

    @property
    def hold_point(self):
        """非制动段的保持点。制动 setpoint 由 hold_setpoint(now) 生成。"""
        if self.state == State.RECOVERING and self.recovery_hold is not None:
            return tuple(self.recovery_hold)
        return self.hold
