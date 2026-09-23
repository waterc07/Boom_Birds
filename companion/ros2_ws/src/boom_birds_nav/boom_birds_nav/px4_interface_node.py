"""规划输出 → Px4Interface → PX4 的高层控制接口与失效处理（脱机 / SITL）。

链路位置
--------
    EGO traj_server ──/position_cmd────► px4_interface_node ──Px4Backend──► PX4
    pose_adapter ────/boom_birds/vio/odom_ego┘        （MAVLink；SITL 或后续实机）
    mavlink_imu_node ─/boom_birds/imu───────┘

不可含糊的边界
------------------
1. **只发高层 setpoint**（需求 FC-004）：本节点与 `Px4Backend` 都没有 PWM/DShot/电机指令接口。
   `px4_backend.assert_no_actuator_surface()` 会在测试里对协议与实现做真实断言。
2. **通信后端被隔离**（需求 SW-001）：本节点只依赖 `Px4Backend` 协议；"控制器放在 Companion
   还是 PX4" 这个未决问题不会因为本节点而被动定死——换后端不触碰算法模块。
3. **对齐未完成不发位置 setpoint**。EGO 的 `world` 与 PX4 局部 NED 是**两个**局部系：
   轴翻转只解决"哪个轴朝哪"，原点与水平朝向不会自动一致。放行位置需要**两项独立证据**：
   - **水平朝向**：`YawAlignmentResidual` 的被动核实，或 `frame_alignment_observed=true`；
   - **原点/平移**：`frame_alignment_origin_evidence=true`（外部视觉融合把 EKF 原点定义在
     VIO 原点上，或已实测标定 `translation_m`）。

   **航向核实不能替代原点证据**：两个系可以朝向完全一致而原点相隔很远。
   任一项缺失即拒绝下发位置 setpoint（原因码 `local_frame_not_aligned`），
   速度/加速度同样不发——只发速度会让位置语义以另一种形式继续误导。
4. **换算是一次完成的**。位置、速度、加速度、偏航、偏航角速率由
   `px4_frames.ros_local_to_ned_setpoint` 统一换算：
   前三者走 `R(φ)`（平移只作用于位置），偏航为 `−(yaw+φ)`，偏航角速率为 `−yaw_dot`。
   只对位置施加旋转、其它分量走零偏移轴映射，会让同一个 setpoint 里各字段方向互相矛盾。
5. **停发 ≠ 飞控已悬停**。规划拒绝、轨迹失效、VIO/IMU/相机断流、链路超时、飞控重启时，
   本节点停止发送 setpoint（`allow_setpoint=False`）并把原因发布到
   `/boom_birds/control/status`。飞控侧的实际反应取决于 PX4 failsafe 配置
   （`COM_OF_LOSS_T` 默认 1.0 s、`COM_OBL_RC_ACT` 默认 Position）——**必须在 SITL/实机验证**。

坐标系与量纲
------------
`/position_cmd` 的 frame_id 是 EGO 的 `world`（重力对齐、Z 轴向上），契约里对应 `global`；
PX4 局部系是 NED。转换与校验的唯一实现在 `px4_frames`（含单测锁定轴向与偏航符号），
本节点只调用它，不自己算第二遍。原点关系未核实时直接闭锁，不假定两系同原点。
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String

from .px4_failsafe import (
    FailsafeConfig,
    Px4FailsafeMonitor,
    SignalId,
)
from .px4_frames import (
    FrameValidationError,
    RosLocalSetpoint,
    TypeMask,
    YawMode,
    LocalFrameAlignment,
    YawAlignmentResidual,
    ros_local_to_ned_setpoint,
)

QOS_SENSOR = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
QOS_RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)

# quadrotor_msgs/PositionCommand.trajectory_flag
FLAG_READY = 1

#: 明确写进状态与日志的边界说明：不要把"我们停发了"读成"飞控已经安全"。
STOP_NOTE = (
    "allow_setpoint=false 只表示本节点停止发送高层 setpoint："
    "既不代表 PX4 已经悬停，也不代表已进入安全接管；"
    "PX4 的实际反应取决于 failsafe 配置（COM_OF_LOSS_T / COM_OBL_RC_ACT），"
    "必须在 SITL 与实机分别验证"
)


@dataclass
class ControlOutcome:
    """一次控制周期的结果，供测试直接断言（不依赖 ROS）。"""

    allow_setpoint: bool
    sent: bool
    state: str
    reasons: list = field(default_factory=list)
    note: str = STOP_NOTE
    detail: dict = field(default_factory=dict)

    def stop_for(self, reason_code: str, detail: str = "") -> None:
        """把结果显式置为"已停发"。

        为什么必须显式：`state` 原先直接沿用失效监控器的判定，于是出现
        「allow_setpoint=false 但 state=OK」这种自相矛盾的输出——闸门拦住了指令，
        状态却报告一切正常。停发原因与状态必须一致，所以由**停发方**强制改写。
        """
        self.allow_setpoint = False
        self.state = "STOPPED"
        self.detail["stopped_by"] = reason_code
        if detail:
            self.detail["stop_detail"] = detail


class Px4InterfaceCore:
    """与 ROS 无关的控制逻辑：失效判定、时间校验、坐标转换、下发。

    ROS 节点只负责订阅/发布与参数；判定与转换都在这里，因此失效行为可以纯函数式单测。
    """

    def __init__(self, backend, config: FailsafeConfig, *, yaw_mode: str = "yaw",
                 send_acceleration: bool = True, max_setpoint_age_s: float = 0.2,
                 alignment: "LocalFrameAlignment | None" = None,
                 position_gate=None) -> None:
        self.backend = backend
        # 局部系对齐：默认 identity 保持既有纯函数行为；真实部署由节点注入
        self.alignment = alignment if alignment is not None else LocalFrameAlignment()
        # position_gate() -> (allow_position: bool, reason_code: str)
        # 说明：对齐未经核实时**连速度也不发**——只发速度会让位置语义以另一种形式继续误导。
        self._position_gate = position_gate
        self.config = config
        self.monitor = Px4FailsafeMonitor(config)
        self.yaw_mode = YawMode(yaw_mode) if not isinstance(yaw_mode, YawMode) else yaw_mode
        self.send_acceleration = bool(send_acceleration)
        self.max_setpoint_age_s = float(max_setpoint_age_s)
        self._boot_id: str | None = None
        self.counters = {
            "position_cmd_received": 0,
            "position_cmd_flagged": 0,
            "position_cmd_frame_mismatch": 0,
            "position_cmd_stale": 0,
            "setpoints_sent": 0,
            "setpoints_blocked": 0,
            "backend_send_failed": 0,
            "conversion_errors": 0,
            "last_block_reasons": None,
            "last_conversion_error": None,
            "last_flag": None,
            "position_blocked_by_alignment": 0,
            "last_alignment_reason": None,
        }

    # ---------------------------------------------------------------- 输入事件

    def on_position_cmd(self, cmd, now_mono_s: float) -> bool:
        """收到 `/position_cmd`。只有 READY 才被接受为可用轨迹。

        失效协议：`traj_server` 在规划拒绝/轨迹失效时**停止发布**；本项目另有显式的
        失效消息（order=0 且 pos_pts 为空）。这里按同一约定处理 flag：非 READY 一律
        视为失效并立即停发，而不是等超时。
        """
        self.counters["position_cmd_received"] += 1
        flag = int(getattr(cmd, "trajectory_flag", 0))
        self.counters["last_flag"] = flag
        if flag != FLAG_READY:
            self.counters["position_cmd_flagged"] += 1
            self.monitor.note_trajectory_invalidated(now_mono_s, detail=f"trajectory_flag={flag}")
            return False
        traj_id = int(getattr(cmd, "trajectory_id", 0))
        # note_setpoint 同时记录新鲜度与轨迹号：轨迹号变化才算"新轨迹"，
        # 用于判定"轨迹失效后是否真的拿到了新轨迹"。
        self.monitor.note_setpoint(now_mono_s, traj_id)
        return True

    def on_planning_rejected(self, now_mono_s: float, reason: str = "") -> None:
        self.monitor.note_planning_rejected(now_mono_s, detail=reason)

    # ---------------------------------------------------------------- 周期评估

    @staticmethod
    def _reason_to_dict(reason) -> dict:
        """`SafetyReason` → JSON 友好字典（不依赖它是否有 as_dict）。"""
        signal = getattr(reason, "signal", None)
        age = getattr(reason, "age_s", None)
        limit = getattr(reason, "limit_s", None)
        return {
            "code": getattr(getattr(reason, "code", None), "value", str(getattr(reason, "code", ""))),
            "severity": getattr(getattr(reason, "severity", None), "value",
                                str(getattr(reason, "severity", ""))),
            "signal": getattr(signal, "value", None),
            "age_s": None if age is None else float(age),
            "limit_s": None if limit is None else float(limit),
            "detail": getattr(reason, "detail", "") or "",
        }

    def step(self, cmd, now_mono_s: float, frame_id_ok: bool = True) -> ControlOutcome:
        """一次控制周期：链路与信号健康 → 时间有效性 → 坐标转换 → 下发。"""
        link_state = {}
        if hasattr(self.backend, "link_state"):
            try:
                link_state = self.backend.link_state() or {}
            except Exception as exc:  # noqa: BLE001
                link_state = {"error": str(exc)}

        # 链路与飞控心跳是两条不同的信号：
        # - LINK 由"后端连接仍在"驱动（socket/串口层面）；
        # - HEARTBEAT 由**后端报告的年龄**驱动，而不是"每周期都当作新鲜"——
        #   否则心跳丢失永远测不出来（会被本函数自己刷新掉）。
        # boot_id 必须稳定：用后端的 restart_epoch 作为"第几次开机"的标识，
        # 只有后端明确报告重启时才变（每次评估换新值会被判成一直重启）。
        epoch = int(link_state.get("restart_epoch", 0) or 0)
        self._boot_id = f"px4-epoch-{epoch}"
        if link_state.get("connected"):
            self.monitor.note_mavlink_link(now_mono_s)
        heartbeat_age = link_state.get("heartbeat_age_s")
        if heartbeat_age is not None and heartbeat_age != float("inf"):
            # 用"现在的本地时钟 − 报告年龄"换算观测时刻，年龄本身仍由监控器按自己的
            # 超时判定，避免这里替它做判断。
            self.monitor.note_heartbeat(now_mono_s - float(heartbeat_age), self._boot_id)

        decision = self.monitor.evaluate(now_mono_s)

        outcome = ControlOutcome(
            allow_setpoint=bool(decision.allow_setpoint),
            sent=False,
            state=decision.state.value,
            reasons=[self._reason_to_dict(r) for r in decision.reasons],
            detail={"signal_ages_s": {k.value: v for k, v in decision.signal_ages_s.items()},
                    "recovery_streak": decision.recovery_streak},
        )
        if not decision.allow_setpoint:
            self.counters["setpoints_blocked"] += 1
            self.counters["last_block_reasons"] = [r.code.value for r in decision.reasons]
            # decision.state 已经是非 OK（监控器判定），这里保持不动
            return outcome

        if cmd is None or not frame_id_ok:
            self.counters["setpoints_blocked"] += 1
            outcome.allow_setpoint = False
            outcome.reasons.append({"code": "no_usable_trajectory"})
            outcome.stop_for("no_usable_trajectory")
            return outcome

        # 时间有效性：只发新鲜的 setpoint。用监控器记录的**到达时刻**衡量，
        # 而不是 header.stamp——ROS 时间与单调时钟之间有映射，年龄必须同一时钟算。
        age = decision.signal_ages_s.get(SignalId.SETPOINT)
        if age is None or not (age <= self.max_setpoint_age_s):
            self.counters["position_cmd_stale"] += 1
            outcome.allow_setpoint = False
            outcome.reasons.append({"code": "setpoint_age", "age_s": age})
            outcome.stop_for("setpoint_age")
            return outcome

        # 局部系对齐闸门（P1-2）：轴翻转只解决"哪个轴朝哪"，原点与水平朝向**不会**
        # 自动一致。对齐未核实就发位置 setpoint，量纲虽对但落点无法证明，
        # 因此这里直接不放行；速度/加速度一并拦住，避免以另一种形式继续误导。
        if self._position_gate is not None:
            allow_position, alignment_reason, alignment_missing = self._position_gate()
            outcome.detail["alignment"] = alignment_reason
            outcome.detail["alignment_missing"] = list(alignment_missing)
            if not allow_position:
                self.counters["position_blocked_by_alignment"] += 1
                self.counters["last_alignment_reason"] = alignment_reason
                self.counters["setpoints_blocked"] += 1
                outcome.reasons.append({
                    "code": "local_frame_not_aligned",
                    "detail": (
                        "VIO 世界系与 PX4 局部 NED 未完成刚体对齐（原点/水平朝向）；"
                        "发位置 setpoint 无法证明落在 PX4 认为的目标处"
                    ),
                    "alignment": alignment_reason,
                    "missing_evidence": list(alignment_missing),
                })
                # 停发与状态必须一致：不能 allow_setpoint=false 却报 OK
                outcome.stop_for("local_frame_not_aligned", alignment_reason)
                return outcome

        # 坐标与量纲转换（唯一实现在 px4_frames，含校验）
        try:
            ros_sp = RosLocalSetpoint(
                position_m=(cmd.position.x, cmd.position.y, cmd.position.z),
                velocity_m_s=(cmd.velocity.x, cmd.velocity.y, cmd.velocity.z),
                acceleration_m_s2=(cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z),
                yaw_rad=float(cmd.yaw),
                # 字段名注意：ROS 侧是 `yaw_dot_rad_s`、PX4 侧是 `yaw_rate_rad_s`；
                # `quadrotor_msgs/PositionCommand` 里的名字是 **`yaw_dot`**（不是 yaw_rate）。
                # 写错属性名会静默取到 0，所以这里显式读 yaw_dot 并保留回退。
                yaw_dot_rad_s=float(getattr(cmd, "yaw_dot", getattr(cmd, "yaw_rate", 0.0))),
            )
            # 加速度前馈是否启用由参数决定；type_mask 因此必须与模式一致
            # （PX4 会把被忽略的轴写成 NaN，模式与掩码不一致就是发错轴）。
            mode = ("position_velocity_acceleration" if self.send_acceleration
                    else "position_velocity")
            # **一次**完整换算：位置（减平移后旋转）、速度/加速度（只旋转）、
            # 偏航（−（yaw+offset））、偏航角速率（−yaw_dot）。
            # 之前只对位置施加了航向旋转，速度/加速度/偏航仍走零偏移轴映射，
            # 于是同一个 setpoint 里各字段朝向互相矛盾。
            setpoint = ros_local_to_ned_setpoint(ros_sp, alignment=self.alignment)
            mask = TypeMask.for_mode(mode, self.yaw_mode)
        except (FrameValidationError, ValueError) as exc:
            self.counters["conversion_errors"] += 1
            self.counters["last_conversion_error"] = str(exc)
            outcome.allow_setpoint = False
            outcome.reasons.append({"code": "frame_conversion", "detail": str(exc)})
            outcome.stop_for("frame_conversion", str(exc))
            return outcome

        # 下发：dry_run 由后端决定是否真的发出；后端返回 False 表示"没有成功发出"，
        # 因此如实反映，不把"没发出去"记成成功。
        sent = bool(self.backend.send_setpoint(setpoint, type_mask=int(mask)))
        outcome.sent = sent
        if sent:
            self.counters["setpoints_sent"] += 1
        else:
            self.counters["backend_send_failed"] += 1
            try:
                diagnostics = self.backend.stream_diagnostics()
            except Exception:  # noqa: BLE001
                diagnostics = {}
            outcome.detail["backend_last_refusal"] = diagnostics.get("last_refusal")
        return outcome


class Px4InterfaceNode(Node):
    def __init__(self) -> None:
        super().__init__("boom_birds_px4_interface")

        self.declare_parameter("position_cmd_topic", "/position_cmd")
        self.declare_parameter("odom_topic", "/boom_birds/vio/odom_ego")
        self.declare_parameter("imu_topic", "/boom_birds/imu")
        self.declare_parameter("depth_topic", "/boom_birds/depth/image")
        self.declare_parameter("status_topic", "/boom_birds/control/status")
        # 允许的输入坐标系名：EGO 的 `world` 与契约里的 `global` 指的是同一个
        # 重力对齐、Z 轴向上的局部系（README 的目标架构里两者混用），因此都接受；
        # 其它名字一律拒绝（不做隐式坐标假设）。
        self.declare_parameter("frame_world", "global")
        self.declare_parameter("frame_world_aliases", ["world", "global", "map"])

        # ---- 局部系对齐（P1-2）：轴翻转之外的原点与水平朝向 ----
        # none                 : 只做轴变换，**不发位置 setpoint**（默认，最保守）
        # declared             : 声明 yaw 偏移（+可选平移），仍要求实测残差核实
        # identity             : 显式声明"两系同向且原点重合"（同源世界系/SITL），仍要求核实
        # unverified_test_only : 仅离线测试；跳过位置闸门，启动时打 ERROR
        self.declare_parameter("frame_alignment", "none")
        self.declare_parameter("frame_alignment_observed", False)
        #: 原点/平移关系的**证据**。航向核实证明不了原点：两个系可以朝向一致但原点相隔很远。
        #: true 表示"原点关系已核实"（例如 PX4 的 EKF 原点已由外部视觉融合定义在 VIO 原点上，
        #: 或已用实测数据标定出 translation_m）。未置位时位置 setpoint 不会被放行。
        self.declare_parameter("frame_alignment_origin_evidence", False)
        #: 原点证据的文字说明（写进状态，便于复核是谁、凭什么核实的）
        self.declare_parameter("frame_alignment_origin_note", "")
        self.declare_parameter("frame_alignment_yaw_offset_rad", 0.0)
        self.declare_parameter("frame_alignment_translation_m", [0.0, 0.0, 0.0])
        self.declare_parameter("frame_alignment_tolerance_rad", 0.0872664626)   # 5°
        self.declare_parameter("frame_alignment_required_samples", 10)
        self.declare_parameter("frame_alignment_max_tilt_rad", 0.35)            # 20°
        self.declare_parameter("frame_alignment_max_yaw_rate_rad_s", 0.5)

        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("yaw_mode", "yaw")              # yaw | yaw_rate
        self.declare_parameter("send_acceleration", True)
        self.declare_parameter("max_setpoint_age_s", 0.2)

        self.declare_parameter("backend", "fake")              # fake | mavlink
        self.declare_parameter("dry_run", True)
        self.declare_parameter("connection", "udpin:127.0.0.1:14540")
        self.declare_parameter("allow_arming", False)
        self.declare_parameter("read_timeout_s", 0.05)
        # 后端自己的心跳超时（决定 is_connected()/能否发 setpoint）。
        # 必须与上面的 heartbeat_timeout_s 同量级，否则失效判定自相矛盾：
        # 默认 1.0 与 PX4 COM_OF_LOSS_T 对齐；不显式传参时后端自身默认 3.0。
        self.declare_parameter("backend_heartbeat_timeout_s", 1.0)
        # 真实后端只在 connect() 里建链/绑定本地端口，不连就没法收发。
        # 因此启动时必须显式连接一次；连不上不致命（节点继续跑并把
        # connected=false / last_error 如实写进状态话题），但绝不静默跳过。
        self.declare_parameter("connect_on_start", True)

        # 失效阈值：setpoint/vio 与契约 pose_timeout_s=0.15 对齐；imu/camera 与
        # depth_timeout_s=1.0 及 PX4 COM_OF_LOSS_T=1.0 的量级对齐。
        self.declare_parameter("setpoint_timeout_s", 0.15)
        self.declare_parameter("vio_timeout_s", 0.15)
        self.declare_parameter("imu_timeout_s", 0.5)
        self.declare_parameter("camera_timeout_s", 1.0)
        self.declare_parameter("heartbeat_timeout_s", 1.0)
        self.declare_parameter("link_timeout_s", 1.0)
        self.declare_parameter("recovery_required_samples", 5)

        self.frame_world = str(self.get_parameter("frame_world").value)
        aliases = self.get_parameter("frame_world_aliases").value
        self.frame_world_ok = {str(a) for a in (aliases or []) if str(a)} | {self.frame_world}
        self._cmd = None
        self._cmd_frame_ok = True

        self.backend = self._make_backend()
        # PX4 重启可重建 EKF 局部原点；原有对齐证据在新启动周期不能沿用。
        self._alignment_epoch: int | None = None
        self._alignment_restart_latched = False
        config = FailsafeConfig(
            timeouts={
                SignalId.SETPOINT: float(self.get_parameter("setpoint_timeout_s").value),
                SignalId.VIO_POSE: float(self.get_parameter("vio_timeout_s").value),
                SignalId.IMU: float(self.get_parameter("imu_timeout_s").value),
                SignalId.CAMERA: float(self.get_parameter("camera_timeout_s").value),
                SignalId.MAVLINK_LINK: float(self.get_parameter("link_timeout_s").value),
                SignalId.PX4_HEARTBEAT: float(self.get_parameter("heartbeat_timeout_s").value),
                SignalId.ODOM_EGO: float(self.get_parameter("vio_timeout_s").value),
            },
            recovery_fresh_samples=int(self.get_parameter("recovery_required_samples").value),
        )
        # 对齐必须在 Core 之前建立好（Core 用它做位置换算与闸门）
        self.alignment = self._make_alignment()
        self._alignment_observed = bool(
            self.get_parameter("frame_alignment_observed").value
        ) or bool(self.alignment.verified)
        self._alignment_residual: YawAlignmentResidual | None = None
        if self.alignment_mode in ("declared", "identity") and not self._alignment_observed:
            # 被动核实：拿 VIO 航向与 PX4 航向的残差判断两系是否真的对齐
            self._alignment_residual = YawAlignmentResidual(
                tolerance_rad=float(self.get_parameter("frame_alignment_tolerance_rad").value),
                required_samples=int(
                    self.get_parameter("frame_alignment_required_samples").value
                ),
                max_tilt_rad=float(self.get_parameter("frame_alignment_max_tilt_rad").value),
                max_yaw_rate_rad_s=float(
                    self.get_parameter("frame_alignment_max_yaw_rate_rad_s").value
                ),
            )
        self.core = Px4InterfaceCore(
            self.backend, config,
            yaw_mode=str(self.get_parameter("yaw_mode").value),
            send_acceleration=bool(self.get_parameter("send_acceleration").value),
            max_setpoint_age_s=float(self.get_parameter("max_setpoint_age_s").value),
            alignment=self.alignment,
            position_gate=self._position_allowed_by_alignment,
        )

        from quadrotor_msgs.msg import PositionCommand

        self.create_subscription(PositionCommand, self.get_parameter("position_cmd_topic").value,
                                 self._on_position_cmd, QOS_RELIABLE)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value,
                                 self._on_odom, QOS_RELIABLE)
        self.create_subscription(Imu, self.get_parameter("imu_topic").value,
                                 self._on_imu, QOS_SENSOR)
        self.create_subscription(Image, self.get_parameter("depth_topic").value,
                                 self._on_depth, QOS_RELIABLE)
        self.pub_status = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )

        rate = max(float(self.get_parameter("control_rate_hz").value), 1.0)
        self.create_timer(1.0 / rate, self._tick)
        # 显式建链：MavlinkPx4Backend 的 socket 只在 connect() 里创建，
        # 之前漏掉这一步会让节点永远收不到心跳、也永远不知道对端地址。
        if bool(self.get_parameter("connect_on_start").value):
            self._connect_backend()
        self.get_logger().info(
            f"Px4Interface 启动：backend={self.get_parameter('backend').value} "
            f"dry_run={self.get_parameter('dry_run').value} "
            f"arming={'ALLOWED' if self.get_parameter('allow_arming').value else 'disabled'} "
            f"rate={rate}Hz cmd={self.get_parameter('position_cmd_topic').value}"
        )
        self.get_logger().warn(
            "只发高层 setpoint；停发 setpoint 不等于 PX4 已悬停或已安全接管——"
            "飞控侧动作取决于 failsafe 配置，必须在 SITL/实机验证"
        )
        # 对齐状态必须在启动时就讲清楚，不能让"能发位置"这件事被默认为已成立
        allowed, reason, missing = self._position_allowed_by_alignment()
        if self.alignment_mode == "unverified_test_only":
            self.get_logger().error(
                "frame_alignment=unverified_test_only：已跳过坐标系对齐闸门，"
                "位置 setpoint 未经验证。此模式**只允许用于离线测试**，绝不可用于实机。"
            )
        elif not allowed:
            self.get_logger().warn(
                f"坐标系对齐不完整（reason={reason}）：**不会下发位置 setpoint**。"
                "缺失的证据：" + "；".join(missing) +
                "。注意航向核实（YawAlignmentResidual）**只**覆盖水平朝向，"
                "证明不了原点——两个系可以朝向一致但原点相隔很远。"
            )

    @property
    def alignment_mode(self) -> str:
        return str(self.get_parameter("frame_alignment").value)

    def _make_alignment(self) -> LocalFrameAlignment:
        mode = str(self.get_parameter("frame_alignment").value)
        if mode not in ("none", "declared", "identity", "unverified_test_only"):
            raise RuntimeError(
                f"未知 frame_alignment：{mode}"
                "（可用：none | declared | identity | unverified_test_only）"
            )
        if mode == "unverified_test_only":
            backend_kind = str(self.get_parameter("backend").value)
            dry_run = bool(self.get_parameter("dry_run").value)
            if backend_kind != "fake" and not dry_run:
                raise RuntimeError(
                    "frame_alignment=unverified_test_only 只允许 Fake 后端或 "
                    "dry_run=true；真实 MAVLink 发送必须提供对齐证据"
                )
        raw_t = self.get_parameter("frame_alignment_translation_m").value or [0.0, 0.0, 0.0]
        translation = tuple(float(v) for v in list(raw_t)[:3])
        if len(translation) != 3:
            raise RuntimeError("frame_alignment_translation_m 必须是 3 个分量")
        yaw = float(self.get_parameter("frame_alignment_yaw_offset_rad").value)
        observed = bool(self.get_parameter("frame_alignment_observed").value)

        # identity 的语义是"两系同向且原点重合"。若同时配了非零偏移/平移，
        # 配置自相矛盾：不能一边声明"同向同原点"一边给出偏移量，否则放行依据就说不清。
        if mode == "identity":
            if abs(yaw) > 1e-12:
                raise RuntimeError(
                    "frame_alignment=identity 却配置了非零 "
                    f"frame_alignment_yaw_offset_rad={yaw}：identity 表示两系同向同原点，"
                    "需要用 declared 表达非零偏移"
                )
            if max(abs(v) for v in translation) > 1e-12:
                raise RuntimeError(
                    "frame_alignment=identity 却配置了非零 "
                    f"frame_alignment_translation_m={translation}：identity 表示原点重合，"
                    "需要用 declared 表达非零平移"
                )
        return LocalFrameAlignment(
            yaw_offset_rad=yaw,
            translation_m=translation,       # type: ignore[arg-type]
            verified=observed,
            source=mode,
        )

    def _origin_evidence(self) -> bool:
        """原点/平移关系是否有证据。**航向核实不能替代它。**"""
        return (
            not self._alignment_restart_latched
            and bool(self.get_parameter("frame_alignment_origin_evidence").value)
        )

    def _refresh_alignment_epoch(self) -> None:
        """PX4 启动周期变化后闭锁旧对齐；须重新核实并重启本节点。"""
        try:
            state = self.backend.link_state() or {}
            value = state.get("restart_epoch")
            if value is None:
                return
            epoch = int(value)
        except Exception:  # noqa: BLE001 —— 后端失联由 Core 处理；状态诊断不得崩溃
            return
        if self._alignment_epoch is None:
            self._alignment_epoch = epoch
        elif epoch != self._alignment_epoch:
            self._alignment_epoch = epoch
            self._alignment_restart_latched = True
            if self._alignment_residual is not None:
                self._alignment_residual.reset()
            self.get_logger().error(
                "PX4 重启：旧航向与原点对齐证据已失效；停止位置 setpoint。"
                "重新核实两项证据后重启本节点。"
            )

    def _yaw_evidence(self) -> tuple[bool, str]:
        """水平朝向是否已核实。返回 (已核实, 说明)。"""
        if self._alignment_restart_latched:
            return False, "alignment_invalidated_after_px4_restart"
        if self._alignment_observed:
            return True, "frame_alignment_observed"
        residual = self._alignment_residual
        if residual is None:
            return False, "yaw_residual_not_configured"
        if not residual.yaw_verified:
            return False, "yaw_not_verified"
        # 首次通过时记录一次，避免每帧重复刷日志
        if not getattr(self, "_yaw_verified_logged", False):
            self._yaw_verified_logged = True
            self.get_logger().info(
                "水平朝向已核实：yaw 残差均值="
                f"{residual.mean_residual_rad:.6f} rad（{residual.samples} 个样本，容差 "
                f"{residual.tolerance_rad:.6f} rad）。注意这**只**覆盖航向，"
                "原点/平移仍需单独证据。"
            )
        return True, "yaw_verified"

    def _position_allowed_by_alignment(self) -> tuple[bool, str, list]:
        """位置 setpoint 是否允许下发 → (allow, reason, missing_evidence)。"""
        self._refresh_alignment_epoch()
        mode = self.alignment_mode
        if mode == "none":
            return False, "frame_alignment_none", [
                "水平朝向未核实", "原点/平移未核实",
            ]
        if self._alignment_restart_latched:
            return False, "alignment_invalidated_after_px4_restart", [
                "PX4 重启后须重新核实水平朝向及原点/平移，并重启本节点",
            ]
        if mode == "unverified_test_only":
            return True, "frame_alignment_unverified_test_only", []

        yaw_ok, yaw_reason = self._yaw_evidence()
        origin_ok = self._origin_evidence()
        missing = []
        if not yaw_ok:
            missing.append("水平朝向未核实（等 VIO/PX4 航向残差达标）")
        if not origin_ok:
            missing.append(
                "原点/平移无证据（需 frame_alignment_origin_evidence=true："
                "外部视觉融合把 EKF 原点定义在 VIO 原点上，或已实测标定 translation_m）"
            )
        if yaw_ok and origin_ok:
            return True, f"yaw_verified+origin_evidence({mode})", []
        reason = "yaw_verified_origin_unknown" if yaw_ok else f"not_aligned({yaw_reason})"
        return False, reason, missing

    def alignment_report(self) -> dict:
        allow, reason, missing = self._position_allowed_by_alignment()
        report = self.alignment.to_dict()
        report.update({
            "mode": self.alignment_mode,
            "observed_yaw": bool(self._alignment_observed and not self._alignment_restart_latched),
            "origin_evidence": self._origin_evidence(),
            "configured_yaw_evidence": bool(self._alignment_observed),
            "configured_origin_evidence": bool(
                self.get_parameter("frame_alignment_origin_evidence").value
            ),
            "origin_note": str(self.get_parameter("frame_alignment_origin_note").value),
            "px4_restart_epoch": self._alignment_epoch,
            "invalidated_after_px4_restart": self._alignment_restart_latched,
            "position_allowed": allow,
            "position_reason": reason,
            "missing_evidence": list(missing),
            "covers": (
                "unverified_test_only" if self.alignment_mode == "unverified_test_only"
                else "yaw_and_origin" if allow else "not_fully_aligned"
            ),
        })
        if self._alignment_residual is not None:
            report["yaw_residual"] = self._alignment_residual.to_dict()
        return report

    def note_yaw_observation(self, ros_yaw_rad: float, ros_mono_s: float | None = None) -> None:
        """用 VIO 航向与 PX4 航向的残差被动核实**航向**（不发任何指令）。

        姿态缺失/过期、或两个消息的本机到达时刻差过大时，采样会被拒绝
        （并计入诊断）。到达时差不是曝光/姿态测量时差，真机仍须核验。
        绝不把缺失的姿态当成 0：那会把"没观测到"读成"水平且不转"。
        """
        self._refresh_alignment_epoch()
        residual = self._alignment_residual
        if residual is None or self._alignment_observed or self._alignment_restart_latched:
            return
        try:
            state = self.backend.read_vehicle_state()
        except Exception:  # noqa: BLE001 —— 读不到就只是"还没法核实"
            return
        if state is None:
            return
        px4_yaw = getattr(state, "yaw_rad", None)
        if px4_yaw is None:
            return
        residual.observe(
            ros_yaw_rad=float(ros_yaw_rad),
            px4_yaw_rad=float(px4_yaw),
            declared_yaw_offset_rad=float(self.alignment.yaw_offset_rad),
            # 缺失就传 None（不可用 0 顶替）
            roll_rad=getattr(state, "roll_rad", None),
            pitch_rad=getattr(state, "pitch_rad", None),
            yaw_rate_rad_s=getattr(state, "yaw_rate_rad_s", None),
            ros_mono_s=ros_mono_s,
            px4_mono_s=getattr(state, "attitude_received_mono_s", None),
            attitude_age_s=getattr(state, "attitude_age_s", None),
        )

    def _make_backend(self):
        kind = str(self.get_parameter("backend").value)
        if kind == "fake":
            from .px4_backend import FakePx4Backend

            return FakePx4Backend(
                heartbeat_timeout_s=float(
                    self.get_parameter("backend_heartbeat_timeout_s").value
                )
            )
        if kind == "mavlink":
            from .px4_backend import MavlinkPx4Backend

            return MavlinkPx4Backend(
                connection=str(self.get_parameter("connection").value),
                dry_run=bool(self.get_parameter("dry_run").value),
                allow_arming=bool(self.get_parameter("allow_arming").value),
                read_timeout_s=float(self.get_parameter("read_timeout_s").value),
                heartbeat_timeout_s=float(
                    self.get_parameter("backend_heartbeat_timeout_s").value
                ),
            )
        raise RuntimeError(f"未知 backend：{kind}（可用：fake | mavlink）")

    def _connect_backend(self) -> bool:
        """启动时建链一次。失败不抛异常：状态话题里的 backend 字段会如实反映。"""
        try:
            ok = bool(self.backend.connect())
        except Exception as exc:  # noqa: BLE001 —— 建链失败必须可见但不应致命
            self.get_logger().error(f"后端建链异常：{exc}")
            return False
        try:
            diagnostics = self.backend.stream_diagnostics()
        except Exception:  # noqa: BLE001
            diagnostics = {}
        self.get_logger().info(
            f"后端建链{'成功' if ok else '失败'}：connection={diagnostics.get('connection')!r} "
            f"local_endpoint={diagnostics.get('local_endpoint')!r} "
            f"connected={diagnostics.get('connected')!r} "
            f"last_error={diagnostics.get('last_error')!r}"
        )
        if not ok:
            self.get_logger().warn(
                "后端未建链：不会下发任何 setpoint，直到链路建立（状态话题里 connected=false）"
            )
        return ok

    # ---------------------------------------------------------------- 回调

    def _on_position_cmd(self, msg) -> None:
        now = time.monotonic()
        frame_id = str(getattr(msg.header, "frame_id", "") or "")
        if not frame_id or frame_id not in self.frame_world_ok:
            # 坐标系不匹配是安全相关错误：不做隐式假设，直接当失效处理
            self._cmd_frame_ok = False
            self.core.counters["position_cmd_frame_mismatch"] += 1
            self.core.on_planning_rejected(now, f"frame_id={frame_id} 不在允许集合")
            self._cmd = None
            self.get_logger().warn(
                f"PositionCommand.frame_id={frame_id!r} 不在允许集合 "
                f"{sorted(self.frame_world_ok)} 中：拒绝该指令（不做隐式坐标假设）",
                throttle_duration_sec=5.0,
            )
            return
        self._cmd_frame_ok = True
        if self.core.on_position_cmd(msg, now):
            self._cmd = msg
        else:
            self._cmd = None      # 非 READY：立即失效，不沿用旧轨迹

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        """四元数 → yaw（仅用于对齐核实，不参与控制换算）。"""
        import math

        x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _on_odom(self, msg) -> None:
        """VIO 里程计到达：记新鲜度 + 用其航向被动核实坐标系对齐。"""
        try:
            self.note_yaw_observation(
                self._yaw_from_quaternion(msg.pose.pose.orientation),
                ros_mono_s=time.monotonic(),
            )
        except Exception:  # noqa: BLE001 —— 核实失败不能影响主链路
            pass
        self._on_odom_impl(msg)

    def _on_odom_impl(self, msg) -> None:
        """VIO 里程计到达：只记新鲜度。

        不用数值判定可信度：位姿是否可用由上游（OpenVINS + pose_adapter 的契约超时）决定；
        本节点只关心"这条数据还在不在流里"。
        """
        now = time.monotonic()
        self.core.monitor.note_vio_pose(now)
        self.core.monitor.note_odom(now)

    def _on_imu(self, msg) -> None:
        self.core.monitor.note_imu(time.monotonic())

    def _on_depth(self, msg) -> None:
        self.core.monitor.note_camera(time.monotonic())

    # ---------------------------------------------------------------- 周期

    def _tick(self) -> None:
        now = time.monotonic()
        outcome = self.core.step(self._cmd, now, frame_id_ok=self._cmd_frame_ok)
        if not outcome.allow_setpoint:
            self._cmd = None      # 失效后必须重新拿到新鲜且 READY 的轨迹才恢复
        self._publish_status(outcome)

    def _publish_status(self, outcome: ControlOutcome) -> None:
        diagnostics = {}
        if hasattr(self.backend, "stream_diagnostics"):
            try:
                diagnostics = self.backend.stream_diagnostics()
            except Exception as exc:  # noqa: BLE001
                diagnostics = {"error": str(exc)}
        stats = {
            "state": outcome.state,
            "allow_setpoint": outcome.allow_setpoint,
            "setpoint_sent_this_tick": outcome.sent,
            "reasons": outcome.reasons,
            "counters": dict(self.core.counters),
            "note": outcome.note,
            "backend": diagnostics,
            "frame_alignment": self.alignment_report(),
        }
        msg = String()
        msg.data = json.dumps(stats, ensure_ascii=False, default=str, sort_keys=True)
        self.pub_status.publish(msg)

    def shutdown(self) -> None:
        try:
            self.backend.close()
        except Exception:  # noqa: BLE001
            pass


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = None
    try:
        node = Px4InterfaceNode()
        rclpy.spin(node)
    except Exception as exc:  # noqa: BLE001
        print(f"[boom_birds_nav] px4_interface_node 启动失败：{exc}")
        raise
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
