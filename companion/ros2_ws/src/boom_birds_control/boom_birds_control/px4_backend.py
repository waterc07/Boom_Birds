"""SW-001 隔离面：飞控通信后端与算法模块之间的唯一接缝。

为什么要这一层
--------------
README 的目标架构写的是「轨迹执行 → 控制接口 → **Px4Interface** → PX4 → ESC」，
并明确「控制器位于 Companion 或 PX4 **尚未确定**；通信后端、外部视觉回传及
EKF2 融合配置随此确定」。SW-001 要求 `Px4Interface` 隔离飞控通信后端与算法模块。

本模块就是那条接缝：算法/节点只依赖 `Px4Backend` 协议，不 import pymavlink、
不碰串口、不知道连接串长什么样。将来若把控制器挪进 PX4，或把 MAVLink 换成
DDS/uxrce，应当只替换本模块中的后端实现，**算法模块一行不改**。

**本模块不实现、也不预设「控制器放哪边」的任何结论。** 它只提供两侧都需要的
最小能力：状态读取、心跳、模式、解锁、高层 setpoint。选择仍待定（SW-001）。

【硬禁令 · FC-004】协议面禁止执行器
-----------------------------------
FC-004：「基础 Offboard 应支持状态读取、心跳、模式、解锁和高层 setpoint，
**不包含直接电机命令**」；README：「Companion 不输出 PWM/DShot」。
因此 `Px4Backend` **不得**出现任何 PWM / DShot / 执行器 / 电机 / 舵机 / 油门
直接命令面——不是「暂未实现」，而是**不允许存在**。本模块用
`assertNoActuatorSurface()` 对协议与两个实现做真实断言
（见 `FORBIDDEN_ACTUATOR_NAME_PATTERNS` 与 `test/test_px4_backend.py`），
不靠注释约束。执行器输出是 PX4 内部职责，Companion 只给高层 setpoint。

setpoint 载体：结构性约定，不引入第二个类型
------------------------------------------
setpoint 类型定义在 `px4_frames.py`
（本任务开始时该文件尚未落地）。为避免造出两个竞争的 setpoint 类型，
本模块**不定义** setpoint 数据类，只按结构消费。

**实测到的真实形状**（读 `px4_frames.py` 得到；该文件仍在开发中，可能再变）：
`Px4LocalSetpoint` 是 frozen dataclass，字段为
`position_m` / `velocity_m_s` / `acceleration_m_s2`（均为必填三元组）、
`yaw_rad`（必填）、`yaw_rate_rad_s`（默认 0.0）——**它不带 `type_mask`**。
`type_mask` 由 `px4_frames.TypeMask.forMode(mode, yaw_mode)` 单独构造，
结果对象有 `.mask` 整数（也支持 `int(...)`）。

因此本模块的 setpoint 取值规则（**不猜**）：

1. `sendSetpoint(setpoint, type_mask=...)` 显式给了 mask → 以它为准；
2. 否则若 setpoint 自带 `type_mask` 属性 → 用该属性（本模块自己的
   `Px4SetpointLike` 形状，便于离线替身与单测）；
3. 两者都没有 → **拒绝**，原因 `setpoint_type_mask_missing`，并提示用
   `px4_frames.TypeMask.forMode(...)` 构造。

「向量为 None」等价于「该三个轴被忽略」是本模块对**替身对象**的便利约定；
真实 `Px4LocalSetpoint` 三个向量恒在，此时 mask 是唯一权威：置位的轴一律
按 MAVLink 惯例填 0.0 并计数，置零的轴必须存在且有限，否则拒绝。

本机 PX4 源码核验过的协议事实（只读 /home/waterc/PX4-Autopilot）
--------------------------------------------------------------
1. `SET_POSITION_TARGET_LOCAL_NED` = MAVLink msg **84**，`MAV_FRAME_LOCAL_NED` = **1**。
   线上字段顺序 time_boot_ms, x,y,z, vx,vy,vz, afx,afy,afz, yaw, yaw_rate,
   type_mask, target_system, target_component, coordinate_frame。
   **注意 pymavlink 的 `_encode` 形参顺序与线上顺序不同**：形参是
   (time_boot_ms, target_system, target_component, coordinate_frame, type_mask,
   x,y,z, vx,vy,vz, afx,afy,afz, yaw, yaw_rate)。写错位置不会报错，只会静默发错值，
   所以本模块只按形参名传参，并在单测里回环解码逐字段断言。
2. type_mask 位值（pymavlink 与本机 PX4 一致）：
   X=1 Y=2 Z=4, VX=8 VY=16 VZ=32, AX=64 AY=128 AZ=256, FORCE_SET=512,
   YAW=1024 YAW_RATE=2048。置位 = **忽略**该轴。
   `mavlink_receiver.cpp` 在 `MAV_FRAME_LOCAL_NED` 分支把被忽略轴写成 **NaN**，
   因此「被忽略轴上填什么」对 PX4 无意义；本模块按 MAVLink 惯例填 0.0 并计数。
3. 同处：若位置/速度/加速度**全部**被忽略，PX4 打印
   "SET_POSITION_TARGET_LOCAL_NED invalid, missing position, velocity or acceleration"
   并丢弃该消息；`type_mask` 带 FORCE_SET 且加速度有效时同样直接丢弃。
   本模块在本地就拒绝这两种情况（不把必然被丢的消息发上线）。
4. 同处：整个处理被 `get_forward_externalsp()` 把关，即参数 **`MAV_FWDEXTSP`
   （默认 1）**。若有人把它置 0，setpoint 会被**静默丢弃**——SITL/真机联调时
   这是「发了没反应」的第一嫌疑项（Lead 侧排查用）。
5. 同处**没有**读取 `target_local_ned.time_boot_ms`：PX4 用
   `hrt_absolute_time()` 给 `trajectory_setpoint.timestamp` 打时间戳。
   所以本模块填的 `time_boot_ms` 仅供日志/复盘，**不参与 PX4 的超时判定**。
6. 解锁：`MAV_CMD_COMPONENT_ARM_DISARM` = **400**，`param1` = 1 解锁 / 0 上锁
   （`Commander.cpp` 按 `lroundf(cmd.param1)` 判 `ARMING_ACTION_*`）。
7. 模式：`MAV_CMD_DO_SET_MODE` = **176**，`param1` = base_mode（须含
   `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED` = 1），`param2` = PX4 自定义主模式，
   `param3` = 子模式（`Commander.cpp` 的 `VEHICLE_CMD_DO_SET_MODE` 分支）。
8. `PX4_CUSTOM_MAIN_MODE_OFFBOARD` = **6**
   （`px4CustomMode.h`：MANUAL=1, ALTCTL, POSCTL, AUTO, ACRO, OFFBOARD）。
   自定义模式整数布局为 `{uint16 reserved; uint8 main; uint8 sub}`，
   故 `custom_mode = (main << 8) | (sub << 16)`。
   AUTO 且 sub=0 时 PX4 落到 `NAVIGATION_STATE_AUTO_MISSION`（已核验）。
9. 解锁状态只认 HEARTBEAT 的 `base_mode & MAV_MODE_FLAG_SAFETY_ARMED`(128)；
   MAVLink 规范明确 DO_SET_MODE 里的该位应被忽略。

未核验的假设（不要当成结论）
----------------------------
- 真机/SITL 是否要求 Companion 先发 GCS 心跳才接受 COMMAND_LONG：本模块默认发
  （`send_heartbeat=True`，1 Hz），但**没有**在 SITL 上验证过必要性。
- `MAV_FWDEXTSP=1` 是 SITL 与真机默认值（源码里的 default），未在实机上确认。
- PX4 对「被忽略轴填 0」的容忍度：源码上被忽略轴直接写 NaN，与入参无关，
  但本任务只在离线回环里验证过我们自己发的内容。
- pymavlink 在本环境的线上版本是 **MAVLink 1.0**（未设 `MAVLINK20`）。
  本模块不改这个全局，只在诊断里报告实测版本；需要 v2 时由环境变量决定。
- pymavlink 的 `mavudp.write()` 会**吞掉** `socket.error`，且在 `udpin` 模式下
  只向已学到的 clients 发送、没有对端时静默不发。因此「write() 没抛异常」
  **不等于**对端收到；成功与否只由 `COMMAND_ACK` / HEARTBEAT 这类回执判定。

时间基准
--------
本模块**不重复 IMU 接收**，也不做 TIMESYNC。`HIGHRES_IMU` 与 `TIMESYNC` 由
`mavlink_imu_node` + `mavlink_clock` 负责。这里只把 PX4 的 `time_boot_ms`
当作**重启证据**与 setpoint 的日志字段使用。
"""

from __future__ import annotations

import ipaddress
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Protocol, Sequence, runtime_checkable

#: 区分「属性不存在」与「属性存在但值为 None」——两者原因不同，不能混为一谈
_MISSING: Any = object()

__all__ = [
    "FORBIDDEN_ACTUATOR_NAME_PATTERNS",
    "FakePx4Backend",
    "ManualClock",
    "MavlinkPx4Backend",
    "Px4Backend",
    "Px4SetpointLike",
    "RecordedCall",
    "VehicleState",
    "assertNoActuatorSurface",
    "px4CustomMainModeName",
]

# ============================================================================
# 硬禁令（FC-004）：这些词一旦出现在协议/实现的名字里，测试直接失败。
# 用子串匹配，覆盖连字符/下划线/驼峰等命名习惯。
# ============================================================================
FORBIDDEN_ACTUATOR_NAME_PATTERNS: tuple[str, ...] = (
    "pwm",
    "dshot",
    "actuator",
    "motor",
    "servo",
    "throttle",
    "esc",
    "rotor",
)


def assertNoActuatorSurface(obj: Any, *, label: str | None = None) -> None:
    """断言 obj 的公开属性名不含任何执行器/电机相关词（FC-004）。

    只检查名字，不检查文档字符串——文档里必须能讨论这条禁令本身。
    下划线开头的一律跳过（内部计数器/句柄不属于对外能力面）。
    """
    name = label or getattr(obj, "__name__", None) or type(obj).__name__
    for attr in dir(obj):
        if attr.startswith("_"):
            continue
        low = attr.lower()
        for pattern in FORBIDDEN_ACTUATOR_NAME_PATTERNS:
            if pattern in low:
                raise AssertionError(
                    f"{name} 暴露了疑似执行器命令面 {attr!r}（命中 {pattern!r}）；"
                    "FC-004 禁止 Companion 直接命令电机/舵机/输出"
                )


# ============================================================================
# MAVLink / PX4 常量
#
# 全部为字面量（本模块在没装 pymavlink 时也必须可 import，好让协议与
# FakePx4Backend 能被离线测试）。每个值的出处见模块 docstring；
# test/test_px4_backend.py 里有一条测试把这里每个字面量与 pymavlink
# 方言对照，写错就红。
# ============================================================================
MAVLINK_MSG_ID_HEARTBEAT = 0
MAVLINK_MSG_ID_SYS_STATUS = 1
MAVLINK_MSG_ID_ATTITUDE = 30
MAVLINK_MSG_ID_LOCAL_POSITION_NED = 32
MAVLINK_MSG_ID_COMMAND_LONG = 76
MAVLINK_MSG_ID_COMMAND_ACK = 77
MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED = 84
MAVLINK_MSG_ID_STATUS_TEXT = 253

MAV_TYPE_GCS = 6
MAV_AUTOPILOT_PX4 = 12
MAV_AUTOPILOT_INVALID = 8
MAV_STATE_ACTIVE = 4

MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_DO_SET_MODE = 176

MAV_RESULT_ACCEPTED = 0
MAV_RESULT_TEMPORARILY_REJECTED = 1
MAV_RESULT_DENIED = 2
MAV_RESULT_UNSUPPORTED = 3
MAV_RESULT_IN_PROGRESS = 5

MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128

MAV_FRAME_LOCAL_NED = 1

ARMING_ACTION_DISARM = 0
ARMING_ACTION_ARM = 1

POSITION_TARGET_TYPEMASK_X_IGNORE = 1
POSITION_TARGET_TYPEMASK_Y_IGNORE = 2
POSITION_TARGET_TYPEMASK_Z_IGNORE = 4
POSITION_TARGET_TYPEMASK_VX_IGNORE = 8
POSITION_TARGET_TYPEMASK_VY_IGNORE = 16
POSITION_TARGET_TYPEMASK_VZ_IGNORE = 32
POSITION_TARGET_TYPEMASK_AX_IGNORE = 64
POSITION_TARGET_TYPEMASK_AY_IGNORE = 128
POSITION_TARGET_TYPEMASK_AZ_IGNORE = 256
POSITION_TARGET_TYPEMASK_FORCE_SET = 512
POSITION_TARGET_TYPEMASK_YAW_IGNORE = 1024
POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE = 2048

# 常用组合（单测里按已知值断言，避免手算错误）
TYPEMASK_POSITION_ONLY = (
    POSITION_TARGET_TYPEMASK_VX_IGNORE
    | POSITION_TARGET_TYPEMASK_VY_IGNORE
    | POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | POSITION_TARGET_TYPEMASK_AX_IGNORE
    | POSITION_TARGET_TYPEMASK_AY_IGNORE
    | POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)  # = 3576 = 0x0DF8
TYPEMASK_VELOCITY_ONLY = (
    POSITION_TARGET_TYPEMASK_X_IGNORE
    | POSITION_TARGET_TYPEMASK_Y_IGNORE
    | POSITION_TARGET_TYPEMASK_Z_IGNORE
    | POSITION_TARGET_TYPEMASK_AX_IGNORE
    | POSITION_TARGET_TYPEMASK_AY_IGNORE
    | POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)  # = 3527 = 0x0DC7

# PX4 自定义主模式（px4CustomMode.h）
PX4_CUSTOM_MAIN_MODE_MANUAL = 1
PX4_CUSTOM_MAIN_MODE_ALTCTL = 2
PX4_CUSTOM_MAIN_MODE_POSCTL = 3
PX4_CUSTOM_MAIN_MODE_AUTO = 4
PX4_CUSTOM_MAIN_MODE_ACRO = 5
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
PX4_CUSTOM_MAIN_MODE_STABILIZED = 7
PX4_CUSTOM_MAIN_MODE_RATTITUDE = 8
PX4_CUSTOM_MAIN_MODE_SIMPLE = 9
PX4_CUSTOM_MAIN_MODE_TERMINATION = 10
PX4_CUSTOM_MAIN_MODE_ALTITUDE_CRUISE = 11

PX4_CUSTOM_MAIN_MODE_NAMES: dict[int, str] = {
    PX4_CUSTOM_MAIN_MODE_MANUAL: "manual",
    PX4_CUSTOM_MAIN_MODE_ALTCTL: "altctl",
    PX4_CUSTOM_MAIN_MODE_POSCTL: "posctl",
    PX4_CUSTOM_MAIN_MODE_AUTO: "auto",
    PX4_CUSTOM_MAIN_MODE_ACRO: "acro",
    PX4_CUSTOM_MAIN_MODE_OFFBOARD: "offboard",
    PX4_CUSTOM_MAIN_MODE_STABILIZED: "stabilized",
    PX4_CUSTOM_MAIN_MODE_RATTITUDE: "rattitude",
    PX4_CUSTOM_MAIN_MODE_SIMPLE: "simple",
    PX4_CUSTOM_MAIN_MODE_TERMINATION: "termination",
    PX4_CUSTOM_MAIN_MODE_ALTITUDE_CRUISE: "altitude_cruise",
}

MODE_NAME_TO_CUSTOM_MAIN_MODE: dict[str, int] = {
    name: mode for mode, name in PX4_CUSTOM_MAIN_MODE_NAMES.items()
}

# PX4 `union px4CustomMode`（px4CustomMode.h）的位域布局：
#   struct { uint16_t reserved; uint8_t mainMode; uint8_t sub_mode; };
# 因此 mainMode 在 bit16-23、sub_mode 在 bit24-31。**不是** >>8 / >>16。
# 这条曾被解错（把真实 0x03040000 解成 main=0/sub=4），SITL 实证后修正。
PX4_CUSTOM_MODE_MAIN_SHIFT = 16
PX4_CUSTOM_MODE_SUB_SHIFT = 24
PX4_CUSTOM_MODE_BYTE_MASK = 0xFF

# PX4_CUSTOM_SUB_MODE_AUTO_*（px4CustomMode.h）
PX4_CUSTOM_SUB_MODE_AUTO_NAMES: dict[str, int] = {
    "ready": 1,
    "takeoff": 2,
    "loiter": 3,
    "mission": 4,
    "rtl": 5,
    "land": 6,
    "follow_target": 8,
    "precland": 9,
    "vtol_takeoff": 10,
}

#: 拒绝原因词汇表（稳定 token，进入 `streamDiagnostics()["last_refusal"]["reason"]`）。
#: 每个 token 的含义见其使用处；测试直接断言这些字符串。
REFUSAL_NON_LOOPBACK_HOST = "non_loopback_host_refused"
REFUSAL_SERIAL_NOT_ALLOWED = "serial_not_allowed"
REFUSAL_UNSUPPORTED_SCHEME = "unsupported_scheme"
REFUSAL_BAD_CONNECTION_STRING = "bad_connection_string"
REFUSAL_ARMING_NOT_ALLOWED = "arming_not_allowed"
REFUSAL_ARMING_NOT_LOOPBACK = "arming_not_allowed_on_non_loopback_link"
REFUSAL_DRY_RUN_COMMAND = "dry_run_command_suppressed"
REFUSAL_TX_PATH_NOT_READY = "tx_path_not_ready"
REFUSAL_WRITE_FAILED = "write_failed"
REFUSAL_CONNECT_FAILED = "connect_failed"
REFUSAL_UNKNOWN_MODE = "unknown_mode"
REFUSAL_SETPOINT_NOT_LIKE = "setpoint_not_setpoint_like"
REFUSAL_SETPOINT_MASK_INVALID = "setpoint_type_mask_invalid"
REFUSAL_SETPOINT_MASK_MISSING = "setpoint_type_mask_missing"
REFUSAL_SETPOINT_MISSING_VALUE = "setpoint_missing_value"
REFUSAL_SETPOINT_NONFINITE = "setpoint_nonfinite"
REFUSAL_SETPOINT_ALL_AXES_IGNORED = "setpoint_all_axes_ignored"
REFUSAL_SETPOINT_FORCE_NOT_SUPPORTED = "setpoint_force_not_supported"
REFUSAL_SETPOINT_YAW_AND_YAW_RATE = "setpoint_yaw_and_yaw_rate"
#: FakePx4Backend 专用（注入故障）
REFUSAL_SETPOINT_REJECTED_BY_FAKE = "setpoint_rejected_by_fake_px4"
REFUSAL_COMMAND_REJECTED_BY_FAKE = "command_rejected_by_fake_px4"


def px4CustomMainModeName(custom_mode: int | None) -> str | None:
    """PX4 `custom_mode` 整数 → 主模式名（未知返回 None，不要编名字）。"""
    if custom_mode is None:
        return None
    main = (int(custom_mode) >> PX4_CUSTOM_MODE_MAIN_SHIFT) & PX4_CUSTOM_MODE_BYTE_MASK
    return PX4_CUSTOM_MAIN_MODE_NAMES.get(main)


def px4CustomSubMode(custom_mode: int | None) -> int | None:
    """PX4 `custom_mode` 整数 → 子模式（AUTO 下才有意义）。"""
    if custom_mode is None:
        return None
    return (int(custom_mode) >> PX4_CUSTOM_MODE_SUB_SHIFT) & PX4_CUSTOM_MODE_BYTE_MASK


#: AUTO 子模式名 → 名字（与 PX4_CUSTOM_SUB_MODE_AUTO_NAMES 互逆）
PX4_CUSTOM_SUB_MODE_AUTO_NAMES_INVERSE: dict[int, str] = {
    value: key for key, value in PX4_CUSTOM_SUB_MODE_AUTO_NAMES.items()
}


def px4CustomSubModeName(custom_mode: int | None) -> str | None:
    """PX4 `custom_mode` → 完整模式名，AUTO 下带子模式。

    只报主模式无法区分 **AUTO Land 与 AUTO Return**（两者 mainMode 都是 auto，
    仅 sub_mode 不同：land=6、rtl=5），而恢复逻辑与验收都必须精确区分，
    所以这里返回 ``auto:land`` / ``auto:rtl`` 这类带子模式的名字。
    未知子模式返回 ``auto:<n>`` 而不是编一个名字。
    """
    main = (int(custom_mode) >> PX4_CUSTOM_MODE_MAIN_SHIFT) & PX4_CUSTOM_MODE_BYTE_MASK \
        if custom_mode is not None else None
    if main is None:
        return None
    name = PX4_CUSTOM_MAIN_MODE_NAMES.get(main)
    if name != "auto":
        return name
    sub = px4CustomSubMode(custom_mode)
    if sub is None:
        return "auto"
    return f"auto:{PX4_CUSTOM_SUB_MODE_AUTO_NAMES_INVERSE.get(sub, sub)}"


def px4CustomMode(mainMode: int, sub_mode: int = 0) -> int:
    """主/子模式 → PX4 `custom_mode` 整数（HEARTBEAT.custom_mode 用的就是这个）。"""
    return (((int(sub_mode) & PX4_CUSTOM_MODE_BYTE_MASK) << PX4_CUSTOM_MODE_SUB_SHIFT)
            | ((int(mainMode) & PX4_CUSTOM_MODE_BYTE_MASK) << PX4_CUSTOM_MODE_MAIN_SHIFT))


@dataclass(frozen=True)
class VehicleState:
    """一次「飞控当前状态」快照。所有字段都可能为 None——不要用默认值冒充观测值。

    `armed` **只**来自 HEARTBEAT 的 SAFETY_ARMED 位；`commanded_armed` 是我们
    发过什么命令。两者分开，是因为「写了命令」不等于「飞控已照做」。
    """

    connected: bool = False
    #: 来自 HEARTBEAT base_mode & MAV_MODE_FLAG_SAFETY_ARMED
    armed: bool = False
    #: 来自 HEARTBEAT
    mode_name: str | None = None
    custom_mode: int | None = None
    custom_main_mode: int | None = None
    custom_sub_mode: int | None = None
    system_status: int | None = None
    autopilot: int | None = None
    mavlink_version: int | None = None
    #: 我们最近一次命令里的意图（不是观测值）
    commanded_armed: bool | None = None
    commanded_mode_name: str | None = None
    #: ATTITUDE
    yaw_rad: float | None = None
    #: 以下姿态量**只**来自 ATTITUDE；未收到就是 None。
    #: 绝不能用 0 代替：0 意味着"水平且不转"，会被下游的倾角/角速率筛选当成合格样本。
    roll_rad: float | None = None
    pitch_rad: float | None = None
    yaw_rate_rad_s: float | None = None
    #: LOCAL_POSITION_NED（NED 米 / 米每秒）
    position_ned_m: tuple[float, float, float] | None = None
    velocity_ned_m_s: tuple[float, float, float] | None = None
    #: 时间戳年龄（秒，基于注入时钟；None = 从未收到）
    heartbeat_age_s: float | None = None
    attitude_age_s: float | None = None
    #: ATTITUDE 到达本机的单调时钟时刻；用于与 VIO 到达时刻做保守配对。
    attitude_received_mono_s: float | None = None
    current_mode_detail: str | None = None
    intended_mode_detail: str | None = None
    current_mode_age_s: float | None = None
    frame_reset_epoch: int = 0
    frame_reset_age_s: float | None = None
    landed_state: int | None = None
    landed_age_s: float | None = None
    position_age_s: float | None = None
    position_received_mono_s: float | None = None
    heartbeat_timeout_s: float | None = None
    #: PX4 启动时钟（毫秒，来自 ATTITUDE/LOCAL_POSITION_NED 的 time_boot_ms）
    boot_time_ms: int | None = None
    #: 每检测到一次 PX4 重启 +1（failsafe 层据此丢弃旧时间线/旧状态）
    restart_epoch: int = 0
    #: 状态数据来源标识（后端名），便于诊断「这份状态是谁给的」
    source: str = "unknown"
    last_error: str | None = None

    @property
    def is_offboard(self) -> bool:
        return self.custom_main_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD

    @property
    def mode_detail(self) -> str | None:
        """完整模式名（AUTO 带子模式）。只报主模式无法区分 Land 与 Return。"""
        return px4CustomSubModeName(self.custom_mode)

    @property
    def isAutoLand(self) -> bool:
        """当前是否处于 **AUTO Land**（且仅在观测到的情况下）。"""
        return (self.custom_main_mode == PX4_CUSTOM_MAIN_MODE_AUTO
                and self.custom_sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_NAMES["land"])

    @property
    def isAutoReturn(self) -> bool:
        """当前是否处于 **AUTO Return（RTL）**。"""
        return (self.custom_main_mode == PX4_CUSTOM_MAIN_MODE_AUTO
                and self.custom_sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_NAMES["rtl"])


@dataclass(frozen=True)
class RecordedCall:
    """`FakePx4Backend` 记录的一次调用。"""

    name: str
    args: tuple = ()
    kwargs: dict = field(default_factory=dict)
    mono_s: float = 0.0
    accepted: bool = False
    reason: str | None = None
    detail: str = ""


class ManualClock:
    """可手动推进的时钟（`FakePx4Backend` 的确定性时间源）。

    真后端不允许用它的单调时钟做业务判定以外的假设；这里只为了让
    「心跳超时」「心跳年龄」这类时间相关行为在测试里完全可复现。
    """

    def __init__(self, start: float = 1000.0) -> None:
        self._t = float(start)

    def __call__(self) -> float:
        return self._t

    def advance(self, dtS: float) -> float:
        self._t += float(dtS)
        return self._t

    def set(self, tS: float) -> None:
        self._t = float(tS)


# ============================================================================
# 协议：算法模块只被允许看到这些
# ============================================================================
@runtime_checkable
class Px4Backend(Protocol):
    """飞控后端的最小能力面（SW-001 的隔离面）。

    【硬禁令 · FC-004】本协议**不允许**出现 PWM / DShot / 执行器 / 电机 /
    舵机 / 油门 直接命令面。Companion 只发高层 setpoint（位置/速度/加速度/
    偏航）与模式、解锁命令；电机输出是 PX4 内部职责。
    `assertNoActuatorSurface()` 会真实断言这一点，见同名单测。

    【待定】控制器在 Companion 还是 PX4 **尚未确定**。本协议刻意只描述
    「读状态 / 发 setpoint / 切模式 / 解锁」，不说话谁算控制律：两种方案
    都只需要这些能力。切换后端实现时算法模块不应改动。

    约定（两个实现必须一致）
    ------------------------
    - `arm()` / `set_mode()` 之类的返回值表示「命令是否已被接受并发出」，
      **不代表飞控已照做**。已确认状态只在 `readVehicleState()` 里
      （`armed` / `mode_name` 来自 HEARTBEAT），另以 `commanded_*` 暴露意图。
    - `sendSetpoint()` 的返回值表示「这次 setpoint 被本后端接受并发出」。
      MAVLink 是无连接协议，没有逐帧回执；是否真的被 PX4 吃下，只能由
      后续状态/模式变化间接判断。
    - 任何拒绝都必须在 `streamDiagnostics()` 里留下带 reason 字符串的记录。
    """

    def connect(self) -> bool:
        """建立链路。返回是否可用。幂等。"""
        ...

    def close(self) -> None:
        """关闭链路并停止后台线程。幂等。"""
        ...

    def isConnected(self) -> bool:
        """**基于心跳年龄**判断是否与飞控通着（不是「socket 还在」）。"""
        ...

    def linkState(self) -> dict:
        """链路摘要：心跳年龄、最近错误、连接串、计数器。"""
        ...

    def readVehicleState(self) -> VehicleState:
        """读取最近一次缓存的飞控状态（不阻塞）。"""
        ...

    def sendSetpoint(self, setpoint: "Px4SetpointLike", type_mask: int | None = None) -> bool:
        """发送一条高层本地 NED setpoint（msg 84）。

        `type_mask` 显式给定时以它为准；为 None 时回退到 `setpoint.type_mask`
        属性；两者都没有则**拒绝**（不猜轴，见模块 docstring 的取值规则）。
        """
        ...

    def arm(self, arm: bool = True) -> bool:
        """解锁/上锁（MAV_CMD_COMPONENT_ARM_DISARM）。"""
        ...

    def disarm(self) -> bool:
        """上锁（等价 `arm(False)`）。"""
        ...

    def setOffboardMode(self) -> bool:
        """切到 PX4 Offboard 主模式。"""
        ...

    def setPositionMode(self) -> bool:
        """切到 PX4 位置控制主模式（非 Offboard 的对照模式）。"""
        ...

    def set_mode(self, name: str) -> bool:
        """按名字切换 PX4 主模式（见 `MODE_NAME_TO_CUSTOM_MAIN_MODE`）。"""
        ...

    def streamDiagnostics(self) -> dict:
        """完整诊断：计数器、拒绝原因、命令收发状态（可直接 JSON 序列化）。"""
        ...


@runtime_checkable
class Px4SetpointLike(Protocol):
    """`Px4LocalSetpoint`-形状 + `type_mask` 的结构约定（离线替身用）。

    真实 `px4_frames.Px4LocalSetpoint` **没有** `type_mask` 字段，此时调用方
    必须把 `px4_frames.TypeMask.forMode(...)` 的 mask 通过 `sendSetpoint`
    的 `type_mask=` 参数显式传来。
    """

    position_m: Sequence[float] | None
    velocity_m_s: Sequence[float] | None
    acceleration_m_s2: Sequence[float] | None
    yaw_rad: float | None
    yaw_rate_rad_s: float | None
    type_mask: int


# ============================================================================
# 共享：计数器与拒绝记录（两个后端保持同名，便于同一个节点代码读）
# ============================================================================
def _newSharedCounters() -> dict[str, int]:
    return {
        # 链路
        "connect_attempts": 0,
        "connect_failures": 0,
        "refusals": 0,
        # 上行
        "messages_rx": 0,
        "messages_other": 0,
        "rejected_source": 0,
        "heartbeats_rx": 0,
        "heartbeats_other_autopilot": 0,
        "heartbeat_gaps": 0,
        "heartbeat_seq_resets": 0,
        "attitude_rx": 0,
        "local_position_rx": 0,
        "sys_status_rx": 0,
        "status_text_rx": 0,
        # 下行
        "heartbeats_tx": 0,
        "setpoints_built": 0,
        "setpoints_sent": 0,
        "setpoints_rejected": 0,
        "setpoints_suppressed_dry_run": 0,
        "setpoints_not_sent": 0,
        "setpoint_ignored_axes_zeroed": 0,
        "commands_sent": 0,
        "commands_suppressed_dry_run": 0,
        "command_acks_rx": 0,
        "commands_accepted": 0,
        "commands_rejected": 0,
        "commands_in_progress": 0,
        "arm_refused": 0,
        # 故障
        "parse_errors": 0,
        "rx_exceptions": 0,
        "handle_exceptions": 0,
        "tx_errors": 0,
        "read_timeouts": 0,
        "px4_restarts": 0,
    }


@dataclass
class _RefusalLog:
    """有界的拒绝记录：每一条都必须能在诊断里看到原因字符串。"""

    maxlen: int = 64
    records: deque = field(default_factory=deque)

    def __post_init__(self) -> None:
        self.records = deque(maxlen=max(int(self.maxlen), 1))

    def add(self, reason: str, detail: str, mono_s: float) -> dict:
        rec = {"reason": reason, "detail": detail, "mono_s": mono_s}
        self.records.append(rec)
        return rec


# ============================================================================
# FakePx4Backend：确定性内存实现
# ============================================================================
class FakePx4Backend:
    """内存版 `Px4Backend`：记录每一次调用，可注入时钟，可注入故障。

    用途：离线测试 ROS 节点与 failsafe 接线。**不碰网络、不碰串口。**

    可模拟的故障/事件
    -----------------
    - 无心跳：`connect()` 前设 `heartbeatActive=False`，或 `simulateHeartbeatLoss()`；
    - 链路丢失：`simulateLinkLoss()` / `restoreLink()`；
    - PX4 重启：`simulatePx4Restart()`（boot 时间归零、解锁与模式状态清空、
      心跳中断，`restart_epoch` +1，事件进 `restart_events`）；
    - setpoint 被拒：`setSetpointAcceptance(False)` 或 `rejectNextSetpoints(n)`；
    - 命令被拒：`setCommandAcceptance(False)`；
    - 模式/解锁迁移：`arm()` 只写 `commanded_armed`；`armed` 只在
      `acknowledgeArm()`（或 `autoAck=True` 时的 `arm()`）后变化——
      和真后端一样，「写了命令」不等于「已解锁」。

    `autoAck=True`（默认）时 `arm()`/`set_mode()` 立即生效，方便节点级测试；
    要测「命令已发但未确认」的窗口，把它设成 False。
    """

    def __init__(
        self,
        *,
        clock: Callable[[], float] | None = None,
        heartbeat_timeout_s: float = 3.0,
        linkUp: bool = False,
        heartbeatActive: bool = True,
        allow_arming: bool = True,
        autoAck: bool = True,
        requireConnectionForSetpoint: bool = True,
        boot_time_ms: int = 1000,
        connectSucceeds: bool = True,
    ) -> None:
        self._clock: Callable[[], float] = clock or time.monotonic
        self.heartbeat_timeout_s = float(heartbeat_timeout_s)
        self.allow_arming = bool(allow_arming)
        self.autoAck = bool(autoAck)
        self.requireConnectionForSetpoint = bool(requireConnectionForSetpoint)
        self.connectSucceeds = bool(connectSucceeds)

        self.linkUp = bool(linkUp)
        self.heartbeatActive = bool(heartbeatActive)
        self.boot_time_ms = int(boot_time_ms)

        self.counters: dict[str, int] = _newSharedCounters()
        self.calls: list[RecordedCall] = []
        self.refusals = _RefusalLog()
        self.restart_events: list[dict] = []
        self.setpoints: list[Any] = []
        self.commands: dict[int, dict] = {}
        self.armed = False
        self.commanded_armed: bool | None = None
        self.custom_mode: int = px4CustomMode(PX4_CUSTOM_MAIN_MODE_MANUAL)
        self.commanded_mode_name: str | None = None
        self.yaw_rad: float | None = None
        self.roll_rad: float | None = None
        self.pitch_rad: float | None = None
        self.yaw_rate_rad_s: float | None = None
        self.position_ned_m: tuple[float, float, float] | None = None
        self.velocity_ned_m_s: tuple[float, float, float] | None = None
        self.last_error: str | None = None
        self.restart_epoch = 0

        self._hbMono: float | None = None
        self._attitudeMono: float | None = None
        self._currentMode = self._intendedMode = self._currentModeMono = None
        self._odomResetCounter = self._odomResetMono = None
        self._frameResetEpoch = 0
        self.sih_setpoint_inhibited = False
        self._observationStreamsRequested = False
        self._landedState: int | None = None
        self._landedMono: float | None = None
        self._positionMono: float | None = None
        self._setpointAcceptance = True
        self._rejectSetpointsRemaining = 0
        self._commandAcceptance = True
        self._hbSeq = 0
        if self.linkUp and self.heartbeatActive:
            self._hbMono = self._clock()

    # ------------------------------------------------------------ 协议实现
    def connect(self) -> bool:
        self.counters["connect_attempts"] += 1
        if not self.connectSucceeds:
            self.refusals.add(REFUSAL_CONNECT_FAILED, "connect_succeeds=False", self._clock())
            self.counters["connect_failures"] += 1
            self.last_error = "connect_failed: connect_succeeds=False"
            self._record("connect", accepted=False, reason=REFUSAL_CONNECT_FAILED)
            return False
        self.linkUp = True
        if self.heartbeatActive:
            self._hbMono = self._clock()
        self._record("connect", accepted=True)
        return True

    def close(self) -> None:
        self.linkUp = False
        self._record("close", accepted=True)

    def isConnected(self) -> bool:
        if not self.linkUp or not self.heartbeatActive or self._hbMono is None:
            return False
        return (self._clock() - self._hbMono) <= self.heartbeat_timeout_s

    def disconnectReason(self) -> str | None:
        """为什么现在不算连通（诊断用；连通时返回 None）。"""
        if not self.linkUp:
            return "link_down"
        if not self.heartbeatActive or self._hbMono is None:
            return "heartbeat_absent"
        if (self._clock() - self._hbMono) > self.heartbeat_timeout_s:
            return "heartbeat_timeout"
        return None

    def linkState(self) -> dict:
        now = self._clock()
        return {
            "connection": "fake:in-memory",
            "connected": self.isConnected(),
            "disconnect_reason": self.disconnectReason(),
            "heartbeat_age_s": None if self._hbMono is None else now - self._hbMono,
            "heartbeat_timeout_s": self.heartbeat_timeout_s,
            "last_error": self.last_error,
            "counters": dict(self.counters),
            "refusal_count": self.counters["refusals"],
            "last_refusal": dict(self.refusals.records[-1]) if self.refusals.records else None,
            "dry_run": False,
            "allow_arming": self.allow_arming,
            "restart_epoch": self.restart_epoch,
            "px4_restart_events": len(self.restart_events),
            "px4_boot_time_ms": self.boot_time_ms,
        }

    def readVehicleState(self) -> VehicleState:
        now = self._clock()
        return VehicleState(
            connected=self.isConnected(),
            armed=bool(self.armed),
            mode_name=px4CustomMainModeName(self.custom_mode),
            custom_mode=int(self.custom_mode),
            custom_main_mode=(int(self.custom_mode) >> PX4_CUSTOM_MODE_MAIN_SHIFT) & PX4_CUSTOM_MODE_BYTE_MASK,
            custom_sub_mode=(int(self.custom_mode) >> PX4_CUSTOM_MODE_SUB_SHIFT) & PX4_CUSTOM_MODE_BYTE_MASK,
            system_status=MAV_STATE_ACTIVE,
            autopilot=MAV_AUTOPILOT_PX4,
            commanded_armed=self.commanded_armed,
            commanded_mode_name=self.commanded_mode_name,
            yaw_rad=self.yaw_rad,
            roll_rad=self.roll_rad,
            pitch_rad=self.pitch_rad,
            yaw_rate_rad_s=self.yaw_rate_rad_s,
            position_ned_m=self.position_ned_m,
            velocity_ned_m_s=self.velocity_ned_m_s,
            heartbeat_age_s=None if self._hbMono is None else now - self._hbMono,
            attitude_age_s=None if self._attitudeMono is None else now - self._attitudeMono,
            attitude_received_mono_s=self._attitudeMono,
            position_age_s=None if self._positionMono is None else now - self._positionMono,
            position_received_mono_s=self._positionMono,
            heartbeat_timeout_s=self.heartbeat_timeout_s,
            boot_time_ms=self.boot_time_ms,
            restart_epoch=self.restart_epoch,
            source="fake",
            last_error=self.last_error,
        )

    def sendSetpoint(self, setpoint: Any, type_mask: int | None = None) -> bool:
        if setpoint is None:
            self._refuse(REFUSAL_SETPOINT_NOT_LIKE, "setpoint 为 None", "send_setpoint")
            self.counters["setpoints_rejected"] += 1
            return False
        if type_mask is None and not hasattr(setpoint, "type_mask"):
            # 与真后端同一条规则：没有 mask 就不猜轴，直接拒绝
            self._refuse(
                REFUSAL_SETPOINT_MASK_MISSING,
                "setpoint 无 type_mask 属性且未显式传 type_mask=",
                "send_setpoint",
            )
            self.counters["setpoints_rejected"] += 1
            return False
        self.counters["setpoints_built"] += 1
        if self.requireConnectionForSetpoint and not self.isConnected():
            reason = self.disconnectReason() or "not_connected"
            self._refuse(reason, "setpoint 被拒：链路未连通", "send_setpoint")
            self.counters["setpoints_not_sent"] += 1
            self._record("send_setpoint", setpoint, accepted=False, reason=reason)
            return False
        if self._rejectSetpointsRemaining > 0:
            self._rejectSetpointsRemaining -= 1
            self.counters["setpoints_rejected"] += 1
            self._refuse(REFUSAL_SETPOINT_REJECTED_BY_FAKE, "注入的 setpoint 拒绝",
                         "send_setpoint")
            self._record("send_setpoint", setpoint, accepted=False,
                         reason=REFUSAL_SETPOINT_REJECTED_BY_FAKE)
            return False
        if not self._setpointAcceptance:
            self.counters["setpoints_rejected"] += 1
            self._refuse(REFUSAL_SETPOINT_REJECTED_BY_FAKE, "setpoint_acceptance=False",
                         "send_setpoint")
            self._record("send_setpoint", setpoint, accepted=False,
                         reason=REFUSAL_SETPOINT_REJECTED_BY_FAKE)
            return False
        self.setpoints.append(setpoint)
        self.counters["setpoints_sent"] += 1
        self._record("send_setpoint", setpoint, accepted=True)
        return True

    def sendAttitudeSetpoint(self, setpoint):
        from .attitude_control import rotation
        try:
            rotation(setpoint.orientation_xyzw)
            if not math.isfinite(setpoint.thrust) or not 0 <= setpoint.thrust <= 1:
                raise ValueError("thrust_bounds")
        except (ValueError, TypeError, AttributeError):
            self.counters["setpoints_rejected"] += 1
            return False
        self.counters["setpoints_built"] += 1
        if ((self.requireConnectionForSetpoint and not self.isConnected())
                or not self._setpointAcceptance or self._rejectSetpointsRemaining > 0):
            self._rejectSetpointsRemaining = max(0, self._rejectSetpointsRemaining-1)
            self.counters["setpoints_not_sent"] += 1
            self._record("send_attitude_setpoint", setpoint, accepted=False)
            return False
        self.setpoints.append(setpoint)
        self.counters["setpoints_sent"] += 1
        self._record("send_attitude_setpoint", setpoint, accepted=True)
        return True

    def arm(self, arm: bool = True) -> bool:
        return self._command(
            "arm" if arm else "disarm",
            MAV_CMD_COMPONENT_ARM_DISARM,
            ARMING_ACTION_ARM if arm else ARMING_ACTION_DISARM,
            isArming=True,
            onAccept=lambda: self._applyArm(bool(arm)),
        )

    def disarm(self) -> bool:
        return self.arm(False)

    def setOffboardMode(self) -> bool:
        return self.set_mode("offboard")

    def setPositionMode(self) -> bool:
        return self.set_mode("posctl")

    def set_mode(self, name: str) -> bool:
        key = str(name).strip().lower()
        main, sub = _resolveModeName(key)
        if main is None:
            self._refuse(REFUSAL_UNKNOWN_MODE, f"name={name!r}", "set_mode")
            self._record("set_mode", name, accepted=False, reason=REFUSAL_UNKNOWN_MODE)
            return False
        custom = px4CustomMode(main, sub)
        return self._command(
            f"set_mode:{key}",
            MAV_CMD_DO_SET_MODE,
            custom,
            isArming=False,
            onAccept=lambda: self._applyMode(key, custom),
        )

    def streamDiagnostics(self) -> dict:
        out = self.linkState()
        out.update(
            {
                "backend": "fake",
                "rx_thread_alive": False,
                "calls": len(self.calls),
                "call_names": [c.name for c in self.calls[-32:]],
                "refusals": [dict(r) for r in self.refusals.records],
                "restart_events": [dict(e) for e in self.restart_events],
                "commands": {str(k): dict(v) for k, v in self.commands.items()},
                "setpoints_sent": len(self.setpoints),
                "last_setpoint": self.setpoints[-1] if self.setpoints else None,
                "commanded_armed": self.commanded_armed,
                "acknowledged_armed": bool(self.armed),
                "commanded_mode_name": self.commanded_mode_name,
                "acknowledged_mode_name": px4CustomMainModeName(self.custom_mode),
            }
        )
        return out

    # ------------------------------------------------------- 注入/模拟接口
    def feedHeartbeat(self) -> float:
        """模拟收到一条心跳：心跳流恢复且时间戳刷新。"""
        self.heartbeatActive = True
        self._hbMono = self._clock()
        self.counters["heartbeats_rx"] += 1
        self._hbSeq = (self._hbSeq + 1) % 256
        return self._hbMono

    def simulateHeartbeatLoss(self) -> None:
        """心跳停止（但链路未必断）：`isConnected()` 应变 False。"""
        self.heartbeatActive = False

    def simulateLinkLoss(self, reason: str = "link_lost") -> None:
        """链路整体丢失：连 socket/串口都没了。"""
        self.linkUp = False
        self.heartbeatActive = False
        self.last_error = reason
        self.counters["refusals"] += 1
        self.refusals.add(reason, "simulate_link_loss", self._clock())

    def restoreLink(self, *, emitHeartbeat: bool = True) -> None:
        self.linkUp = True
        self.heartbeatActive = True
        if emitHeartbeat:
            self.feedHeartbeat()
        else:
            self._hbMono = None

    def simulatePx4Restart(self, *, new_boot_time_ms: int = 0) -> dict:
        """模拟 PX4 重启：boot 时间归零、状态清空、心跳中断。

        `restart_epoch` +1 并追加一条事件——failsafe 层应当据此丢弃旧状态。
        """
        oldBoot = self.boot_time_ms
        self.restart_epoch += 1
        self.counters["px4_restarts"] += 1
        self.boot_time_ms = int(new_boot_time_ms)
        # 重启后 PX4 必然从「未解锁 + 默认模式」开始；旧的「已确认」状态必须丢掉，
        # 否则 failsafe 会以为飞机还停在重启前的模式/解锁状态。
        self.armed = False
        self.commanded_armed = None
        self.commanded_mode_name = None
        self.custom_mode = px4CustomMode(PX4_CUSTOM_MAIN_MODE_MANUAL)
        self._hbMono = None
        self._attitudeMono = None
        self._positionMono = None
        self.yaw_rad = None
        self.position_ned_m = None
        self.velocity_ned_m_s = None
        event = {
            "epoch": self.restart_epoch,
            "reason": "simulated_px4_restart",
            "mono_s": self._clock(),
            "old_boot_time_ms": oldBoot,
            "new_boot_time_ms": self.boot_time_ms,
        }
        self.restart_events.append(event)
        return event

    def setSetpointAcceptance(self, accepted: bool) -> None:
        self._setpointAcceptance = bool(accepted)

    def rejectNextSetpoints(self, count: int) -> None:
        self._rejectSetpointsRemaining = max(int(count), 0)

    def setCommandAcceptance(self, accepted: bool) -> None:
        self._commandAcceptance = bool(accepted)

    def acknowledgeArm(self, armed: bool = True) -> None:
        """模拟 HEARTBEAT 的 SAFETY_ARMED 位变化（这才是「已确认」）。"""
        self.armed = bool(armed)

    def acknowledgeMode(self, custom_mode: int) -> None:
        self.custom_mode = int(custom_mode)

    def feedAttitude(self, yaw_rad: float, *, roll_rad: float | None = None,
                      pitch_rad: float | None = None,
                      yaw_rate_rad_s: float | None = None) -> None:
        """喂一帧姿态。roll/pitch/yaw_rate 不传则保持 None（= 未观测）。

        这与真实后端一致：没收到就是 None，不能默认成 0。
        """
        self.yaw_rad = float(yaw_rad)
        self.roll_rad = None if roll_rad is None else float(roll_rad)
        self.pitch_rad = None if pitch_rad is None else float(pitch_rad)
        self.yaw_rate_rad_s = None if yaw_rate_rad_s is None else float(yaw_rate_rad_s)
        self._attitudeMono = self._clock()
        self.counters["attitude_rx"] += 1

    def feedPositionNed(self, x: float, y: float, z: float) -> None:
        self.position_ned_m = (float(x), float(y), float(z))
        self._positionMono = self._clock()
        self.counters["local_position_rx"] += 1

    def feedVelocityNed(self, vx: float, vy: float, vz: float) -> None:
        self.velocity_ned_m_s = (float(vx), float(vy), float(vz))

    # ------------------------------------------------------------- 记录/内部
    def callsNamed(self, name: str) -> list[RecordedCall]:
        return [c for c in self.calls if c.name == name]

    def clearCalls(self) -> None:
        self.calls.clear()

    def _record(
        self,
        name: str,
        *args: Any,
        accepted: bool,
        reason: str | None = None,
        detail: str = "",
        **kwargs: Any,
    ) -> RecordedCall:
        call = RecordedCall(
            name=name,
            args=tuple(args),
            kwargs=dict(kwargs),
            mono_s=self._clock(),
            accepted=bool(accepted),
            reason=reason,
            detail=detail,
        )
        self.calls.append(call)
        return call

    def _refuse(self, reason: str, detail: str, name: str) -> dict:
        self.counters["refusals"] += 1
        self.counters[f"refused_{reason}"] = self.counters.get(f"refused_{reason}", 0) + 1
        self.last_error = f"refused: {reason}" + (f": {detail}" if detail else "")
        rec = self.refusals.add(reason, detail, self._clock())
        self._record(name, accepted=False, reason=reason, detail=detail)
        return rec

    def _command(
        self,
        name: str,
        command: int,
        param: int,
        *,
        isArming: bool,
        onAccept: Callable[[], None],
    ) -> bool:
        if isArming and not self.allow_arming:
            self.counters["arm_refused"] += 1
            self._refuse(REFUSAL_ARMING_NOT_ALLOWED, "allow_arming=False", name)
            return False
        if not self.isConnected():
            reason = self.disconnectReason() or "not_connected"
            self._refuse(reason, f"命令 {command} 未发出：链路未连通", name)
            return False
        if not self._commandAcceptance:
            self._refuse(REFUSAL_COMMAND_REJECTED_BY_FAKE, f"command={command}", name)
            self._record(name, accepted=False, reason=REFUSAL_COMMAND_REJECTED_BY_FAKE)
            return False
        self.commands[command] = {
            "name": name,
            "command": int(command),
            "param": int(param),
            "sent_mono_s": self._clock(),
            "ack_result": MAV_RESULT_ACCEPTED,
            "ack_result_name": "ACCEPTED",
        }
        self.counters["commands_sent"] += 1
        self.counters["command_acks_rx"] += 1
        self.counters["commands_accepted"] += 1
        onAccept()
        self._record(name, param, accepted=True)
        return True

    def _applyArm(self, arm: bool) -> None:
        self.commanded_armed = bool(arm)
        if self.autoAck:
            self.armed = bool(arm)

    def _applyMode(self, name: str, custom_mode: int) -> None:
        self.commanded_mode_name = name
        if self.autoAck:
            self.custom_mode = int(custom_mode)


# ============================================================================
# MavlinkPx4Backend：真 MAVLink 实现（本任务只在离线回环里测过）
# ============================================================================
def _isLoopbackHost(host: str) -> bool:
    """只认字面量 `localhost` 与回环 IP；刻意不做 DNS 解析。

    为什么不解析名字：把 `my-drone.local` 解析后放行，等于把「允许连谁」
    交给 DNS/hosts；而本后端的误连代价是「可能给真机解锁」。拒绝比解析安全。
    """
    h = (host or "").strip().strip("[]")
    if not h:
        return False
    if h.lower() == "localhost":
        return True
    try:
        return ipaddress.ip_address(h).is_loopback
    except ValueError:
        return False


def _parseConnection(connection: str) -> tuple[str, str | None, int | None]:
    """连接串 → (kind, host, port)。kind ∈ {udpin, udpout, udp, tcp, serial, ...}。

    与 `mavlink_imu_node` 保持同一套写法（`serial:<dev>` / `udpin:ip:port`）。
    `0.0.0.0` 与空 host 一律不是回环——它们会监听所有网卡。
    """
    text = str(connection).strip()
    low = text.lower()
    for prefix, kind in (
        ("udpin:", "udpin"),
        ("udpout:", "udpout"),
        ("udp:", "udp"),
        ("tcpin:", "tcpin"),
        ("tcp:", "tcp"),
        ("serial:", "serial"),
        ("file:", "file"),
    ):
        if low.startswith(prefix):
            rest = text[len(prefix):]
            if kind in ("serial", "file"):
                return kind, rest, None
            host, _, portS = rest.rpartition(":")
            if not host or not portS.isdigit():
                return "bad", None, None
            return kind, host, int(portS)
    # 裸 host:port —— pymavlink 视为 udpout
    host, _, portS = text.rpartition(":")
    if host and portS.isdigit():
        return "udpout", host, int(portS)
    return "bad", None, None


def _resolveModeName(name: str) -> tuple[int | None, int]:
    """模式名 → (custom 主模式, 子模式)。`auto:<sub>` 支持 AUTO 子模式。"""
    key = str(name).strip().lower()
    main = MODE_NAME_TO_CUSTOM_MAIN_MODE.get(key)
    sub = 0
    if main is None and ":" in key:
        head, _, tail = key.partition(":")
        main = MODE_NAME_TO_CUSTOM_MAIN_MODE.get(head)
        if main == PX4_CUSTOM_MAIN_MODE_AUTO:
            # AUTO 且 sub=0 时 PX4 落到 AUTO_MISSION（源码已核验）
            sub = PX4_CUSTOM_SUB_MODE_AUTO_NAMES.get(tail, 0)
    return main, sub


class MavlinkPx4Backend:
    """pymavlink 实现的 `Px4Backend`（PX4 MAVLink，含防误连硬闸门）。

    **这个后端有能力给真机解锁**，所以默认状态是「什么都干不了」：
    只允许回环地址、`allow_arming=False`、`dry_run=True`、不建串口。
    每一项放开都需要显式参数，且每次拒绝都会出现在 `streamDiagnostics()`。

    安全闸门一览（默认值全部是最保守的一侧）
    ----------------------------------------
    ================================  =======  =====================================
    参数                               默认      作用
    ================================  =======  =====================================
    `dry_run`                         True     不发任何字节；setpoint 仍构造+校验+计数
    `allow_arming`                    False    `arm()` 直接拒绝（原因可见）
    `allow_non_loopback`              False    非回环 host 拒绝连接
    `allow_arming_on_non_loopback`    False    非回环/串口链路上再解锁需要它
    `allow_serial`                    False    `serial:` 拒绝连接（绝不默认开串口）
    `requirePx4Autopilot`           True     非 PX4 心跳不算连通
    ================================  =======  =====================================

    接收线程只读上行（HEARTBEAT / ATTITUDE / LOCAL_POSITION_NED / SYS_STATUS /
    STATUS_TEXT / COMMAND_ACK）；**不接收 HIGHRES_IMU**——IMU 由
    `mavlink_imu_node` 单独负责，本模块不重复。

    已知局限（诚实记录）
    --------------------
    - pymavlink 的 `mavudp.write()` 吞掉 `socket.error`，`udpin` 无对端时静默不发。
      所以「发出去了」只表示我们调用了 write 且本地没报错，**不是投递证明**。
      真正的确认只有 `COMMAND_ACK` 与后续 HEARTBEAT。
    - `close()` 只能停线程与关 socket；对端不会收到任何「再见」。
    - 本模块**没有**在 SITL 或真机上跑过（本任务禁止起 SITL/接飞控）。
      已测范围：离线回环 UDP 对端 + 协议/守卫单测。
    """

    def __init__(
        self,
        connection: str = "udpin:127.0.0.1:14540",
        *,
        baud: int = 57600,
        dialect: str = "development",
        target_system: int = 1,
        target_component: int = 1,
        source_system: int = 255,
        source_component: int = 190,
        accept_any_component: bool = True,
        requirePx4Autopilot: bool = True,
        heartbeat_timeout_s: float = 3.0,
        read_timeout_s: float = 0.1,
        restartBootToleranceMs: int = 1000,
        allow_arming: bool = False,
        dry_run: bool = True,
        allow_non_loopback: bool = False,
        allow_arming_on_non_loopback: bool = False,
        allow_serial: bool = False,
        send_heartbeat: bool = True,
        heartbeat_rate_hz: float = 1.0,
        clock: Callable[[], float] | None = None,
        mavlinkConnectionFactory: Callable[..., Any] | None = None,
    ) -> None:
        self._connection = str(connection)
        self._baud = int(baud)
        self._dialect = str(dialect)
        self.target_system = int(target_system)
        self.target_component = int(target_component)
        self._sourceSystem = int(source_system)
        self._sourceComponent = int(source_component)
        self.accept_any_component = bool(accept_any_component)
        self.requirePx4Autopilot = bool(requirePx4Autopilot)
        self.heartbeat_timeout_s = float(heartbeat_timeout_s)
        self.read_timeout_s = max(float(read_timeout_s), 1e-3)
        self.restartBootToleranceMs = int(restartBootToleranceMs)
        self.allow_arming = bool(allow_arming)
        self.dry_run = bool(dry_run)
        self.allow_non_loopback = bool(allow_non_loopback)
        self.allow_arming_on_non_loopback = bool(allow_arming_on_non_loopback)
        self.allow_serial = bool(allow_serial)
        self.send_heartbeat = bool(send_heartbeat)
        self.heartbeat_rate_hz = float(heartbeat_rate_hz)
        self._clock: Callable[[], float] = clock or time.monotonic
        self._factory = mavlinkConnectionFactory

        self._kind, self._host, self._port = _parseConnection(self._connection)
        #: 该链路是否回环。只有回环才允许在 `allow_arming=True` 时解锁。
        self._linkIsLoopback = bool(
            self._host is not None and _isLoopbackHost(self._host)
        )

        self._conn: Any = None
        self._rxThread: threading.Thread | None = None
        self._stop = threading.Event()
        self._txLock = threading.Lock()        # 保护写帧（node 定时器线程 vs 接收线程）
        self._stateLock = threading.Lock()     # 保护状态字段
        self._counterLock = threading.Lock()

        self.counters: dict[str, int] = _newSharedCounters()
        self.counters["bytes_tx"] = 0
        self.counters["packets_tx"] = 0
        self.counters["pymavlink_receive_errors"] = 0
        self.counters["bad_data_bytes"] = 0

        self.refusals = _RefusalLog()
        self.last_error: str | None = None
        self._peerSeen = False
        self._hbTxMono: float | None = None
        self._localEndpoint: str | None = None
        #: 被来源校验拒掉的 (sysid, compid)：设计行为，只作诊断，不进 last_error
        self._rejectedSourceLast: tuple[int, int] | None = None
        self._sourcesRejected: set[tuple[int, int]] = set()

        # 上行状态
        self._hbMono: float | None = None
        self._hbBaseMode: int | None = None
        self._hbCustomMode: int | None = None
        self._hbSystemStatus: int | None = None
        self._hbMavlinkVersion: int | None = None
        self._autopilot: int | None = None
        self._hbSeqLast: int | None = None
        self._attitudeMono: float | None = None
        self._yawRad: float | None = None
        self._rollRad: float | None = None
        self._pitchRad: float | None = None
        self._yawRateRadS: float | None = None
        self._currentMode = self._intendedMode = self._currentModeMono = None
        self._odomResetCounter = self._odomResetMono = None
        self._frameResetEpoch = 0
        self.sih_setpoint_inhibited = False
        self._observationStreamsRequested = False
        self._landedState: int | None = None
        self._landedMono: float | None = None
        self._positionMono: float | None = None
        self._positionNedM: tuple[float, float, float] | None = None
        self._velocityNedMS: tuple[float, float, float] | None = None
        self._bootMs: int | None = None
        self._bootMsMono: float | None = None
        self._sourcesSeen: set[tuple[int, int]] = set()

        # 下行
        self._restartEpoch = 0
        self.restart_events: deque = deque(maxlen=16)
        self._commands: dict[int, dict] = {}
        self.commanded_armed: bool | None = None
        self.commanded_mode_name: str | None = None
        self.last_setpoint: dict | None = None

        # 构造期评估一次安全闸门：节点可以「先启动、把拒绝原因当诊断发出去」，
        # 而不是在构造时炸掉。connect() 会看这个原因并直接返回 False。
        self._guardReason, self._guardDetail = self._evaluateGuards()

    # ------------------------------------------------------------ 安全闸门
    def _evaluateGuards(self) -> tuple[str | None, str]:
        """返回 (拒绝原因, 细节)。None 表示允许建立连接。"""
        kind = self._kind
        if kind == "bad":
            return REFUSAL_BAD_CONNECTION_STRING, f"无法解析连接串 {self._connection!r}"
        if kind == "serial":
            if not self.allow_serial:
                return (
                    REFUSAL_SERIAL_NOT_ALLOWED,
                    "serial: 被拒绝：本后端默认绝不建串口链路（真机解锁风险）。"
                    "确需串口时显式传 allow_serial=True",
                )
            return None, ""
        if kind == "file":
            return (
                REFUSAL_UNSUPPORTED_SCHEME,
                "file: 被拒绝：日志回放源没有心跳，会被误当成实时链路；"
                "离线回放请用 FakePx4Backend 或 mavlink_imu_replay",
            )
        if kind in ("udpin", "udpout", "udp", "tcp", "tcpin"):
            if not self._linkIsLoopback and not self.allow_non_loopback:
                return (
                    REFUSAL_NON_LOOPBACK_HOST,
                    f"host={self._host!r} 不是回环地址：本后端能解锁，默认只允许 "
                    "127.0.0.1/localhost/::1。确需连真机时显式传 allow_non_loopback=True",
                )
            return None, ""
        return REFUSAL_UNSUPPORTED_SCHEME, f"不支持的连接类型 {kind!r}"

    @property
    def guard_reason(self) -> str | None:
        """连接被闸门拦住的原因（None = 允许）。"""
        return self._guardReason

    def _txPathReady(self) -> tuple[bool, str]:
        """当前是否存在可用的发送路径。"""
        if self._conn is None:
            return False, "尚未连接"
        if self._kind == "udpin" and not self._peerSeen:
            # udpin 只有收到过对端数据才知道往哪发；此时 write() 会静默丢弃。
            return False, "udpin 尚未收到对端数据，无从得知回发地址"
        return True, ""

    def _armingAllowed(self) -> tuple[bool, str]:
        if not self.allow_arming:
            return False, "allow_arming=False（默认）：本后端拒绝任何解锁命令"
        if not self._linkIsLoopback and not self.allow_arming_on_non_loopback:
            return False, (
                f"链路 {self._connection!r} 不是回环，且 "
                "allow_arming_on_non_loopback=False：拒绝在非回环链路上解锁"
            )
        return True, ""

    # ------------------------------------------------------------ 链路管理
    def connect(self) -> bool:
        with self._stateLock:
            if self._conn is not None:
                return True
        if self._guardReason is not None:
            self._refuse(self._guardReason, self._guardDetail, "connect")
            return False
        self._bump("connect_attempts")
        try:
            conn = self._makeConnection()
        except Exception as exc:  # noqa: BLE001 —— 任何建链失败都必须可见
            self._bump("connect_failures")
            self.last_error = f"connect_failed: {exc}"
            self._refuse(REFUSAL_CONNECT_FAILED, str(exc), "connect")
            return False
        with self._stateLock:
            self._conn = conn
            self._localEndpoint = self._readLocalEndpoint(conn)
        self._stop.clear()
        self._rxThread = threading.Thread(
            target=self._receiveLoop, name="px4_backend_mavlink_rx", daemon=True
        )
        self._rxThread.start()
        return True

    def _makeConnection(self) -> Any:
        kwargs: dict[str, Any] = {
            "dialect": self._dialect,
            "source_system": self._sourceSystem,
            "source_component": self._sourceComponent,
        }
        if self._kind == "udpin":
            # 只有 udpin 需要 input=True（绑定本地端口等对端）。
            # udpout/udp 若也传 input=True，socket 会变成监听端，发送方向就废了——
            # 这里有意与 mavlink_imu_node 的写法不同，理由如上。
            kwargs["input"] = True
        if self._kind == "serial":
            kwargs["baud"] = self._baud
            kwargs["autoreconnect"] = True
        if self._factory is not None:
            return self._factory(self._connection, **kwargs)
        from pymavlink import mavutil  # 延迟 import：协议与 Fake 不需要 pymavlink

        return mavutil.mavlink_connection(self._connection, **kwargs)

    def _readLocalEndpoint(self, conn: Any) -> str | None:
        """实际绑定的本地地址（udpin 端口可能是内核分配的临时端口）。"""
        try:
            sock = getattr(conn, "port", None)
            if sock is None:
                return None
            addr = sock.getsockname()
            host, port = str(addr[0]), int(addr[1])
            if port == 0:
                # udpout 在第一次发送前没有本地端口，报 None 比报 0.0.0.0:0 诚实
                return None
            return f"{host}:{port}"
        except Exception:  # noqa: BLE001
            return None

    def close(self) -> None:
        self._stop.set()
        thread = self._rxThread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        with self._stateLock:
            conn, self._conn = self._conn, None
            self._rxThread = None
        if conn is not None:
            try:
                conn.close()
            except Exception as exc:  # noqa: BLE001
                self.last_error = f"close_failed: {exc}"

    def isConnected(self) -> bool:
        """基于**心跳年龄**判断连通，而不是 socket 是否存在。"""
        if self._conn is None:
            return False
        with self._stateLock:
            last = self._hbMono
        if last is None:
            return False
        return (self._clock() - last) <= self.heartbeat_timeout_s

    # ------------------------------------------------------------ 接收线程
    def _receiveLoop(self) -> None:
        while not self._stop.is_set():
            conn = self._conn
            if conn is None:
                break
            try:
                self._maybeSendHeartbeat()
                msg = conn.recv_match(blocking=True, timeout=self.read_timeout_s)
            except Exception as exc:  # noqa: BLE001 —— 畸形输入/底层异常都不能让线程死
                self._bump("rx_exceptions")
                self.last_error = f"recv_failed: {exc}"
                time.sleep(min(0.2, self.read_timeout_s))
                continue
            self._syncPymavlinkCounters(conn)
            if msg is None:
                self._bump("read_timeouts")
                continue
            # 收到任何帧（哪怕是坏帧）都说明 udpin 已经知道回发地址了
            self._peerSeen = True
            if msg.get_type() == "BAD_DATA":
                # pymavlink 在 robust_parsing 下把畸形输入变成 BAD_DATA 而不是抛异常
                self._bump("parse_errors")
                data = getattr(msg, "data", b"") or b""
                self._bump("bad_data_bytes", len(data))
                self.last_error = f"bad_data: {len(data)}B"
                continue
            try:
                self._handleMessage(msg)
            except Exception as exc:  # noqa: BLE001
                self._bump("handle_exceptions")
                self.last_error = f"handle_failed: {exc}"

    def _syncPymavlinkCounters(self, conn: Any) -> None:
        """把 pymavlink 自己的解析错误计数同步过来（作为我们计数的交叉校验）。"""
        mav = getattr(conn, "mav", None)
        total = getattr(mav, "total_receive_errors", None)
        if total is None:
            return
        with self._counterLock:
            self.counters["pymavlink_receive_errors"] = int(total)

    def _handleMessage(self, msg: Any) -> None:
        monoNow = self._clock()
        self._bump("messages_rx")
        msgid = int(msg.get_msgId())
        srcSys = int(msg.get_srcSystem())
        srcComp = int(msg.get_srcComponent())
        if not self._acceptSource(srcSys, srcComp):
            # 来源拒绝是设计行为（PX4 会用别的 sysid 发 TIMESYNC 等），
            # 只计数 + 记最近一次，不写进 last_error——否则真实错误会被长期掩盖。
            self._bump("rejected_source")
            with self._stateLock:
                self._rejectedSourceLast = (srcSys, srcComp)
                self._sourcesRejected.add((srcSys, srcComp))
            return
        self._sourcesSeen.add((srcSys, srcComp))
        if msgid == MAVLINK_MSG_ID_HEARTBEAT:
            self._handleHeartbeat(msg, monoNow)
        elif msgid == MAVLINK_MSG_ID_ATTITUDE:
            self._handleAttitude(msg, monoNow)
        elif msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            self._handleLocalPosition(msg, monoNow)
        elif msgid == 436:  # CURRENT_MODE: actual mode and PX4 user intention, not our command cache.
            with self._stateLock:
                self._currentMode = int(msg.custom_mode)
                self._intendedMode = int(msg.intended_custom_mode)
                self._currentModeMono = monoNow
        elif msgid == 331:  # ODOMETRY.reset_counter includes estimator pose/velocity resets.
            counter = int(msg.reset_counter)
            with self._stateLock:
                if self._odomResetCounter is not None and counter != self._odomResetCounter:
                    self._frameResetEpoch += 1
                self._odomResetCounter = counter
                self._odomResetMono = monoNow
        elif msgid == 245:  # EXTENDED_SYS_STATE; UNKNOWN is not an airborne observation.
            value = int(getattr(msg, "landed_state", 0))
            with self._stateLock:
                self._landedState = value if value in (1, 2, 3, 4) else None
                self._landedMono = monoNow
        elif msgid == MAVLINK_MSG_ID_COMMAND_ACK:
            self._handleCommandAck(msg, monoNow)
        elif msgid == MAVLINK_MSG_ID_SYS_STATUS:
            self._bump("sys_status_rx")
        elif msgid == MAVLINK_MSG_ID_STATUS_TEXT:
            self._bump("status_text_rx")
            text = getattr(msg, "text", None)
            if text:
                self.last_error = f"status_text: {text}"
        else:
            self._bump("messages_other")

    def requestObservationStreams(self):
        if self.dry_run or self._observationStreamsRequested or self._conn is None:
            return
        send = getattr(self._conn.mav, "command_long_send", None)
        if send is None:
            return
        try:
            with self._txLock:
                for message_id in (436, 331):
                    send(self.target_system, self.target_component, 511, 0,
                         message_id, 100000, 0, 0, 0, 0, 0)
            self._observationStreamsRequested = True
        except Exception as exc:
            self.last_error = f"observation_stream_request: {exc}"

    def _acceptSource(self, srcSys: int, srcComp: int) -> bool:
        if srcSys != self.target_system:
            return False
        if not self.accept_any_component and srcComp != self.target_component:
            return False
        return True

    # ------------------------------------------------------------ 上行处理
    def _handleHeartbeat(self, msg: Any, monoNow: float) -> None:
        autopilot = int(getattr(msg, "autopilot", -1))
        if self.requirePx4Autopilot and autopilot != MAV_AUTOPILOT_PX4:
            # 防误连：同一端口上可能是别的自驾仪（或地面站），不能当成 PX4
            self._bump("heartbeats_other_autopilot")
            self.last_error = f"unexpected_autopilot: {autopilot}"
            return
        self._bump("heartbeats_rx")
        seq = msg.get_seq()
        with self._stateLock:
            previous = self._hbMono
            if previous is not None and (monoNow - previous) > self.heartbeat_timeout_s:
                self._bump("heartbeat_gaps")
            if seq is not None:
                self._noteHeartbeatSeq(int(seq))
            self._hbMono = monoNow
            self._autopilot = autopilot
            self._hbBaseMode = int(msg.base_mode)
            self._hbCustomMode = int(msg.custom_mode)
            self._hbSystemStatus = int(getattr(msg, "system_status", 0))
            mavlinkVersion = getattr(msg, "mavlink_version", None)
            self._hbMavlinkVersion = None if mavlinkVersion is None else int(mavlinkVersion)

    def _noteHeartbeatSeq(self, seq: int) -> None:
        """心跳序号回退 = 对端 MAVLink 发送侧被重置（重启的旁证之一）。

        只作为旁证计数，**不**单独触发重启事件：正常的 255→0 回绕、
        以及乱序/丢包都可能造成看似回退的序号。真正的重启判定只认
        boot 时间回退（见 `_noteBootTime`），这样事件恰好一次。
        """
        last = self._hbSeqLast
        self._hbSeqLast = seq
        if last is None:
            return
        expected = (last + 1) & 0xFF
        if seq != expected and seq < last:
            self._bump("heartbeat_seq_resets")

    def _handleAttitude(self, msg: Any, monoNow: float) -> None:
        self._bump("attitude_rx")
        with self._stateLock:
            self._yawRad = float(msg.yaw)
            # 同一帧的 roll/pitch/yaw_rate 与 yaw 共享时刻，因此新鲜度一致；
            # 下游用 attitude_age_s 判断这批姿态是否还够新。
            self._rollRad = float(msg.roll)
            self._pitchRad = float(msg.pitch)
            self._yawRateRadS = float(msg.yawspeed)
            self._attitudeMono = monoNow
        self._noteBootTime(getattr(msg, "time_boot_ms", None), monoNow)

    def _handleLocalPosition(self, msg: Any, monoNow: float) -> None:
        self._bump("local_position_rx")
        with self._stateLock:
            self._positionNedM = (float(msg.x), float(msg.y), float(msg.z))
            self._velocityNedMS = (float(msg.vx), float(msg.vy), float(msg.vz))
            self._positionMono = monoNow
        self._noteBootTime(getattr(msg, "time_boot_ms", None), monoNow)

    def _noteBootTime(self, bootMs: Any, monoNow: float) -> None:
        """跟踪 PX4 启动时钟；**显著回退 = PX4 重启**（唯一的重启判据）。

        为什么要专门检测：PX4 重启后 `time_boot_ms` 从 0 重新开始，
        而 Companion 侧缓存的模式/解锁状态、boot 时间线全部作废。
        failsafe 层必须在重启瞬间就知道，不能等心跳超时。
        """
        if bootMs is None:
            return
        try:
            value = int(bootMs)
        except (TypeError, ValueError):
            return
        with self._stateLock:
            old = self._bootMs
            if old is None:
                self._bootMs = value
                self._bootMsMono = monoNow
                return
            if value + self.restartBootToleranceMs < old:
                self._markRestart("px4_boot_time_regressed", monoNow, old, value)
                return
            if value >= old:
                # 允许容差内的乱序/抖动，取观测最大值
                self._bootMs = value
            self._bootMsMono = monoNow

    def _markRestart(self, reason: str, monoNow: float, oldBoot: int, newBoot: int) -> None:
        self._bump("px4_restarts")
        self._restartEpoch += 1
        self._bootMs = newBoot
        self._bootMsMono = monoNow
        # 重启后飞控必然回到未解锁 + 默认模式，且旧心跳属于旧时间线：
        # 全部丢弃，宁可短暂显示「未连通」，也不能沿用重启前的状态。
        self._hbBaseMode = None
        self._hbCustomMode = None
        self._hbSystemStatus = None
        self._hbMono = None
        self._hbSeqLast = None
        self._currentMode = self._intendedMode = self._currentModeMono = None
        self._odomResetCounter = self._odomResetMono = None
        self.sih_setpoint_inhibited = False
        self._observationStreamsRequested = False
        self._landedState = self._landedMono = None
        self._positionMono = self._attitudeMono = None
        event = {
            "epoch": self._restartEpoch,
            "reason": reason,
            "mono_s": monoNow,
            "old_boot_time_ms": int(oldBoot),
            "new_boot_time_ms": int(newBoot),
        }
        self.restart_events.append(event)
        self.last_error = f"px4_restart: {reason}"

    def _handleCommandAck(self, msg: Any, monoNow: float) -> None:
        command = int(msg.command)
        result = int(msg.result)
        self._bump("command_acks_rx")
        if result == MAV_RESULT_IN_PROGRESS:
            # 只是在跑，不是终态：绝不把中间态写成 ack_result
            with self._stateLock:
                entry = self._commands.setdefault(command, {"command": command})
                entry["in_progress_mono_s"] = monoNow
            self._bump("commands_in_progress")
            return
        with self._stateLock:
            entry = self._commands.setdefault(command, {"command": command})
            entry["ack_result"] = result
            entry["ack_result_name"] = _mavResultName(result)
            entry["ack_mono_s"] = monoNow
        if result == MAV_RESULT_ACCEPTED:
            self._bump("commands_accepted")
        else:
            self._bump("commands_rejected")
            self.last_error = f"command_rejected: {command} -> {_mavResultName(result)}"

    # ------------------------------------------------------------ 下行：心跳
    def _maybeSendHeartbeat(self) -> None:
        if not self.send_heartbeat or self.dry_run:
            # dry_run 的定义就是「一个字节都不发」，心跳也不例外。
            return
        interval = 1.0 / max(self.heartbeat_rate_hz, 1e-3)
        now = self._clock()
        if self._hbTxMono is not None and (now - self._hbTxMono) < interval:
            return
        ready, _ = self._txPathReady()
        if not ready:
            return
        conn = self._conn
        if conn is None:
            return
        try:
            msg = conn.mav.heartbeat_encode(
                MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE
            )
        except Exception as exc:  # noqa: BLE001
            self.last_error = f"heartbeat_encode_failed: {exc}"
            return
        if self._write(conn, msg, "heartbeat"):
            self._hbTxMono = now
            self._bump("heartbeats_tx")

    # ------------------------------------------------------------ 下行：发送
    def _write(self, conn: Any, msg: Any, what: str) -> bool:
        """写一帧。注意：成功只代表「本地没报错」，**不是投递证明**。"""
        try:
            data = msg.pack(conn.mav)
            with self._txLock:
                conn.write(data)
        except Exception as exc:  # noqa: BLE001
            self._bump("tx_errors")
            self.last_error = f"write_failed({what}): {exc}"
            self._refuse(REFUSAL_WRITE_FAILED, f"{what}: {exc}", f"write:{what}")
            return False
        self._bump("packets_tx")
        self._bump("bytes_tx", len(data))
        return True

    def _sendCommandLong(
        self,
        name: str,
        command: int,
        *,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
        isArming: bool = False,
        commanded_armed: bool | None = None,
        commanded_mode_name: str | None = None,
    ) -> bool:
        if isArming:
            allowed, why = self._armingAllowed()
            if not allowed:
                self._bump("arm_refused")
                self._refuse(REFUSAL_ARMING_NOT_ALLOWED, why, name)
                return False
        if self.dry_run:
            # 命令是「动作」，没有 ACK 就不能声称成功；dry_run 下连字节都没发，
            # 所以这里必须返回 False，绝不用 True 冒充成功。
            self._bump("commands_suppressed_dry_run")
            self._refuse(REFUSAL_DRY_RUN_COMMAND, f"{name}(cmd={command}) dry_run=True", name)
            return False
        conn = self._conn
        if conn is None:
            self._refuse(REFUSAL_TX_PATH_NOT_READY, "尚未连接", name)
            return False
        ready, why = self._txPathReady()
        if not ready:
            self._refuse(REFUSAL_TX_PATH_NOT_READY, why, name)
            return False
        # 先登记「打算发什么」，再写。ack_result 保持 None，直到收到 COMMAND_ACK。
        with self._stateLock:
            self._commands[command] = {
                "command": command,
                "name": name,
                "param1": float(param1),
                "param2": float(param2),
                "param3": float(param3),
                "sent_mono_s": self._clock(),
                "ack_result": None,
                "ack_result_name": None,
                "ack_mono_s": None,
            }
            if commanded_armed is not None:
                self.commanded_armed = commanded_armed
            if commanded_mode_name is not None:
                self.commanded_mode_name = commanded_mode_name
        try:
            msg = conn.mav.command_long_encode(
                self.target_system,
                self.target_component,
                command,
                0,                      # confirmation
                float(param1),
                float(param2),
                float(param3),
                float(param4),
                float(param5),
                float(param6),
                float(param7),
            )
        except Exception as exc:  # noqa: BLE001
            self.last_error = f"command_encode_failed: {exc}"
            self._refuse(REFUSAL_WRITE_FAILED, f"{name} 编码失败: {exc}", name)
            return False
        if not self._write(conn, msg, name):
            return False
        self._bump("commands_sent")
        return True

    # ------------------------------------------------------------ 下行：API
    def arm(self, arm: bool = True) -> bool:
        return self._sendCommandLong(
            "arm" if arm else "disarm",
            MAV_CMD_COMPONENT_ARM_DISARM,
            param1=float(ARMING_ACTION_ARM if arm else ARMING_ACTION_DISARM),
            isArming=True,
            commanded_armed=bool(arm),
        )

    def disarm(self) -> bool:
        return self.arm(False)

    def setOffboardMode(self) -> bool:
        return self.set_mode("offboard")

    def setPositionMode(self) -> bool:
        return self.set_mode("posctl")

    def set_mode(self, name: str) -> bool:
        """按名字切 PX4 主模式（`auto:<sub>` 支持 AUTO 子模式）。

        非解锁命令也可以被拒绝：dry_run、链路未通、未知模式名。
        返回值只说「命令发出了」，是否接受要看 `streamDiagnostics()["commands"]`
        里的 `ack_result`；是否真的切过去了要看 HEARTBEAT 的 `custom_mode`。
        """
        key = str(name).strip().lower()
        main, sub = _resolveModeName(key)
        if main is None:
            self._refuse(REFUSAL_UNKNOWN_MODE, f"name={name!r}", "set_mode")
            return False
        return self._sendCommandLong(
            f"set_mode:{key}",
            MAV_CMD_DO_SET_MODE,
            param1=float(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            param2=float(main),
            param3=float(sub),
            commanded_mode_name=key,
        )

    def sendSetpoint(self, setpoint: Any, type_mask: int | None = None) -> bool:
        """构造并发送 msg 84（`SET_POSITION_TARGET_LOCAL_NED`）。

        `dry_run=True`（默认）时：**完整构造 + 校验 + 计数，但不发送**，
        返回 True 表示「这次 setpoint 本身是合法的」。命令不走这条路
        （命令在 dry_run 下返回 False，见 `_sendCommandLong`）。

        `type_mask`：真实 `px4_frames.Px4LocalSetpoint` 不带 mask，必须在这里
        显式传入（例如 `type_mask=int(TypeMask.forMode("position_velocity"))`）。
        """
        if self.sih_setpoint_inhibited:
            self._bump("setpoints_not_sent")
            self._refuse("sih_setpoint_inhibited", "authorized SIH fault injection", "send_setpoint")
            return False
        values, mask, reason, detail = self._extractSetpoint(setpoint, type_mask)
        if reason is not None:
            self._bump("setpoints_rejected")
            self._refuse(reason, detail, "send_setpoint")
            return False
        assert mask is not None and values is not None
        bootMs = self._currentBootTimeMs()
        conn = self._conn
        if conn is None:
            # 还没连接：仍然完成构造与计数（dry_run 语义），但如实报告没发出去
            self._bump("setpoints_built")
            self._recordLastSetpoint(values, mask, bootMs)
            self._refuse(REFUSAL_TX_PATH_NOT_READY, "尚未连接", "send_setpoint")
            self._bump("setpoints_not_sent")
            return False
        try:
            msg = conn.mav.set_position_target_local_ned_encode(
                bootMs,
                self.target_system,
                self.target_component,
                MAV_FRAME_LOCAL_NED,
                mask,
                values["x"],
                values["y"],
                values["z"],
                values["vx"],
                values["vy"],
                values["vz"],
                values["afx"],
                values["afy"],
                values["afz"],
                values["yaw"],
                values["yaw_rate"],
            )
        except Exception as exc:  # noqa: BLE001
            self._refuse(REFUSAL_WRITE_FAILED, f"msg84 编码失败: {exc}", "send_setpoint")
            self._bump("setpoints_rejected")
            return False
        self._bump("setpoints_built")
        self._recordLastSetpoint(values, mask, bootMs)
        if self.dry_run:
            self._bump("setpoints_suppressed_dry_run")
            return True
        ready, why = self._txPathReady()
        if not ready:
            self._refuse(REFUSAL_TX_PATH_NOT_READY, why, "send_setpoint")
            self._bump("setpoints_not_sent")
            return False
        if not self._write(conn, msg, "setpoint"):
            self._bump("setpoints_not_sent")
            return False
        self._bump("setpoints_sent")
        return True

    def _recordLastSetpoint(self, values: dict, mask: int, bootMs: int) -> None:
        with self._stateLock:
            self.last_setpoint = dict(values)
            self.last_setpoint["type_mask"] = int(mask)
            self.last_setpoint["coordinate_frame"] = MAV_FRAME_LOCAL_NED
            self.last_setpoint["time_boot_ms"] = int(bootMs)

    def _currentBootTimeMs(self) -> int:
        """msg 84 的 `time_boot_ms` 取值规则（文档化的唯一规则）：

        1. 若已收到过带 `time_boot_ms` 的上行（ATTITUDE / LOCAL_POSITION_NED），
           则 = 最近观测值 + 「自观测以来的注入时钟毫秒数」，即把飞控启动时钟
           外推到**发帧时刻**；
        2. 否则 = **0**。

        为什么可以填 0：本机 PX4 源码里处理 msg 84 的唯一位置
        （`mavlink_receiver.cpp` 的 `handle_message_set_position_target_local_ned`）
        **没有**读这个字段，PX4 用 `hrt_absolute_time()` 给 setpoint 打时间戳。
        所以它只是我们这边的日志/复盘字段，**不参与 PX4 的超时判定**。
        """
        with self._stateLock:
            boot = self._bootMs
            bootMono = self._bootMsMono
        if boot is None or bootMono is None:
            return 0
        elapsedMs = (self._clock() - bootMono) * 1000.0
        if elapsedMs < 0.0:
            # 注入时钟回退：不猜，直接用观测值
            elapsedMs = 0.0
        # time_boot_ms 是 uint32：按 MAVLink 惯例回绕，避免 struct 打包越界
        return int(round(boot + elapsedMs)) & 0xFFFFFFFF

    def _extractSetpoint(
        self, setpoint: Any, type_mask: int | None = None
    ) -> tuple[dict[str, float] | None, int | None, str | None, str]:
        """按 mask 拆字段并校验。mask 来源：显式参数 > setpoint 属性 > 拒绝。

        规则（宁可拒绝也不猜）：
        - mask 某位置位（忽略该轴）：线上填 0.0（PX4 反正会写 NaN），并计数；
          mask 是权威，调用方给了非零值只计数不报错。
        - mask 某位清零（使用该轴）：值必须存在且有限，否则拒绝。
        - 位置/速度/加速度全被忽略 → 拒绝（PX4 会直接丢弃，见源码）。
        - FORCE_SET 且加速度有效 → 拒绝（PX4 明确不支持 force）。
          注：`px4_frames.TypeMask.forMode` 永远不会置 FORCE_SET，这条是
          对「别处手搓 mask」的防御。
        - yaw 与 yaw_rate 同时有效 → 拒绝：MAVLink 规范不允许同时置两个，
          PX4 不会报错但语义有歧义（`ocm.attitude` 与 `ocm.body_rate` 会同时为真）。
          注：`TypeMask.forMode` 恰好忽略一个 yaw 字段，所以正常路径不会命中。
        """
        if setpoint is None:
            return None, None, REFUSAL_SETPOINT_NOT_LIKE, "setpoint 为 None"
        rawMask: Any = type_mask
        if rawMask is None:
            rawMask = getattr(setpoint, "type_mask", _MISSING)
            if rawMask is _MISSING:
                return (
                    None, None, REFUSAL_SETPOINT_MASK_MISSING,
                    "setpoint 没有 type_mask 属性，也没有显式传 type_mask=；"
                    "请用 px4_frames.TypeMask.forMode(...) 构造后显式传入",
                )
        if rawMask is None:
            return None, None, REFUSAL_SETPOINT_MASK_INVALID, "type_mask 为 None"
        try:
            mask = int(rawMask)
        except (TypeError, ValueError):
            return None, None, REFUSAL_SETPOINT_MASK_INVALID, f"type_mask={rawMask!r} 不是整数"
        if mask < 0 or mask > 0xFFFF:
            return None, None, REFUSAL_SETPOINT_MASK_INVALID, f"type_mask={mask} 越界"

        values: dict[str, float] = {}
        axes = (
            ("position_m", ("x", "y", "z"),
             (POSITION_TARGET_TYPEMASK_X_IGNORE, POSITION_TARGET_TYPEMASK_Y_IGNORE,
              POSITION_TARGET_TYPEMASK_Z_IGNORE)),
            ("velocity_m_s", ("vx", "vy", "vz"),
             (POSITION_TARGET_TYPEMASK_VX_IGNORE, POSITION_TARGET_TYPEMASK_VY_IGNORE,
              POSITION_TARGET_TYPEMASK_VZ_IGNORE)),
            ("acceleration_m_s2", ("afx", "afy", "afz"),
             (POSITION_TARGET_TYPEMASK_AX_IGNORE, POSITION_TARGET_TYPEMASK_AY_IGNORE,
              POSITION_TARGET_TYPEMASK_AZ_IGNORE)),
        )
        anyAxisUsed = False
        for attr, names, bits in axes:
            vector = getattr(setpoint, attr, None)
            seq: list[float] | None = None
            if vector is not None:
                try:
                    seq = [float(v) for v in vector]
                except (TypeError, ValueError):
                    return None, None, REFUSAL_SETPOINT_MISSING_VALUE, f"{attr} 不是数值序列"
                if len(seq) != 3:
                    return (None, None, REFUSAL_SETPOINT_MISSING_VALUE,
                            f"{attr} 长度 {len(seq)}，应为 3")
            for index, (fname, bit) in enumerate(zip(names, bits)):
                if mask & bit:
                    values[fname] = 0.0
                    if seq is not None and seq[index] != 0.0:
                        self._bump("setpoint_ignored_axes_zeroed")
                    continue
                if seq is None:
                    return (None, None, REFUSAL_SETPOINT_MISSING_VALUE,
                            f"{attr} 为 None，但 type_mask 未忽略 {fname}")
                value = seq[index]
                if not math.isfinite(value):
                    return None, None, REFUSAL_SETPOINT_NONFINITE, fname
                values[fname] = value
                anyAxisUsed = True

        yawIgnored = bool(mask & POSITION_TARGET_TYPEMASK_YAW_IGNORE)
        rateIgnored = bool(mask & POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
        if not yawIgnored and not rateIgnored:
            return (None, None, REFUSAL_SETPOINT_YAW_AND_YAW_RATE,
                    "type_mask 同时声明 yaw 与 yaw_rate 有效；MAVLink 规范不允许")
        values["yaw"] = 0.0
        values["yaw_rate"] = 0.0
        if not yawIgnored:
            yaw = getattr(setpoint, "yaw_rad", None)
            if yaw is None:
                return None, None, REFUSAL_SETPOINT_MISSING_VALUE, "yaw_rad 为 None"
            yaw = float(yaw)
            if not math.isfinite(yaw):
                return None, None, REFUSAL_SETPOINT_NONFINITE, "yaw_rad"
            values["yaw"] = yaw
        elif not rateIgnored:
            rate = getattr(setpoint, "yaw_rate_rad_s", None)
            if rate is None:
                return None, None, REFUSAL_SETPOINT_MISSING_VALUE, "yaw_rate_rad_s 为 None"
            rate = float(rate)
            if not math.isfinite(rate):
                return None, None, REFUSAL_SETPOINT_NONFINITE, "yaw_rate_rad_s"
            values["yaw_rate"] = rate

        if not anyAxisUsed:
            return (None, None, REFUSAL_SETPOINT_ALL_AXES_IGNORED,
                    "位置/速度/加速度全被 type_mask 忽略；PX4 会丢弃该消息")
        accelUsed = any(not (mask & bit) for bit in
                         (POSITION_TARGET_TYPEMASK_AX_IGNORE,
                          POSITION_TARGET_TYPEMASK_AY_IGNORE,
                          POSITION_TARGET_TYPEMASK_AZ_IGNORE))
        if (mask & POSITION_TARGET_TYPEMASK_FORCE_SET) and accelUsed:
            return (None, None, REFUSAL_SETPOINT_FORCE_NOT_SUPPORTED,
                    "type_mask 带 FORCE_SET 且加速度有效；PX4 明确不支持 force")
        return values, mask, None, ""

    # ------------------------------------------------------------ 诊断
    def linkState(self) -> dict:
        now = self._clock()
        with self._stateLock:
            hbMono = self._hbMono
            bootMs = self._bootMs
            restartEpoch = self._restartEpoch
            eventCount = len(self.restart_events)
            commandedArmed = self.commanded_armed
            commandedMode = self.commanded_mode_name
            endpoint = self._localEndpoint
        return {
            "connection": self._connection,
            "connection_kind": self._kind,
            "connected": self.isConnected(),
            "heartbeat_age_s": None if hbMono is None else now - hbMono,
            "heartbeat_timeout_s": self.heartbeat_timeout_s,
            "last_error": self.last_error,
            "counters": self.countersSnapshot(),
            "refusal_count": self._refusalCount(),
            "last_refusal": self._lastRefusal(),
            "dry_run": self.dry_run,
            "allow_arming": self.allow_arming,
            "allow_non_loopback": self.allow_non_loopback,
            "allow_serial": self.allow_serial,
            "target": f"{self.target_system}/{self.target_component}",
            "local_endpoint": endpoint,
            "rejected_source_last": (
                None if self._rejectedSourceLast is None
                else f"{self._rejectedSourceLast[0]}/{self._rejectedSourceLast[1]}"
            ),
            "sources_rejected": sorted(f"{s}/{c}" for s, c in self._sourcesRejected),
            "guard_reason": self._guardReason,
            "px4_boot_time_ms": bootMs,
            "restart_epoch": restartEpoch,
            "px4_restart_events": eventCount,
            "commanded_armed": commandedArmed,
            "commanded_mode_name": commandedMode,
        }

    def streamDiagnostics(self) -> dict:
        out = self.linkState()
        with self._stateLock:
            rxAlive = self._rxThread is not None and self._rxThread.is_alive()
            sources = sorted(f"{s}/{c}" for s, c in self._sourcesSeen)
            commands = {str(k): dict(v) for k, v in self._commands.items()}
            baseMode = self._hbBaseMode
            customMode = self._hbCustomMode
            systemStatus = self._hbSystemStatus
            mavlinkVersion = self._hbMavlinkVersion
            autopilot = self._autopilot
            yaw = self._yawRad
            roll = self._rollRad
            pitch = self._pitchRad
            yawRate = self._yawRateRadS
            position = self._positionNedM
            velocity = self._velocityNedMS
            lastSetpoint = dict(self.last_setpoint) if self.last_setpoint else None
        out.update(
            {
                "backend": "mavlink",
                "rx_thread_alive": rxAlive,
                "peer_seen": self._peerSeen,
                "wire_protocol_version": self._wireProtocolVersion(),
                "sources_seen": sources,
                "autopilot": autopilot,
                "system_status": systemStatus,
                "mavlink_version": mavlinkVersion,
                "custom_mode": customMode,
                "custom_main_mode": None if customMode is None else (customMode >> PX4_CUSTOM_MODE_MAIN_SHIFT) & PX4_CUSTOM_MODE_BYTE_MASK,
                "custom_sub_mode": None if customMode is None else (customMode >> PX4_CUSTOM_MODE_SUB_SHIFT) & PX4_CUSTOM_MODE_BYTE_MASK,
                "mode_name": px4CustomMainModeName(customMode),
                # 解锁状态**只**来自 HEARTBEAT；commanded_armed 只是我们的意图
                "acknowledged_armed": (
                    None if baseMode is None
                    else bool(int(baseMode) & MAV_MODE_FLAG_SAFETY_ARMED)
                ),
                "yaw_rad": yaw,
                "roll_rad": roll,
                "pitch_rad": pitch,
                "yaw_rate_rad_s": yawRate,
                "position_ned_m": None if position is None else list(position),
                "velocity_ned_m_s": None if velocity is None else list(velocity),
                "commands": commands,
                "last_setpoint": lastSetpoint,
                "refusals": [dict(r) for r in self.refusals.records],
                "restart_events": [dict(e) for e in self.restart_events],
                "tx_enabled": (not self.dry_run),
            }
        )
        return out

    def readVehicleState(self) -> VehicleState:
        now = self._clock()
        # 必须在取 _stateLock **之前**算：isConnected() 自己也要拿这把锁，
        # 在锁内调用会自死锁（threading.Lock 不可重入）。
        connected = self.isConnected()
        with self._stateLock:
            hbMono = self._hbMono
            baseMode = self._hbBaseMode
            customMode = self._hbCustomMode
            if (self._currentModeMono is not None
                    and 0 <= now - self._currentModeMono <= self.heartbeat_timeout_s):
                customMode = self._currentMode
            attitudeMono = self._attitudeMono
            positionMono = self._positionMono
            return VehicleState(
                connected=connected,
                armed=bool(baseMode is not None
                           and (int(baseMode) & MAV_MODE_FLAG_SAFETY_ARMED)),
                mode_name=px4CustomMainModeName(customMode),
                custom_mode=customMode,
                custom_main_mode=None if customMode is None else (customMode >> PX4_CUSTOM_MODE_MAIN_SHIFT) & PX4_CUSTOM_MODE_BYTE_MASK,
                custom_sub_mode=None if customMode is None else (customMode >> PX4_CUSTOM_MODE_SUB_SHIFT) & PX4_CUSTOM_MODE_BYTE_MASK,
                system_status=self._hbSystemStatus,
                autopilot=self._autopilot,
                mavlink_version=self._hbMavlinkVersion,
                commanded_armed=self.commanded_armed,
                commanded_mode_name=self.commanded_mode_name,
                yaw_rad=self._yawRad,
                roll_rad=self._rollRad,
                pitch_rad=self._pitchRad,
                yaw_rate_rad_s=self._yawRateRadS,
                position_ned_m=self._positionNedM,
                velocity_ned_m_s=self._velocityNedMS,
                heartbeat_age_s=None if hbMono is None else now - hbMono,
                attitude_age_s=None if attitudeMono is None else now - attitudeMono,
                attitude_received_mono_s=attitudeMono,
                position_received_mono_s=self._positionMono,
                position_age_s=None if positionMono is None else now - positionMono,
                current_mode_detail=px4CustomSubModeName(self._currentMode),
                intended_mode_detail=px4CustomSubModeName(self._intendedMode),
                current_mode_age_s=None if self._currentModeMono is None else now - self._currentModeMono,
                frame_reset_epoch=self._frameResetEpoch,
                frame_reset_age_s=None if self._odomResetMono is None else now - self._odomResetMono,
                landed_state=self._landedState,
                landed_age_s=None if self._landedMono is None else now - self._landedMono,
                heartbeat_timeout_s=self.heartbeat_timeout_s,
                boot_time_ms=self._bootMs,
                restart_epoch=self._restartEpoch,
                source="mavlink",
                last_error=self.last_error,
            )

    # ------------------------------------------------------------ 内部工具
    def _wireProtocolVersion(self) -> str | None:
        conn = self._conn
        mav = getattr(conn, "mav", None) if conn is not None else None
        version = getattr(mav, "WIRE_PROTOCOL_VERSION", None)
        return None if version is None else str(version)

    def _bump(self, name: str, delta: int = 1) -> None:
        with self._counterLock:
            self.counters[name] = self.counters.get(name, 0) + int(delta)

    def countersSnapshot(self) -> dict[str, int]:
        with self._counterLock:
            return dict(self.counters)

    def _refusalCount(self) -> int:
        return int(self.counters["refusals"])

    def _lastRefusal(self) -> dict | None:
        return dict(self.refusals.records[-1]) if self.refusals.records else None

    def _refuse(self, reason: str, detail: str, name: str) -> dict:
        """登记一次拒绝：计数器 + 有界记录 + ``last_error``。

        每一次拒绝都必须在 `streamDiagnostics()` 里可见（含 reason 字符串）。
        """
        self._bump("refusals")
        self._bump(f"refused_{reason}")
        self.last_error = f"refused: {reason}" + (f": {detail}" if detail else "")
        return self.refusals.add(reason, f"{name}: {detail}" if detail else name, self._clock())


def _mavResultName(result: int) -> str:
    return {
        MAV_RESULT_ACCEPTED: "ACCEPTED",
        MAV_RESULT_TEMPORARILY_REJECTED: "TEMPORARILY_REJECTED",
        MAV_RESULT_DENIED: "DENIED",
        MAV_RESULT_UNSUPPORTED: "UNSUPPORTED",
        MAV_RESULT_IN_PROGRESS: "IN_PROGRESS",
    }.get(int(result), f"RESULT_{int(result)}")
