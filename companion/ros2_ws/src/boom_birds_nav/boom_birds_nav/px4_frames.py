"""ROS 局部 Z-up 坐标系 ↔ PX4 局部 NED 坐标系的显式、受校验的转换。

为什么需要这个模块
------------------
上游（EGO-Planner 的 `traj_server`）发布 `quadrotor_msgs/PositionCommand` 到
`/position_cmd`，`header.frame_id = "world"`，位置/速度/加速度/偏航都表达在
**契约里的局部世界系 global**（`config/contract.yaml`：重力对齐、首帧定义、
Z-up）。上游的位置轨迹就是该系里的三次 B 样条（`traj_server.cpp:191-240`）。
下游 PX4 只接受 **局部 NED**（`SET_POSITION_TARGET_LOCAL_NED`，msg 84，
`MAV_FRAME_LOCAL_NED=1`；PX4 侧解析见
`src/modules/mavlink/mavlink_receiver.cpp` 的 `POSITION_TARGET_TYPEMASK_*` 分支）。

于是"Z-up ↔ NED"这一处轴向与偏航符号的约定，是整个控制下行链路里**唯一**
的坐标翻转点。把它写成散文会散落在后端、节点和文档三处；写成代码 + 单元测试
才能保证"只有一处被审查、被测试锁死"。本模块即该唯一处。

原点与水平朝向（**必须显式建立并核实，不能默认相同**）
--------------------------------------------------------
轴变换只解决"哪个轴朝哪"；**原点与水平朝向是另一回事**：

- ROS 侧契约 `global` 的原点由首帧定义、Z 轴重力对齐，但**水平航向任意**；
- PX4 局部 NED 的原点与航向由 EKF 在上电/初始化时选定，视觉融合或 EKF 重置还会改变它。

两个系都是重力对齐的右手系，也**不会**自动同原点、同航向。完整关系是::

    T_px4_from_ros = R_z(-yaw_offset) · R_axis · T(translation)
    R_axis = diag(1, -1, -1)

其中 `yaw_offset_rad` 是 ROS 局部系 +x 相对 PX4 +x（北）的航向差，
`translation_m` 是从 PX4 原点到 ROS 原点的偏移（以 ROS 系表达）。

对 `SET_POSITION_TARGET_LOCAL_NED`，PX4 把位置解释为**相对 EKF 原点**，
因此正确做法是让 EKF 原点定义在 VIO 原点上（外部视觉融合 / `EKF2_EV_*`），
此时 `translation_m = 0`；但**航向偏移仍然必须建立并核实**。

`LocalFrameAlignment` 把这件事放进代码；未核实或核实失败时，`px4_interface_node`
按 `frame_alignment` 策略**拒绝下发位置 setpoint**（速度/加速度不依赖原点，另行处理）。

坐标系约定（**待硬件核验的假设**，不是已验证事实）
--------------------------------------------------
ROS 侧（REP-103 / 本仓库契约 global 与 body 系）:
    x 前、y 左、z 上；右手系。
    body 系按 ROS 约定为 FLU（Forward-Left-Up）。
PX4 侧（MAVLink / PX4 局部系）:
    x 北、y 东、z 下；右手系。
    body 系为 FRD（Forward-Right-Down）。
两者都是右手系且都把 +x 定义为机头前向，因此唯一差别是 y/z 轴方向对调::

    R_ROS_to_NED = diag(1, -1, -1)      # p_NED = R_ROS_to_NED @ p_ROS

即 ``N = x``（前→北）、``E = -y``（左 = 负东）、``D = -z``（上 = 负下）。
这是一次 **坐标轴重标记**（proper rotation，det = +1），所以
|v|、|a| 不变，无需缩放。

推导（本模块用测试锁死，见 ``test/test_px4_frames.py``）::

    ROS 向量              NED 向量
    (1, 0, 0) 前/上      (1,  0,  0) 北
    (0, 1, 0) 左         (0, -1,  0) 西（东的负方向）
    (0, 0, 1) 上         (0,  0, -1) 下（负 D）
    (0, 0,-1) 重力方向   (0,  0,  1) 正 D

偏航（yaw）推导
---------------
ROS 的 yaw 是绕 +Z（**上**轴）的右手旋转；NED 的 yaw 是绕 +Z（**下**轴）的
右手旋转。Z 轴反向 ⇒ 同样的物理朝向在两种记法下 yaw 差一个符号::

    yaw_NED = -yaw_ROS        yaw_rate_NED = -yaw_rate_ROS

用具体向量验证（同样是测试锁死的内容）：ROS 机头指向 yaw=+90° 时 x 轴转到
(0, 1, 0)「左」，映射到 NED 是 (0, -1, 0)「西」；而 NED 中 yaw=-90° 把北
(1, 0, 0) 转到 (0, -1, 0)。故 +90°(ROS) → -90°(NED)：**ROS 语义上的左转，
在 NED 记法里是负角、朝 +E（右）转**，也就是 PX4 会看到一个右转指令。
这一点必须上机确认（机架装配方向、PX4 机头定义、EGO 首帧朝向都会影响最终
符号），本模块只在 **一处** 汇合这个决定，并对两个方向都提供了测试。

不做什么
--------
- 不导入 rclpy / 任何 ROS 类型：本模块必须能纯离线单元测试（SW-001 的隔离意图）。
- 不做单位换算、不做时间同步、不决定控制模式；控制模式由调用方（MAVLink
  后端）选择，本模块只提供 `TypeMask` 位构造。
- 不宣称任何 SITL/实机验证。上述轴向与符号是**假设**，硬件（或 SITL）确认前
  不得据此上电飞行。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Tuple

import numpy as np

__all__ = [
    "POSITION_TARGET_TYPEMASK_X_IGNORE",
    "POSITION_TARGET_TYPEMASK_Y_IGNORE",
    "POSITION_TARGET_TYPEMASK_Z_IGNORE",
    "POSITION_TARGET_TYPEMASK_VX_IGNORE",
    "POSITION_TARGET_TYPEMASK_VY_IGNORE",
    "POSITION_TARGET_TYPEMASK_VZ_IGNORE",
    "POSITION_TARGET_TYPEMASK_AX_IGNORE",
    "POSITION_TARGET_TYPEMASK_AY_IGNORE",
    "POSITION_TARGET_TYPEMASK_AZ_IGNORE",
    "POSITION_TARGET_TYPEMASK_FORCE_SET",
    "POSITION_TARGET_TYPEMASK_YAW_IGNORE",
    "POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE",
    "MASK_POSITION_VELOCITY_YAW",
    "ANGLE_WRAP_TOL_RAD",
    "ROS_LOCAL_TO_NED",
    "FrameValidationError",
    "RosLocalSetpoint",
    "Px4LocalSetpoint",
    "SetpointLimits",
    "FrameConverter",
    "YawMode",
    "TypeMask",
    "LocalFrameAlignment",
    "YawAlignmentResidual",
    "ros_local_to_ned_rotation",
    "ros_local_to_ned_setpoint",
    "ned_to_ros_local_setpoint",
    "ros_local_to_px4_ned",
    "px4_ned_to_ros_local",
    "normalize_angle_pi",
    "between_frame_rotation",
    "vector_ros_local_to_ned",
    "vector_ned_to_ros_local",
]

# ---------------------------------------------------------------------------
# MAVLink SET_POSITION_TARGET_LOCAL_NED 的 type_mask 位
# ---------------------------------------------------------------------------
# 取值来自 PX4 生成的 MAVLink 头（本机 PX4-Autopilot 构建产物，只读、未修改）：
# build/px4_sitl_default/mavlink/common/common.h 的
# `typedef enum POSITION_TARGET_TYPEMASK`。这里按**名字与数值**同时写出，
# 避免后端自己硬编码魔数；测试断言这些数值等于 PX4 头文件中的定义。
#
# 语义（同一份头文件与 mavlink_receiver.cpp）：
#   置位 = PX4 **忽略**该分量；PX4 把被忽略的轴写成 NAN
#   （mavlink_receiver.cpp: `... ? (float)NAN : target_local_ned.x`）。
#   位 9 (FORCE_SET) 是"把 afx/afy/afz 解释为力而不是加速度"，本工程不使用：
#   带 FORCE_SET 且带加速度分量时 PX4 直接拒收该 setpoint（receiver 里报
#   "force not supported"），所以 `TypeMask` 永远不构造这个位。
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

#: 忽略加速度三轴 = 位置 + 速度 + yaw 控制。
#: EGO 的 PositionCommand 带 position/velocity/acceleration 三组字段，但
#: "控制器位于 Companion 或 PX4 尚未确定"（README），所以后端需要能自由选择
#: 使用哪几组；本常量只是最常用组合的便利别名。
MASK_POSITION_VELOCITY_YAW = (
    POSITION_TARGET_TYPEMASK_AX_IGNORE
    | POSITION_TARGET_TYPEMASK_AY_IGNORE
    | POSITION_TARGET_TYPEMASK_AZ_IGNORE
)

#: |yaw| 超过 π 时按 2π 归一化（不抛异常）。留 0 容差：π 本身合法。
ANGLE_WRAP_TOL_RAD = 0.0

_PI = math.pi


# ---------------------------------------------------------------------------
# 异常
# ---------------------------------------------------------------------------
class FrameValidationError(ValueError):
    """setpoint 校验失败：携带机器可读原因，供诊断/计数使用。

    为什么单独一个异常类型：后端需要在"丢弃这一帧"时给出明确原因（契约
    `invalid_policy` 要求无效与失败分开统计），而不是把 ValueError 与编程
    错误混在一起。

    字段：
        reason: ``"nan_inf" | "yaw_rate_overflow" | "velocity_overflow" |
                 "acceleration_overflow" | "yaw_error"``
        field:  触发字段名（``position_m`` / ``yaw_rad`` ...）
        value:  触发值（非有限时为原值；溢出时为该向量的模）
        limit:  触发的限值（nan_inf 时为 None）
    """

    def __init__(self, reason: str, field_name: str, value: Any, limit: Any = None, detail: str = ""):
        self.reason = str(reason)
        self.field = str(field_name)
        self.value = value
        self.limit = limit
        self.detail = str(detail)
        msg = f"setpoint 校验失败 reason={self.reason} field={self.field} value={value!r}"
        if limit is not None:
            msg += f" limit={limit!r}"
        if detail:
            msg += f" ({detail})"
        super().__init__(msg)

    def to_dict(self) -> Dict[str, Any]:
        """诊断用序列化（不含 numpy 对象，保证可 JSON 化）。"""
        return {
            "reason": self.reason,
            "field": self.field,
            "value": None if self.value is None else float(self.value),
            "limit": None if self.limit is None else float(self.limit),
            "detail": self.detail,
        }


# ---------------------------------------------------------------------------
# 校验限值
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class SetpointLimits:
    """可配置的 setpoint 包线。**缺省值尚未按本机实测核定，上机前必须复核。**

    设计意图不是"限制飞行性能"，而是把三类静默事故变成显式拒绝：
    1) NaN/Inf 渗进 MAVLink 报文（PX4 会原样收下，再在控制器里炸）；
    2) 未初始化或单位错误导致的巨大数值（例如把 cm 当 m）；
    3) 上游规划器输出超包线时不加提示地照发。

    物理依据：
    - ``max_yaw_rate_rad_s`` 默认 π rad/s：与上游执行器自带的限幅一致
      （`traj_server.cpp` 的 ``YAW_DOT_MAX_PER_SEC = PI``），所以正常链路上
      永远不该触发；触发即表示上游被改坏或被污染。
    - ``max_velocity_m_s`` 15、``max_acceleration_m_s2`` 20：**保守占位值**，
      与 README 里"目标推重比至少 2.5"（约 1 g 机动裕度）同量级，但未按
      1103/2216S 动力实测与 EGO 实际输出去核定。真机前必须按实测重设并记录。
      两者都按**向量模**判定：任一轴异常必然被模抓住，且避免逐轴阈值在
      坐标系旋转下产生歧义。
    """

    max_velocity_m_s: float = 15.0
    max_acceleration_m_s2: float = 20.0
    max_yaw_rate_rad_s: float = math.pi

    def __post_init__(self) -> None:
        for name in ("max_velocity_m_s", "max_acceleration_m_s2", "max_yaw_rate_rad_s"):
            v = float(getattr(self, name))
            if not math.isfinite(v) or v <= 0.0:
                raise ValueError(f"SetpointLimits.{name} 必须是有限正数，实得 {v!r}")


# ---------------------------------------------------------------------------
# 角度工具
# ---------------------------------------------------------------------------
def normalize_angle_pi(angle_rad: float) -> float:
    """把角度归一化到 (-π, π]。

    为什么需要：ROS 的 yaw 是连续展开量，可以在轨迹中越过 ±π（上游
    `traj_server.cpp` 自己就带 ±π 环绕处理），跨帧差分还会累积 2π 的整数倍。
    把 3π/2 原样送进 MAVLink 会让 PX4 收到一个与预期相差 2π 的角——多数控制器
    用三角函数所以无感，但这属于"靠下游容忍度活着"，明确归一化更安全。

    ±π 边界：π 映射为 π，-π 也映射为 +π（区间右闭）。两者表示同一朝向，
    所以比较朝向时应使用 |delta| 或四元数，不要直接比较标量。
    """
    a = float(angle_rad)
    if not math.isfinite(a):
        raise FrameValidationError("nan_inf", "yaw_rad", a, None, "yaw 非有限，无法归一化")
    wrapped = math.fmod(a + _PI, 2.0 * _PI)
    if wrapped <= 0.0:
        wrapped += 2.0 * _PI
    return wrapped - _PI


# ---------------------------------------------------------------------------
# ROS Z-up 与 NED 之间的向量/旋转关系
# ---------------------------------------------------------------------------
#: R_ROS_to_NED = diag(1, -1, -1)。见模块 docstring 的推导表。
ROS_LOCAL_TO_NED: Tuple[Tuple[float, float, float], ...] = (
    (1.0, 0.0, 0.0),
    (0.0, -1.0, 0.0),
    (0.0, 0.0, -1.0),
)


def between_frame_rotation() -> np.ndarray:
    """返回 R_ROS_to_NED（3×3，diag(1, -1, -1)）的新副本。

    提供函数而不只给常量，是为了让调用方无法原地改坏共享矩阵。
    """
    return np.array(ROS_LOCAL_TO_NED, dtype=float)


def vector_ros_local_to_ned(v: Any) -> np.ndarray:
    """ROS 局部 Z-up 向量 → NED 向量。位置/速度/加速度共用同一变换。"""
    return between_frame_rotation() @ np.asarray(v, dtype=float).reshape(3)


def vector_ned_to_ros_local(v: Any) -> np.ndarray:
    """NED 向量 → ROS 局部 Z-up 向量（R 是对合矩阵，逆即自身）。"""
    return between_frame_rotation() @ np.asarray(v, dtype=float).reshape(3)


def _tuple3(v: Any) -> Tuple[float, float, float]:
    a = np.asarray(v, dtype=float).reshape(3)
    return (_no_neg_zero(a[0]), _no_neg_zero(a[1]), _no_neg_zero(a[2]))


def _no_neg_zero(v: Any) -> float:
    """把 -0.0 规范成 0.0。

    为什么：``-0.0 == 0.0`` 为真但 ``str(-0.0) == "-0.0"``，日志与诊断字符串里
    出现 "-0.0" 会让人怀疑符号翻转是否出错（本模块的整个要点就是符号）。
    """
    f = float(v)
    return 0.0 if f == 0.0 else f


def _require_finite(field_name: str, value: Any) -> None:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if not bool(np.all(np.isfinite(arr))):
        bad = arr[~np.isfinite(arr)]
        raise FrameValidationError(
            "nan_inf", field_name, float(bad[0]), None, "含 NaN/Inf；MAVLink 报文不得携带"
        )


def _check_norm(field_name: str, value: Any, limit: float, reason: str) -> None:
    n = float(np.linalg.norm(np.asarray(value, dtype=float).reshape(3)))
    if n > limit:
        raise FrameValidationError(reason, field_name, n, limit, "向量模超出配置包线")


def _check_scalar(field_name: str, value: float, limit: float, reason: str) -> None:
    v = float(value)
    if abs(v) > limit:
        raise FrameValidationError(reason, field_name, v, limit, "标量超出配置包线")


# ---------------------------------------------------------------------------
# setpoint 数据类
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class RosLocalSetpoint:
    """ROS 局部 Z-up 系（契约的 global/world）中的 setpoint。

    字段与 `/position_cmd` 一一对应；``yaw_dot_rad_s`` 对应消息里的 ``yaw_dot``
    （绕 +Z「上」轴的右手角速率，即"向左转为正"）。
    """

    position_m: Tuple[float, float, float]
    velocity_m_s: Tuple[float, float, float]
    acceleration_m_s2: Tuple[float, float, float]
    yaw_rad: float
    yaw_dot_rad_s: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "position_m", _tuple3(self.position_m))
        object.__setattr__(self, "velocity_m_s", _tuple3(self.velocity_m_s))
        object.__setattr__(self, "acceleration_m_s2", _tuple3(self.acceleration_m_s2))
        object.__setattr__(self, "yaw_rad", _no_neg_zero(self.yaw_rad))
        object.__setattr__(self, "yaw_dot_rad_s", _no_neg_zero(self.yaw_dot_rad_s))

    @classmethod
    def from_position_command(cls, msg: Any) -> "RosLocalSetpoint":
        """从 `quadrotor_msgs/PositionCommand` 取字段（鸭子类型，不导入 ROS 类型）。

        只读取消息字段，不检查 `trajectory_flag` / `trajectory_id`——那属于
        "该不该发"的判定，由 `px4_failsafe` 负责；本模块只管"怎么换算"。
        frame_id 也不在这里校验：契约规定它是 world 系，若上游换成别的 frame_id，
        应由节点的显式检查拒绝，而不是本模块猜。
        """
        return cls(
            position_m=(msg.position.x, msg.position.y, msg.position.z),
            velocity_m_s=(msg.velocity.x, msg.velocity.y, msg.velocity.z),
            acceleration_m_s2=(msg.acceleration.x, msg.acceleration.y, msg.acceleration.z),
            yaw_rad=msg.yaw,
            yaw_dot_rad_s=msg.yaw_dot,
        )


@dataclass(frozen=True)
class Px4LocalSetpoint:
    """PX4 局部 NED 系（`MAV_FRAME_LOCAL_NED`）中的 setpoint。

    这是唯一允许交给 MAVLink 后端的形态：后端拿到本对象后只做字段搬运，
    不再做任何轴向/符号判断。
    """

    position_m: Tuple[float, float, float]
    velocity_m_s: Tuple[float, float, float]
    acceleration_m_s2: Tuple[float, float, float]
    yaw_rad: float
    yaw_rate_rad_s: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "position_m", _tuple3(self.position_m))
        object.__setattr__(self, "velocity_m_s", _tuple3(self.velocity_m_s))
        object.__setattr__(self, "acceleration_m_s2", _tuple3(self.acceleration_m_s2))
        object.__setattr__(self, "yaw_rad", _no_neg_zero(self.yaw_rad))
        object.__setattr__(self, "yaw_rate_rad_s", _no_neg_zero(self.yaw_rate_rad_s))

    def as_array(self) -> np.ndarray:
        """[x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate]（诊断/日志用）。"""
        return np.array(
            [
                *self.position_m,
                *self.velocity_m_s,
                *self.acceleration_m_s2,
                self.yaw_rad,
                self.yaw_rate_rad_s,
            ],
            dtype=float,
        )


def ros_local_to_px4_ned(
    src: Any,
    limits: "SetpointLimits | None" = None,
    normalize_yaw: bool = True,
) -> Px4LocalSetpoint:
    """ROS Z-up setpoint → PX4 NED setpoint（校验后）。

    ``src`` 可以是 `RosLocalSetpoint`，也可以是任何带同名字段的同形对象
    （例如从消息直接构造的测试替身）。

    ``normalize_yaw=False``：|yaw| > π 时**抛错**而不是归一化。默认 True，
    因为上游会跨 ±π 环绕、跨帧累积 2π，归一化是常规路径而不是异常路径。
    """
    src = _coerce_ros(src)
    lim = limits or SetpointLimits()
    _require_finite("position_m", src.position_m)
    _require_finite("velocity_m_s", src.velocity_m_s)
    _require_finite("acceleration_m_s2", src.acceleration_m_s2)
    _require_finite("yaw_rad", src.yaw_rad)
    _require_finite("yaw_dot_rad_s", src.yaw_dot_rad_s)

    if abs(src.yaw_rad) > _PI:
        if not normalize_yaw:
            raise FrameValidationError(
                "yaw_error", "yaw_rad", src.yaw_rad, _PI, "|yaw| > π 且未开启归一化"
            )
        yaw_rad = normalize_angle_pi(src.yaw_rad)
    else:
        yaw_rad = float(src.yaw_rad)

    _check_norm("velocity_m_s", src.velocity_m_s, lim.max_velocity_m_s, "velocity_overflow")
    _check_norm(
        "acceleration_m_s2",
        src.acceleration_m_s2,
        lim.max_acceleration_m_s2,
        "acceleration_overflow",
    )
    _check_scalar(
        "yaw_dot_rad_s", src.yaw_dot_rad_s, lim.max_yaw_rate_rad_s, "yaw_rate_overflow"
    )

    return Px4LocalSetpoint(
        position_m=vector_ros_local_to_ned(src.position_m),
        velocity_m_s=vector_ros_local_to_ned(src.velocity_m_s),
        acceleration_m_s2=vector_ros_local_to_ned(src.acceleration_m_s2),
        yaw_rad=-yaw_rad,
        yaw_rate_rad_s=-float(src.yaw_dot_rad_s),
    )


def px4_ned_to_ros_local(src: Any, limits: "SetpointLimits | None" = None) -> RosLocalSetpoint:
    """PX4 NED setpoint → ROS Z-up setpoint（**反向**，用于遥测/诊断/测试）。

    反向路径同样校验：PX4 遥测/回放数据不可信（可能含 NaN），在反向路径上静默
    接受 NaN 会让诊断脚本给出看似合理的假结论。
    """
    src = _coerce_ned(src)
    lim = limits or SetpointLimits()
    _require_finite("position_m", src.position_m)
    _require_finite("velocity_m_s", src.velocity_m_s)
    _require_finite("acceleration_m_s2", src.acceleration_m_s2)
    _require_finite("yaw_rad", src.yaw_rad)
    _require_finite("yaw_rate_rad_s", src.yaw_rate_rad_s)
    _check_norm("velocity_m_s", src.velocity_m_s, lim.max_velocity_m_s, "velocity_overflow")
    _check_norm(
        "acceleration_m_s2",
        src.acceleration_m_s2,
        lim.max_acceleration_m_s2,
        "acceleration_overflow",
    )
    _check_scalar(
        "yaw_rate_rad_s", src.yaw_rate_rad_s, lim.max_yaw_rate_rad_s, "yaw_rate_overflow"
    )
    return RosLocalSetpoint(
        position_m=vector_ned_to_ros_local(src.position_m),
        velocity_m_s=vector_ned_to_ros_local(src.velocity_m_s),
        acceleration_m_s2=vector_ned_to_ros_local(src.acceleration_m_s2),
        yaw_rad=normalize_angle_pi(-float(src.yaw_rad)),
        yaw_dot_rad_s=-float(src.yaw_rate_rad_s),
    )


def _coerce_ros(src: Any) -> RosLocalSetpoint:
    if isinstance(src, RosLocalSetpoint):
        return src
    return RosLocalSetpoint(
        position_m=getattr(src, "position_m"),
        velocity_m_s=getattr(src, "velocity_m_s"),
        acceleration_m_s2=getattr(src, "acceleration_m_s2"),
        yaw_rad=getattr(src, "yaw_rad"),
        yaw_dot_rad_s=getattr(src, "yaw_dot_rad_s", 0.0),
    )


def _coerce_ned(src: Any) -> Px4LocalSetpoint:
    if isinstance(src, Px4LocalSetpoint):
        return src
    return Px4LocalSetpoint(
        position_m=getattr(src, "position_m"),
        velocity_m_s=getattr(src, "velocity_m_s"),
        acceleration_m_s2=getattr(src, "acceleration_m_s2"),
        yaw_rad=getattr(src, "yaw_rad"),
        yaw_rate_rad_s=getattr(src, "yaw_rate_rad_s", 0.0),
    )


# ---------------------------------------------------------------------------
# 有状态的转换器（记录归一化/拒绝次数，供诊断）
# ---------------------------------------------------------------------------
@dataclass
class FrameConverter:
    """带诊断计数的转换器：显式持有 limits，并统计归一化与拒绝事件。

    为什么要有状态版本：契约要求"无效与失败分开统计"。后端节点持有一个实例，
    把 `report()` 打进诊断话题即可，不必在后端重复实现计数逻辑。
    """

    limits: SetpointLimits = field(default_factory=SetpointLimits)
    strict_yaw: bool = False
    yaw_normalize_count: int = 0
    convert_count: int = 0
    reject_counts: Dict[str, int] = field(default_factory=dict)

    @property
    def normalize_yaw(self) -> bool:
        return not self.strict_yaw

    def from_ros(self, src: Any) -> Px4LocalSetpoint:
        """ROS Z-up → NED；失败时计数并抛出 `FrameValidationError`。"""
        ros = _coerce_ros(src)
        try:
            out = ros_local_to_px4_ned(ros, self.limits, normalize_yaw=self.normalize_yaw)
        except FrameValidationError as exc:
            self.reject_counts[exc.reason] = self.reject_counts.get(exc.reason, 0) + 1
            raise
        if abs(ros.yaw_rad) > _PI:
            self.yaw_normalize_count += 1
        self.convert_count += 1
        return out

    def to_ros(self, src: Any) -> RosLocalSetpoint:
        """NED → ROS Z-up（诊断方向）。"""
        try:
            out = px4_ned_to_ros_local(src, self.limits)
        except FrameValidationError as exc:
            self.reject_counts[exc.reason] = self.reject_counts.get(exc.reason, 0) + 1
            raise
        self.convert_count += 1
        return out

    def report(self) -> Dict[str, Any]:
        """诊断摘要（JSON 可序列化）。"""
        return {
            "convert_count": int(self.convert_count),
            "yaw_normalize_count": int(self.yaw_normalize_count),
            "reject_counts": {k: int(v) for k, v in self.reject_counts.items()},
            "limits": {
                "max_velocity_m_s": float(self.limits.max_velocity_m_s),
                "max_acceleration_m_s2": float(self.limits.max_acceleration_m_s2),
                "max_yaw_rate_rad_s": float(self.limits.max_yaw_rate_rad_s),
            },
        }


# ---------------------------------------------------------------------------
# type_mask 构造
# ---------------------------------------------------------------------------
class YawMode(str, Enum):
    """setpoint 用 yaw 角还是 yaw 角速率。"""

    YAW = "yaw"
    YAW_RATE = "yaw_rate"


@dataclass(frozen=True)
class TypeMask:
    """`SET_POSITION_TARGET_LOCAL_NED.type_mask` 的显式构造结果。

    为什么让后端用本类而不是自己写位运算：MAVLink 规范里"yaw 与 yaw_rate 都被
    忽略"和"都被使用"都是合法位组合，但后者在 PX4 上的行为**未定义**（接收端
    只是分别按位把两个字段写成 NaN 或原值）。把它做成"必须显式选择 YawMode"
    的构造器，用显式性排除这个歧义。
    """

    mask: int
    mode: str
    yaw_mode: YawMode
    uses_acceleration: bool

    def __int__(self) -> int:
        return int(self.mask)

    @property
    def ignores_yaw(self) -> bool:
        return bool(self.mask & POSITION_TARGET_TYPEMASK_YAW_IGNORE)

    @property
    def ignores_yaw_rate(self) -> bool:
        return bool(self.mask & POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

    @classmethod
    def for_mode(cls, mode: str, yaw_mode: "YawMode | str" = YawMode.YAW) -> "TypeMask":
        """按控制模式构造 type_mask。

        ``mode``:
            ``"position_velocity_acceleration"`` — 位置/速度/加速度全用；
            ``"position_velocity"`` — 忽略加速度（448）；
            ``"position_acceleration"`` — 忽略速度（56）；
            ``"velocity_acceleration"`` — 忽略位置（7）；
            ``"acceleration"`` — 只发加速度（7 + 56）。
        每个模式再叠加"恰好忽略一个 yaw 字段"：``yaw_mode=YAW`` 时置
        YAW_RATE_IGNORE (2048)，``yaw_mode=YAW_RATE`` 时置 YAW_IGNORE (1024)。
        """
        ym = YawMode(yaw_mode)
        acc_bits = 0
        vel_bits = 0
        pos_bits = 0
        if mode == "position_velocity_acceleration":
            pass
        elif mode == "position_velocity":
            acc_bits = (
                POSITION_TARGET_TYPEMASK_AX_IGNORE
                | POSITION_TARGET_TYPEMASK_AY_IGNORE
                | POSITION_TARGET_TYPEMASK_AZ_IGNORE
            )
        elif mode == "position_acceleration":
            vel_bits = (
                POSITION_TARGET_TYPEMASK_VX_IGNORE
                | POSITION_TARGET_TYPEMASK_VY_IGNORE
                | POSITION_TARGET_TYPEMASK_VZ_IGNORE
            )
        elif mode == "velocity_acceleration":
            pos_bits = (
                POSITION_TARGET_TYPEMASK_X_IGNORE
                | POSITION_TARGET_TYPEMASK_Y_IGNORE
                | POSITION_TARGET_TYPEMASK_Z_IGNORE
            )
        elif mode == "acceleration":
            pos_bits = (
                POSITION_TARGET_TYPEMASK_X_IGNORE
                | POSITION_TARGET_TYPEMASK_Y_IGNORE
                | POSITION_TARGET_TYPEMASK_Z_IGNORE
            )
            vel_bits = (
                POSITION_TARGET_TYPEMASK_VX_IGNORE
                | POSITION_TARGET_TYPEMASK_VY_IGNORE
                | POSITION_TARGET_TYPEMASK_VZ_IGNORE
            )
        else:
            raise ValueError(
                f"未知控制模式 {mode!r}；可选 position_velocity_acceleration / "
                "position_velocity / position_acceleration / velocity_acceleration / acceleration"
            )
        yaw_bits = (
            POSITION_TARGET_TYPEMASK_YAW_IGNORE
            if ym is YawMode.YAW_RATE
            else POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        mask = int(pos_bits | vel_bits | acc_bits | yaw_bits)
        if mask & POSITION_TARGET_TYPEMASK_FORCE_SET:  # pragma: no cover - 防御性
            raise AssertionError("本工程不得置 FORCE_SET 位（PX4 会拒收带加速度的力 setpoint）")
        return cls(mask=mask, mode=mode, yaw_mode=ym, uses_acceleration=acc_bits == 0)

    def describe(self) -> str:
        """人类可读的位说明（诊断字符串）。"""
        names = [
            ("X_IGNORE", POSITION_TARGET_TYPEMASK_X_IGNORE),
            ("Y_IGNORE", POSITION_TARGET_TYPEMASK_Y_IGNORE),
            ("Z_IGNORE", POSITION_TARGET_TYPEMASK_Z_IGNORE),
            ("VX_IGNORE", POSITION_TARGET_TYPEMASK_VX_IGNORE),
            ("VY_IGNORE", POSITION_TARGET_TYPEMASK_VY_IGNORE),
            ("VZ_IGNORE", POSITION_TARGET_TYPEMASK_VZ_IGNORE),
            ("AX_IGNORE", POSITION_TARGET_TYPEMASK_AX_IGNORE),
            ("AY_IGNORE", POSITION_TARGET_TYPEMASK_AY_IGNORE),
            ("AZ_IGNORE", POSITION_TARGET_TYPEMASK_AZ_IGNORE),
            ("YAW_IGNORE", POSITION_TARGET_TYPEMASK_YAW_IGNORE),
            ("YAW_RATE_IGNORE", POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE),
        ]
        active = [n for n, b in names if self.mask & b]
        return (
            f"type_mask={self.mask} mode={self.mode} "
            f"yaw={self.yaw_mode.value} ignore=[{','.join(active)}]"
        )

    def require_yaw_input(self, setpoint: Px4LocalSetpoint) -> None:
        """校验 setpoint 提供的就是本 mask 期望的那个 yaw 输入。

        后端最常见的静默错误：选了 YAW_RATE 模式却沿用上一版的 `yaw_rad`
        字段（反之亦然）。本方法把"mask 与数据不匹配"变成显式异常。
        """
        if not isinstance(setpoint, Px4LocalSetpoint):
            raise TypeError("require_yaw_input 需要 Px4LocalSetpoint")
        if not math.isfinite(setpoint.yaw_rad) or not math.isfinite(setpoint.yaw_rate_rad_s):
            raise FrameValidationError(
                "nan_inf", "yaw_rad/yaw_rate_rad_s", setpoint.yaw_rad, None, "yaw 输入非有限"
            )
        if self.yaw_mode is YawMode.YAW_RATE and setpoint.yaw_rate_rad_s == 0.0:
            raise FrameValidationError(
                "yaw_error",
                "yaw_rate_rad_s",
                0.0,
                None,
                "mask 选择了 yaw_rate，但 setpoint 的 yaw_rate 为 0（疑似字段未填）",
            )
        if self.yaw_mode is YawMode.YAW and setpoint.yaw_rate_rad_s != 0.0:
            raise FrameValidationError(
                "yaw_error",
                "yaw_rate_rad_s",
                setpoint.yaw_rate_rad_s,
                None,
                "mask 选择了 yaw，但 setpoint 同时给出非零 yaw_rate（PX4 行为未定义）",
            )


# ============================================================================
# 局部系对齐：轴变换之外的平移与水平朝向
# ============================================================================
def ros_local_to_ned_rotation(yaw_offset_rad: float = 0.0) -> np.ndarray:
    """ROS 局部系 → PX4 局部 NED 的**旋转**部分（含水平朝向差）。

    ``yaw_offset_rad`` 是 ROS 局部系 +x 相对 PX4 +x（北）的航向差::

        p_ned = R_z(-yaw_offset) @ diag(1,-1,-1) @ p_ros

    ``yaw_offset_rad = 0`` 时退化为 ``diag(1,-1,-1)``（即原实现隐含的"两系同向"假设）。
    """
    if not math.isfinite(float(yaw_offset_rad)):
        raise FrameValidationError(
            "yaw_offset_nonfinite", "yaw_offset_rad", yaw_offset_rad, None, "航向偏移必须有限"
        )
    c, s = math.cos(float(yaw_offset_rad)), math.sin(float(yaw_offset_rad))
    rz = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)   # R_z(-yaw)
    return rz @ between_frame_rotation()


@dataclass(frozen=True)
class LocalFrameAlignment:
    """ROS 局部系 ↔ PX4 局部 NED 的刚体对齐（平移 + 水平朝向）。

    - `yaw_offset_rad`：ROS 局部系 +x 相对 PX4 +x 的航向差。
    - `translation_m`：从 PX4 原点到 ROS 原点的偏移（以 ROS 系表达）。
      可通过 PX4 外部视觉融合使其为 0，或实测标定非零值；节点要求
      `frame_alignment_origin_evidence` 显式确认原点关系，缺证据则拒绝位置 setpoint。
    - `verified` 是**元数据**（这份对齐是否经实测核实），不参与数学；是否放行位置由调用方决定。
    """

    yaw_offset_rad: float = 0.0
    translation_m: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    verified: bool = False
    source: str = "unset"

    def rotation(self) -> np.ndarray:
        return ros_local_to_ned_rotation(self.yaw_offset_rad)

    def is_identity_transform(self, tol_m: float = 1e-9, tol_rad: float = 1e-9) -> bool:
        t = np.asarray(self.translation_m, dtype=float)
        return bool(np.max(np.abs(t)) <= tol_m and abs(self.yaw_offset_rad) <= tol_rad)

    def has_translation(self, tol_m: float = 1e-9) -> bool:
        return bool(np.max(np.abs(np.asarray(self.translation_m, dtype=float))) > tol_m)

    def position_ros_to_ned(self, v: Any) -> np.ndarray:
        """ROS 位置（含原点偏移）→ NED 位置。"""
        p = np.asarray(v, dtype=float).reshape(3)
        if not np.all(np.isfinite(p)):
            raise FrameValidationError(
                "position_nonfinite", "position_m", tuple(p), None, "位置分量必须有限"
            )
        t = np.asarray(self.translation_m, dtype=float).reshape(3)
        return self.rotation() @ (p - t)

    def position_ned_to_ros(self, v: Any) -> np.ndarray:
        """NED 位置 → ROS 位置（反向，用于遥测/诊断）。"""
        n = np.asarray(v, dtype=float).reshape(3)
        if not np.all(np.isfinite(n)):
            raise FrameValidationError(
                "position_nonfinite", "position_m", tuple(n), None, "位置分量必须有限"
            )
        t = np.asarray(self.translation_m, dtype=float).reshape(3)
        return self.rotation().T @ n + t

    def to_dict(self) -> Dict[str, Any]:
        return {
            "yaw_offset_rad": float(self.yaw_offset_rad),
            "translation_m": [float(x) for x in self.translation_m],
            "verified": bool(self.verified),
            "source": str(self.source),
        }


class YawAlignmentResidual:
    """被动核实：比较 ROS 侧航向与 PX4 侧航向，看残差是否与声明的 yaw 偏移一致。

    只在接近水平且角速率很小时采样（倾斜会把偏航耦合进姿态，残差不可信）。
    **不发送任何指令**，因此核实过程本身不改变飞行器状态。
    """

    def __init__(
        self,
        *,
        tolerance_rad: float,
        required_samples: int = 10,
        max_tilt_rad: float = 0.35,
        max_yaw_rate_rad_s: float = 0.5,
        max_sample_spread_rad: float = 0.05,
        max_sample_skew_s: float = 0.05,
        max_attitude_age_s: float = 0.2,
    ) -> None:
        if tolerance_rad <= 0.0:
            raise ValueError("tolerance_rad 必须为正")
        self.tolerance_rad = float(tolerance_rad)
        self.required_samples = int(required_samples)
        self.max_tilt_rad = float(max_tilt_rad)
        self.max_yaw_rate_rad_s = float(max_yaw_rate_rad_s)
        self.max_sample_spread_rad = float(max_sample_spread_rad)
        #: 本机接收两份航向的时差上限；测量时间同步仍需真机验证
        self.max_sample_skew_s = float(max_sample_skew_s)
        #: PX4 姿态数据的最大年龄；过期说明这次比较用的姿态不是当前的
        self.max_attitude_age_s = float(max_attitude_age_s)
        self._residuals: list = []
        #: 采样被拒的细分计数：要能看出是"缺姿态"还是"真不一致"
        self.rejected_missing_attitude = 0
        self.rejected_missing_timing = 0
        self.rejected_stale_attitude = 0
        self.rejected_skew = 0

    def reset(self) -> None:
        self._residuals.clear()

    @property
    def samples(self) -> int:
        return len(self._residuals)

    @property
    def mean_residual_rad(self):
        if not self._residuals:
            return None
        return float(sum(self._residuals) / len(self._residuals))

    @property
    def spread_rad(self):
        if len(self._residuals) < 2:
            return None
        return float(max(self._residuals) - min(self._residuals))

    def observe(
        self,
        *,
        ros_yaw_rad: float,
        px4_yaw_rad: float,
        roll_rad: float | None,
        pitch_rad: float | None,
        yaw_rate_rad_s: float | None,
        declared_yaw_offset_rad: float = 0.0,
        ros_mono_s: float | None = None,
        px4_mono_s: float | None = None,
        attitude_age_s: float | None = None,
    ) -> bool:
        """记一次样本；姿态/角速率不合格时不采纳（返回 False）。

        残差 = `wrap(ros_yaw + declared_offset + px4_yaw)`，与 setpoint 的偏航换算
        `yaw_ned = -(yaw_ros + offset)` 使用**同一约定**，因此"核实通过"与"最终发送方向"
        不会互相矛盾（有测试用同一组物理例子同时验证两者）。

        `declared_offset = 0`（identity）时残差就是 `wrap(ros_yaw + px4_yaw)`：
        ROS 的 +Z 与 NED 的 +Z 方向相反，同一朝向的偏航数值应互为相反数。

        采样门槛（任一不满足即不采纳，并计入对应计数）：
          * roll/pitch/yaw_rate 必须**显式提供且非 None**——缺失不当作 0；
          * 节点传入到达时刻时，必须同时提供两个时刻与姿态年龄；
          * `attitude_age_s` 不得超 `max_attitude_age_s`；
          * 两个到达时刻之差不得超 `max_sample_skew_s`。这只能约束接收时差，
            不能替代 VIO/PX4 测量时间戳的真机同步验证。
        """
        # 缺失姿态 ⇒ 不采样。**不得**用 0 代替：0 是"水平且不转"这个观测结论，不是缺省值。
        if roll_rad is None or pitch_rad is None or yaw_rate_rad_s is None:
            self.rejected_missing_attitude += 1
            return False
        for val in (roll_rad, pitch_rad, yaw_rate_rad_s, ros_yaw_rad, px4_yaw_rad):
            if not math.isfinite(float(val)):
                return False
        if ros_mono_s is not None or px4_mono_s is not None:
            if ros_mono_s is None or px4_mono_s is None or attitude_age_s is None:
                self.rejected_missing_timing += 1
                return False
        if attitude_age_s is not None:
            if not math.isfinite(float(attitude_age_s)):
                return False
            if float(attitude_age_s) > self.max_attitude_age_s:
                self.rejected_stale_attitude += 1
                return False
        if ros_mono_s is not None and px4_mono_s is not None:
            skew = abs(float(ros_mono_s) - float(px4_mono_s))
            if not math.isfinite(skew) or skew > self.max_sample_skew_s:
                self.rejected_skew += 1
                return False
        if abs(float(roll_rad)) > self.max_tilt_rad or abs(float(pitch_rad)) > self.max_tilt_rad:
            return False
        if abs(float(yaw_rate_rad_s)) > self.max_yaw_rate_rad_s:
            return False
        residual = normalize_angle_pi(
            float(ros_yaw_rad) + float(declared_yaw_offset_rad) + float(px4_yaw_rad)
        )
        self._residuals.append(residual)
        return True

    @property
    def yaw_verified(self) -> bool:
        """**仅**表示"航向残差已核实"，**不**代表原点/平移正确。

        名字里带 yaw 是刻意的：调用方若把它当成"整个刚体对齐已核实"，
        就会在原点未知时放行位置 setpoint——那正是要避免的错误。
        """
        if len(self._residuals) < self.required_samples:
            return False
        mean = self.mean_residual_rad
        spread = self.spread_rad
        if mean is None or abs(mean) > self.tolerance_rad:
            return False
        if spread is not None and spread > self.max_sample_spread_rad:
            return False
        return True

    @property
    def verified(self) -> bool:
        """**已弃用别名**：只反映航向核实，不代表完整刚体对齐。

        保留它是为了避免静默破坏调用方，同时用 DeprecationWarning 把语义讲清楚。
        新代码请用 `yaw_verified`，并**单独**核实原点/平移。
        """
        import warnings

        warnings.warn(
            "YawAlignmentResidual.verified 只表示航向核实，不代表完整刚体对齐；"
            "请改用 yaw_verified，并单独核实原点/平移。",
            DeprecationWarning,
            stacklevel=2,
        )
        return self.yaw_verified

    def to_dict(self) -> Dict[str, Any]:
        return {
            "samples": self.samples,
            "required_samples": self.required_samples,
            "tolerance_rad": self.tolerance_rad,
            "mean_residual_rad": self.mean_residual_rad,
            "spread_rad": self.spread_rad,
            "yaw_verified": self.yaw_verified,
            "covers": "yaw_only",
            "note": "只核实航向；原点/平移必须另有证据，不能由此推断",
            "rejected": {
                "missing_attitude": int(self.rejected_missing_attitude),
                "missing_timing": int(self.rejected_missing_timing),
                "stale_attitude": int(self.rejected_stale_attitude),
                "time_skew": int(self.rejected_skew),
            },
        }



# ============================================================================
# 对齐感知的完整 setpoint 换算（位置/速度/加速度/偏航/偏航角速率）
# ============================================================================
def _require_finite_scalar(name: str, value: Any, reason: str) -> float:
    v = float(value)
    if not math.isfinite(v):
        raise FrameValidationError(reason, name, v, None, f"{name} 必须有限")
    return v


def ros_local_to_ned_setpoint(
    src: Any,
    alignment: "LocalFrameAlignment | None" = None,
    limits: "SetpointLimits | None" = None,
    normalize_yaw: bool = True,
) -> Px4LocalSetpoint:
    """ROS Z-up setpoint → PX4 NED setpoint，**含完整对齐**（平移/航向偏移）。

    与 `ros_local_to_px4_ned` 的关系：`alignment=None` 或 `yaw_offset=0`、`translation=0`
    时两者结果**逐字段相同**（由测试锁定）。差别只在：
      * 位置减去平移后再旋转；
      * yaw / yaw_rate 减去/带上航向偏移。

    `normalize_yaw=False` 时 |yaw+offset| > π 会抛错而不是归一化。
    """
    align = alignment if alignment is not None else LocalFrameAlignment()
    src = _coerce_ros(src)
    lim = limits or SetpointLimits()
    _require_finite("position_m", src.position_m)
    _require_finite("velocity_m_s", src.velocity_m_s)
    _require_finite("acceleration_m_s2", src.acceleration_m_s2)
    _require_finite("yaw_rad", src.yaw_rad)
    _require_finite("yaw_dot_rad_s", src.yaw_dot_rad_s)

    # --- 位置：减平移 → 旋转 ---
    position_ned = align.position_ros_to_ned(src.position_m)

    # --- 速度/加速度：自由矢量，只旋转（**不减平移**） ---
    rotation = align.rotation()
    velocity_ned = rotation @ np.asarray(src.velocity_m_s, dtype=float).reshape(3)
    acceleration_ned = rotation @ np.asarray(src.acceleration_m_s2, dtype=float).reshape(3)

    # --- 偏航：符号翻转 + 航向偏移 ---
    raw_yaw = float(src.yaw_rad) + float(align.yaw_offset_rad)
    if abs(raw_yaw) > _PI:
        if not normalize_yaw:
            raise FrameValidationError(
                "yaw_error", "yaw_rad", raw_yaw, _PI,
                "|yaw + yaw_offset| > π 且未开启归一化",
            )
        yaw_ned = normalize_angle_pi(raw_yaw)
    else:
        yaw_ned = raw_yaw
    yaw_ned = -yaw_ned
    yaw_rate_ned = -float(src.yaw_dot_rad_s)

    # --- 限值校验（与旧路径同一套限值，避免"换条路径就能绕过限值"） ---
    _check_norm("velocity_m_s", src.velocity_m_s, lim.max_velocity_m_s, "velocity_overflow")
    _check_norm(
        "acceleration_m_s2", src.acceleration_m_s2,
        lim.max_acceleration_m_s2, "acceleration_overflow",
    )
    _check_scalar(
        "yaw_dot_rad_s", src.yaw_dot_rad_s, lim.max_yaw_rate_rad_s, "yaw_rate_overflow"
    )

    return Px4LocalSetpoint(
        position_m=_tuple3(position_ned),
        velocity_m_s=_tuple3(velocity_ned),
        acceleration_m_s2=_tuple3(acceleration_ned),
        yaw_rad=yaw_ned,
        yaw_rate_rad_s=yaw_rate_ned,
    )


def ned_to_ros_local_setpoint(
    src: Any,
    alignment: "LocalFrameAlignment | None" = None,
    limits: "SetpointLimits | None" = None,
) -> RosLocalSetpoint:
    """PX4 NED setpoint → ROS Z-up setpoint（**精确逆变换**，用于遥测/诊断/测试）。

    逆关系：`p_ros = R(φ)ᵀ·p_ned + t`，`ψ_ros = −ψ_ned − φ`，`ψ̇_ros = −ψ̇_ned`。
    """
    align = alignment if alignment is not None else LocalFrameAlignment()
    src = _coerce_ned(src)
    lim = limits or SetpointLimits()
    _require_finite("position_m", src.position_m)
    _require_finite("velocity_m_s", src.velocity_m_s)
    _require_finite("acceleration_m_s2", src.acceleration_m_s2)
    _require_finite("yaw_rad", src.yaw_rad)
    _require_finite("yaw_rate_rad_s", src.yaw_rate_rad_s)

    rotation_t = align.rotation().T
    position_ros = align.position_ned_to_ros(src.position_m)
    velocity_ros = rotation_t @ np.asarray(src.velocity_m_s, dtype=float).reshape(3)
    acceleration_ros = rotation_t @ np.asarray(src.acceleration_m_s2, dtype=float).reshape(3)
    yaw_ros = normalize_angle_pi(-float(src.yaw_rad) - float(align.yaw_offset_rad))
    yaw_rate_ros = -float(src.yaw_rate_rad_s)

    _check_norm("velocity_m_s", src.velocity_m_s, lim.max_velocity_m_s, "velocity_overflow")
    _check_norm(
        "acceleration_m_s2", src.acceleration_m_s2,
        lim.max_acceleration_m_s2, "acceleration_overflow",
    )
    _check_scalar(
        "yaw_rate_rad_s", src.yaw_rate_rad_s, lim.max_yaw_rate_rad_s, "yaw_rate_overflow"
    )

    return RosLocalSetpoint(
        position_m=_tuple3(position_ros),
        velocity_m_s=_tuple3(velocity_ros),
        acceleration_m_s2=_tuple3(acceleration_ros),
        yaw_rad=yaw_ros,
        yaw_dot_rad_s=yaw_rate_ros,
    )
