"""`px4_frames` 的坐标系约定与校验测试。

这些测试不只是"跑通"：**它们锁死 ROS Z-up ↔ PX4 NED 的轴向与偏航符号**。
模块 docstring 里宣称的每一行约定表，在这里都对应一个带具体数值的断言；
谁要改约定，必然先让这些测试变红。

覆盖：
1. 轴向映射表（具体向量，不是公式复述）；
2. 旋转矩阵性质（proper rotation、对合、模长保持）；
3. 偏航符号：用**旋转后的向量**证明，而不是只比公式；
4. ROS ↔ NED 往返；
5. 校验拒绝：非有限、超包线、yaw 归一化（含 ±π 环绕）、strict 模式；
6. type_mask 构造：命名位与数值、模式组合、PX4 头文件取值核对。

全部离线：不连接飞控/串口/UDP，不导入 rclpy。
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from boom_birds_nav import px4_frames as P
from boom_birds_nav.px4_frames import (
    MASK_POSITION_VELOCITY_YAW,
    FrameConverter,
    FrameValidationError,
    Px4LocalSetpoint,
    RosLocalSetpoint,
    SetpointLimits,
    TypeMask,
    YawMode,
)


def _rot_z(angle_rad: float) -> np.ndarray:
    """绕 +Z 的右手旋转（ROS 与 NED 各自的 yaw 定义都是"绕各自 +Z 右手"）。"""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _zero(yaw: float = 0.0, yaw_dot: float = 0.0) -> RosLocalSetpoint:
    return RosLocalSetpoint((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), yaw, yaw_dot)


# ---------------------------------------------------------------------------
# 1) 轴向映射表：具体向量 → 具体向量
# ---------------------------------------------------------------------------
#: (id, ROS Z-up 向量, 期望的 NED 向量)。每一行就是约定表的一行。
AXIS_CONVENTION_TABLE = [
    pytest.param((1.0, 0.0, 0.0), (1.0, 0.0, 0.0), id="forward-to-north"),
    pytest.param((0.0, 1.0, 0.0), (0.0, -1.0, 0.0), id="left-to-west"),
    pytest.param((0.0, 0.0, 1.0), (0.0, 0.0, -1.0), id="up-to-negative-down"),
    pytest.param((0.0, 0.0, -1.0), (0.0, 0.0, 1.0), id="gravity-to-positive-down"),
    pytest.param((1.0, 1.0, 1.0), (1.0, -1.0, -1.0), id="mixed-signs"),
    pytest.param((-3.0, 2.5, -4.0), (-3.0, -2.5, 4.0), id="negative-components"),
]


@pytest.mark.parametrize("ros_vec,ned_vec", AXIS_CONVENTION_TABLE)
def test_axis_convention_vectors(ros_vec, ned_vec):
    """约定表：R = diag(1, -1, -1)，逐行用具体向量断言。"""
    got = P.vector_ros_local_to_ned(ros_vec)
    assert got == pytest.approx(np.array(ned_vec, dtype=float), abs=1e-12)


def test_axis_convention_table_is_complete():
    """约定表必须给出三轴 + 重力的完整映射，避免"漏一行没人发现"。"""
    assert len(AXIS_CONVENTION_TABLE) == 6
    mapped = {tuple(p.values[0]) for p in AXIS_CONVENTION_TABLE}
    for axis in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (0.0, 0.0, -1.0)):
        assert axis in mapped


def test_between_frame_rotation_is_proper_and_involutive():
    """R 必须是 proper rotation（det=+1）且对合（R²=I）——它是一次轴重标记。"""
    R = P.between_frame_rotation()
    assert R == pytest.approx(np.diag([1.0, -1.0, -1.0]))
    assert float(np.linalg.det(R)) == pytest.approx(1.0)
    assert R @ R == pytest.approx(np.eye(3))
    # 返回副本：调用方改不坏模块内部状态
    R[0, 0] = 99.0
    assert P.between_frame_rotation()[0, 0] == 1.0


def test_rotation_preserves_norms_for_all_three_vector_kinds():
    """位置/速度/加速度共用同一变换，且模长不变（无缩放、无剪切）。"""
    vec = (1.5, -2.25, 3.75)
    assert float(np.linalg.norm(P.vector_ros_local_to_ned(vec))) == pytest.approx(
        float(np.linalg.norm(vec))
    )
    sp = P.ros_local_to_px4_ned(RosLocalSetpoint(vec, vec, vec, 0.0))
    assert float(np.linalg.norm(sp.velocity_m_s)) == pytest.approx(float(np.linalg.norm(vec)))
    assert float(np.linalg.norm(sp.acceleration_m_s2)) == pytest.approx(
        float(np.linalg.norm(vec))
    )


def test_gravity_direction_is_consistent_in_both_frames():
    """ROS 重力是 (0,0,-9.81)（Z-up），NED 是 (0,0,+9.81)（D 向下为正）。"""
    assert P.vector_ros_local_to_ned((0.0, 0.0, -9.81)) == pytest.approx((0.0, 0.0, 9.81))


# ---------------------------------------------------------------------------
# 2) 偏航：用向量旋转证明符号，而不是只比公式
# ---------------------------------------------------------------------------
YAW_SIGN_TABLE = [
    pytest.param(0.0, 0.0, id="zero"),
    pytest.param(+math.pi / 2, -math.pi / 2, id="ros-left-90-to-ned-minus-90"),
    pytest.param(-math.pi / 2, +math.pi / 2, id="ros-right-90-to-ned-plus-90"),
    pytest.param(+math.pi / 4, -math.pi / 4, id="ros-left-45-to-ned-minus-45"),
    pytest.param(math.pi, math.pi, id="pi-is-pi"),
]


@pytest.mark.parametrize("ros_yaw,ned_yaw", YAW_SIGN_TABLE)
def test_yaw_sign_flip_by_rotated_forward_vector(ros_yaw, ned_yaw):
    """证明方式：把机头方向向量各转一次，再比较**向量**是否一致。

    ROS 侧：把 (1,0,0) 绕 +Z(上) 转 ros_yaw 得到机头方向，再用 R 映射到 NED。
    NED 侧：把北 (1,0,0) 用 ned_yaw 旋转。
    两者相等，才说明 `yaw_ned = -yaw_ros` 这个取值是对的——只对公式不足以发现
    "轴反向"带来的歧义。
    """
    fwd_ros = _rot_z(ros_yaw) @ np.array([1.0, 0.0, 0.0])
    assert P.vector_ros_local_to_ned(fwd_ros) == pytest.approx(
        _rot_z(ned_yaw) @ np.array([1.0, 0.0, 0.0]), abs=1e-12
    )


def test_ros_left_turn_becomes_negative_yaw_in_ned():
    """关键结论单独写死：ROS 左转（+yaw）⇒ NED 负角 ⇒ 朝向偏 +E（右）。

    这条是整条控制链上最容易被"顺手改成 +"的地方，单独一个测试盯住它，
    并在失败信息里给出两个坐标系下的机头向量。
    """
    sp = P.ros_local_to_px4_ned(_zero(yaw=math.pi / 2))
    assert sp.yaw_rad == pytest.approx(-math.pi / 2)

    ros_headed = _rot_z(math.pi / 2) @ np.array([1.0, 0.0, 0.0])          # 左
    headed_in_ned = P.vector_ros_local_to_ned(ros_headed)                  # 西
    assert headed_in_ned == pytest.approx((0.0, -1.0, 0.0))
    assert _rot_z(sp.yaw_rad) @ np.array([1.0, 0.0, 0.0]) == pytest.approx(headed_in_ned)


def test_yaw_rate_sign_flips_and_matches_small_angle_integration():
    """yaw_rate 符号：用"小角积分"验证，避免只试一个公式。

    取 ROS yaw=0、yaw_dot=+ω（左转），积分 dt 后 ROS 朝向约 +ω·dt；
    同一物理旋转在 NED 记法下应当是 -ω·dt（绕向下轴）。
    """
    omega = 0.7
    dt = 1e-3
    ros = _zero(yaw=0.0, yaw_dot=omega)
    ned = P.ros_local_to_px4_ned(ros)
    assert ned.yaw_rate_rad_s == pytest.approx(-omega)

    fwd_ros_after = _rot_z(ros.yaw_rad + omega * dt) @ np.array([1.0, 0.0, 0.0])
    fwd_ned_after = _rot_z(ned.yaw_rad + ned.yaw_rate_rad_s * dt) @ np.array([1.0, 0.0, 0.0])
    assert P.vector_ros_local_to_ned(fwd_ros_after) == pytest.approx(fwd_ned_after, abs=1e-12)


@pytest.mark.parametrize("ros_rate", [0.0, 1.25, -1.25, 0.5 * math.pi])
def test_yaw_rate_sign_table(ros_rate):
    """yaw_rate 符号表。"""
    sp = P.ros_local_to_px4_ned(_zero(yaw_dot=ros_rate))
    assert sp.yaw_rate_rad_s == pytest.approx(-ros_rate)


# ---------------------------------------------------------------------------
# 3) 往返
# ---------------------------------------------------------------------------
def test_round_trip_ros_ned_ros_is_identity():
    src = RosLocalSetpoint(
        (1.25, -2.5, 0.75), (0.5, 0.25, -0.125), (0.1, -0.2, 0.3), 0.6, -0.4
    )
    back = P.px4_ned_to_ros_local(P.ros_local_to_px4_ned(src))
    assert back.position_m == pytest.approx(src.position_m)
    assert back.velocity_m_s == pytest.approx(src.velocity_m_s)
    assert back.acceleration_m_s2 == pytest.approx(src.acceleration_m_s2)
    assert back.yaw_rad == pytest.approx(src.yaw_rad)
    assert back.yaw_dot_rad_s == pytest.approx(src.yaw_dot_rad_s)


def test_round_trip_ned_ros_ned_is_identity():
    src = Px4LocalSetpoint((3.0, 4.0, -5.0), (0.0, 1.0, 0.0), (0.0, 0.0, 9.81), -1.2, 0.3)
    again = P.ros_local_to_px4_ned(P.px4_ned_to_ros_local(src))
    assert again.position_m == pytest.approx(src.position_m)
    assert again.velocity_m_s == pytest.approx(src.velocity_m_s)
    assert again.acceleration_m_s2 == pytest.approx(src.acceleration_m_s2)
    assert again.yaw_rad == pytest.approx(src.yaw_rad)
    assert again.yaw_rate_rad_s == pytest.approx(src.yaw_rate_rad_s)


def test_round_trip_over_many_angles():
    """扫一圈角度做往返，防止只在 0 附近成立。"""
    for deg in range(-179, 181, 7):
        yaw = math.radians(deg)
        src = RosLocalSetpoint((0.0, 0.0, 0.0), (2.0, -1.0, 3.0), (0.0, 0.0, 0.0), yaw, 0.5)
        back = P.px4_ned_to_ros_local(P.ros_local_to_px4_ned(src))
        assert back.yaw_rad == pytest.approx(yaw, abs=1e-12), f"deg={deg}"
        assert back.velocity_m_s == pytest.approx(src.velocity_m_s), f"deg={deg}"


def test_position_command_extraction_from_duck_typed_message():
    """`from_position_command` 只读字段：用鸭子类型替身验证字段对应关系。"""

    class _V:
        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z

    class _Msg:
        position = _V(1.0, 2.0, 3.0)
        velocity = _V(4.0, 5.0, 6.0)
        acceleration = _V(7.0, 8.0, 9.0)
        yaw = 0.25
        yaw_dot = -0.5

    ros = RosLocalSetpoint.from_position_command(_Msg())
    assert ros.position_m == (1.0, 2.0, 3.0)
    assert ros.velocity_m_s == (4.0, 5.0, 6.0)
    assert ros.acceleration_m_s2 == (7.0, 8.0, 9.0)
    assert ros.yaw_rad == 0.25
    assert ros.yaw_dot_rad_s == -0.5
    assert P.ros_local_to_px4_ned(ros).position_m == (1.0, -2.0, -3.0)


# ---------------------------------------------------------------------------
# 4) 校验：非有限
# ---------------------------------------------------------------------------
def test_non_finite_position_rejected():
    for bad in ((float("nan"), 0.0, 0.0), (0.0, float("inf"), 0.0), (0.0, 0.0, float("-inf"))):
        with pytest.raises(FrameValidationError) as exc:
            P.ros_local_to_px4_ned(RosLocalSetpoint(bad, (0, 0, 0), (0, 0, 0), 0.0))
        assert exc.value.reason == "nan_inf"
        assert exc.value.field == "position_m"
        assert exc.value.to_dict()["reason"] == "nan_inf"


def test_non_finite_velocity_and_acceleration_rejected():
    with pytest.raises(FrameValidationError) as exc:
        P.ros_local_to_px4_ned(RosLocalSetpoint((0, 0, 0), (1.0, float("nan"), 0.0), (0, 0, 0), 0.0))
    assert exc.value.field == "velocity_m_s"
    with pytest.raises(FrameValidationError) as exc:
        P.ros_local_to_px4_ned(
            RosLocalSetpoint((0, 0, 0), (0, 0, 0), (float("nan"),) * 3, 0.0)
        )
    assert exc.value.field == "acceleration_m_s2"


@pytest.mark.parametrize("bad_yaw", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_yaw_rejected(bad_yaw):
    with pytest.raises(FrameValidationError) as exc:
        P.ros_local_to_px4_ned(_zero(yaw=bad_yaw))
    assert exc.value.reason == "nan_inf"
    assert exc.value.field == "yaw_rad"


def test_non_finite_yaw_rate_rejected():
    with pytest.raises(FrameValidationError) as exc:
        P.ros_local_to_px4_ned(_zero(yaw_dot=float("nan")))
    assert exc.value.reason == "nan_inf"
    assert exc.value.field == "yaw_dot_rad_s"


def test_reverse_direction_also_validates():
    """反向（NED→ROS）也必须校验：遥测/回放数据不可信，不能静默接受 NaN。"""
    with pytest.raises(FrameValidationError) as exc:
        P.px4_ned_to_ros_local(
            Px4LocalSetpoint((0, 0, 0), (0, 0, 0), (0, 0, 0), float("nan"), 0.0)
        )
    assert exc.value.reason == "nan_inf"


# ---------------------------------------------------------------------------
# 5) 校验：包线
# ---------------------------------------------------------------------------
def test_velocity_limit_rejected_with_reason_and_measured_norm():
    lim = SetpointLimits(max_velocity_m_s=2.0)
    with pytest.raises(FrameValidationError) as exc:
        P.ros_local_to_px4_ned(
            RosLocalSetpoint((0, 0, 0), (3.0, 4.0, 0.0), (0, 0, 0), 0.0), limits=lim
        )
    assert exc.value.reason == "velocity_overflow"
    assert exc.value.value == pytest.approx(5.0)  # 上报的是向量模，不是单轴
    assert exc.value.limit == pytest.approx(2.0)


def test_acceleration_limit_rejected():
    lim = SetpointLimits(max_acceleration_m_s2=5.0)
    with pytest.raises(FrameValidationError) as exc:
        P.ros_local_to_px4_ned(
            RosLocalSetpoint((0, 0, 0), (0, 0, 0), (0, 0, 6.0), 0.0), limits=lim
        )
    assert exc.value.reason == "acceleration_overflow"
    assert exc.value.value == pytest.approx(6.0)


def test_yaw_rate_limit_rejected_and_default_matches_upstream_clamp():
    """缺省 yaw_rate 限值必须等于上游 `traj_server.cpp` 的 PI 限幅。"""
    assert SetpointLimits().max_yaw_rate_rad_s == pytest.approx(math.pi)
    with pytest.raises(FrameValidationError) as exc:
        P.ros_local_to_px4_ned(_zero(yaw_dot=3.2))
    assert exc.value.reason == "yaw_rate_overflow"
    assert exc.value.value == pytest.approx(3.2)
    # 边界值本身允许（与上游一致：限幅就是 π）
    assert P.ros_local_to_px4_ned(_zero(yaw_dot=math.pi)).yaw_rate_rad_s == pytest.approx(-math.pi)


def test_limits_are_configurable_and_scale_the_whole_vector():
    """限值按向量模生效：单轴不超但合成超也要拒绝。"""
    vec = (1.21, 1.21, 0.0)  # |v| = 1.711
    with pytest.raises(FrameValidationError):
        P.ros_local_to_px4_ned(
            RosLocalSetpoint((0, 0, 0), vec, (0, 0, 0), 0.0),
            limits=SetpointLimits(max_velocity_m_s=1.7),
        )
    out = P.ros_local_to_px4_ned(
        RosLocalSetpoint((0, 0, 0), vec, (0, 0, 0), 0.0),
        limits=SetpointLimits(max_velocity_m_s=1.8),
    )
    assert out.velocity_m_s == pytest.approx((1.21, -1.21, 0.0))


@pytest.mark.parametrize("kwargs", [
    {"max_velocity_m_s": 0.0},
    {"max_acceleration_m_s2": -1.0},
    {"max_yaw_rate_rad_s": float("nan")},
    {"max_yaw_rate_rad_s": float("inf")},
])
def test_limits_reject_non_positive_or_non_finite_configuration(kwargs):
    with pytest.raises(ValueError):
        SetpointLimits(**kwargs)


# ---------------------------------------------------------------------------
# 6) yaw 归一化 / ±π 环绕
# ---------------------------------------------------------------------------
YAW_WRAP_TABLE = [
    pytest.param(0.0, 0.0, id="zero"),
    pytest.param(0.5, 0.5, id="inside"),
    pytest.param(math.pi, math.pi, id="upper-bound-kept"),
    pytest.param(-math.pi, math.pi, id="lower-bound-folds-to-plus-pi"),
    pytest.param(1.5 * math.pi, -0.5 * math.pi, id="three-half-pi"),
    pytest.param(-1.5 * math.pi, 0.5 * math.pi, id="minus-three-half-pi"),
    pytest.param(2.0 * math.pi, 0.0, id="two-pi"),
    pytest.param(3.0 * math.pi, math.pi, id="three-pi"),
    pytest.param(-2.0 * math.pi, 0.0, id="minus-two-pi"),
    pytest.param(5.0 * math.pi, math.pi, id="five-pi"),
    pytest.param(-5.0 * math.pi, math.pi, id="minus-five-pi"),
]


@pytest.mark.parametrize("raw,want", YAW_WRAP_TABLE)
def test_normalize_angle_pi_table(raw, want):
    assert P.normalize_angle_pi(raw) == pytest.approx(want, abs=1e-12)


@pytest.mark.parametrize("raw,want", YAW_WRAP_TABLE)
def test_conversion_normalizes_yaw_then_flips_sign(raw, want):
    """归一化发生在**转换内部**：|yaw|>π 不抛异常，而是归一化后再取负。

    例外：``raw = -π`` 归一化到 ``+π``，取负得 ``-π``，而数据类又把 ``-0.0`` /
    边界值规范回 ``+π``（区间右闭）。此时用**朝向**（cos/sin）判定，而不是比标量：
    ``+π`` 与 ``-π`` 是同一个物理朝向，比标量只会得到假失败。
    """
    sp = P.ros_local_to_px4_ned(_zero(yaw=raw))
    if want == math.pi and raw < 0.0:
        assert math.cos(sp.yaw_rad) == pytest.approx(math.cos(-want))
        assert math.sin(sp.yaw_rad) == pytest.approx(math.sin(-want))
    else:
        assert sp.yaw_rad == pytest.approx(-want, abs=1e-12)
    assert abs(sp.yaw_rad) <= math.pi


def test_wrap_across_pi_preserves_heading():
    """跨 ±π 环绕不能改变朝向：+π 与 -π 是同一个物理朝向。"""
    a = P.ros_local_to_px4_ned(_zero(yaw=math.pi))
    b = P.ros_local_to_px4_ned(_zero(yaw=-math.pi))
    assert math.cos(a.yaw_rad) == pytest.approx(math.cos(b.yaw_rad))
    assert math.sin(a.yaw_rad) == pytest.approx(math.sin(b.yaw_rad))


def test_normalize_rejects_non_finite_directly():
    with pytest.raises(FrameValidationError) as exc:
        P.normalize_angle_pi(float("nan"))
    assert exc.value.reason == "nan_inf"


def test_strict_mode_rejects_instead_of_normalizing():
    """`normalize_yaw=False`（FrameConverter.strict_yaw）时 |yaw|>π 必须报错。"""
    conv = FrameConverter(strict_yaw=True)
    with pytest.raises(FrameValidationError) as exc:
        conv.from_ros(_zero(yaw=1.5 * math.pi))
    assert exc.value.reason == "yaw_error"
    assert conv.reject_counts["yaw_error"] == 1
    assert conv.convert_count == 0


def test_converter_counts_norms_and_rejections():
    conv = FrameConverter()
    conv.from_ros(_zero(yaw=0.1))
    conv.from_ros(_zero(yaw=3.0 * math.pi))
    assert conv.convert_count == 2
    assert conv.yaw_normalize_count == 1
    with pytest.raises(FrameValidationError):
        conv.from_ros(_zero(yaw=float("nan")))
    report = conv.report()
    assert report["reject_counts"] == {"nan_inf": 1}
    assert report["convert_count"] == 2
    assert report["yaw_normalize_count"] == 1


# ---------------------------------------------------------------------------
# 7) type_mask 构造
# ---------------------------------------------------------------------------
def test_type_mask_bit_values_match_px4_header():
    """位值必须与 PX4 生成的 MAVLink 头一致（本项目只读引用）。

    来源：PX4-Autopilot ``build/px4_sitl_default/mavlink/common/common.h`` 的
    ``typedef enum POSITION_TARGET_TYPEMASK``。
    """
    assert P.POSITION_TARGET_TYPEMASK_X_IGNORE == 1
    assert P.POSITION_TARGET_TYPEMASK_Y_IGNORE == 2
    assert P.POSITION_TARGET_TYPEMASK_Z_IGNORE == 4
    assert P.POSITION_TARGET_TYPEMASK_VX_IGNORE == 8
    assert P.POSITION_TARGET_TYPEMASK_VY_IGNORE == 16
    assert P.POSITION_TARGET_TYPEMASK_VZ_IGNORE == 32
    assert P.POSITION_TARGET_TYPEMASK_AX_IGNORE == 64
    assert P.POSITION_TARGET_TYPEMASK_AY_IGNORE == 128
    assert P.POSITION_TARGET_TYPEMASK_AZ_IGNORE == 256
    assert P.POSITION_TARGET_TYPEMASK_FORCE_SET == 512
    assert P.POSITION_TARGET_TYPEMASK_YAW_IGNORE == 1024
    assert P.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE == 2048
    assert MASK_POSITION_VELOCITY_YAW == (64 | 128 | 256) == 448


def test_type_mask_position_velocity_with_yaw():
    """position+velocity+yaw：忽略加速度(64|128|256) + 忽略 yaw_rate(2048) = 2496。"""
    tm = TypeMask.for_mode("position_velocity", YawMode.YAW)
    assert tm.mask == 2496
    assert int(tm) == 2496
    assert tm.ignores_yaw is False
    assert tm.ignores_yaw_rate is True
    assert tm.uses_acceleration is False
    assert "YAW_RATE_IGNORE" in tm.describe()


def test_type_mask_position_velocity_with_yaw_rate():
    """同一模式改用 yaw_rate：yaw 位从 2048 换成 1024 → 448+1024 = 1472。"""
    tm = TypeMask.for_mode("position_velocity", YawMode.YAW_RATE)
    assert tm.mask == 1472
    assert tm.ignores_yaw is True
    assert tm.ignores_yaw_rate is False


def test_type_mask_never_ignores_both_yaw_fields():
    """任何模式/组合都必须**恰好忽略一个** yaw 字段。

    "两者都忽略"是合法位但语义为空；"两者都用"在 PX4 上行为未定义
    （接收端只按位把字段写成 NaN 或原值，见 mavlink_receiver.cpp）。
    """
    for mode in (
        "position_velocity_acceleration",
        "position_velocity",
        "position_acceleration",
        "velocity_acceleration",
        "acceleration",
    ):
        for ym in (YawMode.YAW, YawMode.YAW_RATE):
            tm = TypeMask.for_mode(mode, ym)
            assert tm.ignores_yaw != tm.ignores_yaw_rate, (mode, ym)
            assert not (tm.mask & P.POSITION_TARGET_TYPEMASK_FORCE_SET), (mode, ym)


def test_type_mask_axis_bits_for_each_mode():
    """各模式忽略的轴必须与名字一致（用命名位逐个断言）。"""
    xyz = (
        P.POSITION_TARGET_TYPEMASK_X_IGNORE
        | P.POSITION_TARGET_TYPEMASK_Y_IGNORE
        | P.POSITION_TARGET_TYPEMASK_Z_IGNORE
    )
    vxyz = (
        P.POSITION_TARGET_TYPEMASK_VX_IGNORE
        | P.POSITION_TARGET_TYPEMASK_VY_IGNORE
        | P.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    )
    axyz = MASK_POSITION_VELOCITY_YAW

    full = TypeMask.for_mode("position_velocity_acceleration", YawMode.YAW)
    assert full.mask == 2048
    assert full.uses_acceleration is True

    pv = TypeMask.for_mode("position_velocity", YawMode.YAW)
    assert pv.mask == (axyz | 2048)
    assert not (pv.mask & vxyz)

    pa = TypeMask.for_mode("position_acceleration", YawMode.YAW)
    assert pa.mask == (vxyz | 2048)
    assert not (pa.mask & axyz)

    va = TypeMask.for_mode("velocity_acceleration", YawMode.YAW)
    assert va.mask == (xyz | 2048)
    assert not (va.mask & vxyz)

    acc = TypeMask.for_mode("acceleration", YawMode.YAW)
    assert acc.mask == (xyz | vxyz | 2048)
    assert not (acc.mask & axyz)


def test_type_mask_rejects_unknown_mode_and_yaw_mode():
    with pytest.raises(ValueError):
        TypeMask.for_mode("teleport", YawMode.YAW)
    with pytest.raises(ValueError):
        TypeMask.for_mode("position_velocity", "yaw_rate_and_more")


def test_type_mask_accepts_plain_string_yaw_mode():
    """后端可能从参数服务器拿到字符串，不应因此崩掉。"""
    tm = TypeMask.for_mode("position_velocity", "yaw_rate")
    assert tm.yaw_mode is YawMode.YAW_RATE
    assert tm.mask == 1472


def test_require_yaw_input_catches_mask_payload_mismatch():
    """mask 与数据必须匹配：选 yaw_rate 却给了 0、或选 yaw 却给了非零 rate 都要拒绝。"""
    yaw_mode = TypeMask.for_mode("position_velocity", YawMode.YAW)
    rate_mode = TypeMask.for_mode("position_velocity", YawMode.YAW_RATE)

    ok_yaw = Px4LocalSetpoint((0, 0, 0), (0, 0, 0), (0, 0, 0), 0.3, 0.0)
    ok_rate = Px4LocalSetpoint((0, 0, 0), (0, 0, 0), (0, 0, 0), 0.0, -0.4)
    yaw_mode.require_yaw_input(ok_yaw)
    rate_mode.require_yaw_input(ok_rate)

    with pytest.raises(FrameValidationError) as exc:
        rate_mode.require_yaw_input(ok_yaw)
    assert exc.value.reason == "yaw_error"

    with pytest.raises(FrameValidationError):
        yaw_mode.require_yaw_input(ok_rate)

    with pytest.raises(FrameValidationError):
        yaw_mode.require_yaw_input(
            Px4LocalSetpoint((0, 0, 0), (0, 0, 0), (0, 0, 0), float("nan"), 0.0)
        )


# ---------------------------------------------------------------------------
# 8) 模块卫生：不依赖 ROS、不依赖系统时间
# ---------------------------------------------------------------------------
def test_module_does_not_import_ros_or_io():
    """SW-001 的隔离意图：模块必须保持可离线单测。"""
    src = Path(P.__file__).read_text(encoding="utf-8")
    for forbidden in (
        "import rclpy",
        "from rclpy",
        "import serial",
        "import socket",
        "import time",
        "time.monotonic",
        "time.time",
    ):
        assert forbidden not in src, f"px4_frames.py 不应出现 {forbidden!r}"


def test_setpoint_dataclasses_are_immutable_and_normalize_zero_sign():
    """frozen 数据类：改字段应失败；-0.0 规范成 0.0（日志里不出现 '-0.0'）。"""
    sp = P.ros_local_to_px4_ned(_zero(yaw=0.0, yaw_dot=0.0))
    assert str(sp.yaw_rad) == "0.0"
    assert str(sp.yaw_rate_rad_s) == "0.0"
    with pytest.raises(Exception):
        sp.yaw_rad = 1.0  # type: ignore[misc]
    assert sp.as_array().shape == (11,)
