"""运行参数的**唯一**来源与校验。

分工（不要越界）：

* ``config/contract.yaml``：只描述**接口语义**——话题、消息类型、单位、坐标系、
  无效值约定。它不提供运行阈值，也不作为 ROS 参数文件被加载。
* ``config/runtime.yaml``：本模块校验对象的模板。文件里的值与代码默认值必须
  一致（``test_runtime_config.py`` 强制），任何节点 / launch / 脚本只能从这里
  取值。
* ``config/mavlink_imu.yaml`` / ``config/px4_interface.yaml``：具体 ROS 节点的
  参数文件。它们可以覆盖阈值，但被覆盖的项必须与该节点的实际默认值一致，且由
  测试逐一比对（``test_config_single_source.py``），不允许"文档一套、代码一套"。

为什么要做这件事：此前同一批阈值散落在 contract.yaml、px4_interface.yaml、
px4_failsafe.CONTRACT_TIMING_REFERENCE 与各 launch 的字符串参数里，改一处就
可能让"闸门"和"编排"按不同数字判断。现在数值只有一个定义点，其余都是**引用**。
"""

from __future__ import annotations

from dataclasses import dataclass, fields
import math
import os
from pathlib import Path

import yaml

#: 每个高度量的参考系说明。高度是最容易被不同组件按不同基准解释的量，
#: 因此三者独立命名并在此登记参考系，禁止再用"height"这种含糊名字。
ALTITUDE_REFERENCE = {
    "takeoff_altitude_agl_m":
        "PX4 参数 MIS_TAKEOFF_ALT；相对**起飞点地面**，向上为正（即 PX4 局部 NED 的 -z）",
    "hold_lock_min_altitude_agl_m":
        "相对**本任务起飞点地面**（lifecycle 在 PRECHECK 记录的地面 z），向上为正；"
        "锁定起点设定点前的相对高度下限",
    "image_publish_min_altitude_agl_m":
        "相对**本任务起飞点地面**，向上为正；合成双目/深度开始发布的下限（TEST-ONLY 输入链）。"
        "它只是'不在地面'的护栏，**不得**被当作规划链的启动闸门：栅门一旦在飞行中重新闭合，"
        "深度断流→EGO 丢输入→自行取消目标→编排器闭锁降落，形成自伤回路（SIH 实测："
        "门槛 1.3 m 且起飞 1.5 m 时余量仅 0.2 m，深度帧 39 帧/90 s、EGO 报 153 次丢输入）。"
        "EGO 的启动时机由编排器在确认 Offboard 之后决定（见 lifecycle），不靠这个高度",
    "recovery_min_altitude_agl_m":
        "相对**本任务起飞点地面**，向上为正；自动恢复允许接管的最低相对高度",
}

#: 允许为 0 或负数的字段（场景原点）。其余 float 字段必须为正且有限。
_SIGNED_FIELDS = frozenset({"origin_x", "origin_y", "origin_z"})

#: 场景允许覆盖的字段。除原点外只放开**深度可用量程**：它是场景定义的一部分
#: （合成双目在给定基线与分辨率下的可用距离），不是可以随手在本节点里改的阈值。
_SCENE_OVERRIDE_FIELDS = _SIGNED_FIELDS | {"depth_max_range_m"}


@dataclass(frozen=True)
class RuntimeConfig:
    """一份经校验的运行配置。构造即校验，校验失败抛 ``ValueError``。"""

    # ------------------------------------------------------------ 坐标系
    world_frame: str = "global"
    camera_frame: str = "cam0_rect"
    body_frame: str = "body"

    # ------------------------------------------------------------ 话题
    depth_topic: str = "/boom_birds/depth/image"
    camera_info_topic: str = "/boom_birds/depth/camera_info"
    odom_topic: str = "/boom_birds/vio/odom_ego"
    imu_topic: str = "/boom_birds/imu"
    command_topic: str = "/boom_birds/control/command"
    planner_command_topic: str = "/boom_birds/planner/command"
    planner_request_topic: str = "/boom_birds/planner/request"
    executor_status_topic: str = "/boom_birds/planner/executor_status"
    planner_status_topic: str = "/boom_birds/planner/status"
    status_topic: str = "/boom_birds/control/execution_status"
    mission_status_topic: str = "/boom_birds/mission/status"
    control_status_topic: str = "/boom_birds/control/status"
    action_service: str = "/boom_birds/control/action"
    mission_service: str = "/boom_birds/mission"

    raw_left_frame: str = "cam0"
    raw_right_frame: str = "cam1"
    stereo_left_topic: str = "/boom_birds/stereo/left_raw"
    stereo_right_topic: str = "/boom_birds/stereo/right_raw"
    stereo_stitched_topic: str = "/boom_birds/stereo/stitched"
    stereo_status_topic: str = "/boom_birds/stereo/source_status"
    depth_xyz_topic: str = "/boom_birds/depth/xyz"
    depth_xyz_valid_topic: str = "/boom_birds/depth/xyz_valid"
    depth_compat_topic: str = "/boom_birds/depth/image_compat_uint16_mm"
    depth_preview_topic: str = "/boom_birds/depth/color_preview"
    camera_pose_topic: str = "/boom_birds/vio/camera_pose"
    odom_body_topic: str = "/boom_birds/vio/odom_body"
    odom_imu_topic: str = "/boom_birds/ov/odomimu"
    sim_map_topic: str = "/boom_birds/sitl/map"
    sim_world_cloud_topic: str = "/boom_birds/sitl/world_cloud"

    # ------------------------------------------------------------ 高度（见 ALTITUDE_REFERENCE）
    takeoff_altitude_agl_m: float = 1.5
    hold_lock_min_altitude_agl_m: float = 1.3
    image_publish_min_altitude_agl_m: float = 0.3

    # ------------------------------------------------------------ 深度量程
    #: 深度链路可用量程（米）。同一个值同时用于：
    #:   * depth_node 的 `max_depth_m`（兼容/预览与"超量程"统计）；
    #:   * EGO grid_map 的 `max_ray_length`（`invalid_depth_max_dist_ = 该值 + 0.1`，
    #:     即"至少一个落在量程内的深度像素"才让 `mapReady()` 成立）。
    #: 必须与场景一致：实测 30 m 森林最近障碍 10.39 m，用缺省 5.0 m 时
    #: **一个有效像素都没有**，地图永远不就绪（见 STATUS 第十五轮）。
    #: 注意：合成双目在 0.067 m 基线、240×180 下，10–15 m 的深度误差按 1/16 px
    #: 视差量化估算约 ±1.6 m，该量程下的占据是**粗**的，只用于 SIH 场景。
    depth_max_range_m: float = 5.0

    # ------------------------------------------------------------ 接管
    handoff_max_distance_m: float = 0.5
    #: 接管瞬间允许的**速度连续性**偏差（m/s）。位置接得上但速度跳变，
    #: 同样是"错过轨迹前段"，必须一并拒绝。
    handoff_max_speed_m_s: float = 0.3

    # ------------------------------------------------------------ 时间
    pose_timeout_s: float = 0.15
    depth_timeout_s: float = 1.0
    command_future_tolerance_s: float = 0.03
    command_timeout_s: float = 0.2
    #: setpoint **中断**判定窗口。不取命令有效期（0.2 s）：节点按 50 Hz 评估，
    #: 单帧空洞属调度抖动，按 0.2 s 判会在进入 EXECUTING 后 0.5 s 内误报断流并把
    #: 两次恢复预算耗尽（SIH 实测）。1.0 s 与 PX4 自身的 Offboard 失联超时
    #: `COM_OF_LOSS_T`（缺省 1.0 s）对齐：飞控认定失联的同一尺度上才算中断。
    planning_horizon_m: float = 3.0
    local_target_search_radius_m: float = 1.5
    local_target_search_step_m: float = 0.2
    planning_compute_budget_s: float = 0.15
    planning_max_velocity_m_s: float = 0.5
    planning_max_acceleration_m_s2: float = 0.5
    planning_obstacle_clearance_m: float = 0.8
    planning_obstacles_inflation_m: float = 0.5
    depth_pose_sync_tolerance_s: float = 0.03
    sensor_fault_confirm_s: float = 0.15
    setpoint_interrupt_timeout_s: float = 1.0
    status_timeout_s: float = 1.0
    #: 规划器状态/地图就绪的新鲜度窗口。取 1.0 s（= depth_timeout_s）而不是 0.2 s：
    #: 地图由深度融合而来，SIH 实测深度发布最大间隔 0.208 s，按 0.2 s 要求
    #: "规划器状态新鲜"必然抖动，于是 map_ready 反复翻假、被判成 planning_link
    #: 并白耗恢复预算（实测 t+26 s 触发）。窗口应与它所依赖的深度分支同一量级。
    planning_timeout_s: float = 1.0
    stable_duration_s: float = 1.0
    stable_speed_m_s: float = 0.15
    stable_radius_m: float = 0.10
    goal_tolerance_m: float = 0.3
    precheck_timeout_s: float = 10.0
    takeoff_timeout_s: float = 60.0
    hold_ready_timeout_s: float = 30.0
    #: EXECUTING 之后、规划器尚未接管时，重复请求使能的上限。
    #: EGO 的 projectRequest 在 offboard_confirmed/odom/map 未就绪时会**静默丢弃**，
    #: 单发一次会使任务永久停在 WAIT_TARGET，因此必须允许有界重发。
    planner_activate_timeout_s: float = 30.0
    #: 使能请求的重发间隔。EGO 的 projectRequest 每次都会调用 planNextWaypoint，
    #: 按控制周期（50 Hz）重发会让它的 FSM 反复重启规划（实测：EGO 连续
    #: "traj 1 success / traj 2 success" 后立刻 PROJECT_CANCEL，飞行器根本没动）。
    #: 0.5 s 足够在 30 s 窗口内重试 ~60 次，又不会把规划器打乱。
    planner_enable_retry_s: float = 0.5
    mode_timeout_s: float = 3.0

    # ------------------------------------------------------------ 失效阈值（px4_interface / px4_failsafe）
    setpoint_timeout_s: float = 0.2
    vio_timeout_s: float = 0.15
    imu_timeout_s: float = 0.5
    camera_timeout_s: float = 1.0
    #: PX4 **心跳到达**超时（PX4→Companion 方向）。PX4 MAVLink HEARTBEAT 标称 1 Hz，
    #: 因此窗口必须覆盖至少两次漏拍：取 2.5 s。
    #: 这里曾经被"统一"成 1.0 s（与 PX4 的 `COM_OF_LOSS_T` 混为一谈），结果每次心跳
    #: 抖动（实测 age 1.022 s）都判一次 signal_stale，把迟滞计数清零、闸门反复关闭、
    #: setpoint 流被饿死，最终误报成链路故障。`COM_OF_LOSS_T` 描述的是 PX4 自己丢
    #: offboard 设定点后的动作，与本窗口不是同一件事。
    heartbeat_timeout_s: float = 2.5
    #: MAV_LANDED_STATE（EXTENDED_SYS_STATE）的新鲜度窗口。PX4 侧是约 1 Hz 的
    #: 周期消息，落地状态本身很少变化；窗口取 2.5 s（覆盖至少两次漏发）。
    #: 取 1.0 s 会让 `landed_known` 反复翻假，恢复前置条件被判成 not_airborne。
    landed_state_timeout_s: float = 2.5
    link_timeout_s: float = 1.0
    recovery_required_samples: int = 5

    # ------------------------------------------------------------ 自动恢复（默认关闭）
    #: 真实设备配置默认禁用。只有显式的 SIH 任务入口才置 true。
    recovery_enabled: bool = False
    recovery_attempts: int = 2
    recovery_min_altitude_agl_m: float = 1.0
    recovery_max_speed_m_s: float = 1.0
    recovery_brake_accel_m_s2: float = 1.0
    #: 定位/IMU/深度/地图/坐标对齐必须连续有效的时长
    recovery_valid_duration_s: float = 1.0

    # ------------------------------------------------------------ PX4 参数接线
    #: 起飞高度在飞控侧的**唯一**参数名。声明配置与实际 PX4 参数必须同名同值：
    #: 该值由 `boom_birds_nav.sih_params` 生成 `param set` 命令，供 SIH 脚本使用。
    px4_takeoff_param_name: str = "MIS_TAKEOFF_ALT"

    # ------------------------------------------------------------ 场景原点
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_z: float = 0.0

    def __post_init__(self) -> None:
        for field in fields(self):
            value = getattr(self, field.name)
            name = field.name
            if name == "recovery_enabled":
                if not isinstance(value, bool):
                    raise ValueError(name)
                continue
            if field.type is str or field.type == "str":
                if not isinstance(value, str) or not value.strip():
                    raise ValueError(name)
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(name)
            if not math.isfinite(value):
                raise ValueError(name)
            if name in _SIGNED_FIELDS:
                continue
            if value <= 0:
                raise ValueError(name)
        if not (self.local_target_search_step_m <= self.local_target_search_radius_m <= self.planning_horizon_m) or self.local_target_search_radius_m / self.local_target_search_step_m > 100:
            raise ValueError("local_target_search")
        if type(self.recovery_attempts) is not int:
            raise ValueError("recovery_attempts")

    @property
    def origin(self) -> tuple:
        return (self.origin_x, self.origin_y, self.origin_z)

    def as_mapping(self) -> dict:
        """扁平映射，便于 launch 直接转成 ROS 参数。"""
        return {f.name: getattr(self, f.name) for f in fields(self)}

    def with_overrides(self, **overrides) -> "RuntimeConfig":
        """派生一份新配置（用于场景原点等），仍然经过校验。"""
        return RuntimeConfig(**{**self.as_mapping(), **overrides})

    @classmethod
    def load(cls, path) -> "RuntimeConfig":
        """从 ``runtime.yaml`` 读取。``scenes:`` 段不参与构造，由 ``load_scenes`` 处理。"""
        with open(path, encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        if not isinstance(data, dict):
            raise ValueError(f"运行配置必须是映射：{path}")
        data.pop("scenes", None)
        return cls(**data)

    @classmethod
    def load_scenes(cls, path) -> dict:
        """读取 ``runtime.yaml`` 的 ``scenes:`` 段 → {场景名: RuntimeConfig}。

        场景只能覆盖原点；别的字段写在场景里说明有人想绕过单一来源，直接报错。
        """
        with open(path, encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        scenes = data.get("scenes") or {}
        if not isinstance(scenes, dict) or "local" not in scenes:
            raise ValueError("runtime.yaml 必须定义 scenes.local（基准场景）")
        base = cls.load(path)
        result = {}
        for name, overrides in scenes.items():
            overrides = dict(overrides or {})
            allowed = _SCENE_OVERRIDE_FIELDS
            if name == "recovery_local":
                allowed |= {"takeoff_altitude_agl_m", "hold_lock_min_altitude_agl_m"}
            unknown = set(overrides) - allowed
            if unknown:
                raise ValueError(
                    f"场景 {name} 只允许覆盖原点与 depth_max_range_m，收到：{sorted(unknown)}")
            result[name] = base.with_overrides(**overrides)
        return result


#: 允许用环境变量显式指定运行配置（部署/测试用）。
CONFIG_ENV_VAR = "BOOM_BIRDS_RUNTIME_CONFIG"


def config_candidates() -> list:
    """按优先级列出 ``config/runtime.yaml`` 的可能位置。

    这里踩过一次真实的坑：安装后 Python 模块落在
    ``<prefix>/lib/python3.12/site-packages/boom_birds_nav/``，而配置被安装到
    ``<prefix>/share/boom_birds_nav/config/``。早先只按"模块旁边/上一级"找，
    安装后必然找不到，然后**静默退回**只有 ``local`` 一张场景表 —— 于是 SIH
    launch 在运行时报"缺少 forest_30m 场景"。因此：路径要多候选，找不到必须
    **明确失败**，不允许静默降级。
    """
    here = Path(__file__).resolve()
    pkg = here.parent.name          # 本模块所属包（拆包后是 boom_birds_control）
    candidates = []
    override = os.environ.get(CONFIG_ENV_VAR)
    if override:
        candidates.append(Path(override))
    # 1) 源码树布局：<pkg>/<pkg>/runtime_config.py → <pkg>/config/runtime.yaml
    candidates.append(here.parent.parent / "config" / "runtime.yaml")
    # 2) ament share（colcon/ros2 run 的正式位置）。按**本模块所属包**找：
    #    配置文件随它的所有者模块走；写死包名会在模块迁移后指向别人的 share 目录。
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(Path(get_package_share_directory(pkg)) / "config" / "runtime.yaml")
    except Exception:  # noqa: BLE001 —— 无 ament 环境时继续用其它候选
        pass
    # 3) setup.py install 布局：<prefix>/lib/python3.12/site-packages/<pkg>/ →
    #    <prefix>/share/<pkg>/config/runtime.yaml
    try:
        candidates.append(here.parents[4] / "share" / pkg / "config" / "runtime.yaml")
    except IndexError:
        pass
    # 4) 兜底：模块同级 config/（便于把包与配置一起打包）
    candidates.append(here.parent / "config" / "runtime.yaml")
    return candidates


def config_path() -> Path:
    """解析到实际存在的 ``config/runtime.yaml``；找不到就明确失败。"""
    candidates = config_candidates()
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise RuntimeError(
        "找不到运行配置 config/runtime.yaml；已查找：" +
        "；".join(str(c) for c in candidates) +
        f"。可用 {CONFIG_ENV_VAR} 显式指定。")


#: 默认运行配置（SIH 初始值，**不是**实机验收阈值）。
DEFAULTS = RuntimeConfig()


def _scenes_from_file() -> dict:
    """场景表必须来自配置文件本身；缺文件不允许静默退回单场景表。"""
    return RuntimeConfig.load_scenes(config_path())


#: 场景表。`forest_30m` 的 PX4 局部原点在 ROS global 下的位置。
SCENES = _scenes_from_file()