# EGO fork 补丁索引

本文件索引个人规划器 fork companion/ros2_ws/src/ego-planner-swarm 的项目补丁；当前代码以母仓库 gitlink 为准。日期化段落中的 SHA、行号、工作区状态与回滚清单只对应当次快照。

地图范围参数的母仓库透传见 [bringup](../companion/ros2_ws/src/boom_birds_bringup/README.md#vio-原点与地图-z-范围)；当前验证见 [STATUS](STATUS.md)。

## 2026-10-23 地图膨胀与失败取证

子模块提交：`840ea4910790acd776d0a730fa29ca946a914960`。

- plan_env 按三维索引检查膨胀边界，并重建原始占用变化影响的范围，计入范围外贡献；虚拟顶棚写入前检查 Z 层。未知标记和占用阈值不变。
- path_searching 在非有限输入、越出地图和占用起终点重合时提前拒绝，不修改碰撞半径或计算预算。
- 原生 launch 新增 failure_diagnostics，默认 false；须重建完整 EGO 依赖闭包。使用与回归开关见子模块 [Readme](../companion/ros2_ws/src/ego-planner-swarm/Readme.md#地图失败诊断)，日志分类见 [工具](../companion/ros2_ws/tools/ego_diagnostics/README.md)。
- 脱机场景区分有路、无路和无效起点；局部轨迹成功不等于目标到达或历史现场可达。当前部署与证据见 [STATUS](STATUS.md)。

## 2026-10-23 地图下界默认值

boom_birds_offline.launch.py 的 map_min_z 默认值从 -0.01 m 改为 -0.1 m，与主工程 attitude_hardware.launch.py 一致；仍执行有限值、正地图高度和上下界关系校验。地图下界不是实测地面高度，也不解除占用、碰撞或会话检查。本次只修改 Python launch 与测试，未重建 EGO C++ 或部署 Pi；验证见[10-23 记录](history/STATUS_2026-10.md#2026-10-23-工作区源码集成与脱机验证)。

## 2026-10-09 算力配置入口

`plan_manage/launch/boom_birds_offline.launch.py` 新增 `skip_pixel` 与 `use_distinctive_trajs` 参数，默认仍为 1/true。`skip_pixel < 1` 拒绝启动；项目 `mono_budget` 按 bringup 配置覆盖。碰撞检查与规划计算预算不变。固定版本由母仓库 gitlink 指定；验证见 [STATUS](STATUS.md)。

## 2026-10-03 单机化

单机化提交 `c1ffb98`，基于 `385eb2b`，移除 EGO 的多机实现：

- `plan_manage` 删除轨迹广播/接收、顺序启动和其他无人机碰撞回调；等待目标后直接进入 `GEN_NEW_TRAJ`。
- `bspline_opt` 删除机间椭球距离代价及其重启条件；障碍物代价、动力学约束、多条单机候选和完整曲线检查保留。
- `traj_utils` 删除 `MultiBsplines`、共享轨迹容器和 `Bspline.drone_id`。
- 删除 `drone_detect`、`rosmsg_tcp_bridge`、未使用的 `multi_map_server`、多机 launch 和其他无人机 RViz 显示。保留单机原生仿真工具。
- 节点固定为 `ego_planner_node`，SIH 参数记录脚本同步修改。项目 `/boom_birds/...` 话题不变。

消息定义改变，须使用新的安装前缀重新构建；不能加载旧 `traj_utils` 产物。本次证据见 [STATUS](STATUS.md)。下面的版本号与行数属于各自历史批次。

## 2026-10-01 会话与规划补丁

本轮补丁提交：`385eb2b`，基于 `d16eaeb`，分支 `boombirds-jazzy`。复验结果见 [STATUS](STATUS.md)。精确版本由母仓库 gitlink 固定。

9 月的工作区统计与回滚说明已移至[审查快照](history/EGO_PATCHES_2026-09-29.md)，不用于操作当前版本。

| 路径（相对子模块 `src/planner/`） | 当前行为与验证 |
| --- | --- |
| `plan_env/src/grid_map.cpp`、`include/plan_env/grid_map.h` | 项目 CameraInfo/深度/位姿三路同步；缺匹配内参禁止融合；几何变化清图并闭锁；有效融合才解除断流标志。地图行为回归 99 项 |
| `plan_manage/src/ego_replan_fsm.cpp` | 项目模式以实测位置/速度重建起点；确认 Offboard 才激活目标；重复使能幂等；取消先作废轨迹；规划器进程 ID 随 PlannerStatus 与 SessionBspline 传递 |
| `plan_manage/src/planner_manager.cpp`、`bspline_opt/src/bspline_optimizer.cpp`、`path_searching/src/dyn_a_star.cpp` | A*、优化与最终检查共用单调时钟计算预算，超时拒绝；时间缩放后重新检查完整轨迹速度、加速度、接管连续性与碰撞 |
| `plan_manage/include/ego_planner/trajectory_validation.h`、`plan_manage/src/planner_manager.cpp` | warm start 三个起始控制点匹配实测位置、速度和加速度，保留后续控制点；真实 UniformBspline 回归覆盖三种采样间隔与非法输入 |
| `plan_manage/src/ego_replan_fsm.cpp` | 项目周期重规划同时检查既有时间门限和一个控制点间距的推进，或旧轨迹接近尾部；碰撞回调不延迟，独立入口保留原调度 |
| `plan_manage/include/ego_planner/local_target.h` | 只在项目局部中间目标落入膨胀占据时搜索半径内的自由、向任务目标推进候选；占据的最终目标不迁移；完整轨迹碰撞检查仍执行 |
| `plan_manage/include/ego_planner/process_session.h`、`src/traj_server.cpp` | 每个规划/执行进程生成随机 ID；执行器注册进程 ID；拒绝旧任务、旧生产者、乱序及重复消息；取消屏障清缓存；世界帧严格匹配 |
| `traj_utils/msg/SessionBspline.msg` | `session_id`、`producer_session_id`、序号与内层原始 Bspline；未复制算法消息 |
| `plan_manage/test/test_trajectory_validation.cpp`、母仓库 `boom_birds_nav/test/test_session_trajectory.py` | 完整曲线边界、局部目标选择、实际 C++ 进程会话/取消/重启回归 |
| `plan_manage/launch/boom_birds_offline.launch.py` | 项目话题、世界帧、有效期、预算、动态内参及规划参数显式传入；独立静态内参模式显式互斥，不静默回退 |

项目默认 `planning_horizon_m=3.0`、局部目标搜索半径 1.5 m、步长 0.2 m、计算预算 0.15 s；独立 EGO 入口保留自身默认值。
脱机通过不等于森林目标到达。森林与恢复的本次结果、失败轨迹及限制见 [STATUS](STATUS.md)。

## 2026-09-30 占据地图超时修补

`src/planner/plan_env/src/grid_map.cpp` 的 `updateOccupancyCallback()` 在有效深度完成融合后更新 `last_occ_update_time_` 并清除 `flag_depth_odom_timeout_`。全无效深度不清除超时。`src/planner/plan_env/test/bb_grid_map_test.cpp` 的 S10 覆盖超时、全无效帧和有效帧重新融合；独立构建回归 97 pass / 0 fail。

## 2026-09-30 项目目标使能幂等

`plan_manage/src/ego_replan_fsm.cpp::projectRequest()` 对相同活动目标的重复使能不再调用 `planNextWaypoint()`。禁用请求仍作废轨迹；禁用后的使能重新规划。原独立入口不受此项目接口逻辑影响。该批次的起点试改曾撤回，后续行为见上方 10-01 补丁。

历史审查：[2026-09-29 快照](history/EGO_PATCHES_2026-09-29.md)。

## 开发工具构建开关

`BB_BUILD_EGO_TOOLS` 控制 `plan_env/obj_generator` 和 `ego_planner` 的仿真 launch 安装。开发构建默认开启；运行构建关闭，只安装主链使用的 `boom_birds_offline.launch.py`。地图库与规划接口不受该开关影响。
