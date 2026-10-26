# EGO fork 补丁索引

本文件索引个人规划器 fork companion/ros2_ws/src/ego-planner-swarm 的项目补丁；当前代码以母仓库 gitlink 为准。日期化段落中的 SHA、行号、工作区状态与回滚清单只对应当次快照。

地图范围参数的母仓库透传见 [bringup](../companion/ros2_ws/src/boom_birds_bringup/README.md#vio-原点与地图-z-范围)；当前验证见 [STATUS](STATUS.md)。

本轮固定提交：`937b3e39e9971ed8405c73d002ab458a25157978`，包含下列 10-23 至 10-25 修复。源码提交不改变正式 Pi 版本或导航 PARTIAL 验收。

## 2026-10-25 路径与控制点平面求交

三处路径遍历改用 `findPathPlaneIntersection`：检查相邻路径段的平面跨越、有限性和边界，无交点不修改输出；每个控制点单独判断成功，只在约束已追加后更新该段种子。旧循环的无交点路径已用 ASan 复现堆越界。WSL 几何回归 215 项通过 ASan/UBSan，规划/轨迹 9 项通过；不改变碰撞、膨胀或动力学门限。测试开关 `BB_BUILD_GEOMETRY_TESTS` 默认 OFF，不安装测试程序。ARM 整套运行候选已编译此修复，新增几何测试尚未在 ARM 执行；见[组合验证](history/STATUS_2026-10.md#2026-10-25-整套候选审计与姿态全链仿真)。

## 2026-10-24 地图射线

RayCaster 改用连续方向，按正负方向计算首次越格时间，某轴到终点格后停止该轴步进；零轴使用 infinity。GridMap 的射线输入/输出与 map_origin 对齐，缓存占用更新拒绝非有限或越界坐标。183 项占用/A*/射线回归在 WSL 与独立 ARM 候选通过；WSL 地图 104 项、ARM 基线地图 99 项及两端规划/轨迹 9 项通过。真实数据重建的两轮起点仍 UNKNOWN；正式 scene-06 未切换，见[射线修复](history/STATUS_2026-10.md#2026-10-24-地图射线修复与-scene-06-重建对照)。

## 2026-10-24 A* 负方向索引

`Coord2Index` 改用 floor(offset + 0.5)，避免负偏移被向零截断而偏一格；在整数转换前拒绝非有限/越出搜索池坐标，搜索入口拒绝非正或非有限步长。占用、膨胀和碰撞判据不变。WSL 与独立 Pi ARM 候选均通过占用 177 项、规划/轨迹 9 项回归；正式 scene-06 未切换，现场拒绝原因仍须重放证明。新射线几何探针尚未接入生产实现，见[脱机修复](history/STATUS_2026-10.md#2026-10-24-导航脱机修复与射线几何复核)。

## 2026-10-24 正式版本选择

Pi 已按用户指定将 scene-06 固化为正式版本，复用已测试 ARM 安装树；scene-07 诊断未部署。正式入口仅运行感知与独立规划观察，导航仍 PARTIAL；版本、核验与回退见 [STATUS](STATUS.md)。

## 2026-10-24 同周期起点取证

`failure_diagnostics` 仍默认 false；开启后增加 body、规划边界起点、局部目标与前三初始控制点查询，以 `planning_cycle` 关联原 A* 占用日志。每点保留 raw/unknown/inflated 与融合帧时间，不修改网格或碰撞判据。WSL 隔离构建与回归、Pi 未部署状态见 [STATUS](STATUS.md)。

## 2026-10-23 非零起始状态的动力学修正

在 840ea49 基础上修复。延长 B 样条时间后重新约束起始位置、速度、加速度与终点速度，随后检查导数范数上界与整段扫掠碰撞；重约束改变端部几何，不复用旧碰撞结果。保持三次修复上限与原动力学容差，不将微小 VIO 速度置零。新增现场边界、非零初始加速度、超限初速度回归和深度重建测试；操作见子模块 Readme，证据与未知空间限制见[新场景记录](history/STATUS_2026-10.md#2026-10-23-新场景实机采集与动力学脱机修正)。10-23 已在 Pi 独立 evidence 前缀原生构建并用于不接控制的规划观察；正式 current 未切换，结果及中止原因见[ARM 复核](history/STATUS_2026-10.md#2026-10-23-ego-arm-原生构建与实机规划复核)。同轮停止原因后续归到采集观察器被 SD 写回阻塞（非规划器）；改用 RAM 采集后 scene-05 在放回静止再生成目标，同一原生库 1/1 候选接受、B 样条终点距目标 0.246 m，随后 FSM 转回 WAIT_TARGET，未观测到持续重规划。离线 launch 当前不发布 `/boom_birds/planner/status`（读取该话题的采集校验需显式 allowlist）；见[scene-05 记录](history/STATUS_2026-10.md#2026-10-23-实机静止目标与单次轨迹接受scene-05-ram-采集)。离线 launch 固定 `fsm/thresh_no_replan_meter=1.0`、`fsm/thresh_replan_time=1.0`、`fsm/waypoint_num=1`：`require_session=false` 时只有 `t_cur>1 s` 且距终点 >1 m 才进入 REPLAN_TRAJ，因此 1 m 内目标不会触发周期重规划；这只是重规划调度启发式，不改变碰撞/动力学检查。scene-06 用 1.5–2.0 m 目标触发后，接受轨迹仍会被起点附近体素碰撞作废并进入 EMERGENCY_STOP，见[scene-06 记录](history/STATUS_2026-10.md#2026-10-24-scene-06-周期重规划触发与失效停发观察)。

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
