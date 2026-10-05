# EGO fork 补丁索引

本文件索引个人规划器 fork `companion/ros2_ws/src/ego-planner-swarm` 相对上游的全部改动，供代码审查、
复现与**必要时**回滚使用。每条都以 `文件:行号` 或可直接粘贴的命令为出处。

## 2026-10-03 单机化

单机化提交 `c1ffb98`，基于 `385eb2b`，移除 EGO 的多机实现：

- `plan_manage` 删除轨迹广播/接收、顺序启动和其他无人机碰撞回调；等待目标后直接进入 `GEN_NEW_TRAJ`。
- `bspline_opt` 删除机间椭球距离代价及其重启条件；障碍物代价、动力学约束、多条单机候选和完整曲线检查保留。
- `traj_utils` 删除 `MultiBsplines`、共享轨迹容器和 `Bspline.drone_id`。
- 删除 `drone_detect`、`rosmsg_tcp_bridge`、未使用的 `multi_map_server`、多机 launch 和其他无人机 RViz 显示。保留单机原生仿真工具。
- 节点固定为 `ego_planner_node`，SIH 参数记录脚本同步修改。项目 `/boom_birds/...` 话题不变。

消息定义改变，须使用新的安装前缀重新构建；不能加载旧 `traj_utils` 产物。本次证据见 [STATUS](STATUS.md)。下面的版本号与行数属于各自历史批次。

## 2026-10-01 当前项目补丁

本轮补丁提交：`385eb2b`，基于 `d16eaeb`，分支 `boombirds-jazzy`。复验结果见 [STATUS](STATUS.md)。精确版本由母仓库 gitlink 固定。

下文 L0/L1/L2、行数与回滚清单是 2026-09-29 的历史快照。当前项目补丁以下表及母仓库固定的子模块版本为准；只还原历史的 10 个文件会留下不匹配的接口和启动配置。

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

上述实测起点补丁替代下文“已经撤回”的历史状态。项目默认 `planning_horizon_m=3.0`、局部目标搜索半径 1.5 m、步长 0.2 m、计算预算 0.15 s；独立 EGO 入口保留自身默认值。
脱机通过不等于森林目标到达。森林与恢复的本次结果、失败轨迹及限制见 [STATUS](STATUS.md)。

## 2026-09-30 占据地图超时修补

`src/planner/plan_env/src/grid_map.cpp` 的 `updateOccupancyCallback()` 在有效深度完成融合后更新 `last_occ_update_time_` 并清除 `flag_depth_odom_timeout_`。全无效深度不清除超时。`src/planner/plan_env/test/bb_grid_map_test.cpp` 的 S10 覆盖超时、全无效帧和有效帧重新融合；独立构建回归 97 pass / 0 fail。下文 L1 行数统计是 2026-09-29 的快照，不含本条追加行数。

## 2026-09-30 项目目标使能幂等

`plan_manage/src/ego_replan_fsm.cpp::projectRequest()` 对相同活动目标的重复使能不再调用 `planNextWaypoint()`。禁用请求仍作废轨迹；禁用后的使能重新规划。原独立入口不受此项目接口逻辑影响。当前观测替代旧轨迹预测起点的试改未通过森林 SIH，已经撤回。

## 2026-09-29 历史元信息

| 项 | 值 | 出处 |
| --- | --- | --- |
| 子模块路径 | `companion/ros2_ws/src/ego-planner-swarm` | `.gitmodules` |
| 远端 | `git@github.com:waterc07/ego-planner-swarm.git`（`origin`） | `git remote -v` |
| `.gitmodules` 声明的分支 | `ros2_version` | `.gitmodules` |
| **实际检出分支** | `boombirds-jazzy` | `git submodule status` |
| 母仓库固定的 gitlink | `e96a455da357903505c2ea7e6ea9d0b547fc8186` | `git submodule status` |
| 上游分支点（merge-base） | `origin/ros2_version` @ `a3e14dd1ec3dbcec4619ccc9049b888bbcdcee6d` | `git merge-base HEAD origin/ros2_version` |
| fork 自有提交 | `5e0e1b0`、`e96a455` | `git log --oneline origin/ros2_version..HEAD` |
| 完整 fork 差异（＝L0 已提交层，22 个路径） | 22 files changed, 2384 insertions(+), 62 deletions(-) | `git diff --stat origin/ros2_version..HEAD`（两点差分；`origin/ros2_version` 就是 merge-base `a3e14dd1`，故与 `...` 三点差分结果相同） |
| 工作区未提交改动 | 10 个已修改文件 + 1 个未跟踪文件 | `git status --porcelain` |

### 与任务描述不一致之处（以源码为准）

1. **「全部改动」不等于 `git status --porcelain`。** 任务描述把改动集合等同于未提交的 10+1；实测子模块 HEAD
   `e96a455` 里**已经提交了更早的补丁层**：`git grep -c 'BB-PATCH' HEAD` 给出
   `grid_map.h:8`、`grid_map.cpp:21`、`ego_replan_fsm.cpp:2`。完整 fork 相对上游是 22 个文件 +2384/−62。
   为避免"只改 11 个文件"的误导，本文件分三层写：**L0** 已提交层（索引）、**L1** 未提交层（任务要求逐条详解的 10+1）、
   **L2** 未跟踪文件（属于 L1 但单独列）。
2. **README 的 fork 基线声明曾与实际检出分支不一致（已修订）。** 原 `README.md:68` 只写「当前基线 ros2_version」，
   `.gitmodules` 也写 `branch = ros2_version`，但 `git submodule status` 显示实际检出 `heads/boombirds-jazzy`。
   该句已改为同时写明三件事：母仓库 gitlink 记录的 pin 是 commit `e96a455`、工作副本实际检出 `boombirds-jazzy`、
   `.gitmodules` 声明的跟踪分支仍是 `ros2_version`（因此 `git submodule update --remote` 会按后者走，不会停在
   当前工作副本分支上）。本文件不改 `.gitmodules`——改它会改变所有 clone 的 pin 语义。
3. **`git diff --stat` 的原始数字被行尾转换放大。** `ego_replan_fsm.h` 与 `traj_server.cpp` 在 HEAD 里是 CRLF、工作区是 LF，
   没有 `.gitattributes`。因此原始 `1414 insertions(+), 465 deletions(-)` 里有一部分只是行尾差异，真实语义改动是
   `987 insertions(+), 38 deletions(-)`。两个数字都列在下表；**审查与回滚时用 `--ignore-cr-at-eol` 看语义**。

### `git diff --numstat` 逐文件真实数字

| 文件 | 原始 +/- | `--ignore-cr-at-eol` +/- | 说明 |
| --- | --- | --- | --- |
| `src/planner/plan_env/include/plan_env/grid_map.h` | +60 / −0 | +60 / −0 | 纯新增 |
| `src/planner/plan_env/src/grid_map.cpp` | +342 / −8 | +342 / −8 | 纯新增（8 处删除是同步分支切换） |
| `src/planner/plan_env/test/bb_grid_map_test.cpp` | +326 / −8 | +326 / −8 | 新增 3 个场景 + 1 个工具函数 |
| `src/planner/plan_manage/CMakeLists.txt` | +3 / −2 | +3 / −2 | 依赖 |
| `src/planner/plan_manage/include/ego_planner/ego_replan_fsm.h` | +166 / −144 | **+22 / −0** | 原始数字几乎全是 CRLF→LF |
| `src/planner/plan_manage/launch/boom_birds_offline.launch.py` | +25 / −10 | +25 / −10 | 参数与内参互斥校验 |
| `src/planner/plan_manage/package.xml` | +1 / −0 | +1 / −0 | 依赖 |
| `src/planner/plan_manage/src/ego_replan_fsm.cpp` | +131 / −5 | +131 / −5 | 会话接口 + 几何闭锁 |
| `src/planner/plan_manage/src/traj_server.cpp` | +359 / −288 | **+76 / −5** | 原始数字大部分是 CRLF→LF |
| `src/planner/traj_utils/CMakeLists.txt` | +1 / −0 | +1 / −0 | 注册新 msg |
| **合计** | **+1414 / −465** | **+987 / −38** | `git diff --stat` / `git diff --stat --ignore-cr-at-eol` |

---

## L0：已提交在 HEAD 里的补丁层（索引）

**分层定义**：L0 = **已提交在 fork HEAD 的改动**，判据是 `git diff --name-only origin/ros2_version..HEAD`（两点差分）；
L1 = **工作区未提交改动**，判据是 `git status --porcelain`。两层划分的是**改动内容**，不是文件归属——同一个文件可以同时携带
L0 与 L1 改动（本轮 22 个 L0 路径与 11 个 L1 路径的交集是 8 个文件，逐一标注在下面）。

这一层由 `5e0e1b0`、`e96a455` 两个 commit 带入，**不在**工作区 diff 里。标记用 `grep -n 'BB-PATCH'` 定位，
行号按**当前工作区**文件给出（工作区已在其后追加了 L1 代码，行号会随之下移）。

| 标记 | 文件:行 | 意图 |
| --- | --- | --- |
| `[BB-PATCH-JAZZY]` | `plan_env/include/plan_env/grid_map.h:6` | ROS 2 Jazzy 起 `cv_bridge` 只提供 `cv_bridge.hpp`（Humble 及更早为 `cv_bridge.h`），用宏区分 |
| `[BB-PATCH-1]` | `grid_map.h:79`、`grid_map.h:124`、`grid_map.h:300`；`grid_map.cpp:110`、`grid_map.cpp:254`、`grid_map.cpp:601`、`grid_map.cpp:658`、`grid_map.cpp:676`、`grid_map.cpp:1209`、`grid_map.cpp:1557` | 无效深度观测上限 + 与深度图同形的无效掩码：`NaN` / 非正 / 超量程的像素视为"无信息"，**不产生障碍端点、也不沿其射线清空地图** |
| `[BB-PATCH-2]` | `grid_map.cpp:701`、`grid_map.cpp:751` | 无效观测不产生端点、不清图（上游把 `*row_ptr == 0` 当作"量程外的自由空间"沿射线清空） |
| `[BB-PATCH-3]` | `grid_map.cpp:697`、`grid_map.cpp:716`、`grid_map.cpp:747`、`grid_map.cpp:784` | 按列索引 `u` 取深度 + `proj_points_` 容量边界保护（上游按行推进 `row_ptr`，且预分配只按 640×480） |
| `[BB-PATCH-4]` | `grid_map.h:134`；`grid_map.cpp:253`、`grid_map.cpp:663`、`grid_map.cpp:1244`、`grid_map.cpp:1499` | 语义修正：把"已有有效深度观测"与"首次深度帧"分开；独立里程计只用于尚未获得有效深度观测时预置 `camera_pos_` |
| `[BB-PATCH-5]` | `grid_map.cpp:113`（交叉引用）、`grid_map.cpp:769`；`grid_map.cpp:751` 是双标记 `[BB-PATCH-2/5]`，已列在 BB-PATCH-2 行 | 经深度滤波验证有效的**超量程**观测仍按既有算法延伸到最大量程清空空间（与 `BB-PATCH-1` 的无效观测区分开） |
| `[BB-PATCH-READY]` | `grid_map.h:82`、`grid_map.h:164`、`grid_map.h:215`；`grid_map.cpp:50`、`grid_map.cpp:262`；`ego_replan_fsm.cpp:940`、`ego_replan_fsm.cpp:946` | 建图就绪门控：只有真正完成 N 次"深度→占据"融合（`ready_min_fusion_updates`，缺省 5）才允许生成轨迹 |
| `[BB-PATCH-1/2/3]`（**交叉引用**，非定义） | `boom_birds_offline.launch.py:10` | 文档说明 `use_depth_filter=false` 分支已按 PATCH-1/2/3 跳过无效观测 |

上表覆盖 `git grep -c 'BB-PATCH' HEAD` 的全部 **8（`grid_map.h`）+ 21（`grid_map.cpp`）+ 1（`boom_birds_offline.launch.py`）+ 2（`ego_replan_fsm.cpp`）= 32 行**。
注意 `-c` 输出的是**匹配行数**（`路径:行数`），不是行号；行号见上表逐条列出。

L0 的其余文件（**无 `BB-PATCH` 标记**，未逐个反推意图 — **NOT ANALYZED**），取自 `git diff --name-only origin/ros2_version..HEAD` 去掉上表已列的 3 个文件，共 19 个路径；`†` = 该文件同时带 L1 工作区改动：

| # | 路径 | 跨层 |
| --- | --- | --- |
| 1 | `.gitignore` | |
| 2 | `src/planner/bspline_opt/include/bspline_opt/bspline_optimizer.h` | |
| 3 | `src/planner/bspline_opt/src/bspline_optimizer.cpp` | |
| 4 | `src/planner/plan_env/CMakeLists.txt` | |
| 5 | `src/planner/plan_env/test/bb_grid_map_test.cpp` | † |
| 6 | `src/planner/plan_env/test/bb_map_output_assert.py` | |
| 7 | `src/planner/plan_manage/CMakeLists.txt` | † |
| 8 | `src/planner/plan_manage/include/ego_planner/ego_replan_fsm.h` | † |
| 9 | `src/planner/plan_manage/include/ego_planner/trajectory_validation.h` | |
| 10 | `src/planner/plan_manage/launch/boom_birds_offline.launch.py` | † |
| 11 | `src/planner/plan_manage/scripts/ego_map_traj_checks.py` | |
| 12 | `src/planner/plan_manage/scripts/review_traj_checks.py` | |
| 13 | `src/planner/plan_manage/scripts/run_review_integration.py` | |
| 14 | `src/planner/plan_manage/scripts/test_executor_invalidation.py` | |
| 15 | `src/planner/plan_manage/scripts/test_motion_reset.py` | |
| 16 | `src/planner/plan_manage/src/planner_manager.cpp` | |
| 17 | `src/planner/plan_manage/src/traj_server.cpp` | † |
| 18 | `src/planner/plan_manage/test/test_trajectory_validation.cpp` | |
| 19 | `src/uav_simulator/map_generator/src/random_forest_sensing.cpp` | |

**跨层口径**：L0 的 22 个路径 ∩ L1 的 11 个路径 = **8 个文件**（上表 5 个带 † 的，加标记表里已列的
`plan_env/include/plan_env/grid_map.h`、`plan_env/src/grid_map.cpp`、`plan_manage/src/ego_replan_fsm.cpp`）。
因此：**纯 L0 = 14 个文件**（L0 去掉这 8 个），**纯 L1 = 3 个文件**（`plan_manage/package.xml`、`traj_utils/CMakeLists.txt`、
`traj_utils/msg/SessionBspline.msg`），其余 8 个文件在两层各有改动——按文件看必然两层都出现，这是事实而非归类错误。

---

## L1：工作区未提交改动（10 个已修改文件）

`git -C companion/ros2_ws/src/ego-planner-swarm status --porcelain` 原始输出：

```text
 M src/planner/plan_env/include/plan_env/grid_map.h
 M src/planner/plan_env/src/grid_map.cpp
 M src/planner/plan_env/test/bb_grid_map_test.cpp
 M src/planner/plan_manage/CMakeLists.txt
 M src/planner/plan_manage/include/ego_planner/ego_replan_fsm.h
 M src/planner/plan_manage/launch/boom_birds_offline.launch.py
 M src/planner/plan_manage/package.xml
 M src/planner/plan_manage/src/ego_replan_fsm.cpp
 M src/planner/plan_manage/src/traj_server.cpp
 M src/planner/traj_utils/CMakeLists.txt
?? src/planner/traj_utils/msg/SessionBspline.msg
```

> 标记说明：这一层的代码注释里只有 `[BB-A3]` 一个显式标记（相机几何），会话接口没有自带标记。
> 为便于索引，本文档为这一层拟定两个分组标记：**`[BB-PATCH-A3]`**（对应代码里的 `[BB-A3]`）与
> **`[BB-PATCH-SESSION]`**（会话化规划接口，代码内无标记）。这两个名字**只存在于本文档**，`grep` 不到。

### [BB-PATCH-A3] 相机几何闭锁与显式重建

**意图**：给 `grid_map` 建立"内参来源显式二选一 + 三路同源校验（深度帧 / CameraInfo / 位姿）+ 几何变化即闭锁 +
只能显式重建"的路径；几何一旦变化，旧地图与旧轨迹全部作废，必须由外部重新给目标。规划侧按"几何代次"判断
是否需要重新规划，而不是继续用旧轨迹。

| 文件 | 关键行 | 内容 |
| --- | --- | --- |
| `src/planner/plan_env/include/plan_env/grid_map.h` | `:13`–`:17`（新增 `<sensor_msgs/msg/camera_info.hpp>`、`<sensor_msgs/msg/image.hpp>`、`<std_msgs/msg/empty.hpp>`、`<string>`）、`:244`（`use_camera_info_`）、`:281`（`info_sub_` + 两个三路同步器）、`:218`（`cameraGeometryFault()`）、`:222`（`cameraGeometryGeneration()`）、`:228`（`rebuildCameraGeometry()`）、`:269`（`depthInfoPoseCallback` / `depthInfoOdomCallback`）、`:332`（`geometry_reset_sub_`） | 新增公开查询、私有状态与三路同步回调 |
| `src/planner/plan_env/src/grid_map.cpp` | `:10`（`grid_map/use_camera_info`）、`:64`–`:82`（内参模式互斥校验，非法即 `throw std::invalid_argument`）、`:191`（订阅 `grid_map/geometry_reset`）、`:278`（`validateCameraInfo`）、`:334`（`sameCameraGeometry`）、`:341`（`rejectCameraGeometry`）、`:353`（`clearMapForGeometryChange`）、`:385`（`latchCameraGeometryFault`）、`:396`（`cameraInfoCallback`）、`:445`（`geometryResetCallback`）、`:477`（`rebuildCameraGeometry`）、`:516`（`imageGeometryValid`）、`:1478`（`mapReady` 增加几何门控）、`:1147`/`:1203`/`:1262`/`:1532`（四个回调入口的早退） | 完整实现 |
| `src/planner/plan_env/test/bb_grid_map_test.cpp` | `:205`（`publishCameraFrame`）、`:610`（`scenario7_camera_geometry`）、`:698`（`scenario8_geometry_latch_and_rebuild`）、`:787`（`scenario9_static_mode_is_explicit`）、`:867`–`:869`（注册进 `main`） | 三条回归场景 |

**为什么上游原版不行**

- 上游把 `mp_.fx_/fy_/cx_/cy_` 当作启动期常量：`CameraInfo` 与静态内参同时给出、或都不给出时，代码会**静默用默认值继续建图**
  （本层在 `grid_map.cpp:64`–`:82` 改成显式互斥并 `throw`）。内参错一个基线，`(u-cx)*z/fx` 反投影出的占据就整幅偏移，
  规划器会把"穿障"判成可行。
- 上游没有任何路径能识别"深度帧与内参不是同一次输出"：只要尺寸对就融合。本层在 `imageGeometryValid()`（`grid_map.cpp:516`）里
  比较时间戳（≤0.03 s）、尺寸、光学帧与几何切换时刻，任一不符即拒绝该帧并计数。
- 上游没有"几何变化 ⇒ 整体作废"的概念，也没有解除闭锁的显式入口。本层闭锁**不会被任何自动路径清除**：
  闭锁期间新来的 `CameraInfo` 只作为"候选"暂存（`grid_map.cpp:396`），只有 `grid_map/geometry_reset`（`grid_map.cpp:445`）
  或 `rebuildCameraGeometry()`（`grid_map.cpp:477`）才按新几何重建空地图并把代次 +1。
- `clearMapForGeometryChange()`（`grid_map.cpp:353`）里有一个**上游没有的陷阱**：`proj_points_` 是 `initMap` 预分配的定长缓冲，
  只能把计数归零、**不能** `clear()`；清空后 `size()==0`，重建后每帧都会被容量保护跳过，地图再也建不起来
  （回归用例 `scenario8_geometry_latch_and_rebuild` 锁定这条路径）。
- 上游的 `mapReady()` 只看融合次数；闭锁或 CameraInfo 过期时仍可能返回 `true`（本层在 `grid_map.cpp:1478` 增加三条早退）。

### [BB-PATCH-SESSION] 会话化规划接口（`project/require_session=true`）

**意图**：让 EGO 从"自由飞"变成受编排器会话授权的执行器：只有编排器开了会话、且 `ExecutionStatus` 报告
`offboard_confirmed` 时才接受目标；规划输出的样条带 `session_id` + 单调 `sequence`，由 `traj_server` 转成
带 `trajectory_id` 的高层 `ControlCommand`；轨迹作废时发 `CANCEL` 而不是静默停发。

| 文件 | 关键行 | 内容 |
| --- | --- | --- |
| `src/planner/plan_manage/include/ego_planner/ego_replan_fsm.h` | `:1`–`:4`（新增 4 个 include）、`:142`–`:146`（`require_session_` / `session_id_` / 序号 / 授权时间戳 / `offboard_confirmed_`）、`:151`–`:153`（几何闭锁与代次）、`:154`–`:157`（四个新 pub/sub）、`:158`–`:159`（`publishProjectSpline` / `projectRequest`） | 接口声明 |
| `src/planner/plan_manage/src/ego_replan_fsm.cpp` | `:10`（`project/require_session`）、`:119`–`:135`（会话模式才建 pub/sub，并订阅 `ExecutionStatus`；会话切换即清 `have_target_` 与序号）、`:180`（`publishProjectSpline`：`session_id` 为空就丢，否则包 `SessionBspline`）、`:192`（`projectRequest`：会话/序号/0.2 s 新鲜度校验收口，`enabled=false` 时发 `order=0` 失效样条 + 转 `WAIT_TARGET`，`offboard_confirmed` / `have_odom_` / `mapReady()` 任一不满足就**直接 return**）、`:265`（会话模式下目标变更直接进 `GEN_NEW_TRAJ`/`REPLAN_TRAJ`）、`:525`–`:535`（几何闭锁优先于一切规划动作）、`:542`–`:561`（几何代次变化即作废轨迹并转 `WAIT_TARGET`）、`:564`（每周期 0.2 s 授权窗口门控）、`:818`（`checkCollisionCallback` 里发布 `PlannerStatus`）、`:935`/`:989`/`:1087`（三处发样条改走 `publishProjectSpline`） | 实现 |
| `src/planner/plan_manage/src/traj_server.cpp` | `:20`–`:25`（会话状态与 `/boom_birds/planner/command` 发布者）、`:93`（`sessionSplineCallback`：只接受"会话一致 ∧ `offboard_confirmed` ∧ 0.2 s 内"的样条，会话切换即清 `traj_`；`receive_traj_` 为假时发 `CANCEL`）、`:213`（`cmdCallback` 每周期复核授权，失配即清空轨迹并停发）、`:287`–`:298`（把 `PositionCommand` 包成 `ControlCommand`，`valid_for=200 ms`，`trajectory_id` 取 `project_trajectory_id_`）、`:307`–`:315`（会话模式订阅 `planning/session_bspline`，否则订阅上游 `planning/bspline`）、`:317`–`:326`（授权来源：`/boom_birds/control/execution_status`） | 实现 |
| `src/planner/traj_utils/CMakeLists.txt` | `:28` | 把 `msg/SessionBspline.msg` 注册进 `rosidl_generate_interfaces` |
| `src/planner/plan_manage/package.xml` | `:33` | `<depend>boom_birds_interfaces</depend>` |
| `src/planner/plan_manage/CMakeLists.txt` | `:17`、`:62`、`:77` | `find_package(boom_birds_interfaces REQUIRED)`，并把它加进 `ego_planner_node` 与 `traj_server` 的 `ament_target_dependencies` |

**为什么上游原版不行**

- 上游 `traj_server` 只订阅 `planning/bspline`（`traj_server.cpp:314` 的 else 分支），样条里**没有会话、没有序号**：
  规划器一重启或换目标，执行侧无法区分"新轨迹"与"旧轨迹的续播"。本层用 `SessionBspline`（`session_id` + `sequence`）
  加 `project_trajectory_id_` 建立可判序的轨迹身份。
- 上游 `traj_server` 的 `cmdCallback` 只要 `receive_traj_` 为真就持续发 `/position_cmd`，**不看飞控是否还在 Offboard**。
  本层在 `traj_server.cpp:213` 每周期复核 `offboard_confirmed` 与 0.2 s 授权窗口，失配立刻停发。
- 上游没有"轨迹作废"这个信号：规划失败时 `traj_server` 只是不再收到新样条，`px4_interface` 侧只能靠超时推断。
  本层显式发 `CANCEL`（`traj_server.cpp:107`–`:115`）以及 `order=0` 的空样条，让下游走
  `TRAJECTORY_INVALIDATED` 的闭锁语义而不是"等超时"。
- 上游的目标来源只有 `move_base_simple/goal` / 预设航点（`ego_replan_fsm.cpp:138` 的 else 分支），没有"编排器授权"概念。
  本层 `projectRequest`（`ego_replan_fsm.cpp:192`）把 `offboard_confirmed`、odom、`mapReady()` 三项作为**静默丢弃**的门槛，
  这正是编排器需要 `planner_enable_retry_s` 限频重发的原因。
- 本层还顺带把两处几何语义接到规划侧：`execFSMCallback`（`:525`）里几何闭锁优先于一切规划动作（exec 定时器 10 ms 比
  safety 定时器 50 ms 快，只靠 `checkCollisionCallback` 拦截会漏），以及代次变化必须显式重新规划（`:542`）。
  上游两者都没有。
- 构建层面：上游 `plan_manage` 不认识 `boom_birds_interfaces`，也不生成 `SessionBspline` 消息类型，因此必须同时改
  两个 `CMakeLists.txt` 与 `package.xml`；少任何一个都会在编译期失败。

### [BB-PATCH-CAMINFO-LAUNCH] 内参来源与 `require_session` 的 launch 接线

| 文件 | 关键行 | 内容 |
| --- | --- | --- |
| `src/planner/plan_manage/launch/boom_birds_offline.launch.py` | `:17`–`:18`（文档：两种内参模式）、`:24`、`:49`、`:51`（新增 `require_session` / `use_camera_info` 参数）、`:79`（`grid_map/camera_info` → `/boom_birds/depth/camera_info`）、`:93`–`:94`（`project/require_session`、`grid_map/use_camera_info`）、`:176`（`traj_server` 也吃 `project/require_session`）、`:179`–`:190`（`select_geometry`：动态内参与静态内参**互斥**，静态模式必须显式给 `fx/fy/cx/cy`，用 `OpaqueFunction` 延迟到参数解析后再校验） | 接线与互斥校验 |

**为什么上游原版不行**

- 上游 launch 把标定内参当**默认字面量**写进 `DeclareLaunchArgument`（本层改成 `-1.0` 哨兵 + 显式校验）：
  一旦深度节点的标定/输出尺寸变了，launch 里的静态内参不会跟着变，而 `grid_map` 会照旧用它反投影。
  本层 `use_camera_info=true` 时按图像时间、尺寸、光学帧核对 `CameraInfo`，并**禁止**静态内参覆盖。
- 上游 launch 没有 `project/require_session`，因此 `traj_server` / `ego_planner_node` 会以"无会话"模式启动；
  本层把它作为 launch 参数传进两个节点（`:93`、`:176`），否则会话接口形同虚设。
- `OpaqueFunction` 是必需的：`DeclareLaunchArgument` 的默认值在声明期无法交叉校验"动态模式却给了静态内参"，
  只在 `generate_launch_description()` 里直接读 `LaunchConfiguration` 会拿到未解析的替换串。

---

## L2：未跟踪文件

| 文件 | 内容 | 状态 |
| --- | --- | --- |
| `src/planner/traj_utils/msg/SessionBspline.msg` | `std_msgs/Header header` / `string session_id` / `uint64 sequence` / `traj_utils/Bspline trajectory`（4 行） | 未跟踪；**git 不保护它**，`rm` 或 `git clean` 后无法找回 |

`SessionBspline` 是 `[BB-PATCH-SESSION]` 的载体消息，缺少它时 `traj_utils` 不会生成 `session_bspline.hpp`，
`ego_replan_fsm.h:4` 的 include 与 `traj_server.cpp` 的订阅都无法编译。

---

## 回滚步骤（可执行，**本文档不执行**）

> ⚠️ **不要执行本节命令。** 文档交付不改变工作区；回滚是破坏性操作，且会断开当前 SIH 链路（见下方耦合警告）。

```bash
cd /home/waterc/workspace/Boom_Birds

# 0) 先留档：未跟踪文件 git 不保护，删了就找不回
mkdir -p /tmp/bb_ego_backup
cp companion/ros2_ws/src/ego-planner-swarm/src/planner/traj_utils/msg/SessionBspline.msg /tmp/bb_ego_backup/
git -C companion/ros2_ws/src/ego-planner-swarm diff --numstat > /tmp/bb_ego_backup/numstat_before_rollback.txt

# 1) 还原 10 个已跟踪文件（会同时把 ego_replan_fsm.h / traj_server.cpp 的行尾还原为 HEAD 的 CRLF）
git -C companion/ros2_ws/src/ego-planner-swarm checkout -- \
  src/planner/plan_env/include/plan_env/grid_map.h \
  src/planner/plan_env/src/grid_map.cpp \
  src/planner/plan_env/test/bb_grid_map_test.cpp \
  src/planner/plan_manage/CMakeLists.txt \
  src/planner/plan_manage/include/ego_planner/ego_replan_fsm.h \
  src/planner/plan_manage/launch/boom_birds_offline.launch.py \
  src/planner/plan_manage/package.xml \
  src/planner/plan_manage/src/ego_replan_fsm.cpp \
  src/planner/plan_manage/src/traj_server.cpp \
  src/planner/traj_utils/CMakeLists.txt

# 2) 删除未跟踪的 msg（换 msg 清单必须重建，见第 4 步）
rm companion/ros2_ws/src/ego-planner-swarm/src/planner/traj_utils/msg/SessionBspline.msg

# 3) 校验：应当没有任何输出
git -C companion/ros2_ws/src/ego-planner-swarm status --porcelain

# 4) 重新构建。改了 msg 定义、CMakeLists 与 package.xml 就必须重建相关包，
#    只重编叶子包会用到 install 里过期的 .hpp / 消息类型
bash companion/ros2_ws/tools/build_all.sh --packages-select traj_utils plan_env plan_manage
```

### 回滚不是局部操作（母仓库侧耦合，回滚前必须一起决定）

| 耦合点 | 说明 |
| --- | --- |
| `boom_birds_nav/launch/px4_sih_mission.launch.py:46` | 向 EGO launch 传 `use_camera_info` / `require_session`；回滚后 EGO launch 不再声明这两个参数，`IncludeLaunchDescription` 会因未知 launch 参数失败 |
| `px4_sih_mission.launch.py:33` | 固定 `require_session=true`；回滚后 EGO 不再订阅 `/boom_birds/planner/request`、也不发布 `/boom_birds/planner/command` 与 `/boom_birds/planner/status` |
| `boom_birds_control/px4_interface_node.py:470`–`:477` | `require_session=true` 时**只**订阅 `/boom_birds/control/command`、**不再**订阅 `/position_cmd`；回滚 EGO 的会话接口会让这条链路彻底断开 |
| `boom_birds_bringup/boom_birds_bringup/lifecycle_node.py:69`–`:74` | 编排器只发 `/boom_birds/planner/request`、只收 `/boom_birds/planner/command`；回滚后编排器拿不到任何轨迹 |
| `boom_birds_offline.launch.py:79`、`:94` | `grid_map/camera_info` 重映射与 `grid_map/use_camera_info` 由 L1 提供 |

因此：**要回到上游行为，必须连同母仓库的 SIH 入口一起回退；只回滚子模块会得到一个起不来或永远接管不了的链路。**
本任务不做这件事，也不改任何母仓库源码。

### 回滚后的验证要求（NOT RUN）

- `git -C companion/ros2_ws/src/ego-planner-swarm status --porcelain` 为空（唯一客观判据）。
- 重建后重跑 SIH 矩阵与脱机链路的既有回归。**本文档未执行任何构建、未跑 SIH、未连接真机。**
