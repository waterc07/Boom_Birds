# Boom Birds

RoboMaster 2027 微型空间智能无人机预研。导航选型已确定；视觉定位、避障、返库的集成与飞行尚未验收。

当前进度见 [STATUS](docs/STATUS.md)，操作命令见各模块 README；协作规则见 [AGENTS.md](AGENTS.md)。

## 开发与运行入口

唯一主开发根：WSL Ubuntu-24.04 `/home/waterc/workspace/Boom_Birds`。Git、编辑和构建使用 WSL Linux 工具；在根目录运行 `code .` 并确认编辑器处于 WSL。

```bash
git clone --recurse-submodules https://github.com/waterc07/Boom_Birds.git
# 已有 clone：在项目根执行
git submodule update --init --recursive
```

- [ROS 2 工作空间](companion/ros2_ws/README.md)：开发环境、子模块、Git 与部署约定。
- [双目深度程序](companion/ros2_ws/src/stereo_depth/README.md)：安装与运行；[相机标定](companion/ros2_ws/src/stereo_depth/LIVE_CALIBRATION.md)。
- [PX4](px4/README.md)：独立源码与 FC-001 实板验收步骤。

深度程序保留独立 Python CLI，也已封装为 ROS 2 算法包；WSL 合成链路与分层启动方式见 ROS 工作空间说明。OpenVINS 真数据初始化和硬件链路尚未验收，详见 STATUS。

## 系统架构

```text
USB 同帧双目 → 统一采集 / 左右拆分 / 时间戳 / 标定
                ├─ 左右图像 + 飞控 IMU → OpenVINS → 位姿 / 速度
                └─ 校正 / 双目匹配 → 米制深度 / XYZ
                                      ↓ 与对应时刻位姿结合
任务目标 → EGO-Planner ← 局部地图 + 里程计
                ↓
          轨迹执行 → 控制接口 → Px4Interface → MAVROS → PX4 → ESC
                                  ↑            ↓
                                  └── IMU / 状态
MTF-02P 独立光流 / 测距 ───────────────────→ PX4
```

图示为目标架构；已实现范围见 STATUS。Ubuntu 24.04 / ROS 2 Jazzy 为开发基线；Pi 5 用于验证，RK3576 是后续迁移方向，具体板卡与最终机载适用性待验证。

- VIO 使用图像与飞控加速度/角速度，不能用稠密深度或飞控融合姿态代替输入。IMU 来源已确定为飞控；其消息接口、速率、时间映射、相机—IMU 外参与时间偏移待验证。
- 新实机验证路线选择 Companion 位置/速度闭环 → MAVROS 姿态＋推力 → PX4 姿态/角速度闭环；旧 PX4 位置 setpoint 路线保留。新路线以 VIO 世界系定位，解锁前冻结与 PX4 姿态参考的水平旋转，不要求两套位置原点一致，也不回传外部视觉。物理端口、真实标定、机体参数与 PX4 独立安全接管仍待实机验收。见 [接入与验证入口](companion/ros2_ws/src/boom_birds_bringup/README.md)。
- 算法通过 `Px4Interface` 获取状态和发送 setpoint，不直接依赖串口；Companion 不输出 PWM/DShot。VIO 与安全光流/测距处于不同故障域。
- Companion 超时后由 PX4 执行经验证的安全动作；定位失效时不能默认仍可悬停，不能持续盲冲。
- 软件职责包括采集/记录、估计、深度、建图、规划、轨迹执行和控制适配；下视 AprilTag/平台末段速度降落已有脱机与 SIH 实现，真实速度确认适配器未验证。前视目标检测/跟踪、能源管理与真实设备任务验收待完成；未实现的职责不创建空模块。
- 后续前视任务为检测/跟踪/视觉伺服；下视相机用于软件光流与降落 Tag，配合下视距离传感器辅助返库。多区 ToF 为补充候选，不替代主建图链路。

## 工程基线

`LOCKED` 为已锁定项，变更需新证据或约束冲突并追加决策；`CURRENT BASELINE` 可依据 CAD、手册或实测修订；`CANDIDATE` 未完成选型验证；`TBD` 未定义；`HISTORICAL` 不作为当前要求。

| 子系统 | 状态 | 当前基线 |
| --- | --- | --- |
| 外包络 | CURRENT BASELINE | 118–120 mm；必须通过 CAD 闭合验证，并按最大包络边保守校核当前规则 |
| 整机质量 | CURRENT BASELINE | 仅按当前规则限制：`150 g ≤ m ≤ 322.5-1.225D`；118 mm时上限约177.95 g，120 mm时上限约175.5 g，不另设165 g硬上限 |
| 构型 | CURRENT BASELINE | 同平面、固定前向任务方向、全包围桨保四旋翼；完整涵道非强制；桨叶和机体不得外露，原开放式保护框架须重新审查任意角度碰撞保护 |
| 电机 | LOCKED | 4 × T-MOTOR 1103 8000KV |
| 桨叶 | LOCKED | 4 × Gemfan 2216S-3，2.2 inch 三叶 |
| 开发能源 | LOCKED | 3S 300 mAh、95C、23 g、约 19×18×50 mm；化学体系及接口待确认 |
| 飞控/AIO | LOCKED | 已购入 MicoAir743v2-AIO-45A，运行 PX4 |
| ESC 协议 | CURRENT BASELINE | DShot600 |
| 安全定位 | LOCKED | 已购入 MTF-02P 光流测距一体传感器，直接输入 PX4，不依赖 Companion |
| 独立光流/测距 | LOCKED | MTF-02P；光流和距离在软件中仍作为两类观测管理 |
| Companion | CANDIDATE | Pi 5 用于当前验证，后续迁移 RK3576；具体板卡与最终机载适用性待验证 |
| 当前双目深度输入 | CURRENT BASELINE | USB 免驱、硬件同帧左右拼接；具体模式和标定见双目模块 README，型号、安装方向、快门与距离精度待验证 |
| 前视相机 | CANDIDATE | 彩色全局快门，约 0.5–1 MP+、60–120+ FPS、MIPI CSI |
| 下视相机 | CANDIDATE | 单色全局快门，承担软件光流和降落 Tag；辅助定位待验证，当前主定位改为双目 + 飞控 IMU |
| 主定位 | CURRENT BASELINE | OpenVINS；输入当前双目图像与飞控 IMU，输出位置、姿态、速度；尚未集成验收 |
| 路径规划与避障 | CURRENT BASELINE | 自算双目深度 + 里程计建图，使用个人 fork https://github.com/waterc07/ego-planner-swarm；精确版本由母仓库 gitlink 固定，维护分支为 `boombirds-jazzy`；子模块更新约定见 [ROS 工作空间](companion/ros2_ws/README.md#子模块)；补丁索引见 [EGO fork 补丁索引](docs/EGO_FORK_PATCHES.md)；Jazzy/x86_64 与 Pi 5/aarch64 已构建；设备验证范围见 STATUS |
| 补充避障传感器 | CANDIDATE | 8×8 multi-zone ToF 类传感器，不替代双目建图与规划主线 |
| 最终能源 | TBD | 高概率超级电容 + 独立电容管理模块 |
| 撞击/拦截结构 | CANDIDATE | 必须覆盖直接撞击与主动迎击能力方向；具体判定、载荷路径和实现待细则与实测 |

45A 是 AIO 工程冗余，不代表单电机设计电流。上述为设计基线，采购与测量证据见 [BOM](hardware/bom/README.md)，详细验收项保留在 [需求表](docs/REQUIREMENTS.md)。

比赛规则以 [RULE_BASELINE](docs/RULE_BASELINE.md) 为唯一合规入口：2026-09-09 书面前瞻优先，未述项沿用旧规范，均未定义者 TBD；实测不能放宽规则。人工飞行属于研发验证，比赛要求全自动。规则分析与未采纳设计见 [2026-09-09 评审](docs/RULE_REVIEW_2026-09-09.md)。

动力研究以约 160 g 为参考点而非质量限制，目标推重比至少 2.5、理想 2.5–3.0，总静推力约 400–480 gf、单电机 100–120 gf，须由当前电机/桨/ESC 实测确认。先覆盖现有电池的实测电压范围；推力台记录电压、命令、电流、功率、推力、RPM 和电机/ESC 温度，形成 `T=f(V,u)`、`P=f(V,T)`、`T/P`。

电池化学体系、满充电压与连接器仍未确认，不擅自写成 LiPo/LiHV/XT30。后续 Energy Module 可替换，逻辑电源与动力母线受控；不默认裸 EDLC 直连 ESC。电容稳压、限流、均衡、保护、遥测和总线策略均 TBD，可研究 `remaining_energy_J`，不阻塞当前电池验证。

## 源码与资料位置

源码与当前文档在 WSL 维护；原始资料和交付文件存于 Windows。

| 内容 | 归属与维护方式 |
| --- | --- |
| 源码、Git、子模块、当前项目文档 | WSL 主工程；Windows 不保留另一份可编辑副本 |
| Python/ROS 环境、构建缓存 | WSL Linux 文件系统；不跨系统复制环境或编译产物 |
| 默认标定与必要回归样例 | 运行所需文件随 WSL 代码版本保存；测试日志与过程记录留在本机 |
| 原始视频、照片、ULog、大型采集数据 | Windows `data/`，原件保留；WSL 按需读取 |
| 手册、规则原件与采购资料 | Windows `references/`，作为来源而非实时状态 |
| 正式报告与导出产物 | Windows `reports/`、`artifacts/`，注明日期、代码版本及输入来源 |
| 旧方案与过期交付物 | Windows `archive/`，不参与日常维护 |
| 凭据与备份 | 私有目录，不进入 Git；备份独立于日常资料管理 |

资料流向：Windows 原件 → WSL 开发/分析 → 保存证据或交付物 → 更新 STATUS 或模块说明。少量读取可直接访问 `/mnt/d/...`；大量反复处理可按需复制到 WSL 仓库外的临时工作区，记录来源，缓存不作为唯一原件。

Windows 可通过 VS Code WSL 模式或 `\\wsl.localhost\Ubuntu-24.04\home\waterc\workspace\Boom_Birds` 查看/编辑主工程。树莓派目录属于部署现场，更新前比较差异，现场修改取回 WSL。Git 和同机资料目录不能替代独立备份；现有迁移备份保留，清理策略另行确定。

| 路径 | 职责 |
| --- | --- |
| `companion/ros2_ws/src/boom_birds_interfaces/` | 项目消息、生命周期服务、说明性契约 |
| `companion/ros2_ws/src/boom_birds_sensing/` | 采集、时间同步、深度发布、位姿适配 |
| `companion/ros2_ws/src/boom_birds_control/` | PX4 后端、坐标转换、执行许可、校验后的运行配置 |
| `companion/ros2_ws/src/boom_birds_bringup/` | 生命周期编排、生产集成启动 |
| `companion/ros2_ws/src/boom_birds_sim/` | 合成输入、SIH 真值、场景与仿真启动 |
| `companion/ros2_ws/src/boom_birds_nav/` | Python 模块与 launch 的兼容转发；无第二份实现 |
| `companion/ros2_ws/src/stereo_depth/` | 自研双目深度程序、ROS 2 算法包和默认标定 |
| `companion/ros2_ws/src/open_vins/` | 个人 OpenVINS fork 子模块 |
| `companion/ros2_ws/src/ego-planner-swarm/` | 单机 EGO 规划器 fork 子模块（目录名保留上游名称） |
| `/home/waterc/bb_build/main/{build,install,log}` | WSL 构建产物；仓库外保存，不跨平台复制 |
| `docs/`、`hardware/` | 当前文档、规则、需求、BOM 与硬件证据 |
| `docs/tasks/`、`docs/workflows/`、`px4/manifests/` | 本机历史验证记录与迁移清单，不再纳入新提交 |
| `tools/` | 历史 ULog 分析脚本；旧数据路径待适配 |
| `/home/waterc/PX4-Autopilot` | 独立 PX4 仓库与已有构建目录 |
| `/home/waterc/mavlink` | 独立 MAVLink 仓库，不等同于 PX4 自带依赖 |
| `gmaster@192.168.137.200:/home/gmaster/boombirds/current` | Pi 5 Companion 部署入口；连接前核验地址，版本与检查结果见 STATUS |
| `/home/gmaster/boom_birds_ws/stereo_depth` | Pi 5 旧源码、标定与测量数据，保留原位 |

Windows 根：`D:\Users\Admin\Desktop\G-Master\Boom_Birds`；WSL 对应 `/mnt/d/Users/Admin/Desktop/G-Master/Boom_Birds`。该目录不是 WSL 仓库的父目录。

| 相对资料根的目录 | 内容 |
| --- | --- |
| `data/px4_logs/` | 原始 ULog |
| `data/stereo_depth/windows_snapshot_20260922/` | 原始 captures/depth_outputs |
| `references/manuals/`、`references/rules/` | 厂商手册、比赛规则原件 |
| `reports/flight_analysis/`、`artifacts/calibration/` | 历史分析与棋盘格等产物 |
| `archive/` | 历史临时资料 |
| `.local/` | 凭据、备份与清单，不输出或提交 |

旧 Windows 代码副本已移入回收站，完整备份位于资料根 `.local/backups/windows-retirement-20260922/`。原始证据不因文档清理而删除。

Markdown 相对链接按所在文件解析。日期化历史记录中的旧相对路径按当时目录解释；当前外部数据路径统一使用上述资料根，不沿用 `../data` 等假设。

## 任务生命周期

位置模式的状态判定在 `companion/ros2_ws/src/boom_birds_bringup/boom_birds_bringup/lifecycle.py`：它只编排，输入一份
`Observation`、输出一组动作名，无 ROS、无 I/O、无系统时钟（`lifecycle.py`）。动作由
`.../boom_birds_bringup/lifecycle_node.py` 执行（`lifecycle_node.py`）。`companion/ros2_ws/src/boom_birds_nav/boom_birds_nav/lifecycle.py` 等文件是
**兼容转发层**，不是第二份实现（`boom_birds_nav/boom_birds_nav/lifecycle.py`）。

以下迁移与恢复窗口描述 `px4_position`。`companion_attitude` 使用 `attitude_lifecycle.py`：地面预发 HOLD、确认 Offboard 后请求解锁，起降由 VIO 闭环完成，自动恢复关闭；接入与预算见 [bringup README](companion/ros2_ws/src/boom_birds_bringup/README.md)。可选平台收尾的速度交接见 [control README](companion/ros2_ws/src/boom_birds_control/README.md)。

### 状态集合

| 状态 | 定义 |
| --- | --- |
| `IDLE` / `PRECHECK` / `TAKEOFF` / `HOLD_READY` | 起飞前到锁点（`lifecycle.py`） |
| `OFFBOARD_PENDING` / `EXECUTING` | 等 Offboard 模式回读确认、执行规划轨迹（`lifecycle.py`） |
| `RECOVERING` | 有界自动恢复；缺省**不可达**（`lifecycle.py`） |
| `LANDING` / `COMPLETE` | 降落与完成（`lifecycle.py`） |
| `FAULT_LATCHED` | 闭锁恢复资格；允许人工请求降落收尾，不自动解锁（`lifecycle.py`） |

### 迁移条件与驱动者

| 迁移 | 条件（源码） | 驱动者 |
| --- | --- | --- |
| `IDLE`/`COMPLETE` → `PRECHECK` | 会话非空、goal 三分量有限；由 `Mission.START` 先请求 `VehicleAction.OPEN_SESSION`，服务返回的 `session_id` 交给 `fsm.start()` | `lifecycle_node.py`（START）、`lifecycle_node.py`（应用结果）、`lifecycle.py` |
| `PRECHECK` → `TAKEOFF` | 连接 + `status_age ≤ status_timeout_s` + 未解锁 + `landed == 1` + `pose_age ≤ pose_timeout_s` + 对齐有效 + 位置有限；记录 `ground_z` 与 `boot_epoch`，输出 `arm` | `lifecycle.py` |
| `PRECHECK` 超时 → 闭锁 | `precheck_timeout_s`（缺省 10 s） | `lifecycle.py` |
| `TAKEOFF` → `HOLD_READY` | 解锁确认后发一次 `takeoff`；`landed == 2` 且 `StableWindow` 成立 → 锁点 = 当前观测位置 | `lifecycle.py`、`lifecycle.py` |
| `TAKEOFF` 闭锁 | 未解锁超过 `mode_timeout_s`；或超过 `takeoff_timeout_s` | `lifecycle.py`、`lifecycle.py` |
| `HOLD_READY` → `OFFBOARD_PENDING` | `sending ∧ sensors_ready ∧ map_ready` 且在本状态已停留 `stable_duration_s`，输出 `offboard` | `lifecycle.py` |
| `HOLD_READY` 闭锁 | 对齐失效 → `frame_reset`；或超过 `hold_ready_timeout_s` → `map_or_stream_not_ready` | `lifecycle.py`、`lifecycle.py` |
| `OFFBOARD_PENDING` → `EXECUTING` | **模式回读**确认 OFFBOARD（`offboard_observed`），输出 `enable_planner` | `lifecycle.py` |
| `OFFBOARD_PENDING` 闭锁 | 模式回读超 `mode_timeout_s` → `offboard_not_confirmed` | `lifecycle.py` |
| `EXECUTING` → `LANDING` | 到点判定：`pose_age ≤ pose_timeout_s` ∧ 距 goal ≤ `goal_tolerance_m` ∧ 速度 ≤ `stable_speed_m_s`，且持续 `stable_duration_s` | `lifecycle.py` |
| `LANDING` → `COMPLETE` | 连接 + 状态新鲜 + 已上锁 + `landed == 1` | `lifecycle.py` |
| 任意阶段 → `FAULT_LATCHED` | 时钟回退、状态超 `mode_timeout_s`、`boot_epoch` 变化（飞控重启）、会话变化、命中 `REVOKING_FAULTS`、飞行中 `landed == 1` 或上锁 | `lifecycle.py`、`lifecycle.py`、`lifecycle.py` |

`RECOVERABLE_FAULTS`（允许自动恢复，`lifecycle.py`）与 `REVOKING_FAULTS`（永久撤销本次恢复资格，`lifecycle.py`）是两个不同的集合；
`INTERRUPTIBLE_AUTO_MODES` 只识别 `auto:land` / `auto:rtl` 两个可被**有界**中断的
AUTO 模式（`lifecycle.py`）。自动中断还需新鲜实际/意图模式一致，以及当前 SIH PX4 原生 failsafe 原因匹配；人工模式动作或未知原因闭锁。

### 谁驱动哪一段

| 环节 | 责任方 | 证据 |
| --- | --- | --- |
| 状态判定与动作名 | `boom_birds_bringup/lifecycle.py`（纯状态机，时间由调用者传入） | `lifecycle.py`、`lifecycle.py` |
| ROS 接线、动作→实际调用 | `boom_birds_bringup/lifecycle_node.py` | `lifecycle_node.py`（`_actions`） |
| 解锁 / 起飞 / 模式切换 | **只**经 `VehicleAction` 服务；编排器自身不发模式命令、不碰 MAVLink | `lifecycle_node.py`、`lifecycle_node.py`；服务端 `px4_interface_node.py`，最终落 `backend.arm()` / `backend.set_mode()`（`px4_interface_node.py`） |
| 失效的**最终**判断 | `px4_failsafe.py` 的监控器决定"这一帧允不允许发 setpoint"，结果经 `ExecutionStatus` 回读；编排器只消费 | `px4_interface_node.py`、`px4_interface_node.py`、`px4_interface_node.py`；编排器侧 `lifecycle_node.py` |
| 观测构造 | 由 `ExecutionStatus` 组装 `Observation`（含 `mode_detail`、`sending`、`sensors_ready`） | `lifecycle_node.py` |
| hold / 制动点选择 | 恢复期用故障瞬间位置，其余用锁点 | `lifecycle.py`（`hold_point`）、`lifecycle.py`（`hold_here`）、`lifecycle.py`（`hold_at`） |
| 接管闸门（位置/速度/新鲜度） | `lifecycle_node.py` 在**新轨迹号**首次出现时判四项 | `lifecycle_node.py::_planner_command`；轨迹跳变退役并有界等待，观测异常按传感器故障或闭锁处理 |
| 接管距离独立判定 | `boom_birds_control/handoff.py`（CLI，复用同一 `handoff_max_distance_m`） | `handoff.py`、`handoff.py` |
| SIH 进程授权 | 仅当 `backend == "mavros"` 时校验本地 SIH 进程 | `sih_guard.py`；调用点 `px4_interface_node.py` |

### 恢复窗口与重规划语义

- **缺省不可达**：`RuntimeConfig.recovery_enabled` 缺省 `False`（`runtime_config.py`），只有 TEST-ONLY 的 SIH 入口把它显式打开
  （`px4_sih_mission.launch.py`）。
- **资格四道门**：`recovery_available()` = 开关 ∧ 未被撤销 ∧ 故障属于 `RECOVERABLE_FAULTS` ∧ 预算未耗尽（`lifecycle.py`）。
- **进入前置条件**（fail-closed：观测缺失/过期即拒绝）：未人工取消、仍解锁、明确在空中（`MAV_LANDED_STATE ∈ {2,3}`，`lifecycle.py`）、
  `status_age ≤ status_timeout_s`、`pose_age ≤ pose_timeout_s`、位置与速度有限（`lifecycle.py`）。
- **`recovery_attempts`**：进入 `RECOVERING` 即消耗一次（`lifecycle.py`），缺省预算 `2`（`runtime_config.py`）；耗尽后闭锁
  `detail="exhausted"`，**不循环争抢、不自动重新解锁**；恢复后连续 1 s 健康 EXECUTING 才关闭本次故障并重置事件预算，任务累计次数另行记录；窗口内再次失效共用预算（`lifecycle.py`、`lifecycle.py`）。
- **一次尝试的窗口**：`mode_timeout_s`（缺省 3 s，`runtime_config.py`）同时约束"等条件就绪"与"等模式回读"两段
  （`lifecycle.py`）；超时且预算还有就重试（`lifecycle.py`）。
- **连续有效窗口**：`recovery_valid_duration_s`（缺省 1 s，`runtime_config.py`）要求 `sensors_ready`、`map_ready`、`alignment`、
  两条 age 与有限位置**连续**成立，任一失效即清零并把原因记进 `recovery_window_reset`
  （`lifecycle.py`、`lifecycle.py`）。
- **接管放行**：还需 `Observation.sending`（即 `px4_failsafe` 的放行结果）、已记录 `ground_z`、相对高度 ≥ `recovery_min_altitude_agl_m`
  （1.0）、速度 ≤ `recovery_max_speed_m_s`（1.0）（`lifecycle.py`）。
- **请求 OFFBOARD 只发一次并要求回读确认**：`recovery_request_at` 置位后返回 `offboard` 动作，随后靠 `offboard_observed` 确认，
  **命令被接受不算确认**（`lifecycle.py`、`lifecycle.py`、`lifecycle.py`）。
- **确认后重新规划**：`_confirm_recovery()` 把 hold 移到故障瞬间位置、转 `EXECUTING`，输出 `replan` + `enable_planner`
  （`lifecycle.py`）。`replan`/`recover` 都调 `_retire_trajectory()`：把当前 `trajectory_id` 记入 `ControlIngress.retired`
  并 +1，同 id 命令此后一律被 `retired_trajectory` 拒绝（`lifecycle_node.py`、`control_protocol.py`）。
- **planner 重新使能**：正常路径是 `EXECUTING` 期内**限频重发**使能请求，间隔 `planner_enable_retry_s`（缺省 0.5 s，
  `runtime_config.py`），上限 `planner_activate_timeout_s`（缺省 30 s）后闭锁 `planner_timeout`
  （`lifecycle_node.py`、`lifecycle_node.py`、`lifecycle_node.py`）。重发是必要的：EGO 的 `projectRequest` 在
  offboard/odom/地图未就绪时**静默丢弃**。
- **规划器单次作废 ≠ 任务故障**：收到 `CANCEL` 型规划命令只退役旧轨迹并转入"规划流静默"，由上面的有界等待处理
  （`lifecycle_node.py`）。
- **规划流停止后的 hold 点**：若距 goal 已在 `handoff_max_distance_m`（0.5 m）以内，hold 点放到 **goal**，让飞控收敛最后一段；
  否则放在**当前位置**，不在没有规划的情况下继续飞（`lifecycle_node.py`，理由见 `lifecycle_node.py`）。
- **永久撤销**：人工取消 / 人工模式干预 / 未知故障 / 落地 / 重启 / 坐标系重置 / 几何变化（`lifecycle.py`、`lifecycle.py`）。

## 控制协议（EGO ↔ 控制）

两个 EGO 侧的入口由同一个 launch 参数 `require_session` 切换（EGO 子模块内 `src/planner/plan_manage/launch/boom_birds_offline.launch.py`、
`ego_replan_fsm.cpp`、`traj_server.cpp`）：`false` 走上游的 `planning/bspline` + `/position_cmd` 直连；
`true` 走下面这张会话化接口表。SIH 任务入口固定 `require_session=true`（`px4_sih_mission.launch.py`、`px4_sih_mission.launch.py`）。

| 方向 | 话题/服务 | 类型 | QoS / 深度 | 谁发布 → 谁订阅 |
| --- | --- | --- | --- | --- |
| 编排 → EGO | `/boom_birds/planner/request` | `boom_birds_interfaces/PlannerRequest` | depth 50 | `lifecycle_node.py` → `ego_replan_fsm.cpp` |
| 编排 → 控制 | `/boom_birds/control/command` | `boom_birds_interfaces/ControlCommand` | depth 50（发布）/ reliable depth 10（订阅） | `lifecycle_node.py` → `px4_interface_node.py` |
| EGO → 编排 | `/boom_birds/planner/command` | `boom_birds_interfaces/ControlCommand` | depth 50 | EGO `traj_server.cpp` → `lifecycle_node.py` |
| 执行器 → 编排 | `/boom_birds/planner/executor_status` | `boom_birds_interfaces/PlannerStatus` | depth 10 | `traj_server.cpp` → `lifecycle_node.py` |
| EGO → 编排 | `/boom_birds/planner/status` | `boom_birds_interfaces/PlannerStatus` | depth 10 | `ego_replan_fsm.cpp` → `lifecycle_node.py` |
| 控制 → 编排/EGO | `/boom_birds/control/execution_status` | `boom_birds_interfaces/ExecutionStatus` | 发布 reliable depth 10（`QOS_RELIABLE`）/ 订阅 depth 50 ×3 | `px4_interface_node.py` → `lifecycle_node.py`、`ego_replan_fsm.cpp`、`traj_server.cpp` |
| 编排 → 控制 | `/boom_birds/control/action` | `boom_birds_interfaces/srv/VehicleAction` | 服务 | 客户端 `lifecycle_node.py`；服务端 `px4_interface_node.py` |
| 外部 → 编排 | `/boom_birds/mission` | `boom_birds_interfaces/srv/Mission` | 服务 | 服务端 `lifecycle_node.py` |
| 编排 → 外部 | `/boom_birds/mission/status` | `std_msgs/String`（JSON） | depth 10 | `lifecycle_node.py`（含 `hold_ready_gate` 与 `recovery` 诊断） |
| 控制 → 外部 | `/boom_birds/control/status` | `std_msgs/String`（JSON） | depth 10 | `px4_interface_node.py` |
| EGO 内部（会话模式） | `planning/session_bspline` | `traj_utils/SessionBspline` | depth 50 | `ego_replan_fsm.cpp` → `traj_server.cpp` |
| EGO 内部（上游模式） | `planning/bspline` | `traj_utils/Bspline` | depth 10 | `ego_replan_fsm.cpp` → `traj_server.cpp`；launch 重映射到 `/boom_birds/ego/planning/bspline`（`boom_birds_offline.launch.py`） |
| 直连模式专用 | `/position_cmd` | `quadrotor_msgs/PositionCommand` | depth 50（发布）/ reliable depth 10（订阅） | `traj_server.cpp` → `px4_interface_node.py`；`frame_id` 必须在允许集合内（`px4_interface_node.py`、`px4_interface_node.py`），无会话、无序号 |

QoS 常量：`QOS_SENSOR` = best_effort / depth 5，`QOS_RELIABLE` = reliable / depth 10（`px4_interface_node.py`）。

### session 语义

任务会话由 PX4 接口生成；EGO 与轨迹执行器另各自生成随机进程 ID。
`PlannerStatus.producer_session_id` 在规划器状态与 `/boom_birds/planner/executor_status` 中注册生产者。
编排器绑定本任务的两个进程 ID；执行命令、取消与内部 `SessionBspline` 均携带对应 ID。
活动任务中进程 ID 改变即 `session_changed` 闭锁，旧进程的序号或轨迹号不能重新解释为新命令。
EXECUTE/CANCEL/HOLD 共用可靠、有序的控制话题；取消形成退役屏障，后续旧轨迹不再发送。

`ExecutionStatus` 分开报告 accepted、sending 与模式回读。恢复还需新鲜 CURRENT_MODE 的实际/意图模式、
ODOMETRY reset counter、落地状态及 SIH PX4 原生 failsafe 原因；缺少任一必要观测拒绝恢复。
对齐 yaw/translation 由控制后端回读，编排器按同一变换换算位置和速度；运行中变化闭锁。


- **开会话**：`VehicleAction.OPEN_SESSION` 生成 `uuid4` 会话并把闸门上限绑定到当时的飞控启动周期
  （`px4_interface_node.py`、`control_protocol.py`）。开会话**不是**规划拒绝，它清掉上一轮的 `PLANNING_REJECTED` 闭锁
  （`px4_interface_node.py`）。
- **消息字段**：`ControlCommand.msg` 的字段是 `header / session_id / planner_session_id / executor_session_id / trajectory_id / sequence / valid_for / command_type / position /
  velocity / acceleration / yaw / yaw_rate`——**没有 `frame_id` 字段**；坐标系由 `std_msgs/Header.header.frame_id` 携带，`control_protocol.Command.frame` 即取自它（`lifecycle_node.py`、`lifecycle_node.py`、`lifecycle_node.py`、`px4_interface_node.py`）。`command_type` 常量
  `EXECUTE=1 / CANCEL=2 / HOLD=3`（`control_protocol.py`）。
- **接收校验（顺序即拒绝优先级）**：会话一致 → `sequence` 严格递增 → `trajectory_id > 0` → `command_type` 合法 →
  `CANCEL` 形成屏障（退役并取消，不需延续旧 setpoint）→ 未退役/未回退的轨迹 → 全量有限 → `frame` 非空且 `0 < valid_for ≤ max_ttl`
  （`max_ttl = command_timeout_s`，0.2 s）→ `-0.03 ≤ ros_now - stamp < valid_for` → 时钟回退即 `clock_reset`
  （`control_protocol.py`）。
- **有效期**：`deadline = mono_now + min(valid_for, stamp + valid_for - ros_now)`，`active()` 每周期复核，过期即 `cancel("expired")`
  （`control_protocol.py`、`control_protocol.py`）。`traj_server` 发出的 `ControlCommand.valid_for` 取 `command_timeout_s`（当前 200 ms）
  （`traj_server.cpp`）。
- **trajectory id**：`traj_server` 每接受一条新的 `SessionBspline` 序号即 `++project_trajectory_id_` 并写入消息
  （`traj_server.cpp`、`traj_server.cpp`）；编排器侧在轨迹号变化时再 `+1`，`recover`/`replan` 时也 `+1`
  （`lifecycle_node.py`、`lifecycle_node.py`）。
- **retire / `clear_planning_rejected`**：`PLANNING_REJECTED` 是**外部事件闭锁**，只能由**新** `trajectory_id` 清除——同一 id 继续出现
  只说明规划器还在拒绝同一个目标（`px4_interface_node.py`、`px4_failsafe.py`、`px4_failsafe.py`）。
  `TRAJECTORY_INVALIDATED`（`trajectory_flag != READY`、空 Bspline、`order == 0`）由"带**不同** id 的 setpoint 到达"清除
  （`px4_interface_node.py`、`px4_failsafe.py`）。
- **EGO 侧门限**：`traj_server` 只接受"会话一致 ∧ `offboard_confirmed` ∧ 授权时间戳 0.2 s 内"的样条；`cmdCallback` 每周期复核授权，
  失配即清空轨迹并停发（`traj_server.cpp`、`traj_server.cpp`）。`ego_replan_fsm` 在 `projectRequest` 里对 `offboard_confirmed`、
  odom、`mapReady()` 三项不满足时**直接返回**（`ego_replan_fsm.cpp`），并在 `execFSMCallback` 里同样按 0.2 s 窗口门控
  （`ego_replan_fsm.cpp`）。

### 门限字段

| 字段 | 值 | 生效处 |
| --- | --- | --- |
| `image_publish_min_altitude_agl_m` | 0.3 | **只**用于合成双目/深度开始发布的下限（`px4_sitl_motion.launch.py`、`px4_sih_mission.launch.py`）；被明确禁止当作规划链启动闸门（`runtime_config.py`） |
| `pose_timeout_s` | 0.15 | 锁点窗口 `lifecycle.py`、恢复前置 `lifecycle.py`、接管闸门 `lifecycle_node.py` |
| `status_timeout_s` | 1.0 | 恢复前置 `lifecycle.py`、PRECHECK `lifecycle.py`、接管 `lifecycle_node.py`、LANDING 完成 `lifecycle.py` |
| `setpoint_interrupt_timeout_s` | 1.0 | setpoint **持续**断流才算故障，到点保持仍检查（`lifecycle_node.py`、`lifecycle_node.py`） |
| `heartbeat_timeout_s` | 2.5 | PX4 心跳到达超时，经 `FailsafeConfig` 的 `PX4_HEARTBEAT` 生效（`runtime_config.py`、`px4_failsafe.py`） |
| `landed_state_timeout_s` | 2.5 | `ExecutionStatus.landed_known`（`px4_interface_node.py`） |
| `command_timeout_s` | 0.2 | `ControlIngress` 的 `max_ttl`（`lifecycle_node.py`、`px4_interface_node.py`）与"规划流已停止"判据（`lifecycle_node.py`） |
| `planning_timeout_s` | 1.0 | 规划器状态/地图就绪的新鲜度（`lifecycle_node.py`） |
| `handoff_max_distance_m` / `handoff_max_speed_m_s` | 0.5 / 0.3 | 接管四项判据（`lifecycle_node.py`、`lifecycle_node.py`）、hold 点选择（`lifecycle_node.py`） |
| `max_setpoint_age_s` | 0.2 | 等同 `setpoint_timeout_s`（`px4_interface_node.py`、`px4_interface_node.py`） |
| `mode_timeout_s` | 3.0 | 模式确认、恢复尝试窗口、服务调用截止（`runtime_config.py`） |

> 尚未验证：以上协议只在脱机测试与 SIH 场景中验证；PX4 侧对"停发 setpoint"的实际反应取决于
> `COM_OF_LOSS_T` / `COM_OBL_RC_ACT`，**未在真机验证**（`px4_interface_node.py`、`docs/STATUS.md`）。

## 参数来源

### 单一来源

| 内容 | 位置 | 强制方式 |
| --- | --- | --- |
| 运行阈值、话题名、坐标系、高度量、场景 | `companion/ros2_ws/src/boom_birds_control/config/runtime.yaml` + `.../boom_birds_control/runtime_config.py` 的 `RuntimeConfig` 默认值（`runtime_config.py`） | 两者必须逐字段相等：`test_runtime_config.py::test_packaged_template_matches_code_defaults`、`test_config_single_source.py::test_runtime_yaml_equals_code_defaults` |
| `runtime.yaml` 只允许存在一份，且必须位于所有者包 | `boom_birds_control/config/` | `test_config_single_source.py::test_runtime_yaml_is_the_only_definition_in_source_tree`；`config_path()` 多候选查找并在找不到时**明确失败**（`runtime_config.py`、`runtime_config.py`） |
| 安装树副本必须与源码一致 | ament share | `test_config_single_source.py::test_installed_runtime_yaml_matches_source` |
| 契约语义（话题、类型、单位、坐标系、无效值、timing 语义） | `companion/ros2_ws/src/boom_birds_interfaces/config/contract.yaml` | `timing` 段不允许出现任何数值叶子；`test_config_single_source.py::test_contract_timing_is_semantic_only`；本文件**不作为 ROS 参数文件加载**（`runtime_config.py`） |
| 节点私有参数（`px4_interface`） | `boom_birds_control/config/px4_interface.yaml` | 不重复共享阈值；缺省值取 `RuntimeConfig`，私有参数由测试登记 |
| IMU 私有参数 | `boom_birds_sensing/config/mavros_imu.yaml` | MAVROS 时间同步门控；旧 mavlink_imu.yaml 仅供脱机历史回归 |
| launch / tools 不得硬编码高度与接管字面量 | — | `test_config_single_source.py::test_launch_and_tools_have_no_hardcoded_altitude_or_handoff_literals` |

### PX4 侧参数

PX4 参数**不**在 launch 里写死，统一经 `companion/ros2_ws/src/boom_birds_bringup/boom_birds_bringup/sih_params.py` 生成/下发：

- 起飞高度 → `MIS_TAKEOFF_ALT`，值取 `takeoff_altitude_agl_m`，参数名取 `px4_takeoff_param_name`（`sih_params.py`、`runtime_config.py`）。
- `COM_OBL_RC_ACT`（Offboard 失联动作，SIH 默认 `land`=4）与 `COM_OF_LOSS_T`（显式下发 1.0 s，与 PX4 版本缺省一致，用于**留证不放大**）
  是 SIH 专用常量，**不是** `RuntimeConfig` 字段（`sih_params.py`、`sih_params.py`）。
- shell 只能通过 `--get` 读取 `SHELL_FIELDS` 白名单字段（`sih_params.py`）；脚本必须调用本模块、不得再出现 `MIS_TAKEOFF_ALT` 字面量
  （`test_config_single_source.py::test_takeoff_script_delegates_px4_params_to_sih_params`）。

### 场景覆盖白名单

`runtime.yaml` 定义 `local`、`forest_30m`、`recovery_local`。
普通场景只可覆盖 `origin_x/y/z` 与 `depth_max_range_m`；`recovery_local` 另允许起飞/锁点高度，供 Land/Return 恢复测试留出高度。
未知字段明确失败。30 m 场景原点 `(-15,0,0.1)` 同时用于合成输入、控制对齐和证据汇总，不由 shell 复述。

### 改一个参数要动哪些文件（自检清单）

| 你要改的东西 | 必须改的唯一位置 | 还要同步 | 必跑的回归 |
| --- | --- | --- | --- |
| 任意运行阈值 / 话题名 / 坐标系名 | `boom_birds_control/config/runtime.yaml` **和** `runtime_config.py` 的同名默认值 | 共享字段不在节点参数模板重复；launch/脚本引用配置对象 | `test_runtime_config.py`、`test_config_single_source.py` |
| 新增一个失效阈值 | 同上 | 在共享字段映射中登记；节点与 launch 引用同一值 | 同上 |
| 新增一个"名字像阈值但不参与失效判定"的节点参数 | `px4_interface.yaml` | 在 `NON_THRESHOLD_PARAMETERS` 登记并写理由 | `test_config_single_source.py` |
| 高度量（新增或改名） | `RuntimeConfig` 字段 | 在 `ALTITUDE_REFERENCE` 登记参考系；`test_runtime_config.py::test_declared_altitudes_have_a_reference_frame` 会拒绝含糊的 `*_height_m` | `test_runtime_config.py` |
| 场景专用原点 / 深度量程 | `runtime.yaml` 的 `scenes:` | 无（`depth_max_range_m` 同时被 `depth_node.max_depth_m` 与 EGO `max_ray_length` 引用，见 `runtime_config.py`） | `test_runtime_config.py` |
| 契约语义（话题类型、单位、坐标系、无效值） | `boom_birds_interfaces/config/contract.yaml` | 不许写数值；`timing.declared` 必须保留既有语义名 | `test_config_single_source.py::test_contract_timing_is_semantic_only` |
| PX4 起飞高度 | `runtime.yaml` 的 `takeoff_altitude_agl_m` | 脚本自动经 `sih_params` 生成 `px4-param set`，不要再写第二处 | `test_config_single_source.py::test_sih_params_emits_configured_takeoff_altitude`、`test_config_single_source.py::test_sih_params_tracks_runtime_yaml` |
| PX4 失联动作 / 失联超时 | `sih_params.py` 的常量 | 属于 SIH 行为选择，不进 `RuntimeConfig`；改动要同步 README 与 STATUS（`sih_params.py`） | `test_config_single_source.py::test_sih_params_emits_configured_takeoff_altitude` |
| 节点私有参数（`backend` / `dry_run` / `connection` / `control_rate_hz` / `yaw_mode` / `send_acceleration` / `frame_alignment*`） | `px4_interface.yaml` | 阈值类仍需登记 | `test_config_single_source.py` |
| 自动恢复开关 | `runtime.yaml` 的 `recovery_enabled`（缺省 `false`） | 只有 TEST-ONLY SIH 入口允许覆盖为 `true`（`px4_sih_mission.launch.py`） | `test_runtime_config.py::test_recovery_is_disabled_by_default` |

改完的机械自检（**本文档未执行**，NOT RUN）：

```bash
python -m pytest companion/ros2_ws/src/boom_birds_nav/test/test_runtime_config.py                  companion/ros2_ws/src/boom_birds_nav/test/test_config_single_source.py
git diff --check
```

## 文档维护

- 本页维护目标、架构与稳定基线；[STATUS](docs/STATUS.md) 维护当前能力、阻塞和下一步；AGENTS 只维护工作规则。
- 安装、运行和排障写在模块旁边。一个事实只维护一处，其他页面链接引用；不重复记录实时 Git 状态、目录树和环境检查结果。
- 需求表只在验收要求或结果变化时更新，BOM 只在物料或证据变化时更新；普通文档改动不要求同步所有文件。
- 重要工程取舍追加到 [决策记录](docs/DECISIONS.md)。日期化任务、原始规则与测试记录按需查阅，不作为当前状态，也不要求每轮创建交接文件。
- 技术结论按实测证据、当前厂商/源码资料、最新有效决策、工程基线、历史估算的顺序核对，保留各自适用条件；规则适用顺序以 RULE_BASELINE 为准。
