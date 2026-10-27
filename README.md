# Boom Birds

RoboMaster 2027 微型空间智能无人机预研。导航选型已确定；视觉定位、避障、返库的集成与飞行尚未验收。

当前进度见 [STATUS](docs/STATUS.md)，操作命令见各模块 README；协作规则见 [AGENTS.md](AGENTS.md)。

## 文档入口

| 需要查什么 | 文档 |
| --- | --- |
| 当前完成项、阻塞、标定选择与下一步 | [STATUS](docs/STATUS.md) |
| 环境、构建、部署、子模块 | [ROS 工作空间](companion/ros2_ws/README.md) |
| 双目运行库、相机标定、预览与性能回放 | [运行库](companion/ros2_ws/src/stereo_depth/README.md) / [标定](companion/ros2_ws/tools/camera_calibration/README.md) / [预览](companion/ros2_ws/tools/depth_preview/README.md) / [回放](companion/ros2_ws/tools/depth_benchmark/README.md) |
| 采集、IMU / 相机—IMU标定与噪声 | [感知包](companion/ros2_ws/src/boom_birds_sensing/README.md) / [标定工具](companion/ros2_ws/tools/vio_calibration/README.md) |
| 实机姿态控制、算力配置与验证顺序 | [bringup](companion/ros2_ws/src/boom_birds_bringup/README.md) |
| SIH 集成、兼容入口、接口与参数 | [导航索引](companion/ros2_ws/src/boom_birds_nav/README.md) |
| 平台降落 / Gazebo | [控制包](companion/ros2_ws/src/boom_birds_control/README.md) / [仿真包](companion/ros2_ws/src/boom_birds_sim/README.md) |
| 规则、需求、硬件与决策 | [规则基线](docs/RULE_BASELINE.md) / [需求](docs/REQUIREMENTS.md) / [BOM](hardware/bom/README.md) / [决策](docs/DECISIONS.md) |
| 平台特征跟踪接口与限制 | [局部特征跟踪](docs/PLATFORM_FEATURE_TRACKING.md) |
| EGO fork 修改与历史补丁 | [补丁索引](docs/EGO_FORK_PATCHES.md) |
| 日期化验证与失败证据 | [验证记录](docs/STATUS.md#验证记录) |

## 开发与运行入口

唯一主开发根：WSL Ubuntu-24.04 `/home/waterc/workspace/Boom_Birds`。Git、编辑和构建使用 WSL Linux 工具；在根目录运行 `code .` 并确认编辑器处于 WSL。

```bash
git clone --recurse-submodules https://github.com/waterc07/Boom_Birds.git
# 已有 clone：在项目根执行
git submodule update --init --recursive
```

- [ROS 2 工作空间](companion/ros2_ws/README.md)：开发环境、子模块、Git 与部署约定。
- [双目深度运行库](companion/ros2_ws/src/stereo_depth/README.md)：构建、导入和默认标定；独立操作见[预览](companion/ros2_ws/tools/depth_preview/README.md)与[相机标定](companion/ros2_ws/tools/camera_calibration/README.md)。
- [PX4](px4/README.md)：独立源码与 FC-001 实板验收步骤。

运行部署使用[独立运行构建入口](companion/ros2_ws/README.md#运行与开发隔离)；标定采集、噪声分析、预览和回放位于工作空间 `tools/`，仿真与 TEST-ONLY 配置归 `boom_birds_sim`，均不进入运行安装树。WSL 合成链路与分层启动方式见 ROS 工作空间说明，真实 VIO 与全导航实机闭环的验收状态见 STATUS。

## 系统架构

```text
USB 同帧双目 → 统一采集 / 左右拆分 / 时间戳 / 标定
                ├─ 左右目图像 + 飞控 IMU → 双目 OpenVINS → 位姿 / 速度
                └─ 校正 / 双目匹配 → 米制深度 / XYZ
                                      ↓ 与对应时刻位姿结合
任务目标 → EGO-Planner ← 局部地图 + 里程计
                ↓
          轨迹执行 → 控制接口 → Px4Interface → MAVROS → PX4 → ESC
                                  ↑            ↓
                                  └── IMU / 状态
MTF-02P 独立光流 / 测距 ───────────────────→ PX4
```

无 EGO 的三项 AprilTag 验证使用独立入口：固定航点＋双目 VIO＋下视 Tag/局部特征 PnP＋测距 → Companion 位置闭环 → MAVROS 姿态/推力 → PX4；前视米制深度负责停车、单次绕行和回撤检查，不启动地图/EGO，VIO 全程保留。任务文件、输入与命令见 [bringup](companion/ros2_ws/src/boom_birds_bringup/README.md#独立-apriltag-飞行任务)。

图示为当前开发架构；验收与部署状态见 STATUS。Ubuntu 24.04 / ROS 2 Jazzy 为开发基线，硬件保持 Pi 5 与现有相机。双目 VIO 与双目深度共用图像，分别估计位姿和稠密深度；全链算力与定位质量仍待验收。正式 launch 默认值尚未切换，配置区别见[实机入口](companion/ros2_ws/src/boom_birds_bringup/README.md#双目-vio-与启动配置)。

- VIO 使用图像与飞控加速度/角速度，不能用稠密深度或飞控融合姿态代替输入。IMU 来源已确定为飞控；消息接口与时间门控已实现，已有受限真机短测；正式导航负载、标定候选和真实 VIO 的验收状态见 STATUS。
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
| 整机质量 | CURRENT BASELINE | 比赛设计要求，非当前试飞机实测值；仅按当前规则限制：`150 g ≤ m ≤ 322.5-1.225D`；118 mm时上限约177.95 g，120 mm时上限约175.5 g，不另设165 g硬上限 |
| 构型 | CURRENT BASELINE | 同平面、固定前向任务方向、全包围桨保四旋翼；完整涵道非强制；桨叶和机体不得外露，原开放式保护框架须重新审查任意角度碰撞保护 |
| 电机 | LOCKED | 4 × T-MOTOR 1103 8000KV |
| 桨叶 | LOCKED | 4 × Gemfan 2216S-3，2.2 inch 三叶 |
| 开发能源 | LOCKED | 3S 300 mAh、95C、23 g、约 19×18×50 mm；化学体系及接口待确认 |
| 飞控/AIO | LOCKED | 已购入 MicoAir743v2-AIO-45A，运行 PX4 |
| ESC 协议 | CURRENT BASELINE | DShot600 |
| 安全定位 | LOCKED | 已购入 MTF-02P 光流测距一体传感器，直接输入 PX4，不依赖 Companion |
| Companion | CURRENT BASELINE | Pi 5；硬件固定，全链负载待验收 |
| 当前双目深度输入 | CURRENT BASELINE | USB 免驱、硬件同帧左右拼接；具体模式、转正方向与标定见双目模块 README；型号、快门与距离精度待验证 |
| 前视相机 | CANDIDATE | 彩色全局快门，约 0.5–1 MP+、60–120+ FPS、MIPI CSI |
| 下视相机 | CANDIDATE | 现有 IMX219 用于下视标定与平台验证；单色全局快门为候选要求，未完成选型；下视不替代双目 + 飞控 IMU 主定位 |
| 主定位 | CURRENT BASELINE | OpenVINS；输入同帧左右目图像与飞控 IMU，输出位置、姿态、速度；尚未集成验收 |
| 路径规划与避障 | CURRENT BASELINE | 自算双目深度 + 里程计建图，使用个人 fork https://github.com/waterc07/ego-planner-swarm；精确版本由母仓库 gitlink 固定，维护分支为 `boombirds-jazzy`；子模块更新约定见 [ROS 工作空间](companion/ros2_ws/README.md#子模块)；补丁索引见 [EGO fork 补丁索引](docs/EGO_FORK_PATCHES.md)；设备验证范围见 STATUS |
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

本机目录入口：

| 位置 | 内容与边界 |
| --- | --- |
| /home/waterc/bb_build/ | 构建前缀、环境、诊断脚本与历史验证目录；目录清单见该目录 README.md，不按日期搬动既有前缀 |
| /home/waterc/bb_data/ | 已回收 bag、配置、分析与部署证据；保留原路径和校验信息 |
| /home/waterc/boombirds-data/ | 独立 stereo_depth 工具的采集与输出 |
| /home/waterc/boombirds-hardware/ | VIO 配置候选与 current-vio 选择；不是 Pi 部署状态 |
| Windows reports/README.md、artifacts/README.md | 日期化报告与导出产物索引；不维护当前工程状态 |

WSL 的构建前缀、虚拟环境、安装树及仍被 symlink-install 引用的源文件保持原位。可归档的本机缓存移动时保留 moves.json；Windows 历史临时脚本归入 archive/，打印文件归入 artifacts/。本机清单与外部证据不随 Git 分发。

资料流向：Windows 原件 → WSL 开发/分析 → 保存证据或交付物 → 更新 STATUS 或模块说明。少量读取可直接访问 `/mnt/d/...`；大量反复处理可按需复制到 WSL 仓库外的临时工作区，记录来源，缓存不作为唯一原件。

Windows 可通过 VS Code WSL 模式或 `\\wsl.localhost\Ubuntu-24.04\home\waterc\workspace\Boom_Birds` 查看/编辑主工程。树莓派目录属于部署现场，更新前比较差异，现场修改取回 WSL。Git 和同机资料目录不能替代独立备份；现有迁移备份保留，清理策略另行确定。

| 路径 | 职责 |
| --- | --- |
| `companion/ros2_ws/src/boom_birds_interfaces/` | 项目消息、生命周期服务、说明性契约 |
| `companion/ros2_ws/src/boom_birds_sensing/` | 采集、时间同步、深度发布、位姿适配 |
| `companion/ros2_ws/src/boom_birds_control/` | PX4 后端、坐标转换、执行许可、校验后的运行配置 |
| `companion/ros2_ws/src/boom_birds_bringup/` | 生命周期编排、生产集成启动 |
| [boom_birds_sim](companion/ros2_ws/src/boom_birds_sim/README.md) | 合成输入、SIH 真值与场景、Gazebo 传感器录制和回放评估 |
| `companion/ros2_ws/src/boom_birds_nav/` | Python 模块与 launch 的兼容转发；无第二份实现 |
| `companion/ros2_ws/src/stereo_depth/` | 双目运行库、原生计算核、默认标定与自动测试夹具 |
| `companion/ros2_ws/tools/{camera_calibration,vio_calibration,sensor_replay,depth_preview,depth_benchmark}/` | 相机/VIO 标定、噪声、传感器回放、预览与性能工具；不随运行包安装 |
| `/home/waterc/boombirds-data/stereo_depth/` | 标定过程记录与深度导出；默认写入 `calibration/`、`depth_outputs/`，不进入 Git |
| `/home/waterc/boombirds-hardware/current-vio` | 已选 VIO 开发配置；用途与默认深度标定分开，见 STATUS |
| `companion/ros2_ws/src/open_vins/` | 个人 OpenVINS fork 子模块 |
| `companion/ros2_ws/src/ego-planner-swarm/` | 单机 EGO 规划器 fork 子模块（目录名保留上游名称） |
| `/home/waterc/bb_build/main/{build,install,log}` | WSL 构建产物；仓库外保存，不跨平台复制 |
| `docs/`、`hardware/` | 当前文档、规则、需求、BOM 与硬件证据 |
| `docs/tasks/`、`docs/workflows/`、`px4/manifests/` | 本机历史验证记录与迁移清单，不再纳入新提交 |
| `tools/` | ULog 离线分析；显式传入日志文件与输出目录 |
| `/home/waterc/PX4-Autopilot` | 独立 PX4 仓库与已有构建目录 |
| `/home/waterc/mavlink` | 独立 MAVLink 仓库，不等同于 PX4 自带依赖 |
| Pi `/home/gmaster/boombirds/current` | Companion 部署入口；`current/run.sh` 为特征→EKF dry-run 入口，外部感知/MAVROS 单独提供，`activate.sh` 仅加载环境；SSH 地址与主备 Wi-Fi 见 [设备网络](docs/STATUS.md#设备网络)，版本与检查结果见 STATUS |
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

位置路线的状态迁移、恢复窗口和任务驱动见 [bringup](companion/ros2_ws/src/boom_birds_bringup/README.md#位置路线生命周期)。实机姿态路线见同一模块的接入说明；两条路线的恢复机制不混用。

## 控制协议（EGO ↔ 控制）

会话、序号、有效期、取消屏障与 PX4 出口见[控制包](companion/ros2_ws/src/boom_birds_control/README.md#控制协议ego--控制)；消息定义与单位见[接口契约](companion/ros2_ws/src/boom_birds_interfaces/config/contract.yaml)。

## 参数来源

共享配置、节点覆盖及修改检查见 [ROS 工作空间](companion/ros2_ws/README.md#参数来源)。当前部署与验收只维护在 [STATUS](docs/STATUS.md)。

## 文档维护

README 管架构与入口；STATUS 管当前事实和下一步；模块 README 管接口与操作；日期化证据放在 docs/history/。修改要求见 [AGENTS](AGENTS.md#文档维护)。
