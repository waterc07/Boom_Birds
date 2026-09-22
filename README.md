# Boom Birds

RoboMaster 2027 微型空间智能无人机预研：建立可验证的飞行、视觉定位、避障和返库链路，再扩展任务与能源方案。当前导航选型已确定，集成与飞行尚未验收。

日常只需阅读本页和 [当前状态与下一步](docs/STATUS.md)；AI 另读 [AGENTS.md](AGENTS.md)。操作细节按需进入模块说明。

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

深度程序仍是独立 Python 程序，尚无项目整体启动命令。WSL 依赖阻塞见 STATUS；解决后可从 `companion/ros2_ws/src/stereo_depth` 运行 `python3 depth_preview.py --help` 检查 CLI，设备采集按模块说明执行。

## 系统架构

```text
USB 同帧双目 → 统一采集 / 左右拆分 / 时间戳 / 标定
                ├─ 左右图像 + 飞控 IMU → OpenVINS → 位姿 / 速度
                └─ 校正 / 双目匹配 → 米制深度 / XYZ
                                      ↓ 与对应时刻位姿结合
任务目标 → EGO-Planner ← 局部地图 + 里程计
                ↓
          轨迹执行 → 控制接口 → Px4Interface → PX4 → ESC
                                  ↑          ↓
                                  └─ IMU / 状态
MTF-02P 独立光流 / 测距 ───────────────────→ PX4
```

这是目标架构，已实现范围见 STATUS。Ubuntu 24.04 / ROS 2 Jazzy 为开发基线；Pi 5 用于验证，RK3576 是后续迁移方向，具体板卡与最终机载适用性待验证。

- VIO 使用图像与飞控加速度/角速度，不能用稠密深度或飞控融合姿态代替输入。IMU 来源、速率、时间映射、相机—IMU 外参与时间偏移待定义。
- 控制器位于 Companion 或 PX4 尚未确定；通信后端、外部视觉回传及 EKF2 融合配置随此确定。MAVLink2/UART 仅为参考。
- 算法通过 `Px4Interface` 获取状态和发送 setpoint，不直接依赖串口；Companion 不输出 PWM/DShot。VIO 与安全光流/测距处于不同故障域。
- Companion 超时后由 PX4 执行经验证的安全动作；定位失效时不能默认仍可悬停，不能持续盲冲。
- 软件职责包括采集/记录、估计、深度、建图、规划、轨迹执行、控制适配，以及后续目标检测/跟踪、降落、任务、能源与故障管理；不要求为尚未实现的职责创建空模块。
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
| 路径规划与避障 | CURRENT BASELINE | 自算双目深度 + 里程计建图，使用个人 fork https://github.com/waterc07/ego-planner-swarm；当前基线 ros2_version，具体 SHA 由母仓库 gitlink 固定；Jazzy/ARM64 兼容性待验证 |
| 补充避障传感器 | CANDIDATE | 8×8 multi-zone ToF 类传感器，不替代双目建图与规划主线 |
| 最终能源 | TBD | 高概率超级电容 + 独立电容管理模块 |
| 撞击/拦截结构 | CANDIDATE | 必须覆盖直接撞击与主动迎击能力方向；具体判定、载荷路径和实现待细则与实测 |

45A 是 AIO 工程冗余，不代表单电机设计电流。上述为设计基线，采购与测量证据见 [BOM](hardware/bom/README.md)，详细验收项保留在 [需求表](docs/REQUIREMENTS.md)。

比赛规则以 [RULE_BASELINE](docs/RULE_BASELINE.md) 为唯一合规入口：2026-09-09 书面前瞻优先，未述项沿用旧规范，均未定义者 TBD；实测不能放宽规则。人工飞行属于研发验证，比赛要求全自动。规则分析与未采纳设计见 [2026-09-09 评审](docs/RULE_REVIEW_2026-09-09.md)。

动力研究以约 160 g 为参考点而非质量限制，目标推重比至少 2.5、理想 2.5–3.0，总静推力约 400–480 gf、单电机 100–120 gf，须由当前电机/桨/ESC 实测确认。先覆盖现有电池的实测电压范围；推力台记录电压、命令、电流、功率、推力、RPM 和电机/ESC 温度，形成 `T=f(V,u)`、`P=f(V,T)`、`T/P`。

电池化学体系、满充电压与连接器仍未确认，不擅自写成 LiPo/LiHV/XT30。后续 Energy Module 可替换，逻辑电源与动力母线受控；不默认裸 EDLC 直连 ESC。电容稳压、限流、均衡、保护、遥测和总线策略均 TBD，可研究 `remaining_energy_J`，不阻塞当前电池验证。

## 源码与资料位置

**WSL 负责开发和当前工程事实；Windows 负责原始资料、人工查看与交付文件。** 同一份源码只在 WSL 维护。

| 内容 | 归属与维护方式 |
| --- | --- |
| 源码、Git、子模块、当前项目文档 | WSL 主工程；Windows 不保留另一份可编辑副本 |
| Python/ROS 环境、构建缓存 | WSL Linux 文件系统；不跨系统复制环境或编译产物 |
| 默认标定、小型测试样例与复现证据 | 适合 Git 的内容随 WSL 代码版本保存 |
| 原始视频、照片、ULog、大型采集数据 | Windows `data/`，原件保留；WSL 按需读取 |
| 手册、规则原件与采购资料 | Windows `references/`，作为来源而非实时状态 |
| 正式报告与导出产物 | Windows `reports/`、`artifacts/`，注明日期、代码版本及输入来源 |
| 旧方案与过期交付物 | Windows `archive/`，不参与日常维护 |
| 凭据与备份 | 私有目录，不进入 Git；备份独立于日常资料管理 |

工作顺序：原始资料归档到 Windows → WSL 开发/分析 → 保存可复现证据或交付物 → 更新 WSL STATUS 或模块说明中的结论。少量读取可直接访问 `/mnt/d/...`；大量反复处理可按需复制到 WSL 仓库外的临时工作区，记录来源，缓存不作为唯一原件。

Windows 可通过 VS Code WSL 模式或 `\\wsl.localhost\Ubuntu-24.04\home\waterc\workspace\Boom_Birds` 查看/编辑主工程。树莓派目录属于部署现场，更新前比较差异，现场修改取回 WSL。Git 和同机资料目录不能替代独立备份；现有迁移备份保留，清理策略另行确定。

| 路径 | 职责 |
| --- | --- |
| `companion/ros2_ws/src/stereo_depth/` | 自研双目深度程序、标定和小型验证记录；尚未封装 ROS 2 包 |
| `companion/ros2_ws/src/open_vins/` | 个人 OpenVINS fork 子模块 |
| `companion/ros2_ws/src/ego-planner-swarm/` | 个人规划器 fork 子模块 |
| `companion/ros2_ws/{build,install,log}/` | 构建产物，忽略且不跨平台复制 |
| `docs/`、`hardware/` | 当前文档、规则、需求、BOM 与硬件证据 |
| `docs/tasks/`、`px4/manifests/` | 日期化验证记录与来源清单，不作为实时状态 |
| `tools/` | 历史 ULog 分析脚本；旧数据路径待适配 |
| `/home/waterc/PX4-Autopilot` | 独立 PX4 仓库与已有构建目录 |
| `/home/waterc/mavlink` | 独立 MAVLink 仓库，不等同于 PX4 自带依赖 |
| `gmaster@192.168.137.200:/home/gmaster/boom_birds_ws/stereo_depth` | Pi 5 既有部署位置，连接前核验地址和远端改动 |

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

## 文档维护

- 本页维护目标、架构与稳定基线；[STATUS](docs/STATUS.md) 维护当前能力、阻塞和下一步；AGENTS 只维护工作规则。
- 安装、运行和排障写在模块旁边。一个事实只维护一处，其他页面链接引用；不重复记录实时 Git 状态、目录树和环境检查结果。
- 需求表只在验收要求或结果变化时更新，BOM 只在物料或证据变化时更新；普通文档改动不要求同步所有文件。
- 重要工程取舍追加到 [决策记录](docs/DECISIONS.md)。日期化任务、原始规则与测试记录按需查阅，不作为当前状态，也不要求每轮创建交接文件。
- 技术结论按实测证据、当前厂商/源码资料、最新有效决策、工程基线、历史估算的顺序核对，保留各自适用条件；规则适用顺序以 RULE_BASELINE 为准。
