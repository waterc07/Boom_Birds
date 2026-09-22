# PROJECT_CONTEXT

版本：0.9  
基线日期：2026-09-21  
来源：用户项目上下文、2026-09-09官方书面规则前瞻，以及未被新文明确覆盖的2026-08-15截图原规范；适用顺序见 RULE_BASELINE.md。本文中的器件参数尚未逐项对照厂商资料或实测，相关条目均需按标记核验。

## 当前工程进度入口

当前可用代码、固件、索引与尚未实现能力集中在 [CURRENT_STATUS.md](CURRENT_STATUS.md)。本文件保存架构基线；下文带日期的设备报告按当时证据理解，不覆盖最新导入记录。软件离线开发可与硬件证据补齐并行，不能跳过硬件验收。

## 1. 项目目标

建立一套百克级、全包围四旋翼验证平台，使用 PX4 飞控与独立任务计算机，实现稳定飞行、机载视觉、自主定位/追踪、返库和后续能源方案切换能力。

当前目标不是一次完成最终比赛机，而是优先形成一台：

- 可稳定人工飞行；
- 可记录完整飞行和动力日志；
- 可接入独立光流与下视测距；
- 可通过 MAVLink2 执行基础 Offboard；
- 可持续更换视觉计算平台和能源模块；
- Companion 故障时仍由 PX4 保持安全；
- 严格满足 `RULE_BASELINE.md`；新文未述项沿用原规范，均未定义者保持TBD并可配置。

## 2. 状态词定义

| 状态 | 含义 |
| --- | --- |
| LOCKED | V0 已锁定；只有新证据或约束冲突才能变更，变更必须新增决策记录。 |
| CURRENT BASELINE | 当前工程实施基线；允许通过 CAD、数据手册或实测修订。 |
| CANDIDATE | 候选方案，尚未完成选型验证。 |
| HISTORICAL | 历史研究输入，不得当作当前实施要求。 |
| TBD | 尚无足够信息或尚未设计。 |

## 3. 当前 V0 工程基线

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
| Companion | CANDIDATE | K230 与 AX630C/MaixCAM2 实测比较后决定 |
| Companion 开发环境 | CURRENT BASELINE | Raspberry Pi 5（Pi 5），Ubuntu Server 24.04；主机名与用户名均为 `gmaster`；ROS 2 开发基线为 Jazzy，安装状态待核验。当前使用 Pi 5 验证双目转深度，后续迁移到 RK3576；具体 RK3576 板卡与系统 TBD；详见 `COMPANION_DEV_ENV.md` |
| 当前双目深度输入 | CURRENT BASELINE | 用户确认采用 USB 免驱双目摄像头，左右硬件同帧同步，单帧输出左右拼接图像；当前任务为双目图像转米制深度图。型号、总分辨率/FPS、像素格式、基线、标定与安装方向 TBD；不由硬件同步推定全局快门 |
| 前视相机 | CANDIDATE | 彩色全局快门，约 0.5–1 MP+、60–120+ FPS、MIPI CSI |
| 下视相机 | CANDIDATE | 单色全局快门，承担软件光流和降落 Tag；辅助定位待验证，当前主定位改为双目 + 飞控 IMU |
| 主定位 | CURRENT BASELINE | OpenVINS；输入当前双目图像与飞控 IMU，输出位置、姿态、速度；尚未集成验收 |
| 路径规划与避障 | CURRENT BASELINE | 自算双目深度 + 里程计建图，使用个人 fork https://github.com/waterc07/ego-planner-swarm；ROS 2 分支与提交待兼容验证后锁定 |
| 补充避障传感器 | CANDIDATE | 8×8 multi-zone ToF 类传感器，不替代双目建图与规划主线 |
| 最终能源 | TBD | 高概率超级电容 + 独立电容管理模块 |
| 撞击/拦截结构 | CANDIDATE | 必须覆盖直接撞击与主动迎击能力方向；具体判定、载荷路径和实现待细则与实测 |

45A 是 AIO 的工程冗余选择，不代表单路电机设计电流为 45 A。

比赛约束：全自动、持续悬停至少10 s、柔性桨、紧急停桨遥控器/接收机和安全员、禁止机载发射机构；30 s单次起飞按旧规范沿用。侦察信息在返库后回传；库外通信细则TBD。人工飞行属于研发验证阶段，不作为比赛运行方式。机库300 Wh/30 V不适用于机上，机上能源上限待定。

## 4. 系统架构边界

2026-09-21 用户确认路线（D-024）：Ubuntu 24.04 + ROS 2（沿用 Jazzy 开发基线），OpenVINS + 自算双目深度 + 个人 EGO-Planner fork。选型确定不等于软件、硬件或飞行验收通过。

```text
双目相机 -> 统一采集 / 左右拆分 / 时间戳 / 标定
              |-> 左右图像 + 飞控 IMU -> OpenVINS -> 位置 / 姿态 / 速度
              |-> 极线校正 / 双目匹配 -> 米制深度 / XYZ
                                      |
                          深度 + 对应时刻位姿 -> 局部地图
任务目标 -> EGO-Planner（个人 fork） <--- 地图 + 里程计
              |
           时间参数化轨迹 -> 轨迹执行 -> 控制接口 -> Px4Interface -> PX4 -> ESC
                                                       ^          |
                                                       |-- IMU/状态回传 --|
MTF-02P 光流/测距 -----------------------------------------------> PX4
```

定位使用图像和飞控加速度/角速度测量，不以稠密深度或飞控融合姿态代替输入。通信层同时负责 IMU/状态上行与控制下行。飞控双 IMU 中的具体来源、采样率、传输协议、采样时间戳映射、相机—IMU 外参及时间偏移仍待定义和验证。

控制器位置仍为 TBD：机载位置控制器输出姿态/推力，或由 PX4 内部位置控制器执行高层设定值。ROS 2 通信后端、是否向 PX4 回传外部视觉及对应融合配置随此项设计确定；既有 MAVLink2/UART 仅保留为通信参考基线，不视为本轮最终选定协议。

规划仓库固定为 https://github.com/waterc07/ego-planner-swarm ，不自动改用上游。2026-09-21 `git ls-remote --heads` 确认存在 `ros2_version`（`a3e14dd1ec3dbcec4619ccc9049b888bbcdcee6d`）及 `ros2_lyrical`（`607bfef550f775e88f0b586d16026ab54623e015`）。它们是查询快照，不是锁定依赖；优先核对 `ros2_version` 对 Jazzy/ARM64 的兼容性，不能按分支名认定通过。OpenVINS 来源为 https://github.com/waterc07/open_vins ，版本提交待验证后锁定。

不可违反的分层原则：

1. 任务计算机不直接输出电机 PWM 或 DShot。
2. 算法模块不直接依赖串口；统一通过 `Px4Interface` 访问飞行状态和 setpoint。
3. Companion 超时后停止激进任务，按剩余定位能力由 PX4 执行经验证的接管动作；不能默认定位失效后仍能悬停，阈值和行为待验证。
4. 主 VIO 与安全光流/ToF 位于不同故障域。
5. Linux、相机或 AI pipeline 失效不得导致飞行器持续盲冲。

建议 Companion 模块边界：

```text
SensorHub
StateEstimator (OpenVINS)
StereoDepth
LocalMapping
LocalPlanner (EGO-Planner)
TrajectoryExecutor
ControlAdapter
Recorder
TargetDetector
TargetTracker
LandingDetector
ObstacleSensor
MissionPlanner
Px4Interface
EnergyManager
FailsafeManager
```

## 5. 感知基线

### 5.1 当前双目深度工程输入（2026-09-11）

用户确认：树莓派采用 Pi 5，系统按既有已确认发行版记录为 Ubuntu Server 24.04（本次表述为“linux2404server”）；相机为 USB 免驱双目，硬件同帧同步，输出左右拼接图像。2026-09-11 此项依据为用户报告；2026-09-15 已导入标定、样例及深度历史运行记录，见 CURRENT_STATUS。距离精度与现场长期性能仍需独立验证。平台分工已由用户明确：当前使用 Pi 5 验证，RK3576 为后续迁移计算平台；具体 RK3576 板卡、系统和部署验证仍 TBD。

当前处理链：一帧 USB 拼接图像 → 按实际布局拆分左右目 → 使用双目标定参数去畸变/极线校正 → 双目匹配得到视差 → 转换为米制深度图与有效性标记。无需按两路独立相机流设计采集同步；硬件曝光同步精度仍需资料或实测佐证。输出为距离数据，彩色预览仅用于显示。

2026-09-21 已确认该双目同时服务 OpenVINS 定位与深度分支，安装方向、快门和运动场景适用性仍需验证。Pi 5 验证、后续迁移 RK3576 的平台分工沿用；历史输入见 D-015、D-016，当前定位/规划选择以 D-024 为准。

### 5.2 当前定位与任务分工（2026-09-21）

主定位链路：双目图像 + 飞控 IMU → OpenVINS → 建图、规划与控制接口；是否另向 PX4 EKF2 融合输入由控制方案确定。  
安全定位链路：MTF-02P 独立光流 + 集成 ToF 测距 → PX4。  
前视任务链路：目标检测、跟踪、终端视觉伺服，必要时辅助 VIO。  
返库链路：下视相机识别 AprilTag/landing Tag，并由下视距离传感器辅助低高度控制。  
避障链路：自算双目深度 + OpenVINS 位姿 → 局部地图 → 个人 fork EGO-Planner → 轨迹执行与控制接口；多区 ToF 为补充候选。

## 6. 动力与验证指标

以约 160 g 整机作为动力研究参考点（不是重量限制），当前研究目标为：

- 最大推重比不低于 2.5，理想范围 2.5–3.0；
- 总最大静推力目标约 400–480 gf；
- 单电机静推力目标约 100–120 gf；
- 首先覆盖当前 3S 300 mAh 电池的实测工作电压范围；
- 所有指标必须由当前电机、桨、ESC 的实际推力台数据确认。

推力台至少记录电压、DShot/油门命令、电流、功率、推力、RPM、电机温度和 ESC 温度，并形成 `T=f(V,u)`、`P=f(V,T)` 与 `T/P` 数据。

## 7. 能源架构

V0 使用已确定的 3S 300 mAh、95C 电池，记录质量 23 g，外形约 19×18×50 mm。当前尚未确认其化学体系、满充电压和连接器，不得自行写成 LiPo、LiHV 或 XT30。超级电容属于后续研究，不得阻碍当前飞行验证。

未来统一抽象为可替换 Energy Module：

```text
LiPo Module --------------------+
                                +--> controlled power buses
Supercap + Capacitor Manager ---+       |-- ESC bus
                                        +-- regulated logic rail
```

不得默认裸 EDLC 直接连接 ESC。电容管理模块的稳压程度、限流、均衡、保护、遥测和总线策略均为 TBD。未来任务层可优先使用 `remaining_energy_J`，而非传统电池百分比。

## 8. 已核实硬件资料

### MicoAir743v2-AIO-45A

厂商手册确认：STM32H743VIH6、BMI088 + BMI270 双 IMU、SPL06 气压计、microSD、7 路 UART、8 路电机输出、1 路 I²C、3–6S VBAT、5 V/2 A 与 12 V/2 A BEC、4×45 A AM32 电调，支持 DShot600、双向 DShot 和 PX4。板体 36×36×8 mm、10 g，安装孔距 25.5×25.5 mm，要求 45° 安装。

资料来源：<https://micoair.cn/zh/docs/flight-controller/micoair743-aio-series/micoair743v2-aio-45a-manual>

### MTF-02P

厂商手册确认：光流测距一体设计，5 V 供电、平均 100 mA、LVTTL 3.3 V 串口、115200 baud、50 Hz；尺寸 21.6×16×6.5 mm、重量 1.5 g；MTF-02P ToF 在 90% 反射率条件下标称 0.02–6 m，提供 `Mavlink_px4` 和默认 `AUTO` 协议模式。实际安装方向、串口占用、协议模式、EKF2 参数和真实场景性能仍需实板验证。

资料来源：<https://micoair.cn/zh/docs/sensors/sensors/mtf-02-02p-sensors>

## 9. 历史方案与明确排除项

- HISTORICAL：118 mm、165–168 g 激进尺寸方案；仅用于后续 CAD/规则优化，不是 V0 硬指标。
- HISTORICAL：4×100 F/3.0 V EDLC、约 0.342 Wh、30 s 平均母线功率约 32 W 的理论估算；仅作可行性 benchmark。
- HISTORICAL：裸超级电容宽电压直接驱动 ESC；已被模块化 Energy/Capacitor Manager 方向替代。
- 非主路线：上下错层大桨；除非 CAD、质量和气动实测证明收益，否则保持同平面布局。
- 排除：DJI A3、VOXL、RK3588 作为最终比赛机、任务 Linux 计算机直接控制电机。
- RK3588 只可作为算法开发和性能剖析平台。

## 10. 实施顺序

软件主线：版本与接口核对 → 共享双目/飞控 IMU 数据记录与回放 → OpenVINS、深度与通信分别验证 → EGO-Planner 仿真闭环 → Pi 5 全链路无桨验证 → 受控飞行 → RK3576 单独迁移验收。具体下一步见 NEXT_TASK。

硬件主线保留 PX4/AIO bring-up、动力台架、人工稳定飞行、独立光流/测距与安全接管前置证据。Tag、目标检测跟踪和能源管理在导航基础上扩展。软件仿真不替代硬件或飞行验收。

## 11. 证据优先级

规则合规先按 RULE_BASELINE.md 的新旧规则适用顺序判断，工程证据不得放宽规则。技术性能与硬件事实发生冲突时按以下顺序判断：

1. 实机测试日志、推力台数据和称重结果；
2. 当前硬件的数据手册、原理图、PX4 源码/官方板级支持；
3. 最新带日期的 `DECISIONS.md` 记录；
4. 当前工程基线；
5. 历史理论估算与历史方案。
