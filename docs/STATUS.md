# 当前状态与下一步

更新：2026-10-18。完整机基础飞行已完成；相机—IMU开发标定候选已求解并部署配置，真实 VIO 运行和 Pi 全导航共载尚未验收。

本页维护当前事实、缺口和执行顺序。逐次命令、PASS/FAIL、输入与版本保留在[验证记录](#验证记录)；历史结果只覆盖当次配置和负载。

## 当前路线与验收缺口

正式开发根：WSL Ubuntu-24.04 `/home/waterc/workspace/Boom_Birds`。源码与资料分工见 [README](../README.md#源码与资料位置)，构建与部署见 [ROS 工作空间](../companion/ros2_ws/README.md)。

导航主线：左目＋飞控 IMU → 单目 OpenVINS；同帧双目独立计算深度 → 局部地图/EGO → Companion 位置/速度闭环 → MAVROS 姿态/推力 → PX4。硬件固定，先验收全链算力预算，再验收定位与导航质量（[D-060](DECISIONS.md#d-060固定硬件单目-vio-与双目测深分工)）。

实机采用 `companion_attitude`（[D-058](DECISIONS.md#d-058实机入口采用-companion-位置闭环与-px4-姿态闭环)）；旧 `px4_position` 保留。姿态路线需核实机体安装外参与姿态参考，不要求 VIO/PX4 位置原点一致，也不回传外部视觉。

| 环节 | 已有证据 | 未关闭项 |
| --- | --- | --- |
| 分包、会话、取消屏障、运行配置 | WSL 脱机 PASS；nav 只转发 | 软件证据不包含设备实时性或飞行 |
| 完整机基础飞行 | 用户确认正常飞行；log_27～log_44 已分析 | 定点精度、独立停桨/失联接管、电池检查旁路与供电问题未关闭 |
| 双目采集与深度 | 10-07 台架 1800.17 s、10/10 启停及当次性能门限 PASS；10-17 更新默认标定和显示方向 | 新配置独立距离尺测、曝光时延、有效 VIO/地图/规划共载未验收 |
| 飞控 IMU 与时间门控 | 10-15 精简遥测档下约 50 Hz；共享双目＋IMU短测 PASS | 正式导航遥测预算、曝光与采样时域核验、长测未完成 |
| 相机—IMU标定 | 10-18 联合求解退出 0；选定开发候选并同步 WSL/Pi 配置 | PROVISIONAL_REQUIRES_REVIEW；随机游走为工程假设，动态残差偏大，无独立留出；T_I_B 未测 |
| 真实 OpenVINS | 单目算力配置已准备；配置文件和环境读取 PASS | 实际节点加载、真实初始化、视觉更新、漂移、重置 NOT RUN |
| EGO 与 px4_position | 固定场景 SIH 到目标/降落已有 PASS | 真实 VIO/地图、机体包络净空、跨场景实机闭环 NOT RUN |
| companion_attitude | 控制、预算、起降与故障处置有 WSL/MAVROS/SIH 证据 | 实测机体/推力、姿态参考、独立接管、Pi 全链与实机闭环 NOT RUN；完整姿态森林矩阵未跑 |
| 下视标定与平台捕获 | IMX219 40 组真实样本形成内参候选并部署；70 mm Tag/局部特征为 TEST-ONLY 候选 | 安装外参、新姿态/距离复核、0.5～1.5 m 全捕获区间未通过；像素跟踪未接下降许可 |
| 平台速度交接与计算释放 | 脱机、回放、合成输入 ROS/MAVROS/SIH 已执行 | 真实速度确认适配器 BLOCKED；完整导航往返、真实进程释放和落点精度 NOT RUN |
| 比赛任务与整机约束 | 规则、需求与硬件基线已有记录 | CAD/质量、推力台、MTF-02P 故障独立性、检测跟踪、返库卸载、充电循环、30 s/10 s 及整机合规未关闭 |

完整机基础飞行记录的起飞质量为用户提供的 330 g；它与 README 的比赛设计基线分开记录，不能据此宣称比赛质量/尺寸通过。具体条款见 [RULE_BASELINE](RULE_BASELINE.md)，验收项见 [REQUIREMENTS](REQUIREMENTS.md)。

## 当前配置与标定

| 用途 | 选择入口 | 状态与限制 |
| --- | --- | --- |
| 独立深度预览/模块默认标定 | `stereo_depth/calibration/live_20261017_141337_524478/candidate.npz` | 27 组真实棋盘，含 5 组留出；基线 67.7368 mm；默认加载与几何检查通过，独立尺测未完成 |
| 本次 VIO 开发候选 | WSL `/home/waterc/boombirds-hardware/current-vio`；Pi `/home/gmaster/boombirds/hardware/current-vio` | 均选择 `usb-stereo-vio-20261018-01`；包含同批 mono-budget 与 stereo.npz，基线 67.4664 mm；不与上一行混用 |
| Pi 运行环境 | `/home/gmaster/boombirds/current/activate.sh` | 已接入选定 VIO 的 calibration.env；18 个配置文件 hash 一致、环境读取 PASS；本次未启动 VIO |
| 单目/深度算力候选 | `mono_budget`，共享解码＋64 视差，按需 Z | 左目 640×480、100 跟踪点、6 克隆、0 长期地标；完整有效 VIO 共载未验收 |

VIO 选择清单：[selected-calibration.json](/home/waterc/bb_data/vio-calibration-20261017/selected-calibration.json)。部署证据：[pi-vio-deployment-20261018.json](/home/waterc/bb_data/vio-calibration-20261017/pi-vio-deployment-20261018.json)。配置选择/运行时核对步骤见[感知包](../companion/ros2_ws/src/boom_birds_sensing/README.md#已选开发配置)。

session-05 因 IMU 更新中断提前停止，原始 ABORTED/REJECTED 保留；用户已明确采用，豁免仅限 recording_completed/no_abort/duration。10 分钟静止 IMU 只得到白噪声候选，两个实测 random_walk 仍为 null；本次求解另记录工程权重假设。候选保持 hardware_verified=false，机体外参使用 extrinsics.pending.yaml，不解除控制或飞行闸门。

## 下一步

1. 用现有数据检查选定配置的 OpenVINS 实际加载、初始化、视觉更新、输出与退出；保留标定候选质量限制。当前授权仅用已有数据，配置部署不包含新增采集或硬件测试。
2. 后续设备验证先测有效左目＋IMU VIO与双目深度、EGO、控制计算共载。记录正式遥测档、分段时延、消息年龄、CPU/RSS及退出；无 IMU 或仅启动节点的共载不能代替本项。
3. 全链算力门限：四核整机 CPU P50/P95 ≤70%/85%，深度 ≥15 Hz，年龄 P95/最大 ≤150/300 ms，连续 1800 s 与 10 次启停。此前单项台架结果不覆盖新标定/新负载。
4. 算力满足后验收真实 VIO 米制方向、漂移/重置、独立距离尺测、位姿—深度—地图配对和障碍漏检。质量后验收不等于取消飞行许可条件。
5. 按[实机入口](../companion/ros2_ws/src/boom_birds_bringup/README.md)补齐 T_I_B、机体/推力参数、姿态参考与 PX4 独立安全接管，再依次开展脱桨、系留和短距离闭环；旧位置路线另需航向与原点证据。
6. 平台线补齐下视外参、捕获区间与真实速度回读适配器，再验证往返、末段输出互斥、真实计算释放及近地 Land。CAD/质量、供电、安全光流和停桨证据可并行补齐；比赛完整任务随后集成。

## 验证记录

| 记录 | 查阅内容 |
| --- | --- |
| [10-18 交付前脱机验证](history/STATUS_2026-10.md#2026-10-18-交付前脱机验证) | 18 包构建、13 组脱机检查、失败修复及安装核对 |
| [2026-10 验证与开发记录](history/STATUS_2026-10.md) | Pi 部署/性能、MAVROS、平台、Gazebo、基础飞行和标定 |
| [2026-09 验证与开发记录](history/STATUS_2026-09.md) | 拆包、生命周期、SIH 矩阵、失败与恢复、历史计划 |
| [10-18 VIO 配置部署](history/STATUS_2026-10.md#2026-10-18-按已选标定接入-vio-配置) | 配置选择、路径/hash、环境读取；节点运行 NOT RUN |
| [10-18 相机—IMU候选](history/STATUS_2026-10.md#2026-10-18-采用已有数据的相机imu-标定候选) | 联合求解、残差、工程假设与候选限制 |
| [10-18 Kalibr 与 IMU 噪声](history/STATUS_2026-10.md#2026-10-18-kalibr-环境与-10-分钟-imu-噪声) | 工具/真实 bag 读取、部分噪声估计 |
| [10-17 基础飞行](history/STATUS_2026-10.md#2026-10-17-完整小飞机实机试飞) | 机体配置、ULog、飞行范围与未关闭问题 |
| [10-15 真实 IMU 与双目共载](history/STATUS_2026-10.md#2026-10-15-静态-imu-链路修复与双目共载) | 精简遥测、同步、频率、负载及恢复回读 |
| [10-15 单目优化分项](history/STATUS_2026-10.md#2026-10-15-单目优化-pi-分项测试无飞控) | 共享解码/64 视差选择与无飞控边界 |
| [10-07 深度台架长测](history/STATUS_2026-10.md#2026-10-07-pi-5-感知性能优化与复验) | 1800 s、10 次启停与当次性能口径 |

历史记录正文保留当时结论。例如 10-17 的“Kalibr 缺失/求解 NOT RUN”已由 10-18 的求解与部署记录接续；不能用这些旧句覆盖本页状态，也不能将后续成功回写到原失败记录。
