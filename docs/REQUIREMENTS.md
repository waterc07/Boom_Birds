# REQUIREMENTS

版本：0.6  
基线日期：2026-09-09

状态：`ACCEPTED` 已纳入基线；`PROVISIONAL` 依赖规则或实测；`TBD` 尚未定量。验证：`NOT STARTED`、`IN PROGRESS`、`PASS`、`FAIL`。

## 1. 系统与安全需求

| ID | Priority | Status | Requirement | Verification | Result |
| --- | --- | --- | --- | --- | --- |
| SYS-001 | P0 | ACCEPTED | V0 应为118–120 mm外包络的全包围桨保四旋翼；整机重量不另设工程硬上限，只能按实际尺寸满足当前规则：`150 g ≤ m ≤ 322.5-1.225D`。 | CAD 包络检查 + 完整起飞质量称重 + 规则公式校核 | NOT STARTED |
| SYS-002 | P0 | ACCEPTED | 飞控与任务计算必须分层，Companion 不得直接控制电机。 | 架构审查 + 接口测试 | NOT STARTED |
| SYS-003 | P0 | ACCEPTED | Companion 故障或通信超时后，PX4 必须停止激进任务并进入可验证的安全动作。 | 故障注入飞行/系留测试 | NOT STARTED |
| SYS-004 | P0 | ACCEPTED | V0 必须保存足以复盘姿态、估计器、RC/Offboard、执行器、电源和 failsafe 的日志。 | PX4 ULog 字段审查 | NOT STARTED |
| SYS-005 | P0 | ACCEPTED | 2026-09-09 书面规则优先；未述/未明确项沿用原规范，均未定义者 TBD；来源与继承关系见 RULE_BASELINE.md。 | 规则追踪矩阵与差异审查 | NOT STARTED |

## 2. 飞控与通信需求

| ID | Priority | Status | Requirement | Verification | Result |
| --- | --- | --- | --- | --- | --- |
| FC-001 | P0 | ACCEPTED | 已购入的 MicoAir743v2-AIO-45A 必须确认准确 PX4 board target、固件刷写流程、引脚图和关键接口。 | 厂商资料/PX4 源码核对 + 实板刷写 | IN PROGRESS |
| FC-002 | P0 | ACCEPTED | ESC 执行链路采用 DShot600 基线，并确认四路输出映射与电机顺序。 | 无桨台架测试 + PX4 actuator test | NOT STARTED |
| FC-003 | P1 | ACCEPTED | PX4 与 Companion 首选 MAVLink2 over UART；开发波特率目标 921600，须以链路误码和负载实测确认。 | 长时间链路压力测试 | NOT STARTED |
| FC-004 | P1 | ACCEPTED | 基础 Offboard 应支持状态读取、心跳、模式、解锁和高层 setpoint，不包含直接电机命令。 | SITL + 实机受限测试 | NOT STARTED |
| FC-005 | P1 | PROVISIONAL | 轨迹/速度 setpoint 目标 50–100 Hz，终端视觉伺服目标约 100 Hz；实际频率由端到端延迟测试确定。 | 时间戳/丢包/抖动测试 | NOT STARTED |

## 3. 动力与电源需求

| ID | Priority | Status | Requirement | Verification | Result |
| --- | --- | --- | --- | --- | --- |
| PWR-001 | P0 | ACCEPTED | V0 使用 4×T-MOTOR 1103 8000KV、Gemfan 2216S-3 和 3S 300 mAh、95C、23 g 电池。 | BOM/到货/装配审查 | IN PROGRESS |
| PWR-002 | P0 | PROVISIONAL | 单电机最大静推力目标 100–120 gf；整机最大推重比不低于 2.5。 | 校准推力台测试 | NOT STARTED |
| PWR-003 | P0 | ACCEPTED | 推力台必须记录电压、命令、电流、功率、推力、RPM、电机温度和 ESC 温度。 | 数据文件 schema + 采样检查 | NOT STARTED |
| PWR-004 | P0 | ACCEPTED | 首轮动力测试覆盖当前 3S 300 mAh 电池的实测工作电压范围，不扩展至超级电容宽电压。 | 测试矩阵审查 | NOT STARTED |
| PWR-005 | P3 | ACCEPTED | 未来能源通过可替换 Energy Module 接入，逻辑电源与动力母线策略不得预设裸电容直连。 | 电气架构审查 | NOT STARTED |
| PWR-006 | P3 | TBD | 超级电容模块须定义单体/总压、电流、温度、均衡、充放电保护、总线策略和通信。 | 原理图审查 + 电源台架 | NOT STARTED |

## 4. 感知与定位需求

| ID | Priority | Status | Requirement | Verification | Result |
| --- | --- | --- | --- | --- | --- |
| NAV-001 | P0 | ACCEPTED | 已购入的 MTF-02P 必须将独立光流和集成 ToF 距离数据直接供 PX4 使用，不依赖 Companion 主视觉进程。 | 断开/杀死 Companion 故障测试 | IN PROGRESS |
| NAV-002 | P0 | ACCEPTED | MTF-02P 使用 5 V 供电、LVTTL 串口，按 PX4 MAVLink/AUTO 模式完成方向、更新率、质量输出、距离量测和 EKF2 配置验证。 | 厂商手册 + 实板消息检查 | IN PROGRESS |
| NAV-003 | P1 | ACCEPTED | OpticalFlow 与 DistanceSensor 在软件和日志中必须作为明确数据源管理。 | 消息/参数/日志审查 | NOT STARTED |
| NAV-004 | P2 | ACCEPTED | 主下视相机应支持 VIO、软件光流和降落 Tag；不得将其等同于独立安全光流。 | 相机与算法 benchmark | NOT STARTED |
| NAV-005 | P2 | ACCEPTED | 前视相机应支持目标检测、跟踪和终端视觉伺服，最终 SKU 由实测选择。 | 端到端延迟/FPS/功耗测试 | NOT STARTED |
| NAV-006 | P1 | PROVISIONAL | 前向 8×8 多区 ToF 以短程障碍走廊检查为目标，不要求稠密 3D 建图。 | 场景覆盖测试 | NOT STARTED |

## 5. Companion 与软件需求

| ID | Priority | Status | Requirement | Verification | Result |
| --- | --- | --- | --- | --- | --- |
| CMP-001 | P1 | ACCEPTED | K230 与 AX630C/MaixCAM2 必须用同一数据集、相机输入和指标进行比较。 | 可复现实验报告 | NOT STARTED |
| CMP-002 | P1 | ACCEPTED | benchmark 至少覆盖整板质量、平均/峰值功耗、启动可靠性、双相机与同步、VIO CPU 性能、NPU 性能、延迟和载板复杂度。 | benchmark schema 审查 | NOT STARTED |
| CMP-003 | P1 | PROVISIONAL | AX630C 路线继续保留的研究门槛为计算模块约不超过 10 g、任务平均功耗约不超过 2.5 W、稳定双相机同步。 | 实测 | NOT STARTED |
| SW-001 | P1 | ACCEPTED | `Px4Interface` 必须隔离 MAVLink transport 与算法模块。 | API/依赖审查 | NOT STARTED |
| SW-002 | P1 | ACCEPTED | 软件应保留 StateEstimator、TargetDetector、TargetTracker、LandingDetector、ObstacleSensor、MissionPlanner、EnergyManager 和 FailsafeManager 的清晰边界。 | 架构与依赖审查 | NOT STARTED |

## 6. 规则与机械需求

| ID | Priority | Status | Requirement | Verification | Result |
| --- | --- | --- | --- | --- | --- |
| MEC-001 | P0 | ACCEPTED | 2.2 inch 桨、外围桨保、桨尖间隙、中央硬件堆叠和前向撞击结构必须在118–120 mm三维包络内完成干涉及极限公差检查，不得只按二维直径判断。 | CAD 干涉/公差审查 | NOT STARTED |
| MEC-002 | P0 | ACCEPTED | 完整涵道非强制；任何保护方案均须满足桨叶和机体不外露及任意角度碰撞保护。旧开放式框架待重审。 | CAD/样机审查 + 受载变形检查 | NOT STARTED |
| MEC-003 | P1 | ACCEPTED | 默认使用同平面旋翼；错层布局只有在质量、刚度和气动实测证明收益后才进入候选。 | 设计评审 | NOT STARTED |
| RULE-001 | P0 | ACCEPTED | 按手册第 11 页表 2-8：150≤m≤249 g，且 D≤(322.5-m)/1.225 mm；官方公式已明确，不再使用暂取60–140 mm区间。 | 完整质量称重 + CAD/量规 + 公式复核 | NOT STARTED |
| RULE-002 | P0 | ACCEPTED | 任意情况下长宽高均不超过按质量求出的x；记录L×W×H和最大边D，测量姿态与公差TBD；200 g对应100 mm。 | 全状态CAD包络 + 实物量规 | NOT STARTED |
| RULE-003 | P0 | ACCEPTED | 全包围保护罩覆盖桨叶及机体，不得外露；任何角度和一定水平速度撞击装甲时桨叶不接触装甲；速度与细则TBD。 | CAD极限公差 + 受控结构保护试验（方案待定） | NOT STARTED |
| RULE-004 | P0 | ACCEPTED | 机库具备存储功能，装载上限4架、收纳不超过300×300×200 mm；按实际设计架数验证，不要求必须4架。 | 机库/所装飞机CAD + 实物收纳 | NOT STARTED |
| RULE-005 | P0 | ACCEPTED | 比赛全自动运行并支持从机库起飞；沿用任务后落入场地/返库充电，侦察回传须先返回机库。 | 自主任务场景与返库测试 | NOT STARTED |
| RULE-006 | P0 | ACCEPTED | 沿用2026-08-15旧规范：单次起飞至多30 s；新手册未重述，计时起止及判罚TBD；记录计时并提前安全终止。 | SITL + 受限场地计时 + 日志 | NOT STARTED |
| RULE-007 | P1 | ACCEPTED | 系统应支持反复“降落—充电—起飞”；能源和热设计不得依赖单次任务后人工拆机复位。 | 循环任务测试 | NOT STARTED |
| RULE-008 | P1 | ACCEPTED | 保留撞击、迎击飞镖和弹丸、侦察三类项目能力方向；官方称典型功能，伤害数值/计分TBD；旧42 mm伤害等级细节仅作为继承项。 | 任务架构审查 + 分阶段场景验证 | NOT STARTED |
| RULE-009 | P1 | ACCEPTED | 悬停获取信息，返库后回传并转发雷达等系统；库内有线/近场无线方式沿用；库外通信许可和协议TBD。 | 机载记录 + 返库卸载端到端测试 | NOT STARTED |
| RULE-010 | P0 | ACCEPTED | 原安全绳取消不等于取消失效保护；场内飞行仍受指定部署区和待明确安全边界约束，须由 PX4 执行可验证的通信超时和失控安全动作。 | 安全审查 + 故障注入测试 | NOT STARTED |
| RULE-011 | P0 | ACCEPTED | 持续悬停至少10 s（第11页）；漂移/高度容差TBD。 | 受限场地悬停计时 + 日志 | NOT STARTED |
| RULE-012 | P0 | ACCEPTED | 桨叶为柔性材料；核验现有2216S-3，不以型号直接判合规。 | 材料资料 + 实物检查 | NOT STARTED |
| RULE-013 | P0 | ACCEPTED | 配套紧急停桨遥控器/接收机并配备安全员；停桨链路独立验证。 | 无桨紧急停桨测试 + 职责记录 | NOT STARTED |
| RULE-014 | P0 | ACCEPTED | 空中机器人不得安装发射机构。 | BOM与整机结构审查 | NOT STARTED |
| RULE-015 | P0 | ACCEPTED | 机库最大供电总容量300 Wh，内部任意时刻任意一点对电池负极≤30 V；不得作为机上能源上限。 | 能源清单 + 电气审查 + 台架测量 | NOT STARTED |
| RULE-016 | P1 | ACCEPTED | 机库运行方式不限，最多1个调试遥控器；若提供初始动力，方向遵守竖直法线15°张角以内，测法TBD。 | 设备清单 + 初始动力方向审查 | NOT STARTED |
| RULE-017 | P0 | ACCEPTED | 机库仅安装于战场中央指定红/蓝机库放置区，按图3-4及后续场地图核对。 | CAD安装界面 + 场地检查 | NOT STARTED |
| RULE-018 | P1 | ACCEPTED | 若采用自定义终端：30 V、收纳600×600×600 mm、不得使用无线收发装置；RJ45有线、UDP图传、MQTT+ProtoBuf赛事数据。 | 终端电气/尺寸/通信审查 | NOT STARTED |
| RULE-019 | P1 | ACCEPTED | 感知与规划环境纳入制高点、隧道、工作台和中央机库区；增益适用性及几何缺失项不自行补全。 | 地图与场景数据集审查 | NOT STARTED |

## 7. 当前缺失的验证输入

1. MicoAir743v2-AIO-45A 的实物 PCB revision、准确 PX4 target、固件版本和刷写记录。
2. MTF-02P 的实物协议模式、安装方向、串口分配、PX4 消息、EKF2 参数及实际地面材质、光照和高度性能。
3. T-MOTOR 1103 8000KV + 2216S-3 + 当前 ESC 在 3S 300 mAh 电池下的完整推力台数据。
4. 实际采购电池的品牌型号、化学体系、满充电压、连接器、内阻、放电曲线与线重；23 g 和约 19×18×50 mm 需复测。
5. AIO、接收机、光流、ToF、相机、Companion、线束、紧固件与结构件的实测质量。
6. 118–120 mm 全包围桨保 CAD、中央硬件堆叠、装配公差、重心、桨尖间隙、刚度和碰撞载荷路径。
7. K230 与 AX630C 的同条件 benchmark 数据及最终相机接口需求。
8. Companion 超时阈值、PX4 接管动作及分阶段故障注入测试结果。
9. 后续正式规则、尺寸/重量验收细节、机上储能/电压、保护罩碰撞速度、柔性/悬停判据、30 s计时、库外通信、机库验收及任务判定；见RULE_BASELINE.md第6节。
