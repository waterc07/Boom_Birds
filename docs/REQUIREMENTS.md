# REQUIREMENTS

版本：0.12  
基线日期：2026-09-22（规则依据仍为 2026-09-09 前瞻手册）

2026-10-22：按 D-061 同步 NAV-004/CMP-001 的双目 VIO 输入；固定 Pi 5 与现有相机，保留验收结果。其余 Result 保留各需求的验收口径；模块软件进度见 [STATUS](STATUS.md#当前路线与验收缺口)，不以构建或仿真代替整机验收。

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
| FC-003 | P1 | ACCEPTED | PX4 与 Companion 通信应支持飞控 IMU/状态上行及控制下行；协议、消息、速率和时间同步随控制接口设计确定，MAVLink2/UART 仅为参考。 | 长时间链路压力测试 | NOT STARTED |
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
| NAV-004 | P2 | ACCEPTED | 主定位使用同帧左右目图像与飞控 IMU 输入双目 OpenVINS；这对图像另行计算稠密深度；下视相机用于软件光流/降落 Tag 等扩展，不等同于独立安全光流。 | 相机与算法 benchmark | NOT STARTED |
| NAV-005 | P2 | ACCEPTED | 前视相机应支持目标检测、跟踪和终端视觉伺服，最终 SKU 由实测选择。 | 端到端延迟/FPS/功耗测试 | NOT STARTED |
| NAV-006 | P1 | PROVISIONAL | 前向 8×8 多区 ToF 以短程障碍走廊检查为目标，不要求稠密 3D 建图。 | 场景覆盖测试 | NOT STARTED |
| NAV-007 | P1 | ACCEPTED | Pi 5 验证平台应提供 USB 同帧双目原图调焦和 11×8 内角点、20 mm 棋盘的视频引导标定，自动保存角点并报告独立留出姿态误差。 | 原图/引导标定已实测；默认标定的采样、留出几何与部署证据见 [STATUS](STATUS.md#当前配置与标定)，操作见 [标定说明](../companion/ros2_ws/tools/camera_calibration/README.md)；独立距离验证待完成 | IN PROGRESS |

## 5. Companion 与软件需求

| ID | Priority | Status | Requirement | Verification | Result |
| --- | --- | --- | --- | --- | --- |
| CMP-001 | P1 | ACCEPTED | 按 D-061 固定现有 Pi 5 与相机，验收双目 VIO、双目深度、建图、规划与控制共载；更换板卡须另行决策和验证。 | 可复现实验报告 | NOT STARTED |
| CMP-002 | P1 | ACCEPTED | benchmark 至少覆盖整板质量、平均/峰值功耗、启动可靠性、双相机与同步、VIO CPU 性能、NPU 性能、延迟和载板复杂度。 | benchmark schema 审查 | NOT STARTED |
| SW-001 | P1 | ACCEPTED | `Px4Interface` 必须隔离飞控通信后端与算法模块；生产使用 ROS 2 MAVROS，fake 用于脱机，历史 pymavlink 不作生产入口。 | API/依赖审查；MAVROS、会话/取消及包边界的脱机验证见 STATUS，实机接口待验收 | IN PROGRESS |
| SW-002 | P1 | ACCEPTED | 软件应保留 SensorHub、StateEstimator、StereoDepth、LocalMapping、LocalPlanner、TrajectoryExecutor、ControlAdapter、TargetDetector、TargetTracker、LandingDetector、ObstacleSensor、MissionPlanner、EnergyManager 和 FailsafeManager 的清晰边界。 | 架构与依赖审查 | NOT STARTED |

已移除旧 AX630C 专属门槛 CMP-003；该 ID 不复用，旧内容可通过 Git 历史追溯。

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

完整机已完成基础飞行，质量记录为用户提供的 330 g。该证据不关闭 SYS-001/RULE-001/002，也不代替 NAV 或 Offboard 验收；当前范围见 [STATUS](STATUS.md#当前路线与验收缺口)。

## 7. 当前缺失的验证输入

1. MicoAir743v2-AIO-45A 的实物 PCB revision、准确 target 与刷写记录；日志已有 MICOAIR_H743_V2/固件标识，仍须与实板对应核实。
2. MTF-02P 的实物协议模式、安装方向、串口分配、PX4 消息、EKF2 参数及实际地面材质、光照和高度性能。
3. T-MOTOR 1103 8000KV + 2216S-3 + 当前 ESC 在 3S 300 mAh 电池下的完整推力台数据。
4. 实际采购电池的品牌型号、化学体系、满充电压、连接器、内阻、放电曲线与线重；23 g 和约 19×18×50 mm 需复测。
5. AIO、接收机、光流、ToF、相机、Companion、线束、紧固件与结构件的实测质量。
6. 118–120 mm 全包围桨保 CAD、中央硬件堆叠、装配公差、重心、桨尖间隙、刚度和碰撞载荷路径。
7. 固定现有 Pi 5 的有效 VIO/深度/地图/规划/控制全链资源与延迟数据；当前标定候选的独立验证、T_I_B 和最终相机接口需求。
8. Companion 超时阈值、PX4 接管动作及分阶段故障注入测试结果。
9. 后续正式规则、尺寸/重量验收细节、机上储能/电压、保护罩碰撞速度、柔性/悬停判据、30 s计时、库外通信、机库验收及任务判定；见RULE_BASELINE.md第6节。

## 8. 空中机器人完整形态考核

来源：用户于 2026-10-22 提供的“空中系统展示细则”全文摘录；未提供原文件版本及页码。原文两项均标为“选做”，用户明确要求项目默认全部完成，按必做需求纳入；不改写原文的选做属性。以下为展示考核口径，不替代比赛运行规则。

| ID | Priority | Status | Requirement | Verification | Result |
| --- | --- | --- | --- | --- | --- |
| DEMO-001 | P0 | ACCEPTED | 完成 1.2 m 高、距机库 5 m 的小装甲模块目标模拟打击；装甲角度参考规则手册前哨站中部装甲。该项满分 20 分。 | 场地尺寸/角度检查 + 连续展示视频 + 对应实时自动控制证明材料 | NOT STARTED |
| DEMO-002 | P0 | ACCEPTED | 完成 0.9 m 高、距机库 10 m 的小装甲模块目标模拟打击；装甲角度参考规则手册基地顶部装甲。该项满分 20 分。 | 场地尺寸/角度检查 + 连续展示视频 + 对应实时自动控制证明材料 | NOT STARTED |
| DEMO-003 | P0 | ACCEPTED | 两项均在目标前方 1 m 处设置长 3 m 的判定线，垂直于机库与目标的连线，判定线中点位于该连线上。 | 场地测量 + 视频核对 | NOT STARTED |
| DEMO-004 | P0 | ACCEPTED | 每项测试开始前，至多 4 架空中机器人置于机库内；展示用遥控器逐一控制每架机器人，随后将遥控器放在视频画面内且不再使用。 | 连续展示视频核对 | NOT STARTED |
| DEMO-005 | P0 | ACCEPTED | 每项开始后计时 1 min，期间不得有任何人为干预。 | 连续展示视频 + 自动控制记录核对 | NOT STARTED |
| DEMO-006 | P0 | ACCEPTED | 每项 1 min 内只要有任意一次有效冲击，即以有效冲击次数作为项目成绩；无有效冲击时，按越过判定线的架次计分，每架次 1 分，最高 4 分。同一架越线后返航降落在机库并再次起飞越线，计为新 1 架次。 | 视频逐事件计时/计数 + 飞行记录核对 | NOT STARTED |
| DEMO-007 | P0 | ACCEPTED | 每项结束后提交飞行过程中实时自动控制产生的证明材料，且材料须与视频中的飞行过程一致。 | 证明材料与视频逐段对应审查 | NOT STARTED |

摘录未定义有效冲击的具体判据、有效冲击次数与 20 分满分之间的换算，以及装甲角度数值；这些信息待原手册补齐，不自行设定。不将越线保底得分视为满分验收，也不将两项必做解释为必须使用 4 架机器人。

## 9. 无 EGO 的分阶段飞行验证

来源：用户 2026-10-26 指定的三项任务。前视双目朝前，另有下视相机；每项须有独立任务文件和启动入口。以下为开发验证，不替代第 8 节展示考核或赛事时限。

| ID | Priority | Status | Requirement | Verification | Result |
| --- | --- | --- | --- | --- | --- |
| TAG-001 | P0 | ACCEPTED | 地面 tag 上原地起飞、悬停、对准原 tag 并降落。 | 独立启动 + 地面尺度/视频 + 自动控制记录 + 接地和锁定回读 | 70 mm A4／特征 PnP SIH PASS；实机 NOT RUN |
| TAG-002 | P0 | ACCEPTED | tag 上起飞，沿初始前方飞行 1 m；远端停稳后旋转 180°，沿原路返回原 tag 并降落。 | 独立启动 + 外部核对 1 m 位移/180°转向/落点 + 控制记录 | 70 mm A4／特征 PnP SIH PASS；实机 NOT RUN |
| TAG-003 | P0 | ACCEPTED | 起飞后向初始前方 1 m 处的地面 tag 飞行，在指定 tag 上降落；起点不要求有 tag。 | 独立启动 + 目标 ID/位移/落点核对 + 控制记录 | 70 mm A4／特征 PnP SIH PASS；实机 NOT RUN |
| TAG-004 | P0 | ACCEPTED | 前视米制深度发现障碍时制动，最多尝试一次已重新观测的折线路径；无效深度不得判为自由。 | 深度几何/制动实测 + 绕行与未知区故障注入 + 碰撞检查 | 限定场景 SIH PASS；实机 NOT RUN |
| TAG-005 | P0 | ACCEPTED | 绕行失败则按实际飞过的路径返回起点降落；回撤受阻、定位失效或预算不足时采用后备 Land，不盲目返航。 | 实际轨迹对照 + 回撤二次受阻/断流注入 + 接地和锁定回读 | 限定场景 SIH PASS；实机 NOT RUN |
| TAG-006 | P0 | ACCEPTED | A4 单码 tag36h11 ID7、黑框 70 mm；完整解码确认后，以固定平面特征对应、LK、多点 PnP 和同采样测距/姿态支持下降。边框消失可续跟，身份/点/几何失效撤销许可，不以跟踪证明接地。 | 边框消失 SIH + 过期/对应篡改/退化反例 + 相机/测距与落点实测 | 脱机与限定 SIH PASS；实机 NOT RUN |
| TAG-007 | P0 | ACCEPTED | Tag 必须经过身份、质量、时间新鲜度和连续观测检查；单帧检测不得触发交接或下降。确认速度控制、目标与状态回读、飞控 epoch 一致前必须保留 VIO；交接超时、回读不符或反馈过期不得释放计算或继续下降。 | 交接状态机与节点/SIH 行为测试 + 控制输出互斥检查 + 故障注入 | 交接场景 SIH 有证据；真实控制交接未验收，见 [STATUS](STATUS.md#实现默认入口部署与验收) |
| TAG-008 | P0 | ACCEPTED | 交接确认后 VIO 及其专属计算进程必须实际退出并回读；清理只作用于任务自有进程，不得误停共享深度、通信、测距节点。近地由 PX4 Land 最终接管，任务成功需 PX4 接地与 armed=false 回读。 | 自有进程退出/无关进程存活/CPU 释放证据 + 接地 disarm 回读 | ARM 真实进程手动释放已有证据；Tag 触发释放和接地闭环未验收，见 [STATUS](STATUS.md#实现默认入口部署与验收) |
| TAG-009 | P0 | ACCEPTED | 特征有效时用特征目标，失效后按最后有效视觉目标与飞控 EKF XY 的配对参考持续降落，不设置固定切换高度；测距失效后使用配对的 EKF 垂直位移延续高度，EKF 失效或重置时撤销参考并触发保护。 | 坐标/时序与重置反例 + 特征失效闭环 SIH + 飞控估计、测距、接地与落点实测 | 离线/SIH 及 Pi dry-run 桥接已实现并设为正式部署；ARM 568 项末次结果 PASS；真实输入、正式控制与实飞 NOT RUN |

高度、限速和误差容差是任务文件中的开发参数，尚非实机通过证据；真实验收需独立尺度与连续视频，不只使用 VIO 自报位置。
