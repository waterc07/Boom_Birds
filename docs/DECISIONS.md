# DECISIONS

版本：0.5  
最近更新：2026-08-15

## 记录规则

每次改变 LOCKED 或 CURRENT BASELINE 项时，追加而非覆盖旧记录，并包含：Decision、Date、Status、Reason、Previous option、New option、Evidence、Invalidation criteria。

本文记录的是工程决策，不代表器件参数已经通过厂商资料或实测验证。

## D-001：V0 尺寸与质量基线（已由 D-012 替代）

- Date：2026-08-14
- Status：HISTORICAL（已由 D-012 替代）
- Decision：V0 采用 123–125 mm 外包络级、155–165 g，目标约 160 g。
- Reason：与当前 2.2 inch 动力组和可采购硬件更匹配，优先保证可飞与可迭代。
- Previous option：118 mm、165–168 g 激进优化方案。
- New option：123–125 mm、约 160 g 工程验证平台。
- Evidence：项目交接中记录的最新工程基线；尚待 CAD 与称重闭合。
- Invalidation criteria：正式规则冲突，或 CAD 证明 2.2 inch 全涵道无法在该尺寸/质量内闭合。

## D-002：飞控与任务计算严格分层

- Date：2026-08-14
- Status：LOCKED
- Decision：PX4 承担硬实时飞行与安全；Companion 只承担感知、任务和高层 setpoint。
- Reason：隔离 Linux、视觉和 AI 故障，保证 Companion 失效时仍可安全控制。
- Previous option：上一代 DJI A3；或任务计算机直接控制电机。
- New option：STM32H7/PX4 + 独立 Companion，经 MAVLink2 通信。
- Evidence：项目系统安全原则。
- Invalidation criteria：无；具体硬件和通信实现可替换，但安全边界不得取消。

## D-003：V0 动力组合

- Date：2026-08-14
- Status：LOCKED
- Decision：4 × T-MOTOR 1103 8000KV + Gemfan 2216S-3。
- Reason：对应 123–125 mm、约 160 g 的 2.2 inch 全保护验证机方向。
- Previous option：M1103 11000KV、LAVA 1104 等候选。
- New option：T-MOTOR 1103 8000KV + 2216S-3。
- Evidence：项目交接中的 V0 已定选型；性能尚待推力台验证。
- Invalidation criteria：实测无法达到单电机 100–120 gf、热约束不可接受、接口/几何不兼容，或供应链失效。

## D-004：V0 开发能源

- Date：2026-08-14
- Status：HISTORICAL（已由 D-010 替代）
- Decision：曾计划使用 3S 450 mAh LiHV、至少 90C、XT30。
- Reason：先以稳定电源完成飞控、动力和算法验证。
- Previous option：超级电容直接作为首版能源。
- New option：LiHV 开发电池；超级电容后置研究。
- Evidence：项目阶段划分。
- Invalidation criteria：电池经实测无法满足电流、压降、质量或安全要求；替代品仍须保持稳定开发电源目标。

## D-005：MicoAir743v2-AIO 与 ESC 冗余

- Date：2026-08-14
- Status：LOCKED
- Decision：已实际采用并购入 MicoAir743v2-AIO-45A，运行 PX4，执行链路使用 DShot600。
- Reason：为后续动力与母线实验留工程冗余，避免 AIO 过早成为限制。
- Previous option：35A 与 45A 版本比较。
- New option：优先 45A 版本。
- Evidence：用户确认实际采购；厂商手册确认 PX4、DShot600、主要接口、尺寸和质量。板级 target、刷写和实板运行仍待验证。
- Invalidation criteria：PX4 无可靠板级支持、关键 UART/传感器接口不足、质量/尺寸不闭合，或 45A 版本引入不可接受代价。

## D-006：安全定位独立故障域

- Date：2026-08-14
- Status：LOCKED
- Decision：主 VIO 与独立光流 + 下视 ToF 不共用同一任务计算故障域。
- Reason：Companion、相机或 VIO 失效后，PX4 仍需获得低高度速度/距离观测。
- Previous option：单一下视相机同时承担全部定位与安全能力。
- New option：主下视视觉链路 + 独立安全光流/距离链路。
- Evidence：项目故障隔离原则。
- Invalidation criteria：安全能力可以由另一个同等独立、可验证的传感链路实现；不得仅因减重取消 fallback。

## D-007：Companion 保持实测选型

- Date：2026-08-14
- Status：CANDIDATE
- Decision：K230 与 AX630C/MaixCAM2 均不提前锁定，以实测决定。
- Reason：VIO 依赖 CPU、内存、相机同步和 pipeline，不能仅以 NPU TOPS 选型。
- Previous option：分别存在 K230 主路线与 AX630C 主路线的历史方案。
- New option：建立统一 benchmark 后选择；RK3588 仅作开发平台，VOXL 排除。
- Evidence：当前交接结论。
- Invalidation criteria：某候选无法满足相机、质量、功耗、启动可靠性或延迟门槛，或另一候选形成显著综合优势。

## D-008：超级电容采用模块化能源接口

- Date：2026-08-14
- Status：CURRENT BASELINE
- Decision：未来超级电容通过独立 Capacitor Manager/Energy Module 接入，不在 PX4 或 ESC 架构中假定裸电容直连。
- Reason：最终总线可能稳压、半稳压或仅限流保护，目前尚未确定。
- Previous option：EDLC 宽电压直接连接 ESC，并由 PX4 补偿。
- New option：能源模块抽象 + 受控动力母线 + 稳压逻辑电源。
- Evidence：项目方向的后续修正。
- Invalidation criteria：电源台架证据确认另一拓扑在质量、效率、安全和可控性上更优；仍须保持可替换能源接口。

## D-009：撞击与规则相关功能暂不锁死（已由 D-011 替代）

- Date：2026-08-14
- Status：HISTORICAL（已由 D-011 替代）
- Decision：正式 2027 规则发布前，不锁死撞击机构、判定方式和最终能源形态。
- Reason：现有尺寸—重量公式、30 s 架次、撞击和超级电容要求均属于前瞻信息。
- Previous option：基于历史规则推演具体撞击器或最终能源。
- New option：保持固定前向任务方向与可替换前部结构，但推迟规则耦合设计。
- Evidence：正式规则尚未落地。
- Invalidation criteria：用户提供可作为当前基线的新规则信息并完成逐条需求审查；该条件已于 2026-08-15 满足，现行结论见 D-011。

## D-010：首批实物硬件与开发电池基线

- Date：2026-08-14
- Status：LOCKED
- Decision：实际硬件采用 MicoAir743v2-AIO-45A、MTF-02P、4×T-MOTOR 1103 8000KV、Gemfan 2216S-3；前期电池改为 3S 300 mAh、95C、23 g、约 19×18×50 mm。
- Reason：以已采购或实际采用的硬件替代此前仅基于研究的候选与参考电池。
- Previous option：MTF-02P 为候选；电池为 3S 450 mAh LiHV、至少 90C、XT30。
- New option：MTF-02P 已购入；电池为用户提供的 3S 300 mAh、95C 实物参数，化学体系和连接器待确认。
- Evidence：用户于 2026-08-14 提供的实物采购信息；飞控和传感器参数由微空科技厂商手册核对。
- Invalidation criteria：到货型号与订单不一致，实物称重或尺寸与记录不符，或台架及飞行测试证明硬件不能满足需求。

## D-011：2026-08-15 当前规则成为强制基线

- Date：2026-08-15
- Status：LOCKED
- Decision：用户提供的三张 RoboMaster 2027 规则发布会截图构成当前强制规则基线；在用户提供新规则信息前，项目完成机必须严格符合其中可明确读出的制作、部署、运行与典型功能要求。
- Reason：用户明确要求以当前规则约束完成机，不能继续把尺寸—重量关系、30 s 单次起飞、机库、全包围桨保和任务能力仅视为前瞻假设。
- Previous option：D-009 将尺寸—重量公式、30 s 架次、撞击方式等视为尚未落地的前瞻信息。
- New option：建立 `RULE_BASELINE.md` 和原始截图证据目录；硬约束进入 `REQUIREMENTS.md`，缺失定义标为 TBD 并采用保守工程解释。
- Evidence：用户于 2026-08-15 提供三张规则发布会截图，并明确指示“在我没有提供新的规则信息之前，本项目完成的飞机需要严格符合此要求”。
- Invalidation criteria：仅在用户提供新的规则信息后，通过逐条差异审查修订；不得因实现困难自行放宽。

## D-012：V0 外包络调整为 118–120 mm（重量部分已由 D-013 修正）

- Date：2026-08-15
- Status：HISTORICAL（尺寸结论沿用，重量部分已由 D-013 修正）
- Decision：V0 最大外包络调整为 118–120 mm；质量目标为 155–160 g、工程上限为 165 g。继续使用 2.2 inch Gemfan 2216S-3，并保留规则要求的全包围桨保；完整涵道不再是强制方案，优先评估轻量开放式保护框架。
- Reason：用户明确将尺寸基线定为 118–120 mm，并允许在必要时舍弃涵道。按当前规则工程公式，118–120 mm 对应重量上限约 177.95–175.5 g，当前质量目标仍有规则余量；但2.2 inch桨、保护结构、中央硬件和撞击结构必须通过三维CAD共同闭合。
- Previous option：D-001 的 123–125 mm 外包络、155–165 g 方案，以及默认全涵道方向。
- New option：118–120 mm、目标155–160 g、工程上限165 g，同平面2.2 inch桨与全包围桨保；在满足保护和刚度要求的前提下采用非涵道开放结构。
- Evidence：用户于 2026-08-15 明确指示“把尺寸基线定为118-120”；当前规则尺寸—重量曲线工程校核；已购动力、AIO、电池和MTF-02P尺寸质量记录。
- Invalidation criteria：新规则与该包络冲突；三维CAD无法同时满足桨尖/桨保公差、硬件堆叠、重心和撞击载荷路径；或推力台、称重、结构试验证明当前动力和质量目标不可行。任何变更须追加新决策。

## D-013：重量限制仅按当前规则执行

- Date：2026-08-15
- Status：CURRENT BASELINE
- Decision：保留118–120 mm尺寸基线，但取消155–160 g目标和165 g工程硬上限作为基线约束。完成机重量仅按实际外包络尺寸满足当前规则：`150 g ≤ m ≤ 322.5-1.225D`；118 mm时上限约177.95 g，120 mm时上限约175.5 g。
- Reason：用户明确要求“重量的限制按照规则来”。项目可以为动力、续航或结构优化设置阶段性质量目标，但不得把内部目标写成规则限制或额外验收硬门槛。
- Previous option：D-012 中目标155–160 g、工程上限165 g的内部质量基线。
- New option：尺寸维持118–120 mm；重量合规区间随最终实测尺寸按规则公式动态计算，不设置独立工程硬上限。
- Evidence：用户于2026-08-15的明确指示；`RULE_BASELINE.md`记录的尺寸—重量线性关系。
- Invalidation criteria：用户提供新规则信息并完成差异审查，或规则官方精确公式证明当前工程推导需要修正。任何内部优化目标必须明确标为非规则限制。

## D-014：2026-09-09 官方书面前瞻优先，未述项继承原规范

- Date：2026-09-09
- Status：CURRENT BASELINE
- Decision：在官方后续规则发布前，以“文档”目录的20260909规则变更前瞻手册明确内容为首要依据；未提及或未写明项沿用原规范，两者均未定义者TBD。D-011的来源优先级由本记录更新，历史原文保留。
- Reason：用户明确要求书面规则优先且未述项沿用；手册自身说明仅公布部分关键变更。
- Previous option：2026-08-15截图基线及D-012/D-013尺寸重量工程解释。
- New option：书面公式确认，118–120 mm及对应177.95–175.5 g上限不变，不另设165 g上限；60–140 mm暂取区间改按精确公式。保护罩要求覆盖机体、任意角度碰撞保护，原开放框架须重审；新增10 s悬停、柔性桨、全自动、紧急停桨接收机/遥控器和安全员、禁机载发射机构、机库电气/方向/部署要求。侦察返库后回传；30 s架次及无冲突旧规范继续沿用。
- Evidence：用户2026-09-09指示；官方前瞻手册印刷第5–17页，核心第10–12页；文件SHA256及逐项对应见RULE_REVIEW_2026-09-09.md，旧基线快照见rules/2026-09-09/RULE_BASELINE_before_update.md。
- Impact：同步RULE_BASELINE、REQUIREMENTS、PROJECT_CONTEXT、BOM及项目入口；仅规则/文档核对，未验证实物合规，不更改既有硬件Result为PASS。
- Invalidation criteria：官方发布后续规则文档，逐条差异审查后追加决策；未被覆盖的原项继续继承。缺失定义不得因实现困难自行补全或放宽。

## D-015：Pi 5 与 USB 同帧双目作为当前深度工程输入

- Date：2026-09-11
- Status：CURRENT BASELINE（用户报告，未实机验证）
- Decision：当前树莓派明确为 Raspberry Pi 5，操作系统沿用已确认 Ubuntu Server 24.04；双目相机采用 USB 免驱、左右硬件同帧同步、单帧左右拼接输出。当前研究目标限定为双目图像转米制深度图，并评估 Pi 5 与 RK3576 两个平台。
- Reason：用户要求按实际设备背景筛选工程，并补充缺失项目背景。
- Previous option：树莓派未记录具体型号，前视/下视候选主要按MIPI接口描述，缺少本轮双目采集输入。
- New option：当前深度链采用USB单帧采集、左右拆分、校正、匹配及深度转换。最终机载板、双目安装方向和定位分工未确认，不撤销其他候选，不推定全局快门或标定完成。
- Evidence：用户2026-09-11本任务消息；既有Ubuntu Server 24.04记录见COMPANION_DEV_ENV.md。
- Impact：同步PROJECT_CONTEXT、COMPANION_DEV_ENV和BOM；本次仅补充背景，不将硬件或算法测试标为PASS。
- Invalidation criteria：用户更正设备或用途；相机枚举、资料或实测与报告不一致；后续标定/性能结果要求调整输入或平台。变更追加记录。

## D-016：当前 Pi 5 验证，后续迁移 RK3576

- Date：2026-09-11
- Status：CURRENT BASELINE（用户确认）
- Decision：双目转深度当前使用 Raspberry Pi 5 / Ubuntu Server 24.04 验证；RK3576 为后续需要迁移的计算平台。
- Evidence：用户明确“RK3576仍然为后续需要转移的计算平台，当前使用树莓派验证”。
- Impact：补全D-015中的平台角色，更新PROJECT_CONTEXT、COMPANION_DEV_ENV及Pi 5 BOM条目；工程选型优先满足当前Pi 5验证，并保留迁移接口。具体RK3576板卡、系统与台架性能仍TBD；不将芯片迁移方向当作完整机载硬件验收。
- Invalidation criteria：用户更改平台方向，或后续部署、功耗、质量和性能实测要求调整方案；变更追加决策。

## D-017：本地深度开发副本与 WSL PX4 分工

- Date：2026-09-15
- Status：CURRENT BASELINE（目录整理已实施）
- Decision：深度项目从树莓派完整复制到 `companion/stereo_depth/`，保留内部路径、标定、样例及历史记录；PX4 源码继续使用 WSL 独立仓库，主工程保存固件副本和来源清单。原日志、手册、规则原件不移动。
- Evidence：`docs/workflows/stereo_import_20260915.json` 中 334 个非缓存文件 SHA256 一致；`px4/manifests/wsl_inventory_20260915.json` 中两个固件与 WSL 原件一致，内部 git_hash 与当日源码 HEAD 一致。已有构建不等于本次重新构建。
- Impact：本地可阅读和修改当前深度代码；未实现自动部署或回放入口，不改变距离、性能和飞行验收状态。没有修改树莓派和 WSL 源码，没有刷写。主项目尚未初始化 Git。
- Invalidation criteria：上游设备代码发生新修改时，先比对并合并，不覆盖；平台或目录变更时更新清单和工作流。

## D-018：内层 Boom_Birds 作为正式开发根目录

- Date：2026-09-15
- Status：CURRENT BASELINE（用户明确要求，已实施）
- Decision：开发目录及配置迁入外层下新建的 Boom_Birds/；历史日志、手册、规则原件、分析输出和本机备份留在外层。
- Evidence：移动前后 360 个文件 SHA256 一致，清单位于外层 .local/backups/organization-20260915/nesting-before.json。
- Impact：编辑器打开内层工作区；规则 PDF 引用及日志工具外层路径已调整。未更改树莓派和 WSL，未初始化 Git。深度代码及标定不变。
- Invalidation criteria：需要独立分发开发仓库时，另行安排外部资料获取和数据路径配置；不可默认为外层资料已包含在源码仓库。

## D-019：外层资料分类

- Date：2026-09-15
- Status：CURRENT BASELINE（已实施）
- Decision：外层原日志、手册、文档、analysis、output、tmp 分别归入 data/px4_logs、references/manuals、references/rules、reports/flight_analysis、artifacts/calibration、archive/temporary。
- Evidence：外层 .local/backups/outer-organization-20260915/verification.json。
- Impact：修正规则引用与日志工具路径；不删除原数据，不改变任何硬件验收。旧索引因后台占用保留。
- Invalidation criteria：后续数据管理或独立分发方案改变时更新路径与引用。

## D-020：统一 AI 上下文与 GPT-6 执行约定

- Date：2026-09-15
- Status：CURRENT BASELINE
- Decision：行为规则集中在 AGENTS；新增 CURRENT_STATUS 与 AI_WORKFLOW，更新过时进度和软件候选任务。
- Evidence：当前导入清单、PX4 清单及 CodeGraph 状态；GPT-6 官方参考见 AI_WORKFLOW。
- Impact：只调整文档，不更改模型参数、源码、设备或硬件验收状态；历史决策与规则快照保留。
- Invalidation criteria：实现或设备状态变化时更新事实入口；官方提示指南变更按需复核。

## D-021：建立本地 Git 开发基线

- Date：2026-09-15
- Status：CURRENT BASELINE
- Decision：内层开发目录初始化 main，使用现有提交身份；仓库级换行、快进拉取、冲突显示与忽略配置生效。
- Evidence：本仓库初始提交与 docs/workflows/GIT_WORKFLOW.md。
- Impact：跟踪代码、标定和小型证据；不跟踪原始采集、固件、凭据、缓存及本机进程记录。无远端、无推送，不修改树莓派或 WSL 仓库。
- Invalidation criteria：团队协作或发布方式改变时追加决策；新 clone 需核验本机 Git 配置。
