# DECISIONS

版本：0.6  
最近更新：2026-09-23

## 阅读入口

当前架构与下一步分别以 [README](../README.md)、[STATUS](STATUS.md) 为准。以下按日期保留决策证据，不将旧条目的 Status 或过程描述当作当前状态。

早期条目所指的 `docs/tasks/`、`docs/workflows/`、`px4/manifests/` 与标定过程报告已退出 Git 跟踪，仅在原 WSL 工作区保留；新 clone 不含这些过程记录。既有决策正文及当时数值保留原样。

- 平台方向：D-016（Pi 5 验证、后续 RK3576）覆盖 D-007 的旧候选比较。
- 导航路线：D-024；源码分支/目录：D-025；Windows 退出开发：D-026。
- 当前规则：D-014；质量限制：D-013；标定基线：D-022。
- D-017～D-023 的目录、Git 与环境数据为历史快照，当前事实见 [STATUS](STATUS.md)。

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

## D-022：双目标定改用 20 mm 棋盘实测结果并驱动深度管线

- Date：2026-09-16（2026-09-17 续接核验）
- Status：CURRENT BASELINE
- Decision：当前深度输入基线改用 2026-09-16 实机采集的 11×8 内角点、20 mm 棋盘标定：`calibration/live_20260916_210120_642136/candidate.npz` 成为 `depth_preview.py` 默认；深度程序按所选标定的 `image_size` 请求相同采集模式（总图 2560×960、每目 1280×960），再缩放内参并重算校正映射；新增视频引导标定页（`live_calibration.py`、`calibration_core.py`、`calibration.html`、`test_live_calibration.py`），浏览器预览改为逐帧拉取最新图；旧的 `--calibration` 显式加载入口与 65 mm 标定继续保留为历史证据。
- Reason：既有 65 mm 标定（采集 1280×480）与当前相机实际安装和分辨率不匹配；需要一套可复现、带留出几何验证的自有标定，同时不假定相机各分辨率模式视场一致。
- Previous option：默认沿用 `calibration/20260911_202047/baseline_65mm.npz` 与连续 MJPEG 网页预览。
- New option：20 mm 棋盘新标定作为默认（基线估计 67.671804 mm），深度按标定匹配采集尺寸，预览逐帧拉取并按 `--stream-fps` 限速。
- Evidence：30 组真实采集（24 训练 / 6 留出，两目 12/12 区域、中心 9/9 覆盖，9 组倾斜，尺度跨度 3.93）；单目 RMS A/B 0.2698/0.2861 px、双目 RMS 0.4018 px、留出垂直 P95 0.4924 px、中位 0.1740 px，无筛查警告；`candidate.npz` SHA256 `d27f24de5ba69546274dc6f502ce27a7b36e3aae5f39220252c9e01af675aa7d`（2026-09-19 复核一致）；10 项无硬件回归通过（树莓派 OpenCV 4.6.0 / NumPy 1.26.4）。记录见 `companion/stereo_depth/LIVE_CALIBRATION.md` 与 `docs/tasks/2026-09-16-live-stereo-calibration.md`。
- Impact：同步 CURRENT_STATUS、REQUIREMENTS NAV-007、模块 README 与 BOM；不代表米制距离精度、性能或飞行验收通过；相关源码、页面与标定目录截至 2026-09-19 仍在未提交工作区。
- Invalidation criteria：相机、镜头、安装几何或输入裁剪变化，或独立已知距离测试证明当前标定不满足使用要求时重新标定并追加决策；旧标定不删除。

## D-023：远端仓库已配置（状态记录）

- Date：2026-09-19
- Status：CURRENT BASELINE
- Decision：补记本仓库远端状态：`origin` = `https://github.com/waterc07/Boom_Birds.git`，本地 `main` 与 `origin/main` 同为 `9e3e376`（提交时间 2026-09-15 20:22:25 +0800）；不改变 D-021 的忽略范围与提交身份原则。
- Reason：D-021 记录为“无远端、无推送”，README、NEXT_TASK、GIT_WORKFLOW、PROJECT_LAYOUT 与 CURRENT_STATUS 沿用该说法，与实际状态不符；按“改变 CURRENT BASELINE 项需追加决策”的规则补记。
- Previous option：仅本地 `main`，无远端（D-021）。
- New option：存在远端且已有同名提交；远端用途、可见性与协作方式未在仓库内记录（TBD）。
- Evidence：2026-09-19 复核 `git remote -v`、`git rev-parse HEAD origin/main`、`git log -1`、`git status --short`。
- Impact：只更新文档状态说明，不推送、不改远端设置。GIT_WORKFLOW 要求的“首次远端发布前检查可达历史与资料公开权限”尚未形成记录，列为待办。
- Invalidation criteria：远端地址、可见性或协作方式变化时追加决策；完成公开权限审查后在 GIT_WORKFLOW 记录证据。
- 后续（2026-09-19）：本次一致性修复随 `f1c301f` 提交，本地 `main` 领先 `origin/main` 1 个提交；仍未推送，公开权限审查待办不变。


## D-024：确认 OpenVINS 与个人 EGO-Planner fork 自主导航路线

- Date：2026-09-21
- Status：CURRENT BASELINE
- Decision：使用 Ubuntu 24.04 + ROS 2（沿用 Jazzy 基线）；普通双目相机共享采集后分成两路：左右图像 + 飞控 IMU 输入 OpenVINS，双目匹配生成米制深度/完整 XYZ，结合位姿建图；规划器使用用户个人 fork https://github.com/waterc07/ego-planner-swarm 。
- Reason：用户完成架构讨论后明确确认 OpenVINS 与个人规划仓库；补齐此前仅有深度模块的机载导航架构。
- Previous option：定位算法未定、历史下视主定位候选与 ToF 避障描述；VINS-Fusion 仅作为比较参考。
- New option：OpenVINS 主定位 + 自算双目深度 + 局部地图 + EGO-Planner + 轨迹执行 + 控制/飞控通信层；MTF-02P 保留独立安全观测链路。
- Evidence：本轮用户明确确认；2026-09-21 `git ls-remote --heads https://github.com/waterc07/ego-planner-swarm.git` 成功返回，含 `ros2_version` = `a3e14dd1ec3dbcec4619ccc9049b888bbcdcee6d`、`ros2_lyrical` = `607bfef550f775e88f0b586d16026ab54623e015`。这是远端快照，不是版本锁定或构建证据。
- Impact：更新架构、状态入口和后续任务；未安装/部署算法、未改源码、未连接设备、未刷写或推送。不更新任何能力为 PASS。
- TBD：规划分支/提交与 OpenVINS 提交、Jazzy/ARM64 兼容性、IMU 具体来源及消息、时间同步与时空标定、控制器位置、ROS 2 飞控通信后端、是否回传外部视觉、失效接管与联合资源预算。
- Invalidation criteria：当前平台回放与联合测试证明精度、延迟或资源不满足要求时重新评估并追加决策，不以历史 FAST-250 结果替代本机验证。


## D-025：WSL 主开发目录与 ROS 2 源码布局

- Date：2026-09-21
- Status：CURRENT BASELINE
- Decision：用户确认在 WSL 的 `/home/waterc/workspace/Boom_Birds` 母目录内统一组织 `companion/ros2_ws/src`，包含自研 stereo_depth 与个人 fork OpenVINS、EGO-Planner；后两者用 Git submodule 固定源码版本。
- Reason：统一 Linux 权限与构建环境，保留项目文档及依赖独立历史。
- Sources：https://github.com/waterc07/open_vins （master）、https://github.com/waterc07/ego-planner-swarm （ros2_version）。本次选择为初始检出，不是兼容/飞行验收结论。
- Evidence：stereo_depth 36 个文件迁移前后哈希一致，清单见 workflows/stereo_move_20260921.json；实际依赖 SHA 和检查结果见 tasks/2026-09-21-wsl-layout.md。
- Impact：仅 WSL 工作副本重排，更新当前路径入口；Windows、树莓派、既有 PX4/MAVLink 保留，历史决策及原始记录不改写。尚未实现 ROS 2 深度节点或部署。
- Invalidation criteria：依赖兼容性或架构接口验证发现问题时追加决策；实际升级需审查子模块提交并重新验证。

- D-025 执行补记（2026-09-22）：复用用户普通 clone，登记两个子模块并吸收 Git 目录；OpenVINS master `69488123ed9362dd44b6f28e7f4680abbff1442b`，EGO ros2_version `a3e14dd1ec3dbcec4619ccc9049b888bbcdcee6d`。仅固定源码快照，尚无 Jazzy/ARM64 构建证据。


## D-026：发布 WSL 布局并清理 Windows 重复代码副本

- Date：2026-09-22
- Status：CURRENT BASELINE
- Decision：用户授权整理、提交并推送当前 WSL 工程；Windows 外层继续存放历史资料，内层重复代码目录在备份及远端 SHA 核验后移入回收站，统一在 WSL 开发。
- Evidence：Windows Git 工作区干净且无 stash/独有分支；545 个文件完整备份并逐项校验；305 个采集/深度文件另存外层 data 并验证 SHA256。
- Impact：保留 Windows `.local`、历史资料、个人记忆与任务历史；不动既有 WSL PX4/MAVLink，不连接树莓派，不升级任何硬件能力状态。备份和私有资料不推送。
- Invalidation criteria：需要恢复旧文件时从外层 `.local/backups/windows-retirement-20260922/` 还原，避免同时维护两套主开发副本。


## D-027：深度内参与算法入口收敛

- Date：2026-09-29
- Status：CURRENT BASELINE
- Decision：项目 EGO 启动使用同步 CameraInfo；静态内参只用于显式独立模式，两者互斥。几何变化后旧地图不能继续规划，需重建。
- Interface：`stereo_depth.core.StereoProcessor.process_image()` 为 ROS 和兼容 CLI 共用的逐帧算法入口；匹配器对象不属于 ROS 接口。
- Scope：保留米制 XYZ/NaN 无效值、物理基线和旧 CLI；不构成真机同步、VIO 或飞行验收。

## D-028：运行参数单一来源、模式回读与自动恢复边界

- Date：2026-09-29
- Status：CURRENT BASELINE
- Decision：
  1. **运行参数只有一个定义点**：`boom_birds_nav/runtime_config.py` 的校验对象；
     `config/runtime.yaml` 是它的模板（测试强制逐一相等），任何节点 / launch / 脚本只能引用，
     不得另述一套。`config/contract.yaml` 只记录消息、单位、坐标系与无效值语义，**不写数值**。
  2. **高度量独立命名并登记参考系**：起飞高度（=PX4 `MIS_TAKEOFF_ALT`）、锁点相对高度下限、
     合成输入发布下限、恢复相对高度下限各自命名，全部相对起飞点地面；PX4 侧参数由
     `boom_birds_nav.sih_params` 从配置生成。
  3. **模式必须按回读区分**：`ExecutionStatus.mode_detail` 携带实际观测到的完整模式名
     （AUTO 下带子模式，如 `auto:land` / `auto:rtl`）。只报主模式无法区分 AUTO Land 与 Return，
     而恢复与验收都依赖这个区分；命令被接受不算模式确认。
  4. **会话绑定飞控启动周期**：开会话时记录当时的 `restart_epoch`；之后飞控重启即作废会话、
     清缓存，必须重新开会话。规划拒绝闭锁只能由**新轨迹号**清除，同一轨迹号继续出现不算重新建立。
  5. **自动恢复默认关闭**：`recovery_enabled=False` 为真实设备默认；只有 SIH 任务入口显式打开。
     每次故障最多 2 次尝试、每次模式确认超时 3 s，耗尽即闭锁且不自动重新解锁；人工取消、
     人工模式干预、未知故障、落地、重启、坐标系重置永久撤销本次恢复资格；只中断 PX4 自主进入的
     `auto:land` / `auto:rtl`，不泛化为任意模式争抢。
- Interface：`boom_birds_interfaces/msg/ExecutionStatus.msg` 新增 `mode_detail` / `custom_main_mode` /
  `custom_sub_mode`；`boom_birds_interfaces/srv/VehicleAction.srv` 是编排器请求模式的唯一出口。
- Evidence：`docs/STATUS.md` 的「2026-09-29 DeepSeek 接手轮次」；
  `evidence/deepseek-01/final/report.json`（7 组脱机检查全 PASS）、
  `evidence/deepseek-01/sih/`（SIH 运行记录与恢复路径）。
- Impact：消息定义变化后**整条 ego_planner 依赖链必须重建**——`traj_server` 订阅 `ExecutionStatus`，
  旧二进制配新消息会在运行中崩溃（本轮实测 SIGSEGV）。
- Invalidation criteria：若把运行参数重新散落到各节点/脚本，或让 shell 再次承担高度与模式判定，
  本条失效。

## D-029：SIH 链路的三条硬边界（图像门槛、单一发布者、有界重发与去抖）

- Date：2026-09-29
- Status：CURRENT BASELINE
- Decision：
  1. **合成图像发布高度门槛只是"不在地面"的护栏**，不得作为规划链的启动闸门。飞行中一旦
     重新闭合会造成"深度断流 → EGO 丢输入 → 自行取消目标 → 编排器闭锁"的自伤回路。
     EGO 的目标激活时机由编排器在 Offboard 回读确认后决定。
  2. **同一话题只能有一个数据来源**。SIH 的 `/boom_birds/imu` 由 `sitl_truth_source` 提供；
     不得再起 `vio_source`（它同时发布 odom），否则两个发布者同占一话题。
  3. **对"静默丢弃"的接口必须有界重发**：EGO 的 `projectRequest` 在 Offboard/odom/地图未就绪
     时不回执，编排器必须在 `planner_activate_timeout_s` 窗口内重发使能请求。
  4. **断流判定必须去抖**：`sending` 按 50 Hz 评估、规划流按更高频率发布，单帧空洞是调度抖动；
     必须持续超过 `command_timeout_s` 才判 `setpoint_link`，否则自动恢复预算会被抖动耗尽。
  5. **规划器作废轨迹属于可恢复的规划链路故障**，走既有的有界恢复判定（2 次 / 3 s），
     而不是立即永久闭锁；未启用恢复时仍闭锁。
- Evidence：`evidence/deepseek-01/sih/normal-mockamap-v5`、`diag-mockamap`；
  `docs/STATUS.md` 的「DeepSeek 第二轮」。
- Impact：自动恢复的前置条件（地图/输入连续有效）在规划器停机时**必然不成立**，此时恢复会
  按规定 fail-closed 拒绝接管并闭锁降落。这是合格拒绝，不是合格恢复；不得把"进了 RECOVERING"
  写成"恢复已完成"。
- Invalidation criteria：若把图像门槛重新当作规划闸门、或在同一话题上引入第二个数据来源，
  本条失效。

## D-030：阈值必须与被测链路的实际节奏对齐；测量不得扰动被测系统

- Date：2026-09-29
- Status：CURRENT BASELINE
- Decision：
  1. 任何"链路失效"阈值在定值前必须**先量测**对应流的实际间隔；判据是该阈值不得比被依赖分支的
     正常节奏更紧。实测：odom 0.067 s、深度 0.208 s（均无 >1 s 空洞），因此
     `planning_timeout_s` 取 1.0 s（与 `depth_timeout_s` 同量级），断流判定取
     `setpoint_interrupt_timeout_s = 1.0`（与 PX4 `COM_OF_LOSS_T` 缺省对齐）。
  2. **到达目标是任务完成，不是链路故障**：规划器走完轨迹后会停止发布，编排器必须先判目标容差。
  3. **测量工具不得扰动被测系统**：每秒 spawn 多个 `px4-listener` 会打断 ROS 流并制造假故障；
     时间线证据取自 `ExecutionStatus`，PX4 侧只做低频独立回读（用于交叉核对，不用于判定）。
- Evidence：`evidence/deepseek-01/sih/gap-diag`（间隔统计）、`normal-mockamap-v{7,8,9}`。
- Impact：把"进过 RECOVERING"当成恢复完成、或把阈值收紧当成"更安全"，都会让预算被抖动耗尽并
  闭锁降落。收紧阈值前必须先有间隔测量。
- Invalidation criteria：若在未量测的情况下收紧这些窗口，或让记录器重新以高频 spawn 外部工具，
  本条失效。

## D-031：心跳/周期消息的超时必须按"漏拍次数"定，不得与飞控的失效动作阈值混同

- Date：2026-09-29
- Status：CURRENT BASELINE
- Decision：
  1. PX4 MAVLink HEARTBEAT 标称 **1 Hz**，`heartbeat_timeout_s` 取 **2.5 s**（覆盖至少两次漏拍）。
     它描述的是"飞控还在跟我们说话吗"，与 `COM_OF_LOSS_T`（PX4 自己丢 offboard 设定点后的动作）
     不是同一件事；两者不得互相推导。
  2. 同一规则适用于其它周期消息：`EXTENDED_SYS_STATE`（MAV_LANDED_STATE）约 1 Hz，
     `landed_state_timeout_s` 取 **2.5 s**，否则 `landed_known` 反复翻假、恢复前置条件被误判。
  3. 任何"把代码默认值与节点参数文件统一"的动作，必须先确认哪一个值是对的。
     本轮事故就是**把原本正确的 2.0 s 统一到错误的 1.0 s**。
  4. `allow_setpoint=False` 且 `reasons=[]` 表示**迟滞计数不足**，不是"没有原因"。
     诊断时必须把这个状态与"有具体违规"区分开，否则会误判为无据可查。
- Evidence：`evidence/deepseek-01/sih/normal-mockamap-v10`（原因字典含
  `signal_stale signal=px4_heartbeat age=1.022 limit=1.0`）、`normal-mockamap-v12`
  （连续 EXECUTING 152 s、7971 条 setpoint）。
- Impact：这一条修掉之前，所有"链路断流"故障都是假的；恢复逻辑本身没有问题，是被假故障反复触发。
- Invalidation criteria：若再把心跳超时按 `COM_OF_LOSS_T` 取值、或按"一个标称周期"取值，本条失效。

## D-032：状态新鲜度不得混入被观测对象的固有周期；对可重入的接口必须限频

- Date：2026-09-29
- Status：CURRENT BASELINE
- Decision：
  1. `status_age` 只表示"这份 `ExecutionStatus` 观测有多旧"，**不**取心跳年龄的 max。
     飞控是否在线由 `connected` / `sending` / `reasons` 表达，并由 `px4_failsafe` 用自己的
     心跳窗口判定。把 ~1 Hz 的固有周期算进新鲜度，会让所有 `<= 1 s` 的判定间歇失败。
  2. 对"会被静默丢弃、且调用本身有副作用"的接口（EGO 的 `projectRequest` 每次都重新触发规划），
     重发必须**限频**（`planner_enable_retry_s = 0.5 s`）。按控制周期重发会打乱对端 FSM。
  3. 诊断信息必须能回答"窗口为什么没成立"：恢复窗口清零时记录具体是哪一路
     （`window_reset_by:...`），否则只能看到 `valid_duration_not_met` 而无从下手。
- Evidence：`evidence/deepseek-01/sih/normal-mockamap-v13`
  （`RECOVERING → EXECUTING :: confirmed`、机载最近 0.379 m、实际 x 5.007 m）。
- Impact：这一条修掉之前，自动恢复即使被触发也**永远无法确认**，B6 无从谈起。
- Invalidation criteria：若再把被观测对象的固有周期算进状态新鲜度，或对可重入接口按控制周期重发，本条失效。

## D-033：规划流收尾的保持点选择；瞬时规划失效不等于任务故障；PX4 参数基线必须留证

- Date：2026-09-29
- Status：CURRENT BASELINE
- Decision：
  1. 规划流停止后，保持点按距离选择：离目标已在 `handoff_max_distance_m` 以内 ⇒ 保持点放到
     **目标**（让飞控收敛最后一段并触发到点判定）；否则保持在**当前位置**，不在无规划的情况下继续飞。
  2. EGO 任一次重规划失败都会发失效消息，这**不等于**任务故障。收到后应退役旧轨迹并进入
     "规划流静默"（保持 + 限频重发使能，上限 `planner_activate_timeout_s`），只有静默超窗才闭锁；
     一收到就判 `planning_link` 并进入恢复，会因恢复前置条件必然不成立而白耗预算。
  3. PX4 `parameters.bson` 会跨运行残留：每次 SIH 运行必须显式下发参数基线并回读留证。
     `COM_OF_LOSS_T` 固定为本地版本缺省 **1.0 s**——显式写下的目的正是**证明没有放大**它。
- Evidence：`evidence/deepseek-01/sih/normal-mockamap-v16`
  （COMPLETE、机载最近 0.159 m、Disarmed、`landed: true`、`at_rest: true`、`in failsafe: no`）。
- Impact：本条之前，"完整新 SIH 任务"从未通过；现在正常路径可复现通过。
- Invalidation criteria：若把保持点重新固定为当前位置、或把瞬时规划失效重新当作任务故障、
  或靠放大 `COM_OF_LOSS_T` 掩盖控制断流，本条失效。

## D-034：按职责拆包的最终结构与依赖方向

- Date：2026-09-29
- Status：CURRENT BASELINE
- Decision：
  1. 包结构：`boom_birds_interfaces`（接口）、`boom_birds_control`（**底座**：`runtime_config` /
     `frames` / `control_protocol` + PX4 后端 / 坐标换算 / 执行许可）、`boom_birds_sensing`
     （采集、时间同步、深度、位姿适配、MAVLink IMU、合成场景渲染）、`boom_birds_sim`
     （SIH 真值、TEST-ONLY 合成 VIO/IMU、悬停中继）、`boom_birds_bringup`（生命周期编排、
     配置 IO、SIH 参数入口）、`stereo_depth`（深度算法与兼容 CLI）；
     `boom_birds_nav` 只做**兼容转发**，不再含实现与可执行文件。
  2. 底座放 control 而非 bringup：提示词把"配置"写在 bringup 下，但照做会形成
     control⇄bringup 环。依赖方向固定为 control ← {bringup, sensing, sim}，
     sim → sensing。该方向由测试锁定，不靠约定。
  3. **生产包不得依赖 sim 包**；仿真连接与模式限制仍在运行时实施（真值源只允许回环 14550、
     `dry_run`、`allow_arming=false`），不靠包名隔离。
  4. 兼容转发层**不承诺**对 `inspect.getsource` 与 monkeypatch 透明：源码级结构断言与打桩
     必须绑定实现模块（已按此调整受影响测试）。
  5. 安装必须"清空再装"：先删本包已安装的 site-packages 目录、再删 `build/`，否则源里迁走的
     模块会经 `build/lib` 被复制回安装树，出现"源已迁走但 import 仍拿到旧实现"。
- Evidence：`evidence/deepseek-12/`（15 包 0 失败、12 组脱机检查全 PASS、navigation 730 项）；
  `nav_forwarder` 断言 nav 无实现。
- Impact：拆包前所有模块集中在一个包里，配置与协议的使用者无法从包边界看出依赖方向；
  现在方向可执行地锁定，且"构建成功但入口点/模块不对"这类问题由核对项而非人工发现。
- Invalidation criteria：若把底座重新放回上层包、或在 nav 里重新塞回实现，本条失效。

## D-035：深度量程必须与场景一致；门控失败必须逐项上报

- Date：2026-09-29
- Status：CURRENT BASELINE
- Decision：
  1. **门控类失败必须逐项上报**。只报 `map_or_stream_not_ready` 无法区分是执行许可、
     传感器还是地图：`mission/status` 现在带 `hold_ready_gate`（`sending` / `sensors_ready` /
     `map_ready` / 规划器状态年龄 / `planner_map_ready` / 几何故障）。
  2. **深度量程是链路契约的一部分，必须与场景一致**。实测：30 m 森林最近障碍 10.39 m，
     而 EGO 的 `invalid_depth_max_dist_`（`max_ray_length_ + 0.1`，约 5.1 m）把 10–14 m 的
     返回全部判无效，`mapReady()` 永不成立；`depth_node.max_depth_m = 5.0` 与主话题
     实际发布的 10–14 m 值也不一致。改动量程或场景起点都必须**显式且留证**。
  3. 合成双目在当前基线与分辨率下的可用量程约 5 m 量级；把障碍放到该量程之外时，
     这条链路"看不到场景"，此时任何"绕障通过/失败"的结论都无效。
- Evidence：`evidence/deepseek-15/forest-diag`（`hold_ready_gate`）、
  `evidence/deepseek-15/forest-depth-diag`（`pixels_within_5m = 0`，`min_finite_m = 10.39`）。
- Impact：在此之前，森林矩阵的五次"安全收尾"容易被误读成场景可跑；实际是传感器量程
  与场景不匹配，任务从未开始执行。
- Invalidation criteria：若在未量测深度分布的情况下调整量程或场景，或重新用单一
  `map_or_stream_not_ready` 掩盖门控细节，本条失效。

## D-036：包元数据的依赖方向必须由**实际 import** 决定，且要双向回归

- Date：2026-09-29　Status：现行规则（拆包第三/四批的修正）
- Decision：每个包的 `package.xml` 只声明它**真正 import** 的兄弟包；方向由实现分层决定
  （`control` 是基础层，`nav` 是兼容转发层），并用测试锁死双向一致：用了没声明、声明了没用都算失败。
- Reason：拆包时 `boom_birds_control/sensing/sim` 各留一行旧包名 `<depend>boom_birds_nav</depend>`，
  而 `nav` 反过来 import 这四个包却没有声明。colcon 只因为 nav 没声明而恰好没报环；真实代价是
  `--packages-up-to boom_birds_nav` 只构建 nav，得到一个能构建、import 全断的安装树（第一版 CI 的
  包清单正是如此）。更糟的是三个包自带的 `assert "<depend>boom_birds_nav</depend>" in xml`
  把错误方向**当成规格**锁住，全绿。
- Previous option：每个包保留一行从 nav 拆出时复制的 `<depend>boom_birds_nav</depend>`，靠"构建成功"当作依赖正确。
- New option：`package.xml` ⇄ 实际 import 双向一致（`test_config_single_source.py` 增加兄弟包双向断言、
  三个 `test_package_layout.py` 改为"不得依赖 nav + 必须声明 control"），并显式跳过"包 import 自己"。
- Evidence：`evidence/deepseek-17/check2.log`（真实失败）/`check4.log`（12/12 PASS）；
  `git diff` 中 5 份 `package.xml`。
- Invalidation criteria：若再次出现"声明但不用"或"用而未声明"的兄弟包依赖，或某个包按包名（而非
  实际 import）注册依赖，本条失效。

## D-037：证据生成器必须对"环境不对/被测系统在跑"硬失败，不得产出污染的失败报告

- Date：2026-09-29　Status：现行规则
- Decision：`check_offline.sh`/`check_offline.py` 在两种情况下**拒绝运行并说明原因**：
  ① 安装前缀里没有 `setup.bash`（默认 `~/bb_build/main` 与开发机常用的 `architecture` 前缀不是同一目录）；
  ② 检测到 `px4`/`run_sih_mission`/`sih_record`/`traj_server` 进程（同一 ROS 话题上 SIH 的合成
  深度流会注进被测话题）。强制并发需显式 `BB_ALLOW_BUSY_CHECK=1`。
- Reason：本轮两次"检查结果"其实不是产品结论：漏导出 `INSTALL_BASE` 让 8 组落在陈旧安装树上
  整片 `Package not found`；与 SIH 并发让 `navigation` 组收到 30 Hz 深度流（159 条输出 vs 测试自发
  5 张图）而假失败。假失败比没有结果更危险——它会推动人去修不存在的问题，也会被误当成产品缺陷。
- Previous option：默认前缀 + 假定没有别的任务在跑，失败就记成 FAIL。
- New option：前置条件不满足即退出（2=前缀不对，3=有 SIH 在跑），错误信息给出可直接照做的修法。
- Evidence：`evidence/deepseek-17/check3.log`、`check5.log`（两次假失败）、
  `check4.log`（同一源码 + 正确前缀 → 12/12）；两个前置的自检（退出 3 / 退出 2 / `BB_ALLOW_BUSY_CHECK=1` 放行）。
- Invalidation criteria：若检查再次在错误安装前缀或 SIH 并发下产出 FAIL 报告并被采信，本条失效。

## D-038：CI 只跑它能跑的子集，排除项必须显式且不得声称已跑通

- Date：2026-09-29　Status：现行规则（未验证）
- Decision：`.github/workflows/offline.yml` 构建全部 `boom_birds_*`（拆包后 nav 只是转发层），
  并直接调用本机同一份 `check_offline.sh`（单一入口，不再在 CI 里抄第二份测试清单），
  显式 `--exclude-groups navigation,map_behavior,trajectory_validation`——这三组要求 EGO fork 的
  C++ 产物（`ego_planner/traj_server`、`plan_env/bb_grid_map_test`）已构建。排除项写入
  `report.json` 的 `excluded_groups`，**既不算 PASS 也不静默消失**。
- Reason：第一版 CI 既抄了一份拆包前的测试文件清单，又用了拆包前的包选择，因此在真实 runner 上
  无法通过；而"抄一份清单"必然与 `check_offline.py` 漂移。
- Previous option：CI 内独立列测试文件 + `--packages-up-to ... boom_birds_nav`（拆包后只构建 nav）。
- New option：单一入口 + 显式排除 + 报告留痕。
- Evidence：`evidence/deepseek-17/report_ci_sim.json`（本地模拟 10 组 PASS + 3 组 EXCLUDED，退出 0）；
  `.github/workflows/offline.yml` YAML 语法与引用路径本地校验通过。
- Invalidation criteria：把"本地模拟通过"当作"CI 跑通"；或在 CI 里再写一份测试文件清单。**该工作流至今
  从未在 runner 上执行过**，在真跑通之前本条只能是"未验证"。

## D-039：故障注入必须自证"确实注入了"，否则该运行只能记 NOT RUN

- Date：2026-09-29　Status：现行规则
- Decision：任何故障注入都必须留下可核对的注入证据（命中的进程 pattern + 被杀的 pid + 时间戳 +
  首个 latch 与注入时刻的先后），写进该次运行的 `fault.txt`；拿不到证据的运行**只能记 NOT RUN**，
  不得因为"结束得干净"而记通过。
- Reason：拆包后 `pkill -f 'boom_birds_nav/depth_node'` 匹配不到任何进程（实现在
  `boom_birds_sensing`），pkill 静默失败、`fault.txt` 根本没写：两例 `深度断流` 实际上**从未注入**，
  却被记成了"安全收尾"。同类风险还有"注入晚于首个 latch"——矩阵里 4 例 `handoff_discontinuous`
  在注入之前就已 latch，此时"注入故障是不是首要原因"这个问题本身不成立。
- Previous option：靠 `pkill` 的退出码（被 `|| true` 吞掉）与运行结束状态判断注入是否生效。
- New option：`kill_label`（先 `pgrep` 证明命中再杀，pattern 与 killed pid 落盘）；补跑后两例
  才拿到 `pattern=lib/boom_birds_sensing/depth_node killed_pids=...`。
- Evidence：`evidence/deepseek-17/sih/SIH_REPORT.md`（第 7、12 例原记 NOT RUN；第 16、17 例为补跑）；
  失败/无效批次在 `sih/failures/`。
- Invalidation criteria：若再次出现"注入命令静默失败但运行记为通过"，或注入时刻晚于首个 latch
  却不加区分地归因于注入故障，本条失效。

## D-040：接管速度连续性门是当前最主要的闭锁来源；门限值缺量测依据，不得凭感觉改

- Date：2026-09-29　Status：观察中（未改值）
- Decision：暂**不修改** `handoff_max_speed_m_s = 0.3`；把它标记为"缺量测依据"的待定值，
  下一轮用真机/更高保真链路的实测速度分布来定，并在改动时留证。
- Reason：22 例矩阵里 9 次 `handoff_discontinuous` 的四条判据中**只有 `dist_vel` 越限**
  （0.301–0.673 m/s），`pose_age`/`status_age`/`dist_pos` 一次都没触发；其中 3 例只超
  0.001–0.021 m/s。也就是说"接管被拒"的主要原因可能是一个从未按实测分布定过的阈值，
  而不是链路或规划本身。反过来，若为了"让矩阵好看"直接放宽门限，就会把真正的接管不安全掩盖掉。
- Previous option：按提示词给的 0.3 m/s 直接采信为合理值。
- New option：保留 0.3，标注为待量测重定值，并把"越限幅度"写进每次运行报告（本轮已做）。
- Evidence：`evidence/deepseek-17/sih/SIH_REPORT.md`（9 例 `dist_vel` 越限明细）。
- Invalidation criteria：若在没有实测速度分布证据的情况下调整该门限，或用它之外的判据解释
  "接管被拒"，本条失效。

## D-041：重启轨迹生产者后本任务内**不存在**合法恢复路径（安全侧设计后果）

- Date：2026-09-29　Status：现行行为（已实测确认，非缺陷）
- Decision：不为此新增"生产者重启后续飞"的旁路；若要支持，必须在契约层引入生产者 epoch / 会话语义重绑，
  属需要设计授权的改动。
- Reason：`traj_server` 重启后 trajectory_id 从 1 重新计数，而 `ControlIngress.retired` 是单调水位
  （`control_protocol.py:47/68`，`<= retired` → `retired_trajectory`），前 16 个 id 全被永久拒绝 →
  静默 30 s → `planner_timeout`；重新开会话同样不行——`open_session()` 清零水位（`:37-40`），
  但 `lifecycle.py:587-588` 对会话变化直接 `latch("session_changed")`，且它在 `REVOKING_FAULTS` 里。
  两条路都通向闭锁降落：**安全**（绝不复用已退役轨迹、绝不静默换会话），代价是任务不可续。
- Previous option：（实测走过的弯路）重启 `traj_server` 后用同 argv 拉起，期待任务自动续飞——结构上不可能。
- New option：把该结论写进合同文档；SIH 的 B 类注入改为**挂起/恢复**（SIGSTOP→SIGCONT，pid 与计数都不变）
  或改注入上游（`sitl_truth_source` 造 `sensor_link`）。
- Evidence：`evidence/deepseek-18/sih/`（id 序列 `…14,15,16` → `1,2,3…`；`planner_timeout` 恰好 30.0 s）；
  `evidence/deepseek-17/sih/SIH_REPORT.md` 第 15 例。
- Invalidation criteria：若在没有 epoch/会话重绑设计的情况下，用工具侧手段让任务在生产者重启后"续飞"，本条失效。

## D-042：未登记模式的 fail-closed 由**模式名合法性**保证；`REVOKING_FAULTS` 里的 `"unknown"` 经模式回读不可达

- Date：2026-09-29　Status：事实陈述（是否调整属设计授权）
- Decision：不改动 `REVOKING_FAULTS`；但记录"`unknown` 这个撤销码在当前回读路径上取不到"，
  以免后来者以为它已经覆盖了"未知故障"。
- Reason：实测两种注入——格式合法的未登记模式名 `auto:mission` → `manual_mode` 立即闭锁
  （`attempts=0`、不进 RECOVERING）= **fail-closed 成立**；而字面量 `"unknown"` 被
  `mode_detail_known()` 当成"没有给出模式名"，回退到 offboard 布尔 → `offboard_lost`（**可恢复**），
  真的进了 RECOVERING（attempts 1→2）后才 `exhausted` 闭锁。
- Previous option：以为 `REVOKING_FAULTS` 里的 `unknown` 会在"未知故障"时命中。
- New option：把"未登记模式名 ⇒ manual_mode 闭锁"作为未知故障的验收口径；`unknown` 码标注为不可达。
- Evidence：`evidence/deepseek-18/sih/`（`mode_inject.json`；注入手段为第二发布者改 `mode_detail`，
  属注入手段、非生产契约）。
- Invalidation criteria：若新增产出 `unknown` 故障码的路径而不更新本条，或把"未登记模式名"改成可恢复，本条失效。

## D-043：接管连续性门在**每次重规划**上生效且命中即终态闭锁——冻结值不改，语义待设计评审

- Date：2026-09-29　Status：观察中（本轮不改行为）
- Decision：**不**放宽 `handoff_max_distance_m=0.5` / `handoff_max_speed_m_s=0.3`，**也不**擅自把"终态闭锁"
  改成"拒绝该轨迹、继续飞旧轨迹"。把它作为设计评审项交回，附本轮实测。
- Reason：接管门在轨迹键变化时评估（`lifecycle_node.py:276-302`），而 EGO 每次重规划都换
  `trajectory_id`（`traj_server.cpp:105`；实测约 2 Hz、36 s 内 43 个新 id），于是这条规则在整段飞行中
  反复生效。两轮矩阵 9 次 `handoff_discontinuous` 中**只有 `dist_vel` 越限**（0.301–0.673，3 例仅超 0.001–0.021），
  它是 30 m 森林飞到 9.652 m / 7.254 m 后闭锁、以及 6/10 故障例"注入故障非首要原因"的直接原因。
  行为已被 `test_handoff_thresholds_are_the_frozen_values_and_come_from_one_source`（用 `trajectory_id=2`
  即飞行中重规划）与 `test_discontinuous_or_stale_handoff_is_rejected` **冻结为规格**。
- Previous option：把 9 次闭锁当成"链路不稳"，去调恢复或超时参数。
- New option：如实记录"例行重规划可终止任务"这一任务级后果，交设计评审决定是否改为
  "拒绝该条轨迹 + 有界等待新轨迹（既有 `planner_timeout` 兜底）"；在评审结论出来前不改任何值。
- Evidence：`evidence/deepseek-17/sih/SIH_REPORT.md`、`evidence/deepseek-18/sih/SIH_REPORT.md`
  （`dist_vel` 越限明细；`b-suspend-t1` 首次 `dist_pos=1.718`）。
- Invalidation criteria：在没有实测速度/位置分布依据的情况下调整这两个阈值，或把"终态闭锁"改成"静默忽略"
  而没有有界兜底，本条失效。


## D-044：自动恢复只接受当前已识别故障；回读确认复核全部门限

- Date：2026-09-30　Status：脱机验证通过，SIH 待复验
- Decision：`unknown` 模式闭锁；`auto:land` / `auto:rtl` 需与当前 `sensor_link`、`planning_link` 或 `setpoint_link` 同时观测到，单靠模式名不授予恢复资格。Offboard 回读后再次检查恢复门限，失败即闭锁。恢复 HOLD 按故障瞬间速度和 `recovery_brake_accel_m_s2` 生成加速度有界的减速段。
- Reason：验收反例显示旧实现可在过期位姿、低高度或执行许可关闭时确认恢复；`unknown` 可退化为可恢复的 `offboard_lost`；无故障 Land 可触发恢复，而已知传感器故障后的 Land 会被撤销。参见仓库外 `codex-audit-20260930/ACCEPTANCE.md`。
- Limit：现有 `ExecutionStatus` 没有飞控自动切入 Land/Return 的原因回读，链路故障同周期的人工模式动作仍无法在接口层独立归因。`offboard_confirmed` 布尔另受心跳年龄门限约束，SIH 实测它可短暂变假而完整模式名仍为 `offboard`，因此不用于否定完整模式回读；这次误闭锁记录保留在 `codex-fix-20260930/sih/focused/sensor-land/`。制动命令的加速度上限不等于地图净空或机体碰撞验收。修改后 SIH 只跑了未知模式、传感器进入 Land、规划器挂起三个针对性场景；完整矩阵尚未通过。`planner-rerun` 的二次 setpoint 断流前有 EGO `Depth Lost! EMERGENCY_STOP`，不得通过延长断流容忍时间掩盖。

## D-045：接管跳变退役当前轨迹，等待新轨迹有界接管

- Date：2026-09-30　Status：脱机通过；SIH 仍未完成目标
- Decision：保留接管距离 0.5 m、速度差 0.3 m/s。轨迹本身跳变时退役该 ID，发送取消和当前位置保持；下一 ID 重新接受连续性检查。既有 30 s `planner_activate_timeout_s` 作为等待上限，超时闭锁。位姿、状态过期或非有限值仍立即闭锁。此决定替代 D-043 的“命中即终态闭锁”行为。
- Reason：挂起/恢复 SIH 曾在新轨迹起点距当前状态 2.452 m 时立即闭锁；直接接受该轨迹会跳变，放宽阈值没有依据。
- Evidence：`codex-fix-20260930/report-final.json`、`codex-fix-20260930/sih/planner-handoff-fix.jsonl`。后者恢复后持续发送 setpoint，但最终因 `planner_timeout` 闭锁并 Disarmed，未到目标。

## D-046：规划流停发先退役旧轨迹，再请求剩余目标

- Date：2026-09-30　Status：短距离 SIH 通过
- Decision：规划流停发超过 `min(command_timeout_s, setpoint_timeout_s)/2` 时先发送 HOLD；超过 `command_timeout_s` 后才取消并退役旧轨迹，由既有有界规划窗口重发目标。已接受的新轨迹重新通过位置/速度连续性门。
- Reason：旧 HOLD 在 0.2 s 命令有效期届满才接管，失效检测已报 `setpoint_link`；提前 HOLD 后若不退役旧轨迹，`playing=true` 阻止再规划，SIH 在离目标 0.633 m 处停滞直到超时。
- Evidence：`codex-fix-20260930/sih/normal-short.jsonl`、`normal-short-after-hold.jsonl`、`normal-short-replan.jsonl`。2 m 任务在提前取消版本最近目标距离 0.078 m；延迟退役版本见 `normal-short-grace.jsonl`，最近 0.05 m，恢复 0 次、无闭锁，最终 Disarmed。森林 seed 3 延迟退役复测仍因 `planner_timeout` 失败。

## D-047：HOLD 等待继续观测控制出口；重复目标使能不重置 EGO

- Date：2026-09-30　Status：脱机与短距离正常/恢复 SIH 通过；森林 SIH 未通过
- Decision：EXECUTING 的传感器和 setpoint 断流检查不以 `playing` 为前提，到点也不豁免控制出口断流。相同活动目标的使能重发为幂等操作；编排器拒绝新轨迹或旧流过期后显式禁用目标，随后重新使能剩余目标。
- Reason：HOLD 中 `playing=false` 曾抑制断流观测；使能重发每 0.5 s 重新初始化目标，会打断 EGO 的内部规划重试。
- Evidence：`test_hold_wait_does_not_hide_setpoint_loss`、`test_hold_wait_does_not_hide_sensor_loss`、`test_discontinuous_handoff_explicitly_stops_the_planner_before_retry`；`codex-fix-20260930/sih/final-smoke-recorded.jsonl` 正常与规划器挂起任务分别最近目标距离 0.085/0.112 m，完成降落且最终 Disarmed；`forest-seed3-idempotent.jsonl` 仍为规划超时，不构成森林到达目标的验收。

## D-048：恢复以 PX4 新鲜观测和当前 SIH 故障关联为前提

- Date：2026-10-01
- Decision：Land/Return 自动中断需 CURRENT_MODE 实际/意图模式一致、PX4 SIH 原生 failsafe 原因匹配当前链路故障，以及新鲜 ODOMETRY reset counter、落地、位姿、IMU、深度、地图和对齐观测。缺观测拒绝恢复；硬件入口默认关闭恢复。
- Reason：同周期人工模式动作不能只靠心跳主模式和 Companion 的断流判断归因。SIH 原生原因由受核验本机 PX4 进程只读获取；编排器不新增 MAVLink 控制连接。此实现补充 D-044 的观测限制。
- Budget：每次故障最多 2 次、单次模式确认 3 s；恢复后连续 1 s 健康 EXECUTING 才关闭该故障并重置事件预算。窗口内反复故障共用预算；任务累计次数另行记录。
- Evidence：当前脱机和本机矩阵见 STATUS。中心净空与制动加速度不构成机体碰撞或真机验收。

## D-049：任务会话之外绑定规划器和执行器进程 ID

- Date：2026-10-01
- Decision：EGO 与 traj_server 每次启动生成随机进程 ID，通过 PlannerStatus 注册；SessionBspline、执行/取消命令携带生产者身份。编排器绑定两个 ID，活动任务中变化即 session_changed 闭锁，不能续播旧轨迹。
- Reason：任务 UUID 不会随生产者重启改变，单靠轨迹序号无法区分计数重置与残留消息。本决定落实 D-041 的生产者 epoch 要求，但不新增任务内重绑续飞路径。
- Evidence：C++ 会话轨迹测试、lifecycle_node 回归和 session-final-20261001 本机矩阵。闭锁后重复注册不干扰降落收尾。

## D-050：执行接口回读实际坐标对齐，位置和速度使用同一变换

- Date：2026-10-01
- Decision：ExecutionStatus 携带后端实际 yaw_offset 和 translation；编排器位置按 R 的逆变换再平移，速度只按逆旋转。非有限值拒绝，活动阶段变换改变视为 frame_reset。
- Reason：只用 NED 轴符号换算速度会漏掉非零 yaw_offset；编排器自行复述场景原点也无法观测后端重对齐。
- Evidence：非零航向/平移、运行中对齐变化、非有限观测回归；SIH 日志保存实际变换。

## D-051：配置和仿真实现随所有者迁移，nav 只转发

- Date：2026-10-01
- Decision：contract 属 interfaces；runtime/PX4 参数属 control；采集/IMU 参数属 sensing；生命周期和生产 launch 属 bringup；合成渲染、SIH 场景和仿真 launch 属 sim。生产 stereo_source 不再接受 synth；sim 子类提供合成入口。
- Reason：按包名移动而保留生产入口对合成渲染器的依赖，不能满足生产包不依赖 sim。兼容模块与 Include launch 不保留第二份实现。
- Evidence：包 XML、实际 import、转发身份、安装资源与可执行入口测试；独立前缀构建和 13 组统一脱机验收。

## D-052：项目局部目标不固定在占据体素内

- Date：2026-10-01
- Decision：项目局部中间目标被膨胀地图占据时，在配置半径内搜索自由且向最终目标推进的候选；占据的最终任务目标不能移动。候选选择不能绕过完整轨迹碰撞、速度/加速度和接管检查。
- Reason：森林日志反复报 terminal point in obstacle，固定终点优化无法产生合法轨迹。缩短视距本身未解决，失败证据保留。
- Limit：该修改不能保证森林可达；目标到达与规划拒绝逐 seed 记录在 STATUS。未放宽 0.5 m/0.3 m/s 接管门、0.15 s 位姿新鲜度或 PX4 失联超时。

## D-053：随机森林先核实几何准入，原不可达批次保留为反例

- Date：2026-10-01
- Decision：SIH 森林在 START 前记录完整原始点云和规划器参数，按相同体素尺寸、膨胀和顶棚检查起终点及连通性。占据起点、占据最终目标、不连通和只有角点连通的场景拒绝启动；不移动最终目标。场景生成参数显式按 profile 记录。
- Reason：原 dense seed 1、2、3、5 的最终目标被占据，seed 4 不连通；这些场景不能作为应当到达的正例。reference_30m 仍有 seed 2 起点被占据，不能假定随机场景天然有路。
- Limit：完整场景检查仅为 TEST-ONLY 静态几何证据，不把完整点云或路径提供给 EGO，不替代深度感知、动力学或机体包络验收。准入拒绝不记为飞行到达。
- Evidence：STATUS 的 forest-final 十例批次与可达性单元测试。

## D-054：warm start 锚定实测状态，周期重规划保留加速段推进

- Date：2026-10-01
- Decision：cubic B-spline 首三个控制点按本次实测起点、速度和起始加速度求解，后续 warm start 形状保留。项目会话的周期重规划需满足原时间条件，并推进既有控制点间距或接近轨迹尾部；碰撞检查仍立即触发重规划。
- Reason：每次从静止重建首段并按 1 s 定时替换，会反复退役加速段，30 m 任务无法在 300 s 内完成。直接保留旧曲线起点又会造成接管位置/速度跳变。
- Limit：完整曲线继续通过碰撞、速度/加速度、规划预算与接管门检查。reference_30m 当前 3/5 到达，输入新鲜度失败仍未关闭；该改动不构成森林全部通过。
- Evidence：trajectory_validation 的实曲线起点/导数及周期重规划测试；STATUS 中森林复验。

## D-055：控制状态异步发布，接收端保留完整观测年龄

- Date：2026-10-01
- Decision：Px4Interface 在 rclpy 初始化前默认设置 RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS，显式环境配置优先；可靠 QoS、命令序号与取消屏障保留。ExecutionStatus 接收端将传输年龄加入位姿、姿态、reset counter、实际模式和安全原因的年龄。
- Reason：故障瞬间两个回环端口持续收到新位置报文，Companion 回读却陈旧约 0.32 s；阶段计时将约 0.272 s 阻塞定位到 ExecutionStatus.publish()。仅按接收后的时间更新年龄，会把队列中的旧状态当作新观测。
- Evidence：STATUS 的 wire-repeat、gc-diag、async-final、age-final 与传输年龄回归。发布模式语义见 [rmw_fastrtps](https://github.com/ros2/rmw_fastrtps#change-publication-mode)。
- Limit：异步模式不构成硬实时保证；真实输入中断仍按原门限停发和闭锁。结果只覆盖指定场景及当次 WSL 负载，不替代真机或 Pi 5 验收。
