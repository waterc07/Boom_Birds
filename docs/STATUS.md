# 当前状态与下一步

更新：2026-10-08。当前包括 MAVROS 通信与飞控 IMU 上行、双目采集发布链及 Px4Interface 高层控制接口的脱机验证、Pi 5 原双目感知台架，以及 PX4 SIH 的短距离与固定场景 30 m 绕障仿真；唯一正式开发根为 WSL Ubuntu-24.04 的 `/home/waterc/workspace/Boom_Birds`。下文分别标注 WSL、Pi 5 脱机/感知台架、PX4 SITL 和未执行的传感器/飞控实机验证。详细架构见 [项目入口](../README.md)，运行方式见 [ROS 工作空间](../companion/ros2_ws/README.md)。

## 当前路线与验收缺口

导航主线为同帧双目＋飞控 IMU → OpenVINS → 深度/局部地图 → EGO → Companion 位置/速度闭环 → MAVROS 姿态/推力 → PX4。当前实机接入按 [D-058](DECISIONS.md#d-058实机入口采用-companion-位置闭环与-px4-姿态闭环)；旧 `px4_position` 路线保留，不能把它的原点对齐要求直接套到姿态路线。

平台末段按 [D-059](DECISIONS.md#d-059末段降落使用平台相对速度确认后才释放导航计算) 切换到 PX4 速度闭环：导航输出撤销、零速度预发、飞控实际回读确认后才允许释放 VIO/双目。IMX219 是独立下视内参标定候选，不替换原 USB 双目；局部特征跟踪输出像素目标，尚未进入下降许可。

| 环节 | 当前范围 | 未关闭项 |
| --- | --- | --- |
| 分包、会话、取消屏障、运行配置 | 已实现，已有 WSL 脱机 PASS 记录；nav 只转发 | 软件 PASS 不包含设备、实时性或飞行 |
| Pi 5 原双目采集/深度 | 2026-10-07 台架记录：10/10 启停、1800.17 s 连续运行及记录内性能门限 PASS | 独立距离尺测、曝光时延、OpenVINS 原图共载和全导航负载 NOT RUN |
| 飞控 IMU、采样时间、OpenVINS | MAVROS 上行与时间门控已实现；订阅/脱机验证通过 | 同步真实双目＋飞控 IMU 数据、相机—IMU外参/时间偏移、有效 VIO 初始化/漂移 NOT RUN |
| EGO 与 `px4_position` | 固定场景 SIH 到目标与降落已有 PASS；完整地图仅用于测试准入 | 真实 VIO/地图、机体包络净空、跨场景硬件闭环 NOT RUN |
| `companion_attitude` | 控制、预算、起降与故障处置已有 WSL/MAVROS/SIH 证据 | 实测推力/机体参数、姿态参考、独立安全接管、Pi 全链负载及实机闭环 NOT RUN；完整姿态森林矩阵未跑 |
| 平台速度交接与计算释放 | 脱机、回放、合成输入 ROS/MAVROS/SIH 已执行 | 真实速度确认适配器 BLOCKED；完整 OpenVINS/EGO 往返＋平台收尾、真实进程释放和落点精度 NOT RUN |
| 下视标定与捕获 | IMX219 标定入口已实现；70 mm Tag/局部特征为 TEST-ONLY 候选 | 本页标定入口记录尚未完成真实内参；0.5～1.5 m 全捕获区间未通过，近地续跟踪未接控制 |
| 机体、动力、安全光流 | 选型与验收入口已记录 | CAD/质量闭合、推力台、MTF-02P 融合、独立停桨/失联接管未关闭；见 [需求表](REQUIREMENTS.md) |
| 比赛任务与能源 | 检测/跟踪/视觉伺服、返库卸载、充电循环和能源管理待完成 | 30 s 单次飞行、10 s 悬停及整机规则验收未关闭；长时间 SIH 不能作为规则计时 PASS |

表中设备结果引用下方日期化记录，本轮不连接设备重新核验。历史 PASS 只覆盖各批次注明的版本、输入与负载。

## 下一步

1. 完成真实输入：IMX219 内参与新姿态复核写独立候选目录；导航另准备原双目＋飞控 IMU 的真实数据，核验流频率、TIMESYNC、曝光时域、外参与时间偏移。单目标定不替代双目/IMU标定。
2. 验收 OpenVINS 初始化、米制方向、输出年龄/频率、漂移和重置；以同一套数据核对位姿—深度—地图。失败先查采样、标定与计算积压，不放宽已有门限。
3. 在 Pi 5 测量包含原图订阅、OpenVINS、深度、EGO 与控制的全链负载；保留分段时延、消息年龄、CPU/RSS和退出记录。现有 MJPEG 深度台架不能代替本项。
4. 按 [实机接入入口](../companion/ros2_ws/src/boom_birds_bringup/README.md) 逐步核验机体/推力参数、姿态参考和 PX4 独立接管，再开展脱桨、系留和短距离闭环。旧位置路线另须验证航向与原点；姿态路线不回传外部视觉。
5. 平台先完成真实标定与捕获区间验证，并实现、验证速度回读适配器；再验证完整导航往返、末段输出互斥、真实计算释放及近地 Land。局部特征像素目标不能替代板位姿。
6. 并行完成 CAD/质量、动力/供电、安全光流和停桨证据；按 [RULE_BASELINE](RULE_BASELINE.md) 核对 30 s/10 s、返库卸载和充电循环。RK3576、最终相机与能源选型用实测质量/功耗/负载决定。

## 2026-10-08 项目核查与注释维护（WSL）

对照当前源码核查架构、需求与状态入口；更新四份文档，将旧路线/验收表标为历史。17 个源码文件补充坐标变换、命令剩余有效期、单槽/丢旧帧、交接回读、计算释放与标定限制；修正 R1 方向注释。相对本轮开始时源码，AST 完全相同，未改算法、参数或门限；原未提交双目性能工作保留。

证据根：`/home/waterc/bb_build/project-review-20261008/`。`baseline.diff` 保存原工作区补丁，`review-only.diff` 只含本轮改动；`commands.json` 记录执行命令。七个项目包构建/安装在该目录，EGO/消息依赖使用未改动的 `ego-single` 下层安装；新旧 interfaces 消息/服务定义逐字节一致。

| 检查 | 结果 | 证据 |
| --- | --- | --- |
| 七个项目包独立构建 | PASS | build.log；未重新构建 OpenVINS/EGO |
| 统一脱机入口 | PASS，11/11 组，零跳过 | offline/report.json；nav 853、control 132、sensing 47、bringup 28、sim 26；其它组含重复覆盖，不合计成独立用例总数；source_unchanged=true |
| EGO 地图 / C++ 轨迹 | PASS，99 项 / 1 项 | map-behavior-final.log、trajectory-validation.log；上行统一入口明确排除这两组，另用原 EGO 产物执行 |
| IMX219 单目/RAW | PASS，7 项，零跳过 | mipi-calibration.xml |
| 注释/语法/文档 | PASS | static-audit.json：17 文件 AST 无变化、222 个 Python 编译检查、24 个 shell 语法检查、7 个 package.xml、20 份文档的相对路径/锚点；git diff --check |

地图检查首轮未加载 ROS 动态库环境，启动失败；原日志保留为 map-behavior.log，显式加载 ROS/EGO 安装环境后通过。脱机检查期间源码不变；检查后只补写本节结果。本轮 SIH、Pi/相机/飞控连接与硬件/飞行验收均 NOT RUN，不更新设备实测结论。

## 2026-10-08 IMX219 单目内参标定入口

已读取《验证 MIPI 摄像头接入》的设备上下文，并连接 Pi 5 CAM/DISP 1 的 IMX219。沿用 A4 棋盘：11×8 内角点、20 mm 方格。独立单目标定网页部署于 `/home/gmaster/boombirds/tools/mipi-calibration-20261008/`，通过 SSH 转发访问本机 8083；命令与样本格式见 [标定模块](../companion/ros2_ws/src/stereo_depth/README.md#9-imx219-单目内参标定)。

当前模式为 1640×1232 / pRAA，传感器裁剪回读 (8,8)/3280×2464。曝光 1600 行、模拟增益 192、数字增益 256；开始收样后锁定。RAW 取帧与转图分线程，只保留最新帧；网页 820×616，点击查看原像素局部，检测/保存仍为 1640×1232。Pi 检查未发现其他双目/VIO/控制进程，USB 双目未占用，未暂停系统服务。

WSL 软件回归 PASS，17 项：原双目 10 项，单目/RAW 7 项。实机预览已运行；10 次短状态采样回读取帧约 30 Hz、转图约 8 Hz，主机图像年龄 0.074～0.255 s。时间来自 RAW 管道读完，不是曝光时间或屏幕端到端时延。证据位于 `/home/gmaster/boombirds/evidence/mipi-calibration-20261008/{source-preflight.json,preview-check.json}`。

本次入口可采集真实图像，不代表内参已经完成。准备检查时样本为 0；实际角点覆盖、求解、留出误差和新姿态复核均待用户摆放棋盘，当前 NOT RUN。结果只写独立 `mono_<时间>/` 候选目录，不覆盖原 USB 双目标定或自动启用下降配置。实际距离精度、相机安装外参、同步、VIO 和飞行未执行。

## 2026-10-07 平台降落脱机优化与局部特征原型

有界历史/分段日志、图像单槽处理、异步资源释放及状态新鲜度校验已实现。四包构建通过，1060 项回归通过、零跳过。1152 个图像配置、9 个标定扰动和 18 个引导组合已执行。虚拟耐久为 180000 周期（3600 s 虚拟输入），另有实际 60 s 图像过载；预热后 RSS 范围 308 KiB，仅为 WSL 软件证据。

ROS/MAVROS/SIH 的七项合格记录覆盖位置/姿态导航、返航、服务晚到、失标和测距突变；实际停止两个受管合成输入进程，最终 landed/disarmed。保留板观测中断导致的交接失败，不把整批矩阵记为通过。完整 OpenVINS/EGO、真实 VIO/相机释放和硬件验收仍为 NOT RUN。

按 0.5～1.5 m 完整 Tag 捕获后延续中心附近特征的方案，已补 TEST-ONLY 像素跟踪原型，60 帧裁剪平移测试通过；尚未接入下降许可。全捕获区间尚未通过。当前小尺寸候选为 tag36h11 ID7 黑框边长 70 mm，不含白边，采用独立测试配置；其合成 0.5/1.0 m 位姿有平面歧义，全捕获区间未通过。上述联合/耐久记录仍使用原 300 mm 配置。

证据根 `/home/waterc/bb_build/platform-opt-20261007/`；[详细记录](PLATFORM_OFFLINE_OPTIMIZATION_20261007.md)、[局部跟踪接口](PLATFORM_FEATURE_TRACKING.md)。以下下视板初始实现一节保留上一批证据和范围。

## 2026-10-07 下视 AprilTag 与平台速度交接（WSL 脱机）

配置化下视板检测/PnP、相机外参、平台落点与朝向、测距安装位置、速度/变化率及场地门限已实现。原点起飞和往返保留 VIO；末段通过单一接口撤销导航输出、预发零速度并核对速度控制/实际目标回读。确认前检查 VIO/IMU/双目依赖，确认后才开放计算释放。异常输入撤销下降；持续故障、Tag 近地不可见或测距盲区转原生 Land。disarm 需低高度、低速度及连续 landed 回读，COMPLETE 另需 armed=false。

模块接口、配置和命令见 [控制包](../companion/ros2_ws/src/boom_birds_control/README.md)。默认 platform_config_file 为空，任务平台开关关闭；真实入口拒绝合成/缺证据参数，并因硬件速度确认适配器未验证继续拒绝。没有部署 Pi、连接相机/飞控、修改飞控参数或操作真实解锁/飞行。

本次证据根：`/home/waterc/bb_build/platform-20261007/`；`report.json` 汇总脱机、回放、模拟飞控与 SIH，`sha256-manifest.json` 保存清单。测试源码快照与哈希保存在该证据根；本次平台功能单独提交，此前双目性能工作仍保留在工作区。

| 检查 | 状态 | 本次证据与限制 |
| --- | --- | --- |
| 四包构建/安装入口 | PASS | control、sensing、bringup、sim；build-delivery.log；launch 参数解析通过 |
| 软件回归 | PASS，1,028 项，零跳过 | control 84、sensing 39、bringup 25 + 新资源服务 1、nav 853、sim 26；各 XML；资源服务定向检查与 bringup 原回归的重复项不重复计数 |
| 图像/rosbag 回放 | PASS，4 项（计入 sensing） | replay-pass.xml；保留 Image/CompressedImage CDR、图像、逐帧观测及配置 SHA256；使用 header.stamp |
| 模拟飞控矩阵 | PASS，8/8 | fake-pass：原点、返航、错误 ID、过期、丢标、位姿/测距跳变及交接失败；完整输入/输出和子进程存活记录 |
| 输出互斥/计算释放 | PASS（脱机） | MAVROS 发布者切换和旧输出拒绝；确认前不终止子进程；确认后的进程退出与 ROS release 服务核验 |
| SIH 合成图像起降/返航 | PASS，2/2 | sih-image-final/{origin,return}；真值渲染图像→检测/PnP→速度参考→PX4；Tag 超出近地画面后原生 Land，最终 landed/Disarmed |
| SIH 异常 | PASS，3/3（合格执行） | sih-image-final/{tag_loss,range_jump}、sih-handoff-image-retry；下降许可撤销，交接失败不释放依赖，最终 landed/Disarmed |
| SIH 失败记录 | FAIL，保留 | 最初两次直接位姿输入的 handoff_timeout；图像 handoff_failure 首次因健康检查未就绪拒绝模拟解锁，未进入故障测试。修正回读去重/关联和测试准入后复验，不放宽引导门限 |
| 完整 OpenVINS/EGO + ROS 平台收尾 | NOT RUN | ROS 桥接/任务门控有脱机测试，独立 SIH 用 PX4 位置目标模拟往返；不证明姿态导航到速度降落的整段运动链 |
| 实际 OpenVINS/双目进程释放 | NOT RUN | owned-process 与服务测试用独立 Python 子进程；SIH 无真实 VIO/相机进程，release 标为 logical TEST-ONLY |
| 下视真实标定/精度、Pi 实时性、飞控速度估计/光流测距融合、时延、近地可见范围与落点精度 | NOT RUN | 留待接机；本轮结果不作为硬件或飞行证据 |

OpenVINS 首次回归因测试环境缺动态库路径失败；补载独立 OV 安装环境后 nav 853 项通过。SIH 的五个合格用例使用相同引导器、回读器和板配置哈希；最后交接失败复验仅增加 PX4 pre_flight_checks_pass 等待，不改变控制算法。SIH 启动脚本仍按自身默认参数初始化，测试程序未写参数。

## 2026-10-07 Pi 5 感知性能优化与复验

分段计时定位到原图消息打包、ROS 传输及旧帧积压。采集端改为连续取帧、发布时取最新帧；回放仍按序消费。原图先转为连续内存再交给 CvBridge，XYZ 消息改用字节数组。深度支持单通道校正和匹配，原图路径使用严格同时间戳配对，可选单槽工作线程替换尚未处理的旧输入。新增可选 MJPEG 传输：发送原始相机载荷，深度端按完整尺寸解码，不重新压缩、不降低解码尺寸。原始 Image/CameraInfo 接口保留，默认配置不变。

SIGINT/SIGTERM 由节点处理：使用有限等待的 `spin_once`，先停止取帧/处理线程，再销毁节点和 ROS 上下文。回归覆盖继承 SIGINT 忽略状态、空输入退出、最新帧替换、严格配对及同一 JPEG 在两种输入路径下的深度一致性。WSL / Pi 定向回归各 100 项通过，无跳过；感知/深度两包构建完成，Pi XML 已取回核验。

实机复验保持原双目、旧 `candidate.npz` 和 `2560×960 / MJPG`，驱动回读 60 FPS、发布上限 30 Hz。正常测量使用原 MJPEG 传输、OpenCV 2 线程、采集待发布队列 1、图像 QoS 深度 2、单槽待处理输入，深度仍为 `320×240`；深度、完整 XYZ 和彩色预览均发布。原图按订阅需求发布，不含 OpenVINS 原图共载；接口录包阶段恢复原图发布，单独检查，不计入性能窗口。完整命令见 `bench-commands.jsonl`，参数副本见 `prepared-hardware/camera.yaml`。

| 检查 | 状态 | 证据与限制 |
| --- | --- | --- |
| WSL / Pi 定向回归 | PASS，各 100 项 | `regressions-wsl.xml`、`regressions-pi.xml`；含真实 ROS 子进程退出测试 |
| 飞控接入准备 | PASS（文件准备），5 项静态检查通过 | `prepared-hardware/`、`hardware-preparation.xml`；IMU 仍来自飞控，端口/外参/时间偏移/控制参数待实测 |
| 10 次实机启停 | PASS，10/10 | 所有进程退出码 0，排空后新深度为 0，重启时间戳递增，相机单一占用并释放；`restart-cycles.json` |
| 30 分钟连续运行 | PASS，1800.17 s | 21:33:30–22:03:30；28,227 个深度结果，采集/时间戳/观察检查错误均为 0，发布者各 1；`bench-execution.json` |
| 深度平均 ≥15 Hz | PASS，15.6802 Hz | 完整窗口平均；逐秒计数 P50/P95/max=16/16/17，不表示每秒最低帧率保证 |
| 消息年龄 P95 ≤150 ms / 最大 ≤300 ms | PASS，113.72 / 215.33 ms | P50=95.53 ms；驱动时间戳映射到 ROS 至观察端接收，不是曝光端到端时延 |
| 整机 CPU 中位数 ≤70% / P95 ≤85% | PASS，43.00% / 64.72% | 最大 82.28%；包含观察者和资源采样 |
| 原图/内参及深度接口录包 | PASS | 14.14 s 实录，9 类话题；383 组原图/CameraInfo 四元组、123 组深度/CameraInfo/预览/XYZ，检查错误 0；`bag-validation.json` |
| 最终退出与相机释放 | PASS | 采集/深度/观察者/录包退出码均 0，源码与标定未变化；`completion-audit.json` |

驱动实际取帧 58.79 Hz，序号缺口 11；源状态首尾快照差分主动丢积压 51,801 帧，深度待处理替换 25,773 帧。两个替换计数为不同阶段，不能加总当作设备丢帧；快照窗口与逐帧统计窗口也有边界差异。采集回调耗时 P50/P95/max=2.77/4.37/19.73 ms，深度完整回调为 63.06/70.31/110.25 ms，其中 JPEG 解码 P50=11.37 ms、含缩放/校正的深度处理 P50=35.73 ms；嵌套阶段耗时不能再次相加。

进程 CPU 中位数为采集 17.00%、深度 142.98%、观察者 7.00%（单核 100%）。RSS 峰值分别约 165.14/179.02/67.93 MiB，测量窗口起止变化分别约 +0.625/+0.063/+0.082 MiB；仅描述本次运行，不作长期泄漏结论。最高温度 51.8°C，1800 个资源样本降频回读均为 `throttled=0x0`。此前 30 分钟基线包含更重的观察者和三次录包，本次结果不能当作只改变生产代码的严格 A/B 对比。原图录包阶段深度平均约 8.7 Hz，属于不同负载；本轮通过不覆盖 VIO 共载性能。

WSL 证据根 `/home/waterc/bb_build/camera-opt-20261006-1920/`，设备证据根 `/home/gmaster/boombirds/evidence/camera-opt-20261006-1920/`；`bench-statistics.json` 保存完整分布与门槛判定。设备重启/断电中断的约 17 分钟批次保留在 `interrupted-run-20261007-212954/`，早期短跑强制终止和辅助脚本失败记录也保留，不计作本次通过。完整归档 SHA256 与设备一致，366/366 文件的尺寸与 SHA256 全部通过；`transfer-verification.json` 保存校验结果。

飞控接入文件保留 `dry_run=true`、禁解锁和未实机验证标记，不填入假外参或仿真增益作为实测值。距离精度、地图输入、相机—IMU同步、VIO、飞控控制、物理拔插/遮挡和飞行为 NOT RUN；本轮未启动 MAVROS、OpenVINS、EGO 或控制，也未刷写、提交或推送。

## 2026-10-06 Pi 5 原双目感知台架

原双目沿用 `live_20260916_210120_642136/candidate.npz`，未重新标定、调焦或改变安装几何。标定 SHA256 仍为 `d27f24de5ba69546274dc6f502ce27a7b36e3aae5f39220252c9e01af675aa7d`，与旧设备目录一致。输入为 `/dev/video0`、完整拼接 `2560×960 / MJPG`，每目 `1280×960`；驱动回读 60 FPS，ROS 发布上限 30 Hz、队列深度 2，深度输出 `320×240`。

部署目录仍为 `releases/44b2a44`，本次实际源码 HEAD 为 `83ae60a` 加采集修复。真实 ROS 入口原先把 `StereoFrameClock` 传给需要设备、尺寸和时间基的 `V4L2FrameSource`，启动报 `unexpected keyword argument timeout_s`；现改为按构造接口传参，并把 `capture_timeout_s` 传到底层取帧等待。随后发现 MMAP 忽略 `QUERYBUF.m.offset`，多个缓冲均映射偏移 0，日志出现 JPEG 损坏警告；现按每个缓冲的 length/offset 映射。新增真实节点构造路径、超时传递和不同偏移载荷测试，`m.offset` 由 C 头文件 `offsetof()` 核对。未修改控制链、标定或默认配置。

| 检查 | 结果 | 证据与限制 |
| --- | --- | --- |
| 感知包构建 | PASS，WSL / Pi 各 1 包 | `build_all.sh --packages-select boom_birds_sensing` |
| 定向回归 | PASS，WSL / Pi 各 74 项；WSL ROS 发布/内参另 12 项 | WSL 交付根 `capture-fix.xml`、`ros-regressions.xml`；Pi `capture-fix-pi.xml`（已取回 WSL 交付根）；无跳过 |
| 独立预览 | PASS，5 分钟 | 深度 FPS P50/P95=21.34/21.62；compute P50/P95=45.12/54.01 ms；3 组原图/深度/XYZ，错误 0 |
| V4L2 时间戳 | PASS（驱动时域可追溯） | 修复后 300 帧，MONOTONIC、严格递增；不代表曝光时刻或相机—IMU同步 |
| ROS 连续运行 | PASS（本次运行与数据结构），1807.29 s | 17:25:39–17:55:47；采集错误、JPEG 警告、接口检查错误及重复发布者均为 0 |
| 三段 rosbag 内容 | PASS | 左右图及各自 CameraInfo 与旧标定一致；主深度 `32FC1` 米制/NaN、预览 `bgr8`；592 对 XYZ/Z 与深度、无效值一致 |
| 停发 / 重启 | PASS（行为） | 停止后观察 11.29 s，排空 2 s 后新图像/深度/XYZ 均为 0；原参数重启观察 60.08 s，恢复新数据、时间戳递增、单一发布者 |
| 进程退出 | PARTIAL | SIGINT 使 ROS 上下文失效时，采集/深度回调仍尝试 publish，退出码 1；已释放相机，不能记为优雅退出通过 |

30 分钟包含观察者和三次短录包负载。实际驱动取帧 44.16 Hz，驱动序号缺口累计 26,431；源状态快照差分发布 34,502 帧、主动丢弃积压 45,256 帧。观察端左右原图约 16.99/16.96 Hz，深度约 11.93 Hz，部分原图/CameraInfo 未收齐；仅完整同时间戳四元组可证明配对，不能把观察端缺帧都归因于驱动。

匹配与重投影 compute P50/P95/max=38.86/51.25/99.38 ms（不含校正和 ROS 打包/传输）；深度消息年龄 P50/P95/max=0.692/0.803/2.079 s（V4L2 时间戳映射到 ROS 后，至观察端接收；不是曝光端到端时延）。CPU 中位数为采集 103.8%、深度 152.1%、观察者 56.4%（单核 100%）；RSS 峰值分别约 225.7/237.2/188.6 MiB，起止增长分别约 33.8/21.6/21.0 MiB，不据此认定内存稳定或泄漏。最高温度 59.5°C，1806 次降频回读均为 `throttled=0x0`，CPU 频率回读均为 2.4 GHz。当前丢帧和消息年龄不作为飞行实时性通过证据。

设备证据根 `/home/gmaster/boombirds/evidence/camera-20261006-ros-verified/`，WSL 交付根 `/home/waterc/bb_build/camera-20261006-ros-verified/`；`report.json` 汇总定义与统计，`commands.jsonl` 保存实际命令，`sha256-manifest.json` 核对文件。原录包未及时响应 SIGINT，实录 22.78–23.41 s；原包保留，`excerpt-{start,middle,end}-10s/` 为其首 10 s 的原始 CDR 截取，选择范围和计数见 `excerpt-manifest.json`。

证据传输曾因网络断开中止；用户再次重启设备后恢复，未重跑相机测试。完整归档已断点续传到 WSL，归档 SHA256 与设备一致，77/77 个清单文件的尺寸和 SHA256 全部通过。原始三段包、首 10 s 截取包和测量脚本已取回；校验记录见 `transfer-verification.json`。

早期网络失联、构造失败和 JPEG 警告批次保留于设备 `camera-20261006-{164500,ros-resume,ros-final}/`；用户重启后恢复连接，网络失联根因未确定。后续需分别定位驱动缺帧/处理积压和 SIGINT 退出竞态，不能用本轮运行通过覆盖这些问题。

独立尺测、物理拔插/遮挡、曝光时间偏差、相机—IMU同步、VIO、世界地图、飞控控制及飞行均未验证。本轮未启动 MAVROS、OpenVINS、EGO 或控制任务。

## 2026-10-05 Pi 5 Companion 部署（无相机、无飞控）

设备为 Raspberry Pi 5 Model B Rev 1.1，Ubuntu 24.04.4 LTS / aarch64 / ROS 2 Jazzy。完整源码及固定子模块通过 Git bundle 部署，在设备上构建，未复制 WSL 二进制。构建基线母仓库 `44b2a44`、EGO `c1ffb98`、OpenVINS `7907e70`；本轮只修改观察测试与部署文档，不改生产代码。

部署根 `/home/gmaster/boombirds/releases/44b2a44`；`source/` 保存 Git 源码，`runtime/main/` 与 `runtime/ov/` 为独立构建前缀，`runtime/venv/` 为 Python 环境，`evidence/` 保存全部尝试。入口 `/home/gmaster/boombirds/current/activate.sh`，加载命令见 [工作空间说明](../companion/ros2_ws/README.md#环境与部署)。旧 `/home/gmaster/boom_birds_ws` 保留，15 个旧源码/标定文件的部署前后 SHA256 一致。没有配置开机启动或启动实机任务。

| 检查 | 结果 | 证据与限制 |
| --- | --- | --- |
| 主链原生构建 | PASS，17 包 | `build-main.log`；接口、EGO、项目包、深度、地图工具与可视化 |
| OpenVINS 原生构建 | PASS，3 包 | `build-openvins.log`；`ov_core`、`ov_init`、`ov_msckf`；二进制为 aarch64 |
| 全量脱机 | PASS，13/13 | `offline-final/report.json`；导航 836 项、bringup 23 项、地图 99 项及 C++ 轨迹检查；零跳过，`source_unchanged=true` |
| ROS/MAVROS 定向复验 | PASS，3 项 | `targeted-discovery-final.xml`；实际 ROS 采样、过期状态拒绝与回环 setpoint；WSL 同一观察测试 2 项通过 |
| MAVROS + UDP 假 PX4 | PASS | `wire/report.json`；姿态/位置、坐标、推力缩放、输出互斥、时间配对与故障停发；对端仅在回环地址 |
| 无输入 / 缺实机文件 | PASS（拒绝路径） | `bench-no-input.json` 为 NOT_READY、退出码 2；`hardware-missing-config.log` 在设备进程启动前拒绝 |

首轮 `offline/report.json` 为 11/13，失败报告保留。MAVROS 2.15.1 与设备旧 libmavconn 2.14.0 混用，导致 `mavros_node` 启动时符号查找失败；更新 libmavconn 2.15.1 与 MAVLink 2026.8.8 后通信通过。观察测试的 2 s 窗口曾在 DDS 尚未发现发布者时结束，报告所有输入为零；测试改用工具默认的 15 s 窗口，并等待全部订阅匹配后开始状态停发计时。生产采样年龄门限未调整。

部署期间系统后台更新，SSH 一度中断；恢复后重新核验依赖并执行上述最终检查。当前内核 `6.8.0-1065-raspi`，未发现待重启标记。最终软件清单为 `evidence/deployment-final.json`，日志另存 WSL `/home/waterc/bb_build/pi-deploy-20261005/`。

摄像头与飞控未连接：未验证真实双目输入、TIMESYNC、OpenVINS 初始化、外参/时间偏移、悬停推力、实时负载或飞行。接入后先按 bringup 说明补齐真实标定和端口，执行脱桨 dry-run 与只读报告；默认禁解锁、禁实机动作不变。

## 2026-10-05 提交前全量检查

证据根 `/home/waterc/bb_build/ego-single/evidence/release-1005/`。主工程 build/install/log 均使用 `ego-single` 前缀，OpenVINS 使用 `ov` 前缀。检查未连接设备、启动真实相机或刷写固件。

本轮修正只读接入报告的 ROS 订阅回调：原回调的第二个默认参数会被 rclpy 当作消息元信息参数，覆盖话题名。改为单参数闭包，新增真实 ROS 回调测试，分别检查正常采样与状态过期；输入均为测试替身。

| 检查 / 场景 | 结果 | 证据与限制 |
| --- | --- | --- |
| 主工程构建 | PASS，17 包 | `build-project.log`；EGO、接口、项目包、地图工具、`pose_utils` 与 `odom_visualization` |
| OpenVINS 构建 | PASS，3 包 | `build-openvins.log`；`ov_core`、`ov_init`、`ov_msckf`，不代表真实 VIO 初始化 |
| 全量脱机 | PASS，13/13 | `offline-final/report.json`；导航 836 项、bringup 23 项、地图行为 99 项及本次 build 下 C++ 轨迹校验；零跳过，`source_unchanged=true` |
| MAVROS + UDP 假 PX4 | PASS | `wire/report.json`；姿态/位置报文、坐标转换、推力缩放、输出互斥与 boot 时间配对 |
| Python / shell 语法、文档链接、EGO 多机残留 | PASS | `syntax.json`、`docs-links.json`、`source-audit.json` |
| 姿态模式正常任务 `(1,0,1.5)` | PASS（补跑） | `sih-attitude-final/`；COMPLETE、goal_reached=true，最近目标距离 0.056 m，VIO 闭环降落后 Disarmed |
| 姿态模式定位断流 | PASS（故障处置） | `sih-vio-loss/`；实际终止真值源，sensor_link 闭锁，后备 Land、Disarmed；goal_reached=false |
| 姿态模式人工取消 | PASS（补跑、取消处置） | `sih-cancel-resumed/`；实际注入，manual_cancel 闭锁，最终 Disarmed；goal_reached=false |
| 姿态模式 Offboard 中断 2 s | PASS（补跑、故障处置） | `sih-offboard-final/`；实际注入，setpoint_link 闭锁，未恢复旧轨迹，最终 Disarmed；goal_reached=false |
| 原位置模式正常任务 `(1.5,0,1.5)` | PASS | `sih-position/`；记录器持续确认 COMPLETE、goal_reached=true，最近目标距离 0.045 m，最终 Disarmed |

`sih-summary.json` 核对目标、最终状态及闭锁后发送计数。三个有效故障场景在闭锁 200 ms 后均无新增发送，最终 landed_state=1。正常姿态场景 uORB 回读为 attitude=true、position/velocity/acceleration=false、direct_actuator=false。

本轮 SIH 存在间断失败，不能按补跑结果宣称重复运行稳定：`sih-attitude/` 到达目标后在降落阶段因 VIO 输入间断闭锁；`sih-cancel/`、`sih-cancel-final/`、`sih-offboard-loss/` 在注入前闭锁，故障未注入；`sih-cancel-review/` 已调用取消，但 VIO 故障先闭锁，不计人工取消通过。真值源日志曾记录飞控位置与姿态年龄超过 0.25 s，间断根因未确定，未放宽任何年龄或闭锁门限。位置模式最终一次服务查询超时，任务状态以持续记录器与 PX4 回读核验。

实机台架须检查遥测连续性、相机曝光时间、TIMESYNC、VIO/飞控姿态配对和后备 Land；当前仅具备脱桨 dry-run 接入入口。未验证实机、真实 OpenVINS、Pi 5、姿态模式完整森林矩阵。下面保留前一轮实现验收及历史批次。

## 2026-10-05 Companion 姿态控制与实机接入入口

新增 `companion_attitude`：VIO 世界系位置/速度闭环 → MAVROS 姿态＋归一化推力 → PX4 姿态/角速度闭环。`px4_position` 保留。正常起飞、保持和降落使用 VIO，PX4 位置只保留为独立状态回读；没有外部视觉回传。参见 D-058，操作命令与标定要求见 [实机验证入口](../companion/ros2_ws/src/boom_birds_bringup/README.md)。

软件接入入口已实现：真实文件静态核验、单一双目采集、OpenVINS、位姿适配、EGO、控制和生命周期启动；只读报告检查连接、采样时钟、源数量与姿态参考。默认 dry-run、禁解锁、禁实机动作。实机文件、端口、机体参数和独立安全接管仍需设备侧验证，未执行台架或飞行。

本轮使用 `/home/waterc/bb_build/ego-single/{build,install,log}`；接口新增 VIO 控制反馈、降落 HOLD 字段与受落地回读约束的 DISARM。接口、EGO 消费者及受影响包已重建。OpenVINS 使用独立前缀 `/home/waterc/bb_build/ov/install`。证据根 `/home/waterc/bb_build/ego-single/evidence/attitude-1005/`。

| 检查 / 场景 | 结果 | 证据与限制 |
| --- | --- | --- |
| 受影响包构建 | PASS | 接口/EGO 重建及 `attitude-build-delivery*.log`；日志位于上述构建根 |
| 全量脱机 | PASS，13/13 | `offline-final/report.json`；导航 836 项，零跳过，检查期间 `source_unchanged=true` |
| 契约文档更新后的配置/IMU复验 | PASS，43 项 | `contract-final.xml`；接口契约已重新安装 |
| 新增控制、起降、标定与生产者重启测试 | PASS，24 项 | `targeted-final.xml`；也包含在对应脱机组内 |
| 真实 MAVROS + UDP 假 PX4 | PASS | `wire-final/report.json`；姿态四元数转换、推力缩放、输出互斥、dry-run/心跳失效、PX4 boot 时间配对；不是实机测量 |
| 姿态模式短距离 `(1,0,1.5)` | PASS | `sih-receipt/`；COMPLETE、goal_reached=true，最近目标距离 0.041 m，正常闭环降落后 Disarmed |
| 姿态模式定位断流 | PASS（故障处置） | `sih-vio-loss/`；实际杀死真值源，vio_or_attitude_invalid 闭锁，后备 Land、Disarmed；goal_reached=false |
| 姿态模式人工取消 | PASS（取消处置） | `sih-manual-cancel/`；manual_cancel 闭锁，后备 Land、Disarmed；goal_reached=false |
| 姿态输出中断 2 s | PASS（故障处置） | `sih-offboard-loss/`；setpoint_link 闭锁，中断结束未恢复旧轨迹，最终 Disarmed；goal_reached=false |
| 原位置模式短距离 `(1.5,0,1.5)` | PASS | `sih-position-regression/`；COMPLETE、goal_reached=true，最近目标距离 0.010 m，最终 Disarmed |
| 缺实机配置 / 无输入观察 | PASS（拒绝路径） | `hardware-missing-config.log` 在设备节点启动前拒绝；`bench-no-input.json` 为 NOT_READY |

`sih-final-summary.json` 记录各场景的目标、实际位置、最终状态和停发统计。三个故障场景在闭锁 200 ms 后均无新增发送。正常场景最终 uORB 回读为 attitude=true、position/velocity/acceleration=false、direct_actuator=false；接地收尾推力为 0.08。该推力是 SIH 参数，不能移作实机标定。

开发阶段曾在解锁附近因真值采样时钟变化闭锁，曾到达目标但降落超时；已修正测试时钟路径、重复遥测请求、推力缩放查询频率和接地收尾，并复跑上述最终场景。旧日志保留。`sih-cancel/` 的规划取消在目标完成后才尝试注入，实际未注入，不计入故障验收；人工取消另用有遥测前置条件的运行验证。早期 `offline/` 检查期间修改过源码，不能代替 `offline-final/`。

SIH 使用真值与合成双目，测试时钟路径仅限已核验的 SIH PID，状态标记 TEST_ONLY；不验证真实 OpenVINS、真实采样年龄、Pi 5 或飞行安全。实机入口仍要求 TIMESYNC 和 VIO/飞控姿态采样配对。未复跑姿态模式完整森林矩阵，也未运行远端 CI。

## 2026-10-03 EGO 单机化

单机化基于 EGO `385eb2b`，提交 `c1ffb98`；母仓库 gitlink 固定该版本。删除多机轨迹广播/接收、顺序启动、机间避碰、编号参数、共享轨迹消息与三个专用包；单机建图、障碍避碰、多候选优化、完整曲线检查和会话取消屏障保留。修改路径见 [补丁索引](EGO_FORK_PATCHES.md)。

`Bspline` 删除 `drone_id`，`MultiBsplines` 删除；规划节点固定为 `ego_planner_node`。使用新构建前缀 `/home/waterc/bb_build/ego-single/{build,install,log}`，不要混用旧消息产物。构建/加载命令见 [工作空间 README](../companion/ros2_ws/README.md#单机-ego-安装前缀)。证据根为 `/home/waterc/bb_build/ego-single/evidence/`。

| 检查 / 场景 | 结果 | 证据与限制 |
| --- | --- | --- |
| 新前缀构建 | PASS | 14 个链路包；追加地图行为测试、`pose_utils`、修改后的 `odom_visualization` 与 `mockamap` 构建通过。详细日志在上述 `log/` |
| 多机残留与实际 ROS 图 | PASS | `source-audit.json`、`graph/report.json`；规划器无广播/共享轨迹端点，旧消息和三个专用包不可发现 |
| 全量脱机 | PASS，13/13 | `offline/report.json`；导航 836 项、地图行为 99 项和 C++ 轨迹校验通过，无跳过项，测试期间 `source_unchanged=true` |
| SIH 短距离 `(1.5,0,1.5)` | PASS | `sih-normal-2/`；COMPLETE、goal_reached=true，最近目标距离 0.027 m，最终 Disarmed |
| SIH 人工取消 | PASS（取消处置） | `sih-cancel/`；manual_cancel 闭锁停发，降落收尾后 COMPLETE、goal_reached=false，最终 Disarmed |
| SIH 30 m `reference_30m`，seed 3 | PASS（本次固定场景） | `sih-forest/`；COMPLETE、goal_reached=true，无恢复尝试，最近目标距离 0.096 m，最终 Disarmed |

SIH 的任务链为合成双目 → 深度 → EGO → MAVROS → PX4 SIH，未使用 EGO 原生仿真。采样位置统计见 `sih-summary.json`；森林机体中心到原始静态点云的最小距离为 0.742 m，仅为采样中心距离，不代表机体包络或真机验收。正常与森林任务实际位置和 setpoint 已记录，不能仅按发布样条认定运动。

首轮 `sih-normal/` 因新前缀缺 `mockamap` 在 launch 阶段失败，未进入飞行；中止时确认 Disarmed，日志保留。补齐地图包后重跑通过，构建命令已列出地图依赖。每轮清理后 UDP 14580 空闲；最终无任务链进程残留。

本轮未复跑完整森林/故障矩阵；真机、真实 VIO、Pi 5 与远端 CI 未运行。历史批次结果不替代上述本次结果。

## 2026-10-02 PX4 通信迁移到 MAVROS

- 生产控制 backend 选择 `fake` 或 `mavros`；不再选择 pymavlink backend。模式/解锁调用 MAVROS 服务，setpoint 发布到 MAVROS；SIH 真值和 IMU 复用同一 FCU 连接。旧 pymavlink 代码仅用于脱机历史回归、回放与测试对端。
- 保留项目 NED/FRD 契约、默认 dry_run/禁解锁、回环/SIH 授权、观测年龄与生命周期闭锁门限。MAVROS 插件参数通过服务设置并回读；IMU 使用原始 PX4 采样时间与 MAVROS 同步偏移，未同步或字段不全时拒发。
- 本机 MAVROS 2.15.1 与 GeographicLib egm96-5 已安装。八包构建通过；真实 MAVROS + UDP 假 PX4 的坐标、mask、ACK/状态回读、dry_run、心跳超时、重启与 IMU 时间检查通过。
本轮验证使用 `/home/waterc/bb_build/architecture/{build,install,log}`，OpenVINS 使用独立前缀 `/home/waterc/bb_build/ov/install`。证据根 `/home/waterc/bb_build/mavros-deps/`。

| 检查 / 场景 | 结果 | 证据与限制 |
| --- | --- | --- |
| 八包构建 | PASS | `build-architecture.log`；后续五个修改包重建见 `build-release.log` |
| 全量脱机 | PASS，13/13 | `offline-release/report.json`；导航 836 项、新增 MAVROS 控制/IMU 13 项通过，无跳过项，运行期间 `source_unchanged=true` |
| 真实 MAVROS + UDP 假 PX4 | PASS | `wire-final/report.json`；坐标、mask、ACK/实际解锁回读、dry_run、心跳超时、重启、IMU 时间/字段与插件参数回读 |
| 正常短距离 `(1.5,0,1.5)` | PASS | `sih-normal-3/`；COMPLETE、goal_reached=true，最终 PX4 回读 Disarmed |
| 人工取消 | PASS（取消处置） | `sih-cancel/`；manual_cancel 后闭锁停发，最终 Disarmed；不记目标到达 |
| Offboard 中断 2 s | PARTIAL | `sih-offboard-recovery/`；恢复接管并返回 EXECUTING，后续输入故障与 planner_timeout，未到目标，最终 Disarmed |

失败记录保留：`sih-normal/` 的 raw 订阅队列过小导致解锁回读超时；修正后 `sih-normal-2/` 使用旧 main 前缀未收到 PlannerStatus，改用 architecture 前缀后正常任务通过。脱机首轮 `offline-final/report.json` 为 12/13，导航 832/836：三处测试仍使用旧 backend/参数，另一个 OpenVINS 订阅检查未加载其安装前缀。迁移测试、重建并加载对应前缀后，最终全量通过。

MAVROS 迁移后未复跑完整森林矩阵；迁移前的 30 m 记录不替代本轮验证。真机、真实 VIO、Pi 5 和远端 CI 未验证。故障恢复后任务仍可能规划超时，不能据正常短距离通过认定所有场景正常。

## 2026-10-01 CI 修复与森林可达性复验

本节记录本次修复；下面的结构验收和森林 0/5 为历史批次，保留原始结果。

- CI 在 nav 安装阶段失败：CMake 安装了已迁走、Git 不跟踪的 `config` 目录。兼容包只安装现存 `launch`；构建日志和离线报告改写到已忽略的 `ci-report/`。CI 修复提交 `a2d7832`；集成修复提交 `cff6fbb`，固定 EGO `385eb2b`。
- 干净检出复现原错误，修复后 8 包构建、CI 10 组脱机检查通过。主工作区构建、最终 13/13 脱机检查（导航 836 项、sim 26 项，`source_unchanged=true`、无跳过项）、C++ `trajectory_validation` 和 XML schema 检查通过；完整观测年龄回归 51 项、场景准入回归 12 项通过。历史 lint 失败未记为通过。
- 森林参数分为 `dense`（原失败布局）和 `reference_30m`（20 棵柱状障碍、20 个圆环、40 m 范围、中心 x=0）。START 前保存完整场景点云和规划器实际参数，按同一体素分辨率、膨胀和顶棚检查起终点与自由空间连通；无效点云拒绝准入。此检查不向 EGO 提供地图或路径，不证明传感器可见性、动态轨迹、起飞全过程或机体净空。
- EGO 提交 `385eb2b`：warm start 首段按实测位置、速度和起始加速度锚定；项目周期重规划在既有时间门限之外要求推进一个控制点间距，或接近旧轨迹尾部。碰撞触发重规划不延迟，完整曲线校验与接管门限保留。
- 失败瞬间回环 UDP 位置和姿态报文仍连续到达，控制节点输入年龄却超过门限。分段计时定位到 ExecutionStatus 发布阻塞 0.272 s。控制节点在初始化 ROS 前默认设置 `RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`，显式环境配置优先，可靠 QoS 与取消屏障保留。接收端有效年龄计入消息传输时间，不能用刚收到消息掩盖旧观测。
- 未增大位姿 0.15 s、命令 0.2 s、规划预算 0.15 s 或 PX4 Offboard 丢失门限；未降低恢复最低 AGL 1 m、移动最终目标或减少原 dense 障碍。

本机证据根：`/home/waterc/bb_build/ci-forest-fix-20261001/`。干净检出报告 `ci-final-offline/report.json`；异步发布修复后的离线报告 `offline-async/report.json`；最终源码离线报告 `offline-release-2/report.json`；SIH 汇总 `matrix/async-final.jsonl` 与 `matrix/age-final.jsonl`；逐例核对 `matrix-audit-final.json`。

| 布局 / seed | 几何准入 | 到目标 | 最终 Disarmed | 异步发布修复后结果 |
| --- | --- | --- | --- | --- |
| dense 1、2、3、5 | 拒绝：GOAL_OCCUPIED | 未启动 | 是 | 未发送 START、未解锁 |
| dense 4 | 拒绝：DISCONNECTED | 未启动 | 是 | 未发送 START、未解锁 |
| reference 1 | 通过 | 是 | 是 | 无故障事件；中心采样净空 0.534 m |
| reference 2 | 拒绝：START_OCCUPIED | 未启动 | 是 | 未发送 START、未解锁 |
| reference 3 | 通过 | 是 | 是 | 无故障事件；中心采样净空 0.916 m |
| reference 4 | 通过 | 是 | 是 | 无故障事件；中心采样净空 1.026 m |
| reference 5 | 通过 | 是 | 是 | 无故障事件；中心采样净空 0.784 m |

原 dense 五例在当前完整地图模型下不应通过飞行任务。reference 到达率为 4/5，其中准入通过的 4 例全部到达；拒绝准入不能计为到达目标。中心到原始点云采样距离不是已测机体包络净空，优化器软距离 0.8 m 也不是硬验收门限。

异步发布后的完整 12 例包含正常任务、规划/深度/Offboard 注入恢复、人工取消、未知故障、低高度闭锁和五个 reference seed，全部最终 Disarmed，逐例目标、恢复和闭锁判断通过。完整观测年龄修复后追加 4 例：正常、深度恢复及 seed 4 两次复跑，均到达目标并最终 Disarmed；两次 seed 4 无故障事件。

最终检查首轮 `offline-release/report.json` 为 12/13：会话测试在规划器 ID 变化消息处理前收到一条在途旧命令，误判序号增加。测试改为先收齐在途命令，再在 Offboard 授权仍有效时验证序号持续停止、未接受新生产者；执行器代码未改。定向回归及最终 13/13 全量检查通过，首轮失败报告保留。

历史 `matrix/forest-final.jsonl` 中 seed 4 的低高度故障闭锁、`matrix/wire-repeat/` 中偶发失败及其诊断均保留。当前复跑只证明本机 WSL、固定场景和记录负载下的结果；异步发布不构成硬实时保证。真机、真实 VIO、Pi 5 性能和全新容器仍未验证。

## 2026-10-01 结构修整与最终独立验收

本节是结构修整时的历史验收状态；后续修复见上节，历史 PASS 不替代新修改的验收。
验收时源码和已有未提交修改均保留，验收基线母仓库 HEAD `c11761e8`，EGO HEAD `e96a455d`；PX4 `ff5b9484369b714763db9638517c08df0c242237`，未修改 PX4 源码。

### 计划关闭边界

| 阶段 | 当前结果 | 证据与限制 |
| --- | --- | --- |
| 1 固定回归、独立环境、依赖、CI | PASS（当前 WSL 独立 venv/构建） | 13 组全 PASS；导航 834、可移植行为 221、标定 10、地图行为 99，C++ trajectory_validation PASS。无用户目录隐式导入，系统 NumPy/OpenCV 保留；当前容器复验 BLOCKED，不称新 OS 镜像复现 |
| 2 配置与深度算法接口 | PASS（脱机/SIH） | contract 只写语义；runtime 校验共享话题、帧、阈值、原点；CameraInfo 三路同步与几何闭锁；StereoProcessor 公共逐帧接口及旧 CLI 兼容 |
| 3 控制协议与生命周期 | PASS（脱机/SIH） | 任务 UUID + 规划/执行进程 ID；可靠取消屏障；实际对齐回读；稳定锁点、Offboard 确认后激活、最终执行许可；本次取消回调修补另有定向测试 |
| 4 自动恢复与 SIH 矩阵 | PARTIAL | 正常/故障/反向 21/21 PASS；森林 0/5 到达目标。26 次最终均 PX4 回读 Disarmed；完整运行不等于任务成功 |
| 5 按职责拆包 | PASS（迁移与脱机归属检查） | interfaces/sensing/control/bringup/sim 已迁移；nav 只有模块/launch 转发；生产包不依赖 sim。整体飞行场景验收仍受上行森林结果限制 |

### 本次实现

- `ExecutionStatus` 回读新鲜实际/意图模式、落地、重启、ODOMETRY reset counter、实际坐标对齐 yaw/translation。SIH 原生 failsafe 原因通过受核验本机 PX4 进程只读获取；缺少观测拒绝自动恢复。
- 深度/里程计/setpoint 故障允许有界中断 Land/Return。恢复需连续 1 s 有效输入、AGL ≥1 m、速度 ≤1 m/s、最终执行许可；重新制动、确认 Offboard、重规划，旧轨迹退役。
- 每次故障最多 2 次、每次确认 3 s。恢复后连续 1 s 健康 EXECUTING 才重置事件预算；任务累计次数另行记录。人工取消/未知故障/落地/重启/坐标变化均撤销恢复资格。
- EGO 与 traj_server 各自生成进程 ID；活动任务中 ID 变化闭锁，重复变化状态不打断人工降落收尾。控制日志逐条保存序号、会话、轨迹、时间和 setpoint。
- 规划器 CANCEL 在同一回调发布控制 CANCEL，清缓存并形成序号屏障；下一 HOLD 使用新轨迹 ID。较高序号也不能续播已退役轨迹。
- 项目规划以实测起点重建，A*/优化/最终校验共用 0.15 s 预算；完整轨迹仍检查速度、加速度、碰撞与接管连续性。局部中间终点被占据时做有界搜索，最终任务目标不迁移。森林可达性仍未通过，未以放宽阈值掩盖。
- 共享配置和资源随所有者迁移；生产 stereo_source 只处理 v4l2/replay/file，合成渲染在 sim 的 synthetic_stereo_source。控制运行配置、interfaces 契约、sensing 标定/IMU、sim 场景各只有一份源码定义。

### 证据和复跑

本机证据根：`/home/waterc/bb_build/architecture/evidence/codex-complete-20260930/`（本轮跨 9 月 30 日和 10 月 1 日）。

- `offline-cancel-barrier-20261001/report.json`：13/13 PASS，`source_unchanged=true`，无跳过项；记录 SHA、dirty 文件散列、配置、依赖、命令、退出码和日志。
- `cancel-barrier-tests-20261001.log`：49 项 lifecycle_node 回归 PASS；`build-cancel-barrier-20261001.log`：独立前缀构建 PASS。
- `matrix/{session,faults,forest}-cancel-barrier-20261001.jsonl`：逐例原始汇总；对应批次目录保存源码/安装/PX4 清单、参数回读、PX4 模式/位置/setpoint/failsafe、注入、全部控制命令和最终 commander 状态。
- `matrix-verdict-20261001.json`：逐例行为判断；`audit_final_matrix.py` 保存判断脚本。`run_exit=0` 只表示运行收尾与 Disarmed 核对，不能单独计 PASS。
- `source-freeze-20261001.json`：26 例源码、安装、PX4、venv 与依赖一致，用户 site 禁用。

```bash
cd /home/waterc/workspace/Boom_Birds
export BOOM_BIRDS_VENV=$HOME/bb_build/architecture/venv
export BUILD_BASE=$HOME/bb_build/architecture/build
export INSTALL_BASE=$HOME/bb_build/architecture/install
export LOG_BASE=$HOME/bb_build/architecture/log
export OV_INSTALL=$HOME/bb_build/ov/install
bash companion/ros2_ws/tools/setup_python_env.sh --install
bash companion/ros2_ws/tools/build_all.sh --packages-up-to ego_planner boom_birds_nav map_generator mockamap --cmake-args -DBB_BUILD_GRIDMAP_TESTS=ON
# 输出目录须为新目录，不能覆盖本次报告。
bash companion/ros2_ws/tools/check_offline.sh --out "$HOME/bb_build/architecture/evidence/recheck/report.json"
export BB_SIH_MATRIX_EVID=$HOME/bb_build/architecture/evidence/recheck/matrix
bash companion/ros2_ws/tools/run_sih_matrix.sh session companion/ros2_ws/tools/sih_cases/session.spec
bash companion/ros2_ws/tools/run_sih_matrix.sh faults companion/ros2_ws/tools/sih_cases/faults.spec
bash companion/ros2_ws/tools/run_sih_matrix.sh forest companion/ros2_ws/tools/sih_cases/forest.spec
```

SIH 原始参数 `COM_OF_LOSS_T=1.0` 未增大；模式动作只有 PX4 接口发送。恢复场景使用同一固定 mockamap，显式起飞 2.5 m/锁点 2.3 m；常规 local 场景起飞 1.5 m/锁点 1.3 m 也单独回归。

### SIH 逐例判断

全部行最终 Disarmed；飞控重启用例停在 FAULT_LATCHED：旧会话的 LAND 被新飞控接口拒绝，符合重启后不能继续控制的约束，原始状态与拒绝原因均保留。`goal=false` 在反向用例是预期结果，在正常/森林任务是未达目标。距离只报告机体中心到原始场景点云的采样距离，没有已测机体包络，不是机体碰撞验收。

| 用例 | 验收 | 到目标 | 恢复后执行 | 累计恢复次数 | 最近目标 m | 采样中心净空 m | 未通过项 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| normal-local | PASS | True | False | 0 | 0.175 | 1.103 | — |
| normal | PASS | True | False | 0 | 0.132 | 1.126 | — |
| executor-restart | PASS | False | False | 0 | 1.97 | 2.279 | — |
| planner-restart | PASS | False | False | 2 | 1.883 | 2.179 | — |
| recover-mode-reject | PASS | False | False | 1 | 1.848 | 2.166 | — |
| cancel | PASS | True | False | 0 | 0.135 | 1.114 | — |
| planner-pause | PASS | True | True | 2 | 0.054 | 1.149 | — |
| depth-land | PASS | True | True | 2 | 0.177 | 1.128 | — |
| depth-rtl | PASS | True | True | 2 | 0.044 | 0.997 | — |
| odom-land | PASS | True | True | 2 | 0.053 | 0.989 | — |
| odom-rtl | PASS | True | True | 2 | 0.131 | 1.16 | — |
| offboard-land | PASS | True | True | 1 | 0.061 | 1.087 | — |
| offboard-rtl | PASS | True | True | 1 | 0.149 | 1.121 | — |
| manual-cancel | PASS | False | False | 0 | 1.762 | 2.166 | — |
| unknown | PASS | False | False | 0 | 1.736 | 2.158 | — |
| restart | PASS | False | False | 1 | None | None | — |
| mode-reject | PASS | False | False | 0 | 1.941 | None | — |
| budget-exhausted | PASS | False | False | 2 | 1.704 | 2.089 | — |
| low-height | PASS | False | False | 2 | 0.043 | 1.075 | — |
| landed | PASS | True | False | 0 | 0.135 | 1.05 | — |
| manual-mode | PASS | False | False | 0 | 1.403 | 1.802 | — |
| forest-current-seed1 | FAIL | False | True | 2 | 12.339 | 0.379 | goal_not_reached |
| forest-current-seed2 | FAIL | False | True | 2 | 12.93 | 0.611 | goal_not_reached |
| forest-current-seed3 | FAIL | False | False | 0 | 12.903 | 0.324 | goal_not_reached |
| forest-current-seed4 | FAIL | False | False | 0 | 14.269 | 1.071 | goal_not_reached |
| forest-current-seed5 | FAIL | False | False | 0 | 11.214 | 0.533 | goal_not_reached |

### 保留的失败与中断

- 旧森林 seed 1–5、缩短 horizon 的试验和 local-target 首次 seed 3 仍未到目标，保留在旧批次；不得被新结果覆盖。日志包含起点/终点占据、优化拒绝与 planner_timeout。
- `offline-observed-alignment-20261001/report.json`：831 PASS/2 FAIL；两例 IMU 测试把固定 boot_ref=1234.5 与刚重启 WSL 的真实 uptime 混用。已改测试模型为当前单调时间减 60 s，生产代码及未来/过期拒绝阈值未改；该失败报告保留。
- `faults-final-20261001`：运行 11/16 后，在已完成 Disarmed 的用例之间停止，修补规划器 CANCEL 未立即发布控制屏障的问题；余下用例没有冒记 PASS。修复后以新批次重跑全部矩阵。
- 早期 `session-smoke` 的后续 3 例被锁护栏拒绝；watchdog 子进程继承锁导致残留。现由 flock 父进程持锁并关闭子进程 FD，清理只处理本轮进程树；最终批次护栏独立留证。
- `offline-local-target` 被中断，没有完整 report，不计 PASS。plan_env 广口径 CTest 的 4 个历史 lint 失败仍保留；99 项行为回归与 trajectory_validation 的通过不覆盖这些失败。

### 未关闭与回退

- 森林 seed 到达目标结果见上表；反复规划拒绝区域仍需进一步处理，不能称 30 m 森林穿越验收通过。
- 当前 Docker Desktop 引擎 IPC 故障，最新代码容器复验 BLOCKED；旧容器结果属于旧快照。自动审批拒绝删除 dockerInference IPC 文件（blocked by policy），未执行删除、未绕过；未重置 Docker 或 Windows。未创建新 OS 镜像。
- 真机同步、真实双目/IMU、真实 VIO/OpenVINS 初始化、ARM64/Pi 5 性能、飞行验收：NOT RUN。无机体包络测试。
- 母仓库/子模块初始 SHA、状态和 diff 保存在 `submodules-before.txt`、`root-before.patch`、`ego-before.patch`；`pre-resource-migration.tar.gz` 保留资源/合成实现迁移前包目录。`untracked-snapshot.tar.gz` 在本轮首个编辑之后创建，不能称精确初始未跟踪状态。
- 回退前先保存当前 dirty/untracked，再按上述补丁和包归档逐路径恢复；消息定义与 EGO/Python 包必须一起重建，不能仅回退叶子包或覆盖已有用户修改。既有正式 venv 与 main 构建前缀保留，当前独立前缀未替换系统 ROS 环境。验收轮未执行 git reset、提交或推送；后续版本整理不改变本节验收结果。

### 2026-10-01 版本整理

- EGO 项目补丁固化为 `d16eaeb`（`boombirds-jazzy`）；母仓库 gitlink 与接口、Python 包一起更新。OpenVINS 固定版本不变。
- 修正工作空间 README 的节点归属、依赖安装和构建命令；迁移后的配置链接指向资源所有者。历史结果保留，森林 0/5 不改为 PASS。
- 提交前脱机复验 13/13 PASS，`source_unchanged=true`；报告为 `/home/waterc/bb_build/architecture/evidence/git-delivery-20261001/offline/report.json`。本次未重跑 SIH。
- Python/pytest 缓存归档到上述证据目录的 `workspace-cache/`，移动清单为 `workspace-cache-moves.json`。日志、原始数据、标定和已有构建环境保留；运行证据不纳入 Git。
- 母仓库提交号以 `git log -1` 为准；公开获取使用递归子模块检出，不使用 `--remote`。

## 2026-09-30 Codex 验收后修复（历史记录）

- `unknown` 模式闭锁；自动 Land/RTL 只有与当前 `sensor_link`、`planning_link` 或 `setpoint_link` 同时观测到才进入恢复。恢复确认前复核新鲜度、空中状态、AGL、速度、传感器、地图、坐标和执行许可。恢复 HOLD 按 `recovery_brake_accel_m_s2=1.0` 生成限加速度减速段。
- EGO 占据地图超时标志只在有效深度再次完成融合后清除；全无效深度不能清除超时。地图 C++ 回归场景 S10 覆盖两条路径。
- 新轨迹位置或速度超过接管连续性阈值时，退役该轨迹并发送取消和当前位置保持；维持 0.5 m / 0.3 m/s 阈值，30 s 内没有可接管新轨迹则 `planner_timeout` 闭锁。位姿或状态过期仍立即闭锁。规划命令停发超过 0.1 s 时提前切换 HOLD；超过 0.2 s 命令有效期才退役旧轨迹并有界请求剩余目标。
- 独立环境构建 `ego_planner`、`boom_birds_bringup`、`boom_birds_nav` 通过。最终运行代码统一脱机报告 `codex-fix-20260930/report-final-lifecycle.json`：12/12 PASS，导航 759 项、地图 97 项；随后验收工具坐标修补的 4 项定向测试 PASS，森林与短距离历史日志重新汇总验证通过。
- HOLD 等待期也检查传感器和 setpoint 持续断流，不能由 `playing=false` 跳过；到点后也必须维持 HOLD，直到生命周期进入降落。相同活动目标的 EGO 使能重发为幂等操作；需要重建起点时显式禁用再使能。
- SIH 字面量 `unknown` 注入后 `FAULT_LATCHED`，最终 Disarmed。`sensor_link` 进入 Land 后满足故障关联，但高度/空中/许可条件不足，恢复预算耗尽，最终 Disarmed。规划器挂起复测 `planner-handoff-fix`：`RECOVERING→EXECUTING`、4029 条 setpoint、持续生成新轨迹；最后 `planner_timeout`，最近目标距离 3.259 m，最终 Disarmed。**不计为完成目标或完整恢复通过。** 报告见 `codex-fix-20260930/sih/`。
- 无故障 2 m 短距离 SIH `normal-short-replan`：最近目标距离 0.078 m，`EXECUTING→LANDING→COMPLETE`，恢复 0 次、无闭锁，最终 PX4 回读 Disarmed。此前 `normal-short-after-hold` 无故障闭锁但在 0.633 m 停滞后超时降落，失败记录保留。
- 森林 30 m seed 1–5 的原始 SIH 均进入 Offboard，但最终分别距目标 11.007、11.314、12.402、8.049、12.724 m，未完成目标；seed 1/2/3/5 为 `planner_timeout`，seed 4 为 `manual_mode`，其前有超过 2 s 的 setpoint 断流和 PX4 转入 Land，不能据此声称是人工操作；全部最终 Disarmed。`sih_matrix_row.py` 原先漏加场景原点，把距离误报为约 0.2 m；原始 `forest-seeds-final.jsonl` 保留，按日志中记录的平移/航向重算摘要为 `forest-seeds-final-recorded.jsonl`；缺少对齐、对齐变化或 PX4 重启时拒绝计算目标距离。退役宽限复测 seed 3 最近 12.461 m，仍 `planner_timeout`。最终时序短距离复测 `normal-short-grace` 最近 0.05 m、无闭锁、恢复 0 次、最终 Disarmed。
- HOLD 失效检查与幂等目标复测：`forest-seed4-hold-gate` 为 `planner_timeout`；`forest-seed3-diagnostic` 有两次 `sensor_link` 恢复后仍超时；`forest-seed3-idempotent` 最近目标距离 11.616 m、最终超时，均 Disarmed。当前里程计作为重规划起点的试改在 `forest-seed3-measured-start` 中仍未完成目标并耗尽恢复预算，已经撤回；证据保留，不包含于最终源码。
- 最终源码短距离 SIH（`sih/final-smoke-recorded.jsonl`）：正常任务最近目标距离 0.085 m，无恢复/闭锁；规划器挂起 2 s 后两次 `RECOVERING→EXECUTING`，最近目标距离 0.112 m，随后正常降落，无闭锁。两例最终 PX4 回读 Disarmed。摘要补齐“规划器挂起”关键词，并排除 `fault=none` 的伪注入时间，原摘要保留。
- 自动 Land/RTL 原因没有独立 PX4 回读，链路故障与人工模式动作同周期时不能归因。完整故障矩阵和恢复后到达目标仍未通过；30 m 森林 seed 1–5 已执行但全部 FAIL。真机、真实 VIO、ARM64 性能与飞行验收 NOT RUN。

## 2026-09-29 DeepSeek 接手轮次（A–E 收口）

本轮在已有结构修整之上继续实现，证据目录
`/home/waterc/bb_build/architecture/evidence/deepseek-01/`。

### A 配置、环境与生命周期：PASS（脱机）
- **唯一运行参数来源**：`boom_birds_nav/runtime_config.py`（构造即校验）+ `config/runtime.yaml`
  （模板，测试强制它与代码默认值逐一相等）。`config/contract.yaml` 的 `timing:` 改为**纯语义**
  （只写含义/无效值处理，不再写数值）；`px4_failsafe.CONTRACT_TIMING_REFERENCE` 与
  `FailsafeConfig.from_runtime()` 改为引用同一来源。节点参数默认值也改成引用它。
- **高度独立命名并登记参考系**（`runtime_config.ALTITUDE_REFERENCE`）：
  `takeoff_altitude_agl_m`(1.5，即 PX4 `MIS_TAKEOFF_ALT`)、`hold_lock_min_altitude_agl_m`(1.3)、
  `image_publish_min_altitude_agl_m`(1.3)、`recovery_min_altitude_agl_m`(1.0)；全部相对起飞点地面。
  PX4 起飞参数由 `boom_birds_nav.sih_params` 从配置生成，shell 不再出现高度字面量。
- `ExecutionStatus` 增 `mode_detail` / `custom_main_mode` / `custom_sub_mode`：**AUTO Land 与 AUTO
  Return 现在可区分**（`auto:land` / `auto:rtl`），此前只报主模式 `auto`。
- 生命周期 FSM 完整：IDLE→PRECHECK→TAKEOFF→HOLD_READY→OFFBOARD_PENDING→EXECUTING→
  RECOVERING/LANDING→COMPLETE，任意活动阶段可入 FAULT_LATCHED（终态，不自动解锁）。
  锁点以 PRECHECK 记录的地面 z 为 AGL 基准，须达起飞高度且连续 1 s 内速度 ≤0.15 m/s、
  位置波动 ≤0.10 m，**不得用首次越过高度的样本**；接管保留 0.5 m 距离并新增速度连续性 0.3 m/s。
- 取消当周期清缓存、下一周期不再发旧 setpoint；只有 PX4 接口发模式命令，编排器只请求
  `VehicleAction` 服务；旧 shell 的 NED 高度判定与 `px4-commander` 模式控制已删除。
- 验证：`build_all.sh --packages-up-to ego_planner boom_birds_nav map_generator mockamap
  --cmake-args -DBB_BUILD_GRIDMAP_TESTS=ON` → **11 包 0 失败**；统一脱机入口 7 组全 PASS
  （navigation **723 项 0 失败**、stereo_calibration 10 项、config_single_source 20 项、interfaces_import、launch_executables）。报告：
  `evidence/deepseek-01/final/report.json`。

### A3 内参与几何（CameraInfo 三路同步）：PASS（脱机）
- `camera_geometry.py` 成为 Python 侧唯一校验口径（标定指纹、期望深度几何、不同源即拒绝启动 EGO）；
  EGO `grid_map` 严格校验 P 矩阵。`frame_id`/内参/尺寸任一变化 ⇒ 闭锁并清空占据体素、膨胀、
  深度缓存与就绪标志，只有 `grid_map/geometry_reset` 能解除并要求显式重新规划；FSM 的 10 ms
  回调也拦截闭锁，堵住原先只有 50 ms 回调才拦截的竞态。grid_map C++ 回归 94 项 0 失败。
- 深度无效值保持 NaN；`P[0][3] = -fx_当前分辨率 · B_物理`，物理基线不缩放。

### B 自动恢复：PARTIAL
- 默认**关闭**（`recovery_enabled=False`），只有 SIH 入口显式打开。
- SIH 实测**已进入 `RECOVERING`**：故障 `setpoint_link`（来源 `status.fault`），阻塞项如实上报
  （`valid_duration_not_met` → `status_stale,inputs_not_valid,valid_duration_not_met,
  execution_gate_closed`），2 次预算耗尽后闭锁（`detail=exhausted`，不循环争抢、不自动解锁），
  随后 LANDING→COMPLETE、Disarmed。
- **未完成**："恢复成功并重新规划剩余目标回到 EXECUTING"未在 SIH 中走通；Land/Return 两种进入
  方式的中断与完整反向矩阵未跑。

### C SIH 验证：PARTIAL
- 新编排入口 `px4_sih_mission.launch.py` 已**实际跑通**：TAKEOFF→HOLD_READY→OFFBOARD_PENDING→
  **EXECUTING**（`mode_detail=offboard`，实际下发 setpoint），结束 Disarmed、`landed: true`、
  `at_rest: true`。
- 4 次运行证据在 `evidence/deepseek-01/sih/`：前两次（含 `goal51`）进入 OFFBOARD 后因 EGO 判自身
  轨迹 `collision_free=0` 走 `planning_cancel` 闭锁降落；`normal-mockamap-res02`（修正 mockamap
  地图分辨率后）进入 EXECUTING 并触发上述恢复路径。
- 本轮修掉 **4 个只有在实跑中才暴露的集成缺陷**：
  ① `OPEN_SESSION` 曾调用 `on_planning_rejected`，而该闭锁无人清除 ⇒ 会话链路 `setpoints_sent`
  恒为 0、任务停在 HOLD_READY 直到超时；现改为"开会话清闭锁"，且拒绝闭锁只由**新轨迹号**清除
  （同一轨迹号继续出现不算重新建立，既有回归测试锁定）。
  ② 安装后 `config_path()` 找不到 `share/boom_birds_nav/config/runtime.yaml` 且**静默退回**只有
  `local` 的场景表 ⇒ SIH launch 报"缺少 forest_30m 场景"；现多候选解析 + 找不到**明确失败**。
  ③ SIH 链没有 `/boom_birds/imu` 发布者，而 `require_session=true` 时 IMU 是必需信号 ⇒
  `allow_setpoint` 恒假、永远进不了 OFFBOARD；现补 TEST-ONLY 合成 IMU 源。
  ④ `bin/install_package.sh` 硬编码入口点清单，遗漏 `lifecycle_node`/`sih_params` ⇒ 构建"成功"
  但 `ros2 pkg executables` 看不到、launch 运行时报 executable not found；现从安装记录推导并交叉核对。
  另修正 `px4_sih_mission.launch.py` 把 mockamap 的 0.2 m 点云按 0.1 m 采样（生成器分辨率不一致）。
- **未完成（NOT RUN，不是 PASS）**：30 m 森林与固定 seed 1–5；规划取消/深度断流/里程计断流/
  setpoint 中断/模式确认失败各自"进入 Land"与"进入 Return"后的合格恢复；全部反向用例
  （人工取消、落地、飞控或节点重启、低高度、未知故障、次数耗尽、模式命令被拒）。
- 没有实测机体包络，因此只报告中心到障碍距离，不写机体碰撞验收。

### D 按职责拆包：NOT RUN
`boom_birds_sensing/control/bringup/sim` 本轮**未执行**。`boom_birds_nav` 仍是唯一实现，
未出现"第二份实现"；拆包顺序与准入条件见文末「下一步」。

### E 文档
本条即本轮记录；重要接口决定追加 `DECISIONS.md` D-028。既有决策正文与历史测试内容不改写。

### 本轮仍未验证（始终单列）
真机同步、真实双目/IMU、真实 VIO/OpenVINS 初始化、ARM64/Pi 5 性能与真实飞行验收：**NOT RUN**；
未建立新 OS 镜像，因此不声称全新机器可复现。

## 2026-09-29 DeepSeek 第十七轮：B 收口（恢复→重规划闭环）与接管门的系统性影响

证据目录 `evidence/deepseek-18/`（16 次运行，**全部有 PX4 读回的最终 Disarmed**）。

### 1. B：恢复→重规划闭环**机制走通 5/6**，闭合后持续飞行 1/6
- 注入方式定为**规划器挂起**（`kill -STOP ego_planner_node` → 2 s → `kill -CONT`，pid 不变、id 计数不丢），
  故障码 `planning_link`（属 `RECOVERABLE_FAULTS`）；执行许可不受影响，PX4 保持 offboard。
- 实测（5 个样本）：`enable_planner#1 → EGO 产出水位之上的新 id` = **1530 / 1682 / 1748 / 2054 / 2087 ms**
  （中位 1748）；`disable→enable#1` 2440–2600 ms；enable 重发节奏 500–520 ms（设定 `planner_enable_retry_s=0.5`）；
  **5 例均未出现 `planner_timeout`**。⇒ 三个时间参数与实际节奏**是对齐的，无可改依据**（不改值）。
- `b-suspend-t3` 完整闭环：注入自证 → RECOVERING（`waiting:map_not_ready,valid_duration_not_met,window_reset_by:map_not_ready`）
  → 回读确认 `detail=confirmed, revoked=false` → replan+enable_planner → **活着的 planner 产出 id=4（enable 后 2087 ms）**
  → EXECUTING 且 setpoint 继续流（**8123 条**，执行到 deadline）→ **不是 30 s 静默闭锁**。
- 另 4 例同样走完"确认 + 新 id"，但约 5–8 s 后又出现**第二个 `setpoint_link`**，第 2 次恢复条件不连续 → `exhausted` 闭锁（预算耗尽属设计行为）。
  ⇒ **B 记 PASS（机制）/ PARTIAL（恢复后的持续执行）**，不写成"恢复后能稳定飞完任务"。

### 2. 两个结构性发现（都指向"设计边界"，不是工具问题）
- **重启轨迹生产者无法在同一任务内恢复**：`traj_server` 重启后 trajectory_id 从 1 重新计数，
  而 `ControlIngress.retired` 是单调水位（`control_protocol.py:47/68`，`<= retired` 直接 `retired_trajectory`），
  于是前 16 个 id 全被永久拒绝 → 静默 30 s → `planner_timeout`；**重新开会话也不是出路**——
  `open_session()` 虽清零水位（`control_protocol.py:37-40`），但 `lifecycle.py:587-588` 对会话变化直接
  `latch("session_changed")`，而它在 `REVOKING_FAULTS` 里。两条路都通向闭锁降落：**安全侧设计后果**（见 D-041）。
- **`REVOKING_FAULTS` 里的字面量 `"unknown"` 经模式回读不可达**：未登记模式名被 `mode_detail_known()`
  当成"没有模式名"，回退到 offboard 布尔 → `offboard_lost`（**可恢复**）并真的进了恢复；
  真正 fail-closed 的是格式合法的未登记模式名（`auto:mission` → `manual_mode`，`attempts=0`）（见 D-042）。

### 3. 系统性发现：接管连续性门是矩阵最主要的任务终止机制（本轮不改，需设计评审）
- `lifecycle_node.py:276-302` 的接管门在**每次轨迹键变化**时评估，而 EGO 每次重规划都换
  `trajectory_id`（`traj_server.cpp:105` 每个 B 样条 `++project_trajectory_id_`；实测约 2 Hz、36 s 内 43 个新 id），
  所以"新轨迹必须与当前状态在 0.5 m / 0.3 m/s 内连续"这条规则**在整段飞行中反复生效**，且命中即**终态闭锁**。
- 两轮矩阵里 9 次 `handoff_discontinuous` 的**四条判据中只有 `dist_vel` 越限**（0.301–0.673 m/s，
  其中 3 例只超 0.001–0.021），`pose_age`/`status_age`/`dist_pos` 一次都没触发（唯一例外见下）；
  这也是 30 m 森林两次飞到 9.652 m / 7.254 m 后闭锁、以及 6/10 故障例"注入故障不是首要原因"的直接原因。
- 该行为**被测试冻结为规格**：`test_handoff_thresholds_are_the_frozen_values_and_come_from_one_source`
  断言"距离 0.5 / 速度 0.3 是任务冻结值"，并用 `trajectory_id=2`（即**飞行中重规划**）验证超限即闭锁；
  `test_discontinuous_or_stale_handoff_is_rejected` 同样按规格断言。
  ⇒ **本轮不擅自改这条安全规则**（既不放宽阈值、也不把终态闭锁改成"拒绝该轨迹"）——
  后者的语义差别是任务级后果（例行重规划可终止任务）vs 安全级后果（绝不执行不连续轨迹），
  属设计授权范围，记为待评审项（D-043）。
- **首次出现位置差越限**：`b-suspend-t1` 的唯一失败原因是 `dist_pos=1.718>0.5, dist_vel=0.820>0.3`；
  此前 9 次都只有速度越限，位置差 1.7 m 需单独定位（可能与挂起期间 EGO 内部状态有关），本轮只记录。

### 4. 低空门限：**NOT DIRECTLY VERIFIED**（含一处工具自身缺陷）
- 工具缺陷（已修）：高度门的 AGL 基准取的是"门开始轮询后第一条位置样本"，而门在 `EXECUTING` 之后才启动，
  于是把空中的 1.486 m 当地面、`agl` 读成 ≈0 —— **那一次运行不算证据**；现改为"起飞前未 armed 的地面样本"
  并留 `ground_source`。上一轮 deepseek-17 的低空用例用 `--fault-at TAKEOFF`，不受此缺陷影响。
- 修正后仍取不到 `below_min_altitude`：低 z 目标下飞行器始终停在约 1.5 m（EGO 59 次重规划不下降），
  先 `planner_timeout` 闭锁，门是在**闭锁后的下降段**才触发。该阻塞项逻辑本身有单测覆盖
  （`test_lifecycle.py:417 test_recovery_below_min_altitude_never_requests_offboard_and_exhausts_budget`）。
  ⇒ 路径逻辑有单测、SIH 未直接走到，如实记 NOT DIRECTLY VERIFIED。

### 5. 顺带修掉的两处自身缺陷
- 状态条件触发落地后，旧的"固定时刻注入前 `sleep 8`"仍在执行，导致实际注入比门确认晚 8.1 s
  （第 1 批两次因此先踩 `handoff_discontinuous`）；现门确认即注入。
- `--inject-when-ready` 缺 `shift` 导致参数循环空转、首批 3 次运行零输出；整批留档 `failures/b-setpoint-transient-argparse-bug/`，修复后复验。
- 离线检查抓出 SIH 工具注释里的"硬编码高度"误判（`# 那时飞行器已经在 1.5 m 空中`）：
  `_hardcoded_offenders` 原先把注释也算违规，会逼人删掉有用的解释来讨好扫描器；现改为**只扫 `#` 之前的代码**，
  并用"注入违规"反向验证判据仍能抓住真违规（注释不算、`ALT=1.5`/`HANDOFF=0.5` 算）。

### 6. 最终验收（本轮冻结树）
- `check_offline.sh` → `evidence/deepseek-18/report_final4.json`：**12/12 PASS，退出 0，844 项测试 0 失败 0 跳过**。
- 中间一次真实失败已留档：`check_final3.log`（2 项失败＝同一条扫描器误判经 navigation/config_single_source 两个组各报一次），修复后 `check_final4.log` 全绿。
- 机器卫生：无 px4/ros2/sih 残留、UDP 空闲、`git diff --check` 干净。

## 2026-09-29 DeepSeek 第十六轮：拆包元数据与证据链修正；30 m 森林量程打通

本轮在第十五轮的实测量根因之上收口：把 `depth_max_range_m` 从配置贯通到深度节点与 EGO，
并修掉拆包第三/四批留下的**元数据与证据链缺陷**。证据目录
`/home/waterc/bb_build/architecture/evidence/deepseek-17/`。

### 1. 深度量程贯通（第十五轮结论的落地）
- `RuntimeConfig` 新增 `depth_max_range_m`（默认 5.0 m），`local` 场景 5.0、`forest_30m` 覆盖 16.0；
  场景覆盖白名单 = `origin_x/y/z + depth_max_range_m`，其余字段按场景覆盖直接 `ValueError`。
- 发布路径贯通：`runtime.yaml → RuntimeConfig → px4_sih_mission.launch.py →
  px4_sitl_motion.launch.py（`depth_node.max_depth_m`）→ EGO `max_ray_length`（`4.5 → 场景量程`）。
- `config/runtime.yaml` 随其所有者迁到 `boom_birds_control/config/`（nav 只留兼容转发），
  `config_path()` 改为按所有者包解析，找不到即**明确失败**（不再静默退回单场景表）。

### 2. 拆包遗留的元数据缺陷（本轮实修，全部有回归锁定）
- **陈旧配置副本**：安装只增不删，`runtime.yaml` 迁走后 nav 的 `share/` 里仍留着旧副本，
  而它与真配置同名同结构——正是 `config_path()` 最怕的「看起来合法」。5 份
  `bin/install_package.sh` 增加「清理陈旧配置」（`share/<pkg>/config` 下源码已不存在的文件）。
- **依赖方向写反**：`boom_birds_control/sensing/sim` 的 `package.xml` 各留一行
  `<depend>boom_birds_nav</depend>`（拆包时旧包名没换），而 nav 反过来 import 了这四个包却
  **一个都没声明**。后果不是纸面的：`colcon --packages-up-to boom_birds_nav` 只构建 nav，
  得到一个「能构建、import 全断」的安装树（第一版 CI 的包清单就是这么写的）。现按**实际
  import** 双向修正：control 去掉 nav 依赖，nav 声明它转发的四个包，sensing/sim/bringup 补上
  control（sensing 另需 bringup）。
- **把错误当规格的断言**：`test_*_package_layout.py` 里三条
  `assert "<depend>boom_birds_nav</depend>" in xml` 正是上述错误方向的看守者，已改为
  「不得依赖上层转发包 nav + 必须声明基础层 control」。
- **A9 盲区**：`test_config_single_source.py` 的依赖清单里没有四个兄弟包，「用了没声明」查不出来；
  补入后立刻又暴露一个潜在缺陷——包内绝对导入自己会被当成第三方依赖（现显式跳过自引用）。

### 3. 证据链修正（保证报告说的是真事）
- `check_offline.py` 的配置指纹原来只扫 `boom_birds_nav/config/*.yaml`，于是**唯一真值**
  `boom_birds_control/config/runtime.yaml` 根本不在报告里；现覆盖全部 `boom_birds_*` 的 config。
- 新增 `--exclude-groups`：被排除的组记入 `report.json` 的 `excluded_groups`，既不算 PASS 也不静默消失。
- 两处**硬前置**（本轮实测教训的固化）：
  ① 安装前缀里没有 `setup.bash` 就停下并说明（`check_offline.sh` 退出 2 / `check_offline.py` 退出 1）——
  默认前缀是 `~/bb_build/main`，与开发机常用的 `architecture` 前缀不是同一个目录；
  ② 检测到 `px4`/`run_sih_mission`/`sih_record`/`traj_server` 进程时**拒绝运行**（退出 3，
  `BB_ALLOW_BUSY_CHECK=1` 可强制）——同一套 ROS 话题上，SIH 的合成深度流会注进被测话题。

### 4. 本轮检查结果（含假失败的完整轨迹，不隐藏）
- `check.log`（`report.json`）：**12/12 PASS**，退出 0——修掉 `runtime.yaml` 定位后的第一份全绿。
- `check2.log`（`report2.json`）：4 组 FAIL——**真实失败**，即第 2 条里 package.xml 方向错误与自引用缺陷被新断言抓出。
- `check3.log`：8 组 FAIL——**假失败**：本轮命令漏导出 `INSTALL_BASE`，检查落在陈旧前缀 `~/bb_build/main`，表现为整片 `Package not found`。
- `check4.log`（`report4.json`）：**12/12 PASS**，退出 0（与 check3 同一份源码，唯一差别是安装前缀）。
- `check5.log`：11/12——`navigation` **假失败**：与并发 SIH 运行相撞，`test_pairing_and_frames.py`
  收到 30 Hz 合成深度流（159 条输出 vs 测试自发的 5 张图）。此为第 3 条①②两个硬前置的直接来因。
- CI 覆盖模拟（`--exclude-groups navigation,map_behavior,trajectory_validation`）：
  **10 组 PASS + 3 组 EXCLUDED，退出 0**（`report_ci_sim.json`）。
- 构建：`build.log` / `build2.log` 均 **15 包 0 失败**。

### 5. SIH 矩阵实测（22 例，全部有 PX4 读回的最终 Disarmed）

完整逐场景表见 `evidence/deepseek-17/sih/SIH_REPORT.md`；证据根 `evidence/deepseek-17/sih/`。

| 交付项 | 判定 | 关键实测 |
| --- | --- | --- |
| forest_30m × 2 seed（目标 30 m） | **PARTIAL** | `depth_max_range_m=16.0` **确实修好了 mapReady**：两 seed `hold_ready_gate_at_hold` 均为 `map_ready=true / planner_map_ready=true`（上一轮全 false）；都真进 EXECUTING，飞了 **9.652 m / 7.254 m**，EGO 13–16 次规划成功、674–835 条 setpoint；**两次都在飞行中闭锁、未到目标 → 不记通过** |
| 故障矩阵 10 例（Land×5 / Return×5） | **PARTIAL（按场景钉死）** | 注入生效的 10 例中 **6 例首要原因是 `handoff_discontinuous`**（其中 4 例在注入之前就已 latch），4 例注入故障=首要原因（Land 组 1/5、Return 组 4/5 是首要） |
| 反向矩阵 4 例 | **PASS** | `reverse-px4-restart` 改为**同一次运行内重启**（kill 772307 → new 773094）：首个 latch `flight_controller_restart`（+1.73 s）、恢复按设计撤销、**最终 Disarmed = 新实例读回**，上一轮的 NOT OBSERVABLE 消除 |
| 低空门限 1 例 | **PASS（注入时机）/ 撤销路径 NOT DIRECTLY VERIFIED** | `fault_telemetry.json`：注入瞬间 `agl_m=-0.004`（注入前 20 样本最大 0.008 m）；该次 `attempts=0`、无自动接管；但 latch 是 `map_or_stream_not_ready`（非可恢复故障），"低于 1 m ⇒ 撤销恢复"这条路径**未被直接走到**，不写成已验证 |

**本轮三个最重要的发现都是缺陷，不是通过**：
1. **`深度断流` 的注入方式在拆包后已失效**：`pkill -f 'boom_birds_nav/depth_node'` 匹配不到任何进程（实现已迁到 `boom_birds_sensing`），pkill 静默失败、`fault.txt` 未写，**故障从未注入**却"干净结束"。改为 `kill_label`（先 pgrep 证明命中再杀，并把 pattern 与 killed pid 写进 `fault.txt`），补跑 2 例。这条对"注入故障是不是首要原因"是前提性的**先证明注入了，才谈得上是不是首要原因**。
2. **接管速度门 `dist_vel>0.3` 是全部矩阵最主要的闭锁来源**：9 次 `handoff_discontinuous` 的四条内嵌判据里**只有 `dist_vel` 越限**（0.301–0.673），`pose_age`/`status_age`/`dist_pos` 一次都没出现，其中 3 例只超 0.001–0.021 m/s。门限是否有量测依据未定，本轮**不改门限**，只把量摆出来（见 D-040）。
3. **串行护栏自身修掉三个真问题**（`$$` 不是脚本 pid、短命子 shell 竞态、一个会活过整次运行的记录器轮询子 shell），并新增 `RUNNING` 标记与每次运行的 `leftover_after_cleanup.txt` 自检。

seed3 的 CPU 争用已按 Lead 要求在真空窗干净复跑分离：受争用 `depth` 最大空洞 **4.609 s（>1 s 空洞 3 次）/ `odom_or_depth_lost=101` / `session_changed`**；真空窗 **0.213 s（0 次）/ 86 / `sensor_link→handoff_discontinuous(0.392)`** ⇒ 那 4.6 s 断流是 CPU 争用造成，两组证据都留（`failures/forest-batch2-pre-vacuum/`）。**但两组都到不了目标，森林绕障仍未通过。**

### 6. 独立核验（对抗式复核）及其修复

独立核验者（task-8）在冻结快照（HEAD `c11761e`）上复核 33 条声明：**26 VERIFIED / 4 REFUTED / 3 UNVERIFIABLE**；报告 `evidence/deepseek-17/verification.md`。拆包 DAG、单一来源、深度量程通路、EGO 补丁索引均**经独立复算成立**（如 `runtime.yaml` ↔ `RuntimeConfig` 55/55 字段相等、EGO numstat 逐格一致、77/77 条符号断言命中）。四条 REFUTED 已全部处理：

- **R1**：`boom_birds_sensing`/`boom_birds_sim` 的 `<depend>boom_birds_interfaces</depend>` 全包零使用 → 按 D-036「声明了就要用」删除。
- **R2/R3**：README 把 `RECOVERABLE_FAULTS`/`REVOKING_FAULTS` 的名字与行号配反；`ControlCommand` 字段清单写了不存在的 `frame_id`、漏了 `header` → 已按源码改（frame 由 `header.frame_id` 承载）。
- **R4**：参数来源表 11/15 处行号引用指向无关断言，根因是**本轮给 `test_config_single_source.py` 追加内容导致行号整体下移**而文档未重算 → 改为按 `模块::测试函数` 引用（19 处），并对 235 处行号引用做了 0 越界自证。**这是指针错误而非断言错误**：表中描述的行为经独立复算全部为真。
- **F1（核验者附加发现，已修）**：深度量程还有两处硬编码 `5.0` 的第二定义点（`depth_node.py`、`offline_chain.launch.py`）→ 改为 `DEFAULTS.depth_max_range_m`，并新增 `test_depth_range_has_no_second_definition`（判据经"注入违规"反向验证，确认非空转）。
- **F6（已修）**：安装副本一致性断言在 `config_path()` 命中源码树时是自反的 → 增加从 ament share 独立取值的第二条路径。
- 未采纳/留档：F5（陈旧的 `~/bb_build/main` 安装前缀内容与源码分歧）**不删**（可能是历史证据），改由 `check_offline.sh` 的前置条件把"前缀不对"变成显式失败，见 D-037。

### 7. 最终验收（冻结树）

- 构建 `build3.log`：**15 包 0 失败**。
- `check_offline.sh` → `report_final.json` / `check_final.log`：**12/12 PASS，退出 0**；JUnit 合计 **844 项、0 失败、0 跳过**（navigation 739、config_single_source 34、sensing 24、control 16、bringup 11、stereo_calibration 10、sim 8、nav_forwarder 2）。
- 机器卫生：无 px4/ros2/sih_record/run_sih/ego_planner/traj_server/lifecycle 残留；UDP 14580/14540/14550 未占用；`git diff --check` 干净。
- **重建对 SIH 证据的影响**：本轮改动中的量程默认值替换是**等值**的（`DEFAULTS.depth_max_range_m = 5.0` 与原字面量相同；forest 由 launch 显式传 16.0），因此 22 例 SIH 数字不需要重跑；核验者也独立核对了安装树模块 mtime 与 EGO 二进制未变。

### 8. 仍未完成（NOT RUN / 未定论，不得当作 PASS）
- **`geometry_changed` 的字段级成因未定论**：种子 1 两次出现（t+7.0 s、t+36.0 s，都早于探针），EGO 日志 `[BB-A3] 拒绝相机几何：内参或尺寸变化`；真空窗两例的 `caminfo_watch` 逐条记录 161/171 条 CameraInfo 的 `width/height/frame_id/P` 完全相同、发布者恒为 1 ⇒ **有探针的运行没复现、复现的运行没探针**。探针已常驻，下次复现即可给字段级答案；不写成"已定位"。
- **接管速度门限缺量测依据**（D-040）：`dist_vel` 0.3 是否应按实测分布重定值，需要真机/更高保真链路的量测。
- **`RECOVERING→EXECUTING` 确认恢复只有 1 例样本**（`fault-setpoint-break-rtl`，之后仍以 `planner_timeout` 结束），不足以称"恢复已打通"。
- **真机同步、真实双目/IMU、真实 VIO、ARM64/Pi 5 性能、真实飞行验收**：NOT RUN。

### 9. 本轮（收口前）仍未完成项（保留原文）
- **30 m 森林任务路径**：`forest_30m`（量程 16 m）的种子运行仍在进行，结果与判定见后续小节；
  在拿到证据前不声称森林绕障可用。
- **恢复（B）未打通**：「恢复成功并重新规划回到 EXECUTING」未在 SIH 中建立。
- **故障矩阵首要原因**：需要按内嵌诊断值重跑，区分「注入故障是主因」与「`handoff_discontinuous`/`status_missing` 先 latch」。
- **反向矩阵**：`reverse-px4-restart` 上一轮因 FC 被杀而 **NOT OBSERVABLE**，需改为同一次运行内重启；「未知故障」仍未在 SIH 注入。
- 真机同步、真实双目/IMU、真实 VIO、ARM64/Pi 5 性能、真实飞行验收：**NOT RUN**。


- **30 m 森林任务路径**：`forest_30m`（量程 16 m）的种子运行仍在进行，结果与判定见后续小节；
  在拿到证据前不声称森林绕障可用。
- **恢复（B）未打通**：「恢复成功并重新规划回到 EXECUTING」未在 SIH 中建立。
- **故障矩阵首要原因**：需要按内嵌诊断值重跑，区分「注入故障是主因」与「`handoff_discontinuous`/`status_missing` 先 latch」。
- **反向矩阵**：`reverse-px4-restart` 上一轮因 FC 被杀而 **NOT OBSERVABLE**，需改为同一次运行内重启；「未知故障」仍未在 SIH 注入。
- 真机同步、真实双目/IMU、真实 VIO、ARM64/Pi 5 性能、真实飞行验收：**NOT RUN**。

### 10. 本轮文件与回滚
- 写入面：`boom_birds_control/{config/runtime.yaml,runtime_config.py}`、`boom_birds_{nav,control,sensing,sim,bringup}/{package.xml,bin/install_package.sh,test/*}`、
  `tools/{check_offline.py,check_offline.sh}`、`.github/workflows/offline.yml`、`README.md`、`docs/EGO_FORK_PATCHES.md`。
- 回滚：按轮次证据目录互不覆盖（`evidence/deepseek-01`…`deepseek-17`）；本轮之前的检查点见
  `checkpoints/`；不执行 `git reset`/`clean`。

## 2026-09-29 DeepSeek 第十五轮：森林场景卡点的**实测量**根因

证据：`evidence/deepseek-15/forest-diag`、`forest-depth-diag`（含 `recorder_topics.jsonl` 的深度量程统计）。

### 门控逐项上报（新增）

只在状态里报 `map_or_stream_not_ready` 无法定位。给 `mission/status` 加上三项闸门的逐项值后，
一次运行就拿到：

```
hold_ready_gate: {sending: true, sensors_ready: true, map_ready: false,
                  planner_seen: true, planner_map_ready: false,
                  planner_age_s: 0.029, geometry_fault: false,
                  pose_age_s: 0.043, status_age_s: 0.018}
```

**执行许可与传感器都正常，卡在 EGO 自己的 `mapReady()`。**

### 实测量：森林场景的深度全部在量程之外

记录器直接解析 `32FC1` 深度并按量程统计（`evidence/deepseek-15/forest-depth-diag`）：

```
pixels_total            = 11 016 000
pixels_finite_positive  =  4 498 200   (40.8% —— 与 depth_node 的"有效率"一致)
pixels_within_5m        =          0
min_finite_m            = 10.390
max_finite_m            = 14.386
```

森林最近障碍约 **10.4 m**，而 EGO 的 `invalid_depth_max_dist_`
（`[BB-PATCH-1]`：`max_ray_length_ + 0.1`，约 5.1 m）把 10–14 m 的返回**全部判为无效**：
`has_valid_depth_obs_` 永远为 false ⇒ `mapReady()` 永远为 false ⇒ HOLD_READY 30 s 超时闭锁。

**这解释了三件事**：① 五次种子全部"深度在跑但不建图"；② 旧实现 2026-09-28 的 30 m 记录
为什么看起来能飞（旧 `grid_map` 没有这条显式量程门）；③ 为什么单位测试全绿——这是**场景与
量程的配置矛盾**，只有在真实链路上跑才暴露。

### 这是配置矛盾，不是"绕障失败"

- `depth_node.max_depth_m = 5.0`（用于兼容/预览裁剪）与主话题实际发布的 10–14 m 值**不一致**；
- 合成双目在该基线与分辨率下可用量程本就在 5 m 量级，而 30 m 森林把障碍放在 10 m 以外；
- 因此当前配置下**这台"传感器"看不到这个场景**。

下一轮要做的是一次**显式且留证的场景/量程决策**（二选一，不能悄悄调参）：

1. 把森林起点移近障碍（调整 `map/center_x` 使最近障碍落在 3–4 m），使场景落在现有量程内；或
2. 为该场景显式提高 `grid_map/max_ray_length` 与 `depth_filter_maxdist` 覆盖到 ~15 m，
   并在文档里写明"SGBM 在 0.067 m 基线、240×180 下 10–15 m 的深度精度未标定"。

在此之前，**30 m 森林任务面的结论仍是 NOT PASS**，不得因为"五次都 Disarmed"就写成通过。

### 判定

| 段 | 判定 |
| --- | --- |
| C 30 m 森林 / 种子 1–5 | 安全面 PASS（5/5 Disarmed）；**任务面 NOT PASS，根因已实测量定位** |
| 其余 | 同前（C 正常 PASS；故障/反向矩阵安全面成立；B 未确认成功恢复；A-CI 未在 runner 执行） |

## 2026-09-29 DeepSeek 第十四轮：30 m 森林种子矩阵（1–5）与一处被掩盖的主因

证据：`evidence/deepseek-14/`（`sih-matrix/forest-seeds.jsonl`，逐场景证据同目录）。

### 30 m 森林固定场景，seed 1–5（起点 ROS `(-15,0,0.1)`、终点 ROS `(15,0,1.0)`）

| seed | 最终状态 | 最终 Disarmed | 落地下 | 距目标最近 | setpoint |
| --- | --- | --- | --- | --- | --- |
| 1 | LANDING | **yes** | true | 29.975 m | 1501 |
| 2 | COMPLETE | **yes** | true | 29.973 m | 1500 |
| 3 | COMPLETE | **yes** | true | 29.973 m | 1500 |
| 4 | LANDING | **yes** | true | 29.973 m | 1500 |
| 5 | COMPLETE | **yes** | true | 29.973 m | 1500 |

**必须说清楚这五个数意味着什么**：距目标仍近 30 m，即起点到终点这 30 m **一步都没飞**。
五次全部在 HOLD_READY 阶段闭锁，`reason = map_or_stream_not_ready` —— 编排器在
`hold_ready_timeout_s`（30 s）内没有看到"执行许可 + 传感器就绪 + 地图就绪"同时成立。

所以本轮的准确结论是：

- **安全面 PASS**：5/5 起飞、稳定悬停、最终 **Disarmed**（PX4 独立回读）、落地下；
- **任务面 NOT PASS**：一次都没有执行目标，**不能**写成"30 m 森林绕障通过"，
  也不能用旧实现 2026-09-28 的 seed=3 记录充当本轮证据（那是旧链路的，提示词已明确警告）。

排查线索（**尚未定论**）：深度链本身在跑（`depth_node` 累计 304 帧、有效率 40.8%），
`random_forest` 也生成了场景点云；卡点在 EGO 的 `mapReady`。假设是起点处森林障碍
在 17 m 以外、超出合成深度量程，起始位置附近没有占据体素可累积。这条**只是假设**，
需要下一轮用 EGO 侧的地图就绪诊断证实或推翻。

### 顺带发现：注入故障此前被另一个主因掩盖

回看故障矩阵各场景的闭锁原因，注入故障**大多没有成为主因**：

| 场景 | 实际闭锁原因 |
| --- | --- |
| `fault-depth-loss` | `handoff_discontinuous` |
| `fault-odom-loss` | `handoff_discontinuous` |
| `fault-mode-confirm` | `status_missing` |
| `fault-plan-cancel` | `manual_cancel`（即注入的动作本身，正确） |
| `fault-setpoint-break` | 无闭锁 |

即：多数场景在注入故障之前（进入 EXECUTING 后 8 s 内）就已经被**接管闸门**
（`handoff_max_distance_m` / `handoff_max_speed_m_s` / 新鲜度）闭锁。
所以"注入故障 → 合格恢复"不成立，一部分原因是**故障压根没被触发**。

为此给接管闸门加了逐项诊断（把实际的距离/速度/新鲜度写进闭锁说明），
下一轮据此判断该改的是闸门还是轨迹起点。

### 判定

| 段 | 判定 |
| --- | --- |
| C 正常（mockamap 固定场景） | PASS；拆包后回归 PASS |
| **C 30 m 森林 / 种子 1–5** | **安全面 PASS（5/5 Disarmed）；任务面 NOT PASS（0/5 执行目标）** |
| C 故障矩阵（Land/Return 10 例） | PARTIAL：安全收尾成立；主因多为接管闸门而非注入故障 |
| C 反向矩阵（5 例） | PARTIAL：4/5 确认不自动接管；PX4 重启例的 Disarmed 不可观测 |
| B | PARTIAL（仅 1 例进入 RECOVERING 且未确认成功率） |
| D | PASS |
| A-CI | 未在 runner 执行 |

## 2026-09-29 DeepSeek 第十三轮：拆包后 SIH 回归 + 反向矩阵第一批（C）

证据：`evidence/deepseek-13/`（构建 15 包 0 失败、统一脱机检查 **12 组全 PASS**、
navigation **730 项 0 失败**）；SIH `sih/post-split-normal`、`sih-matrix/reverse.jsonl`。

### 拆包后 SIH 回归：PASS

`post-split-normal`（mockamap 固定场景、目标 `(5.0, 1.0, 2.5)`）：

```
IDLE → TAKEOFF → HOLD_READY → OFFBOARD_PENDING → EXECUTING → LANDING → COMPLETE
```

setpoint 发 **9325**、最终 **Disarmed**（PX4 `px4-commander status` 独立回读）。
拆包把入口点分散到三个包后，真实飞行链没有回归。顺带修掉 runner 里两处过期路径
（`boom_birds_nav.synthetic` → `boom_birds_sensing.synthetic`；`sih_guard` 探测改为
`boom_birds_control.sih_guard` 并把 `sys.path` 指向新包目录）。

### 反向矩阵第一批（5 例，注入故障后**不得自动接管**）

| 场景 | 注入 | 状态序列 | 最终 Disarmed | 落地下 | 恢复尝试 |
| --- | --- | --- | --- | --- | --- |
| `reverse-manual-cancel` | `Mission.CANCEL` | …EXECUTING→FAULT_LATCHED→LANDING→COMPLETE | **yes**（commander 回读） | true | 0 |
| `reverse-orchestrator-restart` | kill `lifecycle_node` | …EXECUTING→FAULT_LATCHED | **yes** | true | 0 |
| `reverse-low-altitude` | TAKEOFF 阶段 kill `depth_node` | …TAKEOFF→HOLD_READY→EXECUTING→FAULT_LATCHED→LANDING→COMPLETE | **yes** | true | 0 |
| `reverse-camera-loss` | kill `stereo_source` | …EXECUTING→**RECOVERING**→FAULT_LATCHED→LANDING→COMPLETE | **yes** | true | **1** |
| `reverse-px4-restart` | kill PX4 | …EXECUTING→FAULT_LATCHED | **NOT OBSERVABLE** | NOT OBSERVABLE | 0 |

- **4/5 例确认"没有自动重新接管"**：全部以闭锁或降落收尾，没有任何一例在无人干预下重新起飞或争抢模式；
  Disarmed 取自 PX4 独立回读，不是"软件进程退出"。
- **`reverse-px4-restart` 的最终 Disarmed 属于 NOT OBSERVABLE，不是"未解锁"**：
  该用例把飞控进程本身杀掉，之后没有任何对象可以回读解锁状态，`px4-commander status`
  返回 "PX4 server not running"。这是**场景设计问题**：正确做法是"杀掉后在同一轮内重新拉起 PX4"，
  再核对"不自动接管 + 最终 Disarmed"。已记为待修正项，不得把这一格写成通过或失败。
- `reverse-camera-loss` 是**第一个进入 `RECOVERING` 的注入故障**（fault=`sensor_link`，attempts=1），
  随后闭锁降落。比上一批的 0/10 有进展，但仍**不是**"合格恢复"。
- `reverse-low-altitude` 未触发恢复（attempts=0），与"低高度拒绝接管"一致，但
  该次注入时机下飞行器未必真的低于 `recovery_min_altitude_agl_m`，因此**不能据此断言低高度门槛已被验证**。
- "未知故障"这一条本轮**未单独注入**：FSM 的 `REVOKING_FAULTS` 对 unknown 的处理由脱机测试覆盖，
  SIH 侧尚无可靠的注入方式（会让状态变成已分类故障），如实标为未执行而不是通过。

### 判定更新

| 段 | 判定 |
| --- | --- |
| D | PASS（第十二轮） |
| C 正常 | PASS；拆包后回归 PASS |
| C 故障（Land/Return 两批 10 例 + 反向 5 例） | **PARTIAL**：安全收尾、"不自动接管"成立；合格恢复仅 1 例进入 RECOVERING 后仍闭锁 |
| C 种子矩阵 / 30 m 森林 | NOT RUN |
| B | PARTIAL：注入故障下进入 RECOVERING 但未确认成功恢复 |
| A-CI | 未在 runner 执行 |

## 2026-09-29 DeepSeek 第十二轮：拆包收口 `boom_birds_bringup`，nav 成为纯转发（D）

证据：`evidence/deepseek-12/`（构建 **15 包 0 失败**、统一脱机检查 **12 组全 PASS**、
navigation **730 项 0 失败**、`bringup_layout` / `nav_forwarder` 新增）。

### 最终包结构与可执行文件分布

```
boom_birds_nav     :（无可执行文件，纯兼容转发）
boom_birds_bringup : lifecycle_node sih_params
boom_birds_control : px4_interface_node
boom_birds_sim     : sitl_hold_relay sitl_truth_source vio_source
boom_birds_sensing : camera_timestamp_probe depth_node mavlink_imu_node pose_adapter stereo_source
```

`nav_forwarder` 组断言 nav 下**每个模块都是转发层**（首行标记 + AST 里没有任何函数/类定义），
即"不复制第二份实现"是可执行的约束，不只是文档承诺。

### 底座归属：`runtime_config` / `frames` / `control_protocol` 放在 **control**

提示词把"配置"写在 bringup 名下，但直接照做会形成包级环：control 需要 `runtime_config`
取阈值默认值，而 bringup 的 `lifecycle_node` 需要 control 的 `px4_frames.LocalFrameAlignment`
做 NED→ROS 换算。实践取法是**把无项目内依赖的底座放进 control**（它是唯一的零依赖层），
bringup / sensing / sim 单向依赖 control：

```
control (runtime_config, frames, control_protocol, px4_*)
   ↑            ↑              ↑
bringup      sensing          sim ──► sensing(synthetic)
```

`bringup_layout` 组把这条方向钉成测试：control 不得引用 bringup，bringup 不得引用 sensing/sim。

### 本轮修掉的三个"拆包才暴露"的真缺陷

1. **包级环**（上面那条）。第一版把 `runtime_config` 放 bringup，立刻被新加的 DAG 断言抓住。
2. **`build/lib` 会把删掉的模块运回安装树**：`setup.py install` 只增不删，
   `install_lib` 从 `build/lib` 复制，源里已迁走的 `runtime_config`/`frames`/`control_protocol`
   仍留在 bringup 的 site-packages 里——"源里已迁走"和"import 还能拿到旧实现"同时成立。
   安装脚本现在先清本包 site-packages 目录、再清 `build/`，然后才安装。
3. **`check_launch_executables.py` 的自我假设过期**：它硬编码"必须找到
   `package=boom_birds_nav` 的 Node"，而 nav 已不再提供任何可执行文件，扫描直接失效报错。
   已改为覆盖**所有** `boom_birds_*` 包，并逐包核对该包的 `setup.py` 声明与
   `ros2 pkg executables`。

### 判定

| 段 | 判定 |
| --- | --- |
| **D** | **PASS**：`control` / `sim` / `sensing` / `bringup` 四包已拆出，`stereo_depth` 与 `boom_birds_interfaces` 原样保留，nav 为纯兼容转发；生产包不依赖 sim（三处测试锁定）；仿真连接与模式限制仍在运行时实施 |
| 其余 | 同前（C 正常 PASS；C 故障 PARTIAL；反向/种子/30 m NOT RUN；B 注入故障下未复现；A-CI 未在 runner 执行） |

## 2026-09-29 DeepSeek 第十一轮：拆包第三批 `boom_birds_sensing`（D）

证据：`evidence/deepseek-11/`（构建 **15 包 0 失败**、统一脱机检查 **10 组全 PASS**、
navigation **727 项 0 失败**、`sensing_layout` 26 项）。

### 迁移内容

新包 `companion/ros2_ws/src/boom_birds_sensing`，迁入 11 个模块：
`stereo_source`、`stereo_capture`、`camera_timestamp`、`timebase`、`depth_node`、
`depth_core`、`pose_adapter`、`mavlink_imu_node`、`mavlink_imu_core`、`mavlink_clock`、
`mavlink_imu_replay`；5 个入口点随包迁移（`stereo_source` / `depth_node` / `pose_adapter` /
`mavlink_imu_node` / `camera_timestamp_probe`）。

安装后可执行文件分布：

```
boom_birds_nav     : lifecycle_node sih_params
boom_birds_control : px4_interface_node
boom_birds_sim     : sitl_hold_relay sitl_truth_source vio_source
boom_birds_sensing : camera_timestamp_probe depth_node mavlink_imu_node pose_adapter stereo_source
```

### 兼容转发的两条真实代价（本轮暴露并处理）

1. **`inspect.getsource` 看不穿转发层**：`test_stereo_capture` 的结构性断言
   （"只允许一处真实采集源构造点"）读的是模块源码，转发层只有转发代码，断言必然失败。
2. **monkeypatch 打在转发层上不生效**：`test_camera_timestamp` 打桩模块级 `decode_stitched`
   时，实现里的调用不会看到补丁。

处理方式：把这几个**感知自有**的测试绑定到实现模块（`boom_birds_sensing.*`）而不是兼容层——
转发层只保证导入路径与对象身份，不承诺对 `inspect`/patch 透明。这条边界写进 README。

另外把"package.xml 必须声明实际用到的第三方依赖"改为**按包参数化**：修好 nav 不代表
sensing 也声明了 `pymavlink`；现在每个包各自核对（因此断言数从 723 增至 727）。

### 判定更新

| 段 | 判定 |
| --- | --- |
| D | PARTIAL：`control` + `sim` + `sensing` 已拆出；`bringup` 与 nav 里剩余的共享模块（`frames`/`camera_geometry`/`synthetic`/`pointcloud_scene`/`deep_checks`/`ros_msg`）未归位 |
| 其余 | 同前（C 正常 PASS；C 故障 PARTIAL；反向/种子/30 m NOT RUN；A-CI 未在 runner 执行） |

## 2026-09-29 DeepSeek 第十轮：拆包第二批 `boom_birds_sim`（D）

证据：`evidence/deepseek-10/`（构建 **13 包 0 失败**、统一脱机检查 **9 组全 PASS**、
navigation **723 项 0 失败**、`sim_layout` 8 项）。

### 迁移内容

新包 `companion/ros2_ws/src/boom_birds_sim`（独立 package.xml / CMakeLists.txt / setup.py /
install_package.sh）：

| 迁入模块 | 说明 |
| --- | --- |
| `sitl_truth_source.py` | SIH 真值源（只回环 14550、dry_run） |
| `vio_source.py` | TEST-ONLY 合成 VIO/IMU 源 |
| `sitl_hold_relay.py` | 旧分段启动的悬停中继（正式入口已用 lifecycle） |

安装后可执行文件分布（`ros2 pkg executables`）：

```
boom_birds_nav     : camera_timestamp_probe depth_node lifecycle_node mavlink_imu_node pose_adapter sih_params stereo_source
boom_birds_control : px4_interface_node
boom_birds_sim     : sitl_hold_relay sitl_truth_source vio_source
```

### 包边界（本轮确定，写进 README）

`synthetic.py`（合成场景渲染器）与 `pointcloud_scene.py` **暂不迁入 sim**：生产入口
`stereo_source` 的 synth 模式仍直接使用它们（`pointcloud_scene` 是 `stereo_source` 的
运行期依赖）。把它们迁走会让**生产包依赖 sim 包**，违反提示词 D 的约束。
正确的收敛方向是把渲染器做成可注入接口，留待 sensing 拆包时处理；本轮把这条边界写明并
用测试锁定（`sim_layout` 断言生产包不得出现 `boom_birds_sim`）。

### 本轮修掉的两个真缺陷

1. **入口点载体被注释吞掉（我上一轮引入）**：第一批移除 `px4_interface_node` 入口点时，
   删除正则用了 `\s*`，它跨行匹配并吃掉了前一行注释的换行，把
   `"lifecycle_node = boom_birds_nav.lifecycle_node:main",` **并进了注释行**。
   于是 nav 的 setup.py 实际只声明 6 个入口点，而旧的可执行文件还在 libexec 里，
   表面上一切正常；第二批的清理一跑就把它删掉，`ros2 pkg executables` 才暴露出来。
   已修回独立一行。
2. **安装白名单不能用安装记录**：重建时 `setup.py install` 会跳过"已是最新"的脚本，
   `--record` 写出的清单可能不完整。我上一轮用记录当白名单做清理，就会把**仍然声明着**的
   入口点误删。现在改为：白名单取自 `setup.py` 的 console_scripts 声明，并强制
   `rm -rf <pkg>.egg-info` 让脚本每次都重新生成，最后逐条核对"声明即必须有"。

> 教训：这两个缺陷都属于"构建成功但可执行文件不对"。真正把它抓住的是新加的
> "声明了但 libexec 里没有就必须失败"核对，而不是单元测试。

### 判定更新

| 段 | 判定 |
| --- | --- |
| D | PARTIAL：`control` + `sim` 已拆出且兼容转发生效；`sensing` / `bringup` 未拆 |
| 其余 | 同前（C 正常 PASS；C 故障 PARTIAL；反向/种子/30 m NOT RUN） |


证据：`evidence/deepseek-09/`（构建 **12 包 0 失败**、统一脱机检查 **8 组全 PASS**、
navigation **723 项 0 失败**、新增 `control_layout` 16 项）。

### 迁移内容

新包 `companion/ros2_ws/src/boom_birds_control`（独立 `package.xml` / `CMakeLists.txt` /
`setup.py` / `bin/install_package.sh`，入口点 `px4_interface_node`）：

| 迁入模块 | 说明 |
| --- | --- |
| `px4_backend.py` | PX4 后端（Fake / MAVLink），只发高层 setpoint |
| `px4_frames.py` | 坐标与 setpoint 换算 |
| `px4_failsafe.py` | 信号超时、迟滞与最终发送许可 |
| `px4_interface_node.py` | 控制接口节点（**唯一**发模式命令的出口） |
| `control_protocol.py` | 会话/序号/轨迹/有效期 |
| `handoff.py`、`sih_guard.py` | 接管判定、SIH 进程守卫 |

- **兼容转发**：`boom_birds_nav/<模块>.py` 改为转发层，**不复制第二份实现**——
  显式把实现的全部公共名与 `__doc__` 绑定到兼容模块，并保留 `python -m` 转发。
  只用 `from ... import *` 是不够的：实现若定义了 `__all__`，星号导入会被截断，
  "文档说转发、实际少一半名字"，只在导入时才炸（本轮实测踩到）。
- **依赖方向**：`boom_birds_control` → `boom_birds_nav`（取 `runtime_config`）；
  生产包**不依赖 sim 包**，由 `test_package_layout.py` 强制断言。
- **入口点回收**：`install_package.sh` 现会删除本包 libexec 中已不在安装记录里的陈旧入口。
  否则迁走 `px4_interface_node` 后旧文件仍留在 `<prefix>/lib/boom_birds_nav/`，
  `ros2 pkg executables` 仍列得出来，看起来像"入口还在"。
- 启动文件与子进程环境同步更新：`px4_sitl_motion.launch.py`、`px4_interface.launch.py`
  指向 `boom_birds_control`；`test/conftest.py` 的 `child_env()` 现在把**同工作空间其它包**
  的源码目录也放进子进程 `PYTHONPATH`（否则兼容入口在子进程里 ImportError 并立刻退出，
  表现得像"节点起不来"）。
- 统一验收入口已纳入新包：`check_offline.py` 新增 `control_layout` 组。

### 拆包后的行为核对

- 结构：12 包构建通过；8 组脱机检查全 PASS；`ros2 pkg executables` 中
  `px4_interface_node` 归 `boom_birds_control`，nav 下已无残留；`launch_executables` 组
  PASS 说明启动文件引用的入口都能解析。
- 一次真实 SIH：`d-batch1-normal` 走到 IDLE→…→**EXECUTING**，随后因
  `handoff_discontinuous`（首条规划轨迹与当前位置超出 0.5 m 接管距离）**合格拒绝**并降落，
  最终 **Disarmed**、`landed: true`。**这次没有复现 v16 的到点闭环**——是同一场景的
  逐次差异，不是拆包引入的回归；但也不能据此说拆包后的正常闭环已通过。

### 判定更新

| 段 | 判定 |
| --- | --- |
| D | **PARTIAL**：`boom_birds_control` 已拆出且兼容转发生效；sensing / bringup / sim 三包未拆 |
| 其余 | 同第八轮（C 正常 PASS；C 故障 PARTIAL；反向/种子/30 m NOT RUN） |

## 2026-09-29 DeepSeek 第八轮：故障矩阵第二批（失联动作 = Return/RTL）

证据：`evidence/deepseek-08/sih-matrix/fault-return.jsonl`（逐场景证据同目录 `fault-return/<场景>/`）。

| 场景 | 注入方式 | 状态序列 | 最近距目标 | 最终 Disarmed | 落地下 |
| --- | --- | --- | --- | --- | --- |
| `fault-plan-cancel-rtl` | `Mission.CANCEL` | …EXECUTING→FAULT_LATCHED→LANDING | 1.237 m | **yes** | true |
| `fault-depth-loss-rtl` | kill `depth_node` | …EXECUTING→FAULT_LATCHED→LANDING→COMPLETE | 3.546 m | **yes** | true |
| `fault-odom-loss-rtl` | kill `sitl_truth_source` | …EXECUTING→FAULT_LATCHED→LANDING | 3.244 m | **yes** | true |
| `fault-setpoint-break-rtl` | kill `traj_server` | …EXECUTING→FAULT_LATCHED→LANDING→COMPLETE | 3.495 m | **yes** | true |
| `fault-mode-confirm-rtl` | kill `px4_interface_node` | …EXECUTING→FAULT_LATCHED | 2.836 m | **yes** | true |

### 两批合并结论（10 个注入故障场景）

- **10/10 最终 Disarmed、`landed: true`**（取 PX4 `px4-commander status` 独立回读）；
- **10/10 没有自动重新接管**：全部走"闭锁 → 请求 Land → 降落"，没有出现无人干预的重新起飞或模式争抢；
- **0/10 出现"合格恢复"**：`recovery.attempts` 均为 0。也就是说**"注入故障 → 合格恢复"仍未成立**，
  目前恢复被触发并确认的证据只来自 `normal-mockamap-v13/v16` 的自然故障。
- 其中深度断流/里程计断流属于**合格拒绝**：恢复前置条件要求定位、深度、地图连续有效
  （提示词 B3），源被杀掉后该条件必然不成立，因此拒绝接管、直接降落是正确行为，
  不应记作缺陷，也不应记作"恢复通过"。
- 失联动作 Land 与 Return 两批的状态序列形态一致：编排器闭锁后显式请求 Land，
  没有把"PX4 自己会 RTL"当作恢复手段。

### 备份副本（本轮 PX4 侧参数）

两批均显式下发并回读：`COM_OF_LOSS_T = 1.0000`（本地版本缺省，**未放大**）、
`MIS_TAKEOFF_ALT = 1.5000`；`COM_OBL_RC_ACT` 分别取 4（Land）与 3（Return）。

### 判定

| 段 | 判定 |
| --- | --- |
| C 正常 | PASS |
| C 故障（Land / Return 两批） | **PARTIAL**：安全收尾与"不自动接管"成立；合格恢复未成立 |
| C 反向矩阵 | PARTIAL（"不得自动接管"由上述 10 例间接覆盖；人工取消/落地/重启/低高度/未知故障/次数耗尽/模式命令被拒仍需逐条显式用例） |
| C 种子矩阵 / 30 m 森林 | NOT RUN |
| D 拆包 | 进行中（本轮开始第一批） |

## 2026-09-29 DeepSeek 第七轮：故障矩阵第一批（失联动作 = Land）

证据：`evidence/deepseek-08/`（构建 11 包 0 失败、统一脱机检查 7 组全 PASS、navigation 723 项）；
矩阵 `evidence/deepseek-08/sih-matrix/fault-land.jsonl`，逐场景证据在同目录 `fault-land/<场景>/`。

### 新增可复现的矩阵入口

- `tools/run_sih_matrix.sh <batch> <spec>`：按 spec 逐行跑 `run_sih_mission.sh`，每行
  `场景|scene|goalx|goaly|goalz|故障|失联动作|超时`；
- `tools/sih_matrix_row.py`：把每个场景的证据汇总成一行 JSON（状态序列、最近距目标、
  setpoint 计数、最终 armed/landed、Disarmed、EGO 事件）；
- `sih_params --obl-action land|rtl`：失联动作可显式选择（故障矩阵要分别验证 Land 与 Return
  之后的恢复），数值仍只有单一来源，脚本零字面量。

### 第一批结果（固定 mockamap、目标 `(5.0, 1.0, 2.5)`、失联动作 Land）

| 场景 | 注入方式 | 状态序列 | 最近距目标 | 最终 Disarmed | 落地下 |
| --- | --- | --- | --- | --- | --- |
| `fault-plan-cancel` | `Mission.CANCEL` | …EXECUTING→FAULT_LATCHED→LANDING→COMPLETE | 0.569 m | **yes** | true |
| `fault-depth-loss` | kill `depth_node` | …EXECUTING→FAULT_LATCHED→LANDING→COMPLETE | 2.985 m | **yes** | true |
| `fault-odom-loss` | kill `sitl_truth_source` | …EXECUTING→FAULT_LATCHED→LANDING | 3.009 m | **yes** | true |
| `fault-setpoint-break` | kill `traj_server` | …EXECUTING→LANDING→COMPLETE | 0.882 m | **yes** | true |
| `fault-mode-confirm` | kill `px4_interface_node` | …EXECUTING→FAULT_LATCHED | 2.048 m | **yes** | true |

**五个场景全部最终 Disarmed、`landed: true`**；Disarmed 取自 PX4 `px4-commander status`
独立回读（不是"软件进程退出"）。

### 诚实说明（本轮暴露的缺口）

- 这五项里 `recovery.attempts` 多为 0：故障大多走"闭锁 + 降落"或直接收尾，**没有**进入
  `RECOVERING`。目前"恢复被触发并确认"的证据来自 `normal-mockamap-v13/v16` 的自然故障，
  而不是本批注入故障。也就是说：**"注入故障 → 合格恢复"尚未成立**。
- 其中一部分是**合格的拒绝**而非缺陷：深度断流 / 里程计断流时，恢复的前置条件
  （定位、深度、地图连续有效，见提示词 B3）必然不成立，因此拒绝接管、直接降落是正确行为，
  应当报告为"合理拒绝"。
- `fault-mode-confirm` 的闭锁原因 `revoked:control_service_missing` 是预期语义：模式命令的
  唯一出口消失时不得继续。
- 本批**未验证** PX4 进入 Return（`COM_OBL_RC_ACT=3`）之后的恢复；也未验证"次数耗尽"以外的
  反向用例。这些是下一批。

### 判定

| 段 | 判定 |
| --- | --- |
| C 正常 | PASS（第六轮） |
| C 故障（Land） | **PARTIAL**：5/5 安全收尾 + Disarmed；但"合格恢复"未成立 |
| C 故障（Return） | NOT RUN |
| C 反向矩阵 / 30 m 森林 / seed 1–5 | NOT RUN |
| B | PARTIAL（自然故障下的恢复确认已实测；注入故障下未复现） |
| D | NOT RUN |

## 2026-09-29 DeepSeek 第六轮：完整新 SIH 任务首次通过（C 正常路径）

证据：`evidence/deepseek-07/`（构建 11 包 0 失败、统一脱机检查 7 组全 PASS、navigation 723 项）；
SIH `evidence/deepseek-01/sih/normal-mockamap-v16`。

### 结果：`normal-mockamap-v16`（mockamap 固定场景 seed 511，目标 `(5.0, 1.0, 2.5) m`）

```
t+  0.0  IDLE
t+  4.0  TAKEOFF
t+ 20.5  HOLD_READY
t+ 21.5  OFFBOARD_PENDING
t+ 22.0  EXECUTING
t+ 31.5  RECOVERING :: waiting:valid_duration_not_met
t+ 32.5  EXECUTING  :: confirmed      ← 自动恢复确认后重新规划
t+ 35.5  LANDING    :: revoked:landed ← 到点判定自动降落
t+ 45.5  COMPLETE
```

| 指标 | 值 | 来源 |
| --- | --- | --- |
| 最终状态 | **COMPLETE** | `/boom_birds/mission/status` |
| 机载最近距目标 | **0.159 m**（ROS `[5.154, 1.038, 2.505]`，容差 0.3 m） | `recorder_execution.jsonl` |
| 实际水平位移 | x 到 5.0 m 目标（5.007 m 量级） | PX4 `vehicle_local_position` 独立回读 |
| setpoint | 发 691 / 收 1076 | `/boom_birds/control/status` counters |
| 最终 | **Disarmed**、`landed: true`、`at_rest: true`、`in failsafe: no` | PX4 `px4-commander status` / `vehicle_land_detected` |
| 自动恢复 | 1 次尝试即 `confirmed` | 同上 timeline |

**这是本提示词「C 正常」路径的首次通过**：起飞 → 稳定锁点 → Offboard → 目标执行 → 降落，
且中途完成了一次合格的自动恢复（B6「回读确认后重新规划」）。测试数量不作数，此处给的是事件序列与回读值。

### 本轮修的两处

1. **保持点要按距离选**：规划流停止后此前一律保持在**当前位置**，于是飞行器停在离目标
   0.379 m 处不再收敛（容差 0.3 m）。现在离目标已在 `handoff_max_distance_m`（同"近距离
   接管点"语义）以内 ⇒ 保持点放到**目标**，让飞控收敛最后一段；否则保持在当前位置，
   不在没有规划的情况下继续飞。
2. **瞬时规划取消不再是故障**：EGO 在任一次重规划失败时都会发失效消息，此前一收到就判
   `planning_link` → 恢复，而恢复的前置条件（地图/输入连续有效）此刻必然不成立，两次预算
   被白耗并闭锁降落（实测同一任务里反复出现）。现在改为：退役旧轨迹 → 进入"规划流静默" →
   保持 + 限频重发使能，上限 `planner_activate_timeout_s`；只有静默超过该窗口才闭锁。

### PX4 参数基线固化并留证

`parameters.bson` 会跨运行残留（早先实验改过失效动作），因此每次运行都显式下发 + 回读，
数值仍只有一个定义点（`RuntimeConfig` / `sih_params`，脚本无字面量）：

| 参数 | 本次值 | 说明 |
| --- | --- | --- |
| `COM_OF_LOSS_T` | **1.0000** | 本地 PX4 版本缺省；显式下发即为**留证不放大** |
| `COM_OBL_RC_ACT` | 4 | Land（SIH 的失效动作选择） |
| `MIS_TAKEOFF_ALT` | 1.5000 | 等于 `takeoff_altitude_agl_m` |

### 判定更新

| 段 | 判定 |
| --- | --- |
| A | PASS（CI 未在 runner 执行 ⇒ 该子项 PARTIAL） |
| B | **PARTIAL→接近 PASS**：恢复触发、阻塞上报、预算耗尽闭锁、安全降落、以及**确认后重新规划回 EXECUTING**均已实测；尚缺 Land/Return 中断与耗尽反向用例 |
| C | **正常路径 PASS**（如上）；故障矩阵、反向矩阵、30 m 森林、seed 1–5 仍 NOT RUN |
| D | NOT RUN |
| E | PASS |

## 2026-09-29 DeepSeek 第五轮：状态新鲜度混入心跳年龄 + 使能重发限频

证据：`evidence/deepseek-05/`（构建 11 包 0 失败、统一脱机检查 7 组全 PASS、navigation 723 项）；
SIH `evidence/deepseek-01/sih/normal-mockamap-v13`。

### 又一处同类混同：status_age 把心跳年龄算了进去

上一轮修掉 px4_failsafe 的心跳窗口后，恢复窗口仍攒不满。给恢复窗口加"清零原因"上报后拿到确证：

```
RECOVERING :: waiting:valid_duration_not_met,window_reset_by:status_age=1.02
```

`lifecycle_node.observation()` 里 `status_age = max(报文年龄, 心跳年龄)`，而 PX4 心跳本来就是
~1 Hz，于是 `status_age` 恒为 ≈1.0 s，任何 `<= status_timeout_s (1.0)` 的判定都会间歇失败——
恢复的"连续 1 s 有效"永远不成立，接管闸门也会被误拒。

改法：`status_age` 只表示**这份控制接口观测有多旧**（报文自身新鲜度）。飞控是否在线由
`connected` / `sending` / `reasons` 单独表达，并由 `px4_failsafe` 用它自己的心跳窗口
（`heartbeat_timeout_s=2.5`）判定，不在这里二次计入。

### 使能重发限频（我上一轮引入的缺陷）

上一轮加的"有界重发 enable_planner"按控制周期（50 Hz）触发，而 EGO 的 `projectRequest`
每次都调用 `planNextWaypoint` —— 实测 EGO 连续 `traj 1 success / traj 2 success` 后立刻
`PROJECT_CANCEL`，飞行器根本没动。新增 `planner_enable_retry_s = 0.5` 限频：30 s 窗口内
仍可重试约 60 次，但不会把规划器的 FSM 打乱。

### 本轮的里程碑（`normal-mockamap-v13`，目标 `(5.0, 1.0, 2.5)`）

| 指标 | 结果 |
| --- | --- |
| **自动恢复确认** | `RECOVERING → EXECUTING :: confirmed`（第 1 次尝试即确认，满足 B6 的"回读确认后重新规划"） |
| **飞抵目标** | 机载最近距离 **0.379 m**（ROS `[4.951, 0.635, 2.586]`，`armed=True`、`landed=2` 在空中） |
| 飞行 | 5.0 m 目标、实际 x 到 **5.007 m** |
| setpoint | 发 665 / 收 1092 |
| 结束 | 第二次故障（`setpoint_link`）恢复预算耗尽 → 降落 → **Disarmed** |

**这是首次同时拿到"合格恢复确认"与"飞抵目标"的 SIH 运行。** 仍未拿到一次"到点判定 → 自动降落"
的完整正常任务：最近距离 0.379 m 略大于 `goal_tolerance_m`（0.3 m），随后第二次故障打断了收敛。

### 仍未完成

- **C 正常闭环**：差最后 0.08 m 的收敛与到点判定。
- **C 故障矩阵、反向矩阵、30 m 森林、seed 1–5**：全 NOT RUN。**D 拆包**：未开始。

## 2026-09-29 DeepSeek 第四轮：心跳超时是整条故障链的根因

证据：`evidence/deepseek-04/`（构建 11 包 0 失败 + 统一脱机检查 7 组全 PASS，navigation 723 项）；
SIH `evidence/deepseek-01/sih/normal-mockamap-v{10,11,12}`、`normal-mockamap-goal1`。

### 根因：把"心跳到达超时"和 PX4 的 `COM_OF_LOSS_T` 混为一谈

`ExecutionStatus` 的原因字典给出了决定性证据：

```
signal_stale signal=px4_heartbeat age=1.0223610899993218 limit=1.0
```

**PX4 的 MAVLink HEARTBEAT 是 1 Hz**，而 `heartbeat_timeout_s` 在上一轮"单一来源收敛"时
被统一成 **1.0 s**（`config/px4_interface.yaml` 原为 2.0 s，代码默认 1.0 s，我按代码取值）。
于是**几乎每一次心跳抖动**（age 1.022 s）都判一次 `signal_stale`：

- 每次 `signal_stale` 都把 `px4_failsafe` 的迟滞计数清零；
- 计数攒不满 5 次，闸门就持续关闭（`allow=False state=STOPPED reasons=[]`——**这正是它一直
  没有原因码、极难定位的原因**：门是被"计数不够"关的，不是被某条违规关的）；
- setpoint 流被周期性饿死，编排器据此判成链路故障并进入恢复，两次预算耗尽后闭锁降落。

修法（不是放宽阈值，是纠正语义）：

| 项 | 旧 | 新 | 依据 |
| --- | --- | --- | --- |
| `heartbeat_timeout_s` | 1.0 | **2.5** | PX4 HEARTBEAT 标称 1 Hz，窗口须覆盖至少两次漏拍 |
| `FailsafeConfig` 模块默认心跳超时 | 0.50 | **2.5** | 同上；旧注释"取 1 个周期"是错的 |
| `px4_heartbeat_period_s` 参考值 | 0.5（当成 2 Hz） | **1.0** | PX4 侧实测 1 Hz |
| `landed_state_timeout_s`（新增） | 1.0（硬编码） | **2.5** | `EXTENDED_SYS_STATE` 约 1 Hz；1.0 s 会让 `landed_known` 翻假，恢复前置条件被判 `not_airborne` |

同时修正 `test_px4_failsafe.py` 里那条**把两者混为一谈的断言**：心跳看门狗不再要求
"小于 `COM_OF_LOSS_T`"，改为要求"覆盖至少两次标称漏拍"；其余主动停发所依赖的信号仍保持
"小于 `COM_OF_LOSS_T`"。

### 编排器新增：规划流停止后自己维持保持段

规划器走完轨迹后会停止发布，此前 setpoint 断流先触发故障判定，把已到点的任务闭锁降落。
现在 EXECUTING 期间若规划指令静默超过 `command_timeout_s`，编排器调用
`lifecycle.hold_here()` 把保持/制动点设到**当前观测位置**并持续发布 HOLD，
使飞控保持而不是进入失联。

### 结果

| 运行 | EXECUTING 时长 | setpoint 发送 | 结束 |
| --- | --- | --- | --- |
| v10/v11 | ~3 s | 403 / 463 | 误报故障 → 降落 |
| **v12** | **152 s** | **7971** | 持续执行；末段相机断流才进恢复 |

`normal-mockamap-v12` 是迄今最接近完整正常任务的一次：**连续执行 152 s、实际下发 7971 条
setpoint、全程无心跳误报**。未能到点是因为目标 `(2.0, 0.0, 1.6)` 落在障碍里，EGO 的轨迹
在 1.64 m 处结束（随后编排器按新逻辑正确保持而非误报）。

### 仍未完成

- **C 正常任务**：还差一次"目标可达 → 到点判定 → 降落 → Disarmed"的完整闭环。
- **B**：成功恢复未走通；**C 故障/反向矩阵、30 m 森林、seed 1–5** 全 NOT RUN；**D 拆包**未开始。

## 2026-09-29 DeepSeek 第三轮：阈值与实际链路节奏对齐

证据：`evidence/deepseek-03/`（构建 11 包 0 失败 + 统一脱机检查 7 组全 PASS，navigation 723 项）；
SIH 运行 `evidence/deepseek-01/sih/normal-mockamap-v{6,7,8,9}`、`gap-diag`。

### 先量测，再改阈值

对已发布的流做了**间隔统计**（不再只看计数）：

| 流 | 消息数 | 最大间隔 | >1 s 的空洞 |
| --- | --- | --- | --- |
| `/boom_birds/vio/odom_ego` | 1076 | **0.067 s** | 0 |
| `/boom_birds/vio/camera_pose` | 1076 | 0.067 s | 0 |
| `/boom_birds/imu` | 1076 | 0.067 s | 0 |
| `/boom_birds/depth/image` | 116 | **0.208 s** | 0 |

结论：传输层**没有**断流。此前所有"链路失效"都是**阈值与被测链路节奏不匹配**造成的假报。
据此修掉 3 处，并各写明依据（都不是为了"让测试过"而放宽）：

1. `setpoint_interrupt_timeout_s = 1.0`（新增）：断流判定不再用命令有效期 0.2 s，而与 PX4 自身的
   Offboard 失联超时 `COM_OF_LOSS_T`（缺省 1.0 s）对齐。0.2 s 会在进入 EXECUTING 后 0.5 s 内误报，
   两次恢复预算被白耗。
2. `planning_timeout_s: 0.2 → 1.0`：地图由深度融合而来，深度最大间隔 0.208 s，比旧窗口还大，
   于是 `map_ready` 必然抖动并被判成 `planning_link`。
3. **到达目标不是故障**：规划器走完轨迹后本来就会停止发布，此前被计入 `setpoint_link`
   并闭锁降落（实测飞到目标半径内 0.11 m 后触发）。已在 `lifecycle_node` 加目标容差判定。
4. 记录器降载：每秒 spawn 3 个 `px4-listener` 会打断 ROS 位姿/命令流，反而打出假的"链路断流"；
   时间线证据改由 `ExecutionStatus` 记录器提供，PX4 侧只做低频独立回读。**测量工具不得扰动被测系统。**

### 本轮 SIH 飞行结果（全部最终 Disarmed）

| 运行 | EGO | 实际飞距 | setpoint | 结束 |
| --- | --- | --- | --- | --- |
| `normal-mockamap-v7` | `traj 1 success` ×2 | 1.11 m | 发 402 | FAULT_LATCHED → 降落 → Disarmed |
| `normal-mockamap-v8` | `traj 1 success` ×2 | **1.905 m** | 发 153 | 到点附近后 `manual_mode` 撤销 → 降落 → Disarmed |
| `normal-mockamap-v9` | 规划成功 | 1.74 m | 发 404 | `setpoint_link` 耗尽 → 降落 → Disarmed |

目标点为 `(2.0, 0.0, 1.6) m`（ROS global）。**飞距已到目标的 87–95%**，但**仍未完成一次
"到点→判定到达→降落"的完整正常任务**：EGO 轨迹走完后 setpoint 流停止与目标判定之间仍有
时序缺口。

### 判定

| 段 | 判定 |
| --- | --- |
| A | PASS（CI 未在 runner 执行 ⇒ 该子项 PARTIAL） |
| B | PARTIAL（恢复进入/阻塞上报/预算耗尽/安全降落有实测；成功恢复未走通） |
| C | PARTIAL（正常路径飞到目标 87–95%；故障矩阵、反向矩阵、30 m 森林、seed 1–5 全 NOT RUN） |
| D | NOT RUN |
| E | PASS |

## 2026-09-29 DeepSeek 第二轮：SIH 集成缺陷收敛

证据：`evidence/deepseek-02/`（构建 + 统一脱机检查 7 组全 PASS）；SIH 运行在
`evidence/deepseek-01/sih/`（`normal-mockamap` … `normal-mockamap-v5`、`diag-mockamap`）。

### 本轮修掉的 4 个"只有实跑才暴露"的集成缺陷

1. **合成图像发布高度门槛不能当规划链闸门**。门槛 1.3 m 与起飞高度 1.5 m 的余量只有 0.2 m，
   飞行中门槛反复闭合：90 s 只发出 39 帧深度，EGO 报 153 次"odom or depth lost"并
   `PROJECT_CANCEL`，编排器随即闭锁降落——一条自伤回路。该门槛只是"不在地面"的护栏，
   EGO 的启动时机由编排器在确认 Offboard 后决定。已恢复为 0.3 m，并在
   `runtime_config.ALTITUDE_REFERENCE` 写明理由。
2. **同一话题两个发布者**。上一轮为补 IMU 加入的 `vio_source` 与既有的 `sitl_truth_source`
   同时发布 `/boom_birds/imu`，违反契约"合成替身与其它来源不得同时占用同一话题"。已移除；
   实测 IMU 由 `sitl_truth_source` 单独提供（单次任务 784–1303 条）。
3. **单发使能会被静默丢弃**。EGO 的 `projectRequest` 在 `offboard_confirmed`/odom/地图未就绪时
   直接 `return`（无日志、无回执），而编排器只在进入 EXECUTING 时发一次 `enable_planner`，
   任务于是永久停在 WAIT_TARGET。已改为窗口内**有界重发**（新增 `planner_activate_timeout_s=30 s`）。
4. **单帧抖动被当成断流**。编排器按单帧 `sending=False` 判 `setpoint_link`，而节点发点占空
   约 88%（实测 381 发 / 572 收），于是每次调度抖动都触发一次自动恢复并白耗两次预算。
   已改为"持续超过 `command_timeout_s` 未发出"才算断流。

语义修正：规划器作废轨迹（`PROJECT_CANCEL`）属提示词允许恢复的**规划链路故障**，改为走既有
有界恢复判定，不再立即永久闭锁（未启用恢复时仍闭锁，失败语义不变）。

### 本轮 SIH 结果

| 运行 | 到达 | EGO | setpoint | 最终 |
| --- | --- | --- | --- | --- |
| `normal-mockamap-v5` | TAKEOFF→HOLD_READY→OFFBOARD_PENDING→**EXECUTING** | `traj 1 success` ×2、`plan_success=1` | 发 381 / 收 572 | **Disarmed**、落地下 |

**仍未到达目标点**：EGO 在 `EXEC_TRAJ` 中 `PROJECT_CANCEL`（伴随 `odom or depth lost`，
占据更新在 t+54 s 后停止），setpoint 流随之停止；恢复因地图就绪不成立、
`valid_duration_not_met` 无法累计而**按规定 fail-closed 拒绝接管**，2 次预算耗尽后闭锁降落。
这是**合格拒绝**，不是合格恢复。

### 判定（本提示词 A–E）

| 段 | 判定 | 说明 |
| --- | --- | --- |
| A | PASS（CI 除外） | 配置/环境/生命周期/协议均收敛并有测试；CI 已写但**从未在 runner 执行**，新增 apt 清单未在容器验证 ⇒ CI 部分 PARTIAL |
| B | PARTIAL | 进入 RECOVERING、阻塞项如实上报、预算耗尽闭锁、安全降落均有实测；"恢复成功→重新规划→回 EXECUTING"未走通；Land/Return 中断未验证 |
| C | PARTIAL | 正常路径到 EXECUTING 并有实际 setpoint；**故障矩阵、反向矩阵、30 m 森林、seed 1–5 全部 NOT RUN** |
| D | NOT RUN | `boom_birds_nav` 仍是唯一实现，未拆包 |
| E | PASS | 本表 + README 生命周期表/控制协议表/参数来源/EGO fork 补丁索引 + DECISIONS D-028/D-029 + 回退步骤 |

### 回退步骤

1. 各轮证据互不覆盖：`evidence/deepseek-01/`、`evidence/deepseek-02/`；失败日志在
   `evidence/deepseek-01/failures/`，后续通过不覆盖失败证据。
2. 回到接手前状态：以 `checkpoints/deepseek-handoff-20260929_173140` 的补丁与未跟踪归档为准，
   **先在单独目录核验**（`git apply --check` 或逐文件比对）再落地；不要直接反向覆盖，
   因为接手后还有本轮修改，直接覆盖会丢掉别人后续的工作。
3. 本轮改动全部是文件级且未提交；需要单独撤销时按文件回退：
   `runtime_config.py`、`config/runtime.yaml`、`lifecycle.py`、`lifecycle_node.py`、
   `px4_interface_node.py`、`px4_failsafe.py`、`bin/install_package.sh`、
   `launch/px4_sitl_motion.launch.py`、`launch/px4_sih_mission.launch.py`、
   `tools/run_sih_mission.sh`、`tools/sih_record.py`。
4. 原 venv 备份：`checkpoints/20260928_234207/previous-venv.tar.gz`；当前正式 venv
   `bb_build/architecture/venv`。两者都保留，不要用补丁反向覆盖环境。
5. EGO 子模块为 10 修改 + 1 未跟踪（`SessionBspline.msg`），未提交；回退同样先在临时目录核验。

## 2026-09-29 结构修整（进行中）

- 已保留原未提交修改；基线补丁、未跟踪文件和原 venv 备份：`/home/waterc/bb_build/architecture/checkpoints/20260928_234207`。
- 独立目录：`/home/waterc/bb_build/architecture/{venv,build,install,log}`；NumPy/OpenCV 来自系统，MAVLink 来自声明依赖，不注入 `~/.local`。OpenVINS 订阅测试仍复用 `/home/waterc/bb_build/ov/install`；未建立新 OS 镜像。
- 阶段二报告：`/home/waterc/bb_build/architecture/evidence/stage2-final/report.json`。导航 526、标定 10、地图行为 48 项及 C++ 轨迹检查 PASS；此前编译失败与一次迁移路径测试失败单独保留，未覆盖。
- 已提取 `stereo_depth.core`，项目 EGO 默认同步 CameraInfo；几何变化闭锁地图/轨迹，静态模式显式选择且与 CameraInfo 互斥。
- 阶段三已有 `boom_birds_interfaces`、会话封装、控制入口、生命周期纯状态机和 ROS 封装。2026-09-29 复查后验收报告：`/home/waterc/bb_build/architecture/evidence/stage3-reviewed/report.json`；导航 561、标定 10、地图行为 49 项及 C++ 轨迹检查 PASS，Python 测试无跳过。9 个相关包构建通过。交接前核对报告文件散列与当前源码一致。
- 这些结果仅证明已覆盖的脱机行为。**（2026-09-29 DeepSeek 轮次已部分取代本条：共享参数已收敛到 `runtime_config.py`、旧 shell 高度/模式判定已删除、Land/Return 已可由 `mode_detail` 区分、新 SIH 入口已实跑到 EXECUTING；自动恢复只到 PARTIAL，故障矩阵与拆包仍未完成。详见上一节。）**
- DeepSeek 接手前工作区快照：`/home/waterc/bb_build/architecture/checkpoints/deepseek-handoff-20260929_173140`（母仓库/子模块补丁、SHA、状态、未跟踪文件）。下一执行方先核对当前工作区再继续；Codex 后续独立验收。
- 本轮真机、同步真实双目/IMU、真实 VIO、ARM64 性能与飞行验收：NOT RUN。

## 历史软件与仿真记录（截至 2026-09-28）

以下保留当时的实现、命令和结果；后续拆包、MAVROS、控制路线与 Pi 验证见页首记录。

- ROS 2 Jazzy/x86_64 已构建 `stereo_depth`、`boom_birds_nav`、EGO 依赖链和 `ego_planner`，以及 OpenVINS 的 `ov_core`、`ov_init`、`ov_msckf`。OpenVINS 已启动并订阅合成双目与 IMU；尚无有效 VIO 初始化/里程计输出证据。
- 工作空间隔离 Python 环境可导入 NumPy 1.26.4、OpenCV 4.6.0、rclpy、cv_bridge；深度 CLI 和既有 10 项标定单测通过。构建脚本对 colcon/CMake 限并发，并默认使用仓库外持久目录 `/home/waterc/bb_build/main/{build,install,log}`。
- `boom_birds_nav` 提供合成/文件双目源、`32FC1` 米制深度（无效 NaN）、`16UC1` 毫米兼容深度（无效整数 0）、完整 H×W XYZ 点云、CameraInfo、按采集时间缓存/插值的位姿适配。TEST-ONLY 合成 IMU/VIO 不能作为飞控或精度证据。话题、坐标系和时间参数以 [契约](../companion/ros2_ws/src/boom_birds_interfaces/config/contract.yaml) 为准。
- Companion 端 **真实双目采集 → ROS 发布链代码完成**（回放链脱机通过，真机未测）：`stereo_source`
  以 `mode` 区分 `v4l2`/`replay`/`file`/`synth`，是**唯一**采集与发布入口；只有
  `stereo_capture.V4L2FrameSource` 打开设备，左右原始图 + **各自与图像配对**的 CameraInfo
  供深度与 OpenVINS 共用，同帧左右共享同一 V4L2 采集时间戳；时域不可核实即**拒绝发布并诊断**，
  不回退 OpenCV/发布时间。跨分辨率缩放只缩放 `fx/fy/cx/cy`，**米制物理基线不缩放**
  （曾错误地把基线也乘 scale，使右目 `P[0][3]` 在 scale=0.5 时衰减到 1/4，已修正并有单测锁定）。
  回放模式发布语义与真实采集一致，但记录帧没有曝光时间戳，**不能**用于证明相机-IMU 同步。
  本节点不做矫正/深度（由 `stereo_depth`/`depth_node` 负责）。配置见
  [stereo_camera.yaml](../companion/ros2_ws/src/boom_birds_sensing/config/stereo_camera.yaml)。
- Companion 端 **规划输出 → Px4Interface → PX4 高层控制接口代码完成**（脱机通过；SITL 部分通过，真机未测）：
  `px4_interface_node` 订阅 `PositionCommand`，经 `px4_frames` 换算到 NED 并配 `type_mask`，由
  `Px4Backend` 协议下发；`FakePx4Backend` 用于确定性脱机测试，生产使用 `MavrosPx4Backend`。
  只允许高层 setpoint，协议无 PWM/DShot/电机/执行器面；默认 `dry_run=true`、`allow_arming=false`、
  仅回环地址。**坐标系对齐是位置 setpoint 的前置条件**：EGO 的 `world` 与 PX4 局部 NED 是两个局部系，
  轴翻转只解决"哪个轴朝哪"，原点与水平朝向不会自动一致。放行位置需要**两项独立证据**：
  ①水平朝向（VIO/PX4 航向残差被动核实，或 `frame_alignment_observed=true`）；
  ②原点/平移（`frame_alignment_origin_evidence=true`，即外部视觉融合把 EKF 原点定义在 VIO 原点上，
  或已实测标定 `translation_m`）。**航向核实不能替代原点证据**——两系可以朝向一致而原点相隔很远，
  因此 `YawAlignmentResidual` 只暴露 `yaw_verified`，单独达标不放行位置。
  任一项缺失即不下发位置 setpoint（原因码 `local_frame_not_aligned`，状态 `STOPPED`，速度/加速度同样不发）。
  setpoint 的位置/速度/加速度/偏航/偏航角速率由 `px4_frames.ros_local_to_ned_setpoint`
  **一次**换算（前三者走同一 `R(φ)`，平移只作用于位置；偏航 `−(yaw+φ)`；偏航角速率 `−yaw_dot`）。规划拒绝/轨迹失效/VIO/IMU/相机断流/链路超时/飞控重启均停发并写状态；
  恢复需迟滞且需看到新的 boot_id/trajectory_id。**停发 setpoint ≠ PX4 悬停或安全接管**——
  飞控侧动作取决于 `COM_OF_LOSS_T`/`COM_OBL_RC_ACT`，必须在 SITL 与实机分别验证。
  上述为 `px4_position` 路线；新实机入口采用 Companion 位置闭环，见 D-058。配置见
  [px4_interface.yaml](../companion/ros2_ws/src/boom_birds_control/config/px4_interface.yaml)。
- Companion 端 **PX4 MAVLink IMU 上行 + 时间同步第一版代码完成**（脱机验证通过，真机未测）：`mavlink_imu_node` 接收 `HIGHRES_IMU` 并发布契约话题 `/boom_birds/imu`；`mavlink_clock` 用 `TIMESYNC` 往返估计「PX4 启动时钟 − Companion 单调时钟」偏移；`timebase` 把单调时钟映射到 ROS 时间域。`stereo_source` 的 V4L2/回放模式已接入 `camera_timestamp` 的采集时间戳判定与拼接帧切分，并发布左右图；同帧左右共享时间戳，时域不可核实时拒发。编译期 `offsetof()` 测试核对 `v4l2_buffer` 布局。**真实曝光时刻及相机与 IMU 的真机同步尚未验证。**串口/波特率/sysid-compid/流频率/话题全部配置化；无可靠映射、字段或时间校验失败时拒绝发布并给出诊断。运行与参数见 [boom_birds_nav README](../companion/ros2_ws/src/boom_birds_nav/README.md)，配置见 [mavlink_imu.yaml](../companion/ros2_ws/src/boom_birds_sensing/config/mavlink_imu.yaml)。
- EGO 地图跳过无效深度观测，完成融合/膨胀且达到占据阈值后才放行规划。规划器对整段轨迹做动态约束与保守碰撞校验；拒绝时向 `traj_server` 发送失效消息并停发 PositionCommand。停发命令不等于 PX4 悬停或安全接管。
- **PX4 SIH 前台运动仿真（TEST-ONLY）**：以 PX4 自身局部位置/姿态回读作为仿真真值，驱动合成双目随相机平移和旋转，复用 `depth_node`、EGO 与 `px4_interface_node`。2026-09-24 在仅回环 `-i 0` 的 SIH 中，起飞并切 OFFBOARD 后，实际回读位置从约 `x=0.04 m` 到 `x=0.97 m`，目标 `x=1.0 m`，最后 `commander land` 自动 Disarmed；RViz2 前台窗口显示深度及占据点云。运行说明见 [导航包](../companion/ros2_ws/src/boom_birds_nav/README.md#前台运动仿真test-only)。该链使用仿真真值与测试 IMU，绕过 OpenVINS，不构成真机同步、VIO 或一般绕障证据。
- **EGO mockamap 复杂场景接入（TEST-ONLY）**：EGO 原有 `mockamap` 以固定 seed 生成 4,880 点场景；仿真相机从世界点云生成左右图，保持 `depth_node` 深度计算和 EGO 深度建图路径。先 SIH 起飞再启动规划链的本机试验中，深度约 5 Hz、EGO 膨胀占据点云非空，目标 `(1.0, 0.0, 2.5) m` 的 PX4 回读约 `(1.04, -0.01, 2.49) m`，最后降落并 Disarmed。地面阶段先启动规划链曾导致切 OFFBOARD 后下降；当前起飞目标改为 1.5 m，图像发布门槛改为 1.3 m。运行顺序和边界见 [导航包](../companion/ros2_ws/src/boom_birds_nav/README.md#使用-ego-mockamap-复杂场景保留合成双目链)。该单次轨迹不证明一般绕障。
- **EGO 原生 random_forest 场景（TEST-ONLY）**：已接入 250 柱体、250 环体的默认森林生成器，保持合成左右图→StereoSGBM→深度→EGO 的链路。首次 `(4,-3,1.5) m` 试飞发生 PX4 Offboard 信号丢失并进入 Return，已安全降落 Disarmed，该次判 FAIL。后续将 SIH Offboard 丢失动作改为 Land、场景体素参数对齐 0.1 m，并把深度发布尺寸调为 240×180、同步缩放 EGO 内参，新增 `/boom_birds/depth/color_preview` 彩色预览。2026-09-27 按分段启动顺序复测同一森林生成器，目标 `(4.0, -3.0, 2.5) m`：PX4 巡航路径距原始森林点云最近 `0.536 m`，起点至目标直线基准最近 `0.180 m`，目标最近误差 `0.014 m`，ulog 未见 failsafe，降落后 Disarmed。日志和场景点云在本机 `companion/ros2_ws/log/forest_sih_20260927/`（Git 忽略）。这是单次固定场景仿真通过，不代表多 seed 成功率。
- **random_forest 30 m 绕障（TEST-ONLY，固定 seed 单次通过）**：规划世界起点 `(-15,0,0.1) m`、终点 `(15,0,1.0) m`；SIH 起飞到约 1.5 m 后启动双目/深度/悬停链，进入 OFFBOARD 后启动 EGO。`seed=3`、20 柱体 + 20 环体的 2026-09-28 试飞中，PX4 巡航中心距原始点云最近 `0.508 m`，直线基准 `0.041 m`，最大侧向偏移 `0.708 m`，目标最近误差 `0.004 m`；87.7 s 巡航内 failsafe 0 次，Offboard 消息最大间隔 `0.544 s`，降落 Disarmed。点云、ulog、节点输出和计算口径在本机 `companion/ros2_ws/log/forest_30m_seed3_20260928/`（Git 忽略）。之前的密集 seed 发生轨迹拒绝/Offboard 失联，另一次 `seed=2`、10+10 障碍因建图未就绪未起步，均不计入通过。点云距离未扣机体外廓；未验证多 seed 成功率。
- **mockamap 长距离绕障复测（TEST-ONLY，固定场景通过一次）**：2026-09-27 先用旧启动顺序测试 5 m 目标，PX4 在 Hold 中已播放完 EGO 轨迹，切 OFFBOARD 后首个巡航设定点约为 `x=5 m`，使飞行器近似直线追目标：轨迹距原始点云最小 `0.064 m`，直线理论最小约 `0.100 m`，判为 **FAIL**。现增加 SIH 起点悬停设定点与 0.5 m 接管门槛，先切 OFFBOARD、再启动 EGO 目标规划。同一 `seed=511`、目标 `(5.0, 1.0, 2.5) m` 的新测试中，PX4 巡航轨迹距原始 mockamap 点云最小 `0.719 m`，设定点最小 `0.671 m`，最大侧向绕行约 `0.906 m`，目标附近误差约 `0.063 m`，降落 Disarmed；本机记录在 `companion/ros2_ws/log/mockamap_long_20260927/`（Git 忽略）。这证明**固定场景单次长距离绕障仿真**，不证明多场景成功率、连续安全性或真机能力。
- 默认标定 `stereo_depth/calibration/live_20260916_210120_642136/candidate.npz` 随工程保存；2026-09-16/17 的 Pi 5 旧记录仅证明当时的几何校验，真实米制距离精度仍须独立尺测。

## 2026-09-28 验收范围和结果（历史）

第一阶段规划范围暂定为侧向可达目标、占据目标拒绝、长期门控、轨迹失效、运动中重置；**正后方目标绕障暂不纳入本阶段**。旧的正后方测试在 25 s 内 42 次规划均被安全拒绝，记录留在本机 `companion/ros2_ws/log/review_fix/final_center/`，不把该用例写成 PASS。

| WSL 验证项 | 结果 |
| --- | --- |
| 导航 Python 单测 | PASS：35 项 |
| EGO 地图行为回归 | PASS：41 项；当前参数至少 6 次正命中后才放行 |
| C++ 轨迹校验 | PASS：完整区间导数上界、时间拉伸和保守扫掠碰撞 |
| 侧向目标 (2.5, 1.2, 1.2) m | PASS：3 条运动轨迹，速度/加速度/障碍碰撞违规 0/0/0；最小障碍距离 0.304 m，终点误差 0.058 m |
| 占据目标 (3.0, 1.2, 1.2) m | PASS（安全拒绝）：120 s 轨迹/位置指令 0 |
| 地图就绪门控关闭 | PASS：120 s 规划尝试/运动指令 0 |
| 连续拒绝及执行端失效 | PASS：120 s 失败重试最短间隔约 0.5 s、位置指令 0；60 s 执行端失效测试停发并可恢复 |
| 运动中重置 | PASS：闭锁、停旧链、新坐标系重建地图及重新规划；旧/新占据体素重叠 0 |
| OpenVINS 初始化/漂移 | NOT RUN：无同步真实双目与飞控 IMU 数据集 |
| MAVLink IMU 上行（代码/脱机） | PASS（脱机）：时钟映射收敛与符号、TIMESYNC 配对（PX4 主动请求插入不丢样、超时/回显/来源校验）、无同步拒发、`v4l2_buffer` 布局与编译期 `offsetof()` 逐字段核对、模拟 ioctl 取帧、真实记录帧的拼接切分、相机与 IMU 共用 ROS 时间映射、契约一致性；`colcon build --packages-select boom_birds_nav` 通过 |
| 真实双目采集→ROS 发布链（代码/回放） | PASS（脱机）：`boom_birds_nav` 全量 504 项通过（本轮复跑 1 次，71.05 s）；其中回放链用**真实 `depth_node` 进程**订阅左右图与 CameraInfo 并实际收到；V4L2 缓冲布局、拼接帧切分、单入口语义、时域不可用即拒发均有断言 |
| 物理基线缩放（P1 修正） | PASS：`P[0][3] = -fx_当前分辨率 · B_物理`；基线不随分辨率变化，`test/test_camera_info_scaling.py` 锁定；反向验证过（注入旧 bug 该测试必失败） |
| 左右 CameraInfo 配对发布 | PASS：各自话题 `/boom_birds/stereo/{left,right}_raw/camera_info`；旧合并话题默认关闭、保留为可选过渡 |
| OpenVINS 订阅发布链话题 | PASS（仅订阅关系）：真实 `run_subscribe_msckf` 进程 remap 后连接到左右图与 `/boom_birds/imu`；**不证明 VIO 初始化**（回放无曝光时间戳、无同步真实 IMU） |
| 坐标系对齐闸门（P1 修正） | PASS（脱机）：默认 `frame_alignment=none` ⇒ 位置与速度一个都不发、原因码 `local_frame_not_aligned`；PX4 启动周期变化使旧对齐证据失效并闭锁；核实逻辑含残差/样本数/倾角/角速率/绕圈/集中度测试 |
| 完整 setpoint 换算（非零 yaw_offset） | PASS（脱机）：位置/速度/加速度/偏航/偏航角速率一致性校验；"速度旋转后的朝向 == 偏航换算结果"自洽性；正反变换互逆。旧实现只转位置，相关用例在旧代码上失败 |
| 航向核实 ≠ 完整对齐 | PASS（脱机）：只声明航向已核实、原点无证据时**仍然拦住**（reason=`yaw_verified_origin_unknown`）；两项证据齐备才放行 |
| 姿态缺失/过期不得充当合格样本 | PASS（脱机）：真实 MAVLink 后端从 ATTITUDE 解析并返回 yaw/roll/pitch/yaw_rate 与本机到达时刻；缺字段、姿态过期、缺到达时刻或 VIO/PX4 到达时差超限均拒绝采样。到达时差不证明测量时间同步 |
| 停发与状态一致 | PASS（脱机）：闸门拦下时 `state=STOPPED`（不再沿用监控器 OK），`stopped_by` 指明停发方；空或未知 `frame_id`、无轨迹等路径同样拒绝 |
| 测试模式边界 | PASS（脱机）：`unverified_test_only` 只允许 Fake 后端或 MAVLink `dry_run=true`；状态明确标记未核实，不再误报 `yaw_and_origin` |
| 原点/航向的真实对齐 | NOT RUN：两系之间完整刚体变换未在真机（或融合后 EKF 状态）上核实；不伴随 PX4 重启的 EKF 原点重置目前无可靠上行事件可自动识别；**未核实前不得用位置 setpoint 控制实机** |
| 真实双目采集→ROS 发布链（真机） | NOT RUN：未开真实相机；V4L2 与曝光之间的偏差未核验；1280×960 标定配 640×480 采集只做内参缩放并告警，真机须重新标定 |
| 规划输出→Px4Interface→PX4（代码/脱机） | PASS（脱机）：NED/偏航/type_mask 换算、各失效路径停发、迟滞恢复、坐标系不匹配拒绝、进程级回环 MAVLink 实收 `msg 84`、默认 dry_run 零发送均有断言 |
| PX4 SITL：链路/状态/msg 84/type_mask | PASS（SIH，仅回环，`-i 0`）：真实 HEARTBEAT 解析、`connect()`/`read_vehicle_state()`、ulog `offboard_control_mode` 证实 `msg 84` 被接收且 type_mask 位对应、缺 type_mask 被拒、`arm()` 被拒（`allow_arming=false`）、心跳超时翻假、无误判重启 |
| PX4 SITL：短距离位置响应 | PASS（SIH，TEST-ONLY）：已 arm/切 OFFBOARD；从约 `x=0.04 m` 移动至 `x=0.97 m`，目标 `x=1.0 m`，落地后 Disarmed。合成双目 + PX4 EKF 真值；尚未证明绕障通用性或实机控制 |
| PX4 SIH：mockamap 5 m 绕障 | PASS（固定 seed 单次，TEST-ONLY）：先进入 OFFBOARD 再启动 EGO；实际巡航轨迹对原始点云最小距离 0.719 m，设定点 0.671 m，终点误差约 0.063 m；旧启动顺序 0.064 m 判 FAIL |
| PX4 SIH：random_forest 绕障 | PASS（固定场景单次，TEST-ONLY）：分段启动，目标 `(4,-3,2.5) m`；巡航轨迹对原始点云最小距离 0.536 m，直线基准 0.180 m，目标最近误差 0.014 m，降落 Disarmed；此前未分段测试发生 Offboard 丢失，仍记录为 FAIL |
| PX4 SIH：random_forest 30 m 绕障 | PASS（seed=3 单次，TEST-ONLY）：起点 `(-15,0,0.1)`、终点 `(15,0,1.0)` m；巡航距原始点云最近 0.508 m，直线基准 0.041 m，目标误差 0.004 m，failsafe 0 次，降落 Disarmed |
| PX4 SITL：offboard 失联后的 failsafe | PARTIAL：长距离失败试飞中实际出现 `offboard_control_signal_lost` 并进入 Land；尚未主动注入断流、核对恢复/重复接管及完整飞控动作 |
| PX4 custom_mode 位域解码 | PASS（SITL 实证）：SITL 实收 `0x03040000`=AUTO/LOITER(3) 曾把工程 `>>8`/`>>16` 的错误暴露出来（解成 main=0/sub=4、OFFBOARD 恒判假）；已按 PX4 `px4_custom_mode.h` 改为 main=bit16-23/sub=bit24-31 并补真实值回归单测 |
| ARM64 构建 / Pi 5 性能 | NOT RUN（原因已核实）：本包零编译扩展（`*.so`=0、`setup.py` 无 `ext_modules`），故无「本包自身的交叉构建」；本机缺 `aarch64-linux-gnu-gcc`/`qemu-aarch64` 且禁止联网安装。已知架构相关点仅 `camera_timestamp` 的 64 位 `v4l2_buffer` 布局，仓库自带 gcc+`offsetof()` 探针可在 aarch64 上判定。ARM64 通过也不等于 Pi 5 实时性/驱动验收通过 |
| MAVLink IMU 真机（频率/带宽/同步误差/OpenVINS 初始化） | NOT RUN：未连接飞控与相机 |
| 相机曝光时间戳 / 图像-IMU 真机同一时间域发布 | NOT RUN：V4L2 采集时间戳判定与左右图 ROS 发布路径已脱机验证；真实曝光时刻、V4L2 时间戳与曝光的偏差以及与飞控 IMU 的真机同步均未测。记录帧的 `capture.json` 明确「host save time, not exposure time」 |
| 真机距离精度、端到端时延、ARM64/Pi 5、实机 PX4 闭环与飞行 | NOT RUN |

以上 PASS 覆盖 WSL 合成/文件输入的软件行为，以及固定场景的 SIH 短距离与 30 m 绕障仿真；完整 VINS–深度–规划链、跨场景绕障成功率和真机安全性**尚未通过总体验收**。测试 JSON、日志和过程记录均保留本机并由 Git 忽略，不随 GitHub clone 分发。

## 2026-09-28 架构修整（第一批）

- SIH 内置规划入口从标定与实际输出尺寸生成 EGO 内参；分段目标脚本读取新鲜的深度 CameraInfo，超时或无效则拒绝启动。主深度仍为米制 `32FC1` / NaN。运行中改变标定或尺寸仍需重启规划链。
- OFFBOARD 前检查与 relay 接管检查共用 `handoff.py`，NED 转换复用 `LocalFrameAlignment`；两个阶段分别保留检查。起飞、relay 锁点、图像发布高度不合并。
- 补充 ROS 依赖与 MAVLink Python 版本清单；SIH 脚本路径可配置。工作空间仍兼容既有用户目录 Python 依赖，干净环境复现未执行。
- 新增统一脱机入口，加载独立 OpenVINS 安装环境，汇总命令、Git 状态、日志和跳过项。初次运行发现 OpenVINS 动态库路径缺失；新增测试曾污染默认 ROS 上下文，已隔离，并为 IMU 节点测试增加非回环拒绝检查。

本次执行：

```bash
bash companion/ros2_ws/tools/build_all.sh --packages-select boom_birds_nav
bash companion/ros2_ws/tools/check_offline.sh --out companion/ros2_ws/log/architecture_checks/verified/report.json
```

| 检查 | 本次结果 |
| --- | --- |
| 导航包构建 | PASS |
| 导航 pytest | PASS：516 项，跳过 0 项；59.32 s |
| 双目标定 pytest | PASS：10 项，跳过 0 项 |
| C++ trajectory_validation | PASS：1 项 |
| 安装后的 SIH launch 参数解析、shell/Python/XML 语法、Git diff 检查 | PASS |
| SIH 运动与完整分段接管流程 | NOT RUN：本次只做脱机验证 |
| 硬件、真实 VIO 与干净机器环境复现 | NOT RUN |

本机汇总与日志：[report.json](../companion/ros2_ws/log/architecture_checks/verified/report.json)（Git 忽略）。
统一生命周期状态机、取消/恢复协议、稳定悬停锁点、轨迹 ID 分配和包拆分尚未实现；
本批不把现有分段脚本视为完整编排。当前流程与参数来源见 [导航包说明](../companion/ros2_ws/src/boom_birds_nav/README.md#sih-启动边界与参数来源)。

## 2026-09-28 下一步（历史）

以下计划已被页首“下一步”替代；拆包、恢复与 Pi 验证状态按后续日期化记录读取。

1. 联机验收本次新增链路：核对串口设备/波特率/heartbeat，记录实际 `imu_rate_hz`、`interval_max_s`、`gaps` 与 TIMESYNC `rtt_median_s`/`error_bound_s`（115200 是否够用由实测决定）；用 `camera_timestamp_probe` 核验相机帧时间戳时域，再标定 `camera_imu_offset_s`（符号 = `t_cam_ros − t_imu_ros`）；最后用同步的真实双目 + 飞控 IMU 验收 OpenVINS 初始化、输出频率、重置和漂移。当前无实机飞控连接，不把 TEST-ONLY 合成 IMU 或 MAVLink 回放结果充作真实数据。
2. 核验相机独立距离精度、端到端延迟和长期性能；位姿插值上限、队列容量及门控次数按实测重新定值。
3. 扩展 mockamap/random_forest 障碍布局、起终点与随机 seed，复测成功率和机体外廓碰撞余量；同时验证轨迹接管异常、地图断流和 OFFBOARD 失联的动作。后续单独处理正后方目标曲线优化。当前固定 seed 的 5 m 与 30 m 通过不证明跨场景绕障能力。
4. Pi 5/ARM64 构建、PX4 控制接口和安全接管需另立硬件证据门槛；协调重启仅用于地面流程，不能用于飞行中连续控制。
5. **用位置 setpoint 控制实机之前必须先完成坐标系对齐核实**（当前**未通过**）：需要
   ①实测 VIO 与 PX4 的航向残差；②原点/平移的证据（PX4 侧融合外部视觉使 EKF 原点等于 VIO 原点，
   或实测标定 `frame_alignment_translation_m`），并把证据记入验收。当前默认 `frame_alignment=none`，
   两项证据都不采信，因此只会拦住位置指令。PX4 重启后必须重新核实两项证据并重启接口节点；
   联机时还需确认如何识别不伴随重启的 EKF 原点重置。
   **脱机换算与订阅通过不等于实机可控制。**

### 2026-09-29 交回计划（历史）

1. **C 未跑完的 SIH 矩阵**：先查清 `normal-mockamap-res02` 里 `setpoint_link` 的成因
   （EGO `traj_server` 何时停止发布、轨迹是否提前走完），再依次跑固定 seed 1–5 与 30 m 森林；
   然后按"规划取消 / 深度断流 / 里程计断流 / setpoint 中断 / 模式确认失败"分别验证进入 Land
   与进入 Return 后的合格恢复；最后补齐反向用例（人工取消、落地、重启、低高度、未知故障、
   次数耗尽、模式命令被拒）。每次都要核对最终 Disarmed 并保留失败证据。
2. **B 的收口**：把"恢复成功并重新规划剩余目标回到 EXECUTING"在 SIH 中走通；
   当前只证明到"进入 RECOVERING + 阻塞项如实上报 + 预算耗尽闭锁 + 安全降落"。
3. **D 按职责拆包**（本轮 NOT RUN，准入条件是 C 的行为稳定）：`boom_birds_sensing`（采集/时间同步/
   深度发布/位姿适配）→ `boom_birds_control`（PX4 后端/坐标转换/执行许可）→
   `boom_birds_bringup`（生命周期编排/配置/集成启动）→ `boom_birds_sim`（合成输入/SIH 真值/场景与
   故障注入）。`boom_birds_nav` 只作兼容转发入口，不复制第二份实现；生产包不得依赖 sim 包；
   仿真连接与模式限制仍在运行时实施。小批次迁移，每批跑受影响测试并更新 `build_all.sh`
   的包选择与统一验收入口。

飞控/动力硬件验收入口为 [PX4 清单](../px4/README.md#fc-001-实板验收清单)。OpenVINS 和 EGO 为独立 Git 子模块；需要发布新的子模块提交时，应先推送子模块，再推送引用这些提交的母仓库。
