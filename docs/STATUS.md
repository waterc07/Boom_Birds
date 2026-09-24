# 当前状态与下一步

更新：2026-09-24。当前包括 MAVLink IMU 上行、双目采集发布链及 Px4Interface 高层控制接口的脱机验证，以及 PX4 SIH 的短距离运动仿真；唯一正式开发根为 WSL Ubuntu-24.04 的 `/home/waterc/workspace/Boom_Birds`。下文分别标注 WSL 脱机、PX4 SITL 和未执行的真机验证。详细架构见 [项目入口](../README.md)，运行方式见 [ROS 工作空间](../companion/ros2_ws/README.md)。

## 已完成的软件与仿真部分

- ROS 2 Jazzy/x86_64 已构建 `stereo_depth`、`boom_birds_nav`、EGO 依赖链和 `ego_planner`，以及 OpenVINS 的 `ov_core`、`ov_init`、`ov_msckf`。OpenVINS 已启动并订阅合成双目与 IMU；尚无有效 VIO 初始化/里程计输出证据。
- 工作空间隔离 Python 环境可导入 NumPy 1.26.4、OpenCV 4.6.0、rclpy、cv_bridge；深度 CLI 和既有 10 项标定单测通过。构建脚本对 colcon/CMake 限并发，并默认使用仓库外持久目录 `/home/waterc/bb_build/main/{build,install,log}`。
- `boom_birds_nav` 提供合成/文件双目源、`32FC1` 米制深度（无效 NaN）、`16UC1` 毫米兼容深度（无效整数 0）、完整 H×W XYZ 点云、CameraInfo、按采集时间缓存/插值的位姿适配。TEST-ONLY 合成 IMU/VIO 不能作为飞控或精度证据。话题、坐标系和时间参数以 [契约](../companion/ros2_ws/src/boom_birds_nav/config/contract.yaml) 为准。
- Companion 端 **真实双目采集 → ROS 发布链代码完成**（回放链脱机通过，真机未测）：`stereo_source`
  以 `mode` 区分 `v4l2`/`replay`/`file`/`synth`，是**唯一**采集与发布入口；只有
  `stereo_capture.V4L2FrameSource` 打开设备，左右原始图 + **各自与图像配对**的 CameraInfo
  供深度与 OpenVINS 共用，同帧左右共享同一 V4L2 采集时间戳；时域不可核实即**拒绝发布并诊断**，
  不回退 OpenCV/发布时间。跨分辨率缩放只缩放 `fx/fy/cx/cy`，**米制物理基线不缩放**
  （曾错误地把基线也乘 scale，使右目 `P[0][3]` 在 scale=0.5 时衰减到 1/4，已修正并有单测锁定）。
  回放模式发布语义与真实采集一致，但记录帧没有曝光时间戳，**不能**用于证明相机-IMU 同步。
  本节点不做矫正/深度（由 `stereo_depth`/`depth_node` 负责）。配置见
  [stereo_camera.yaml](../companion/ros2_ws/src/boom_birds_nav/config/stereo_camera.yaml)。
- Companion 端 **规划输出 → Px4Interface → PX4 高层控制接口代码完成**（脱机通过；SITL 部分通过，真机未测）：
  `px4_interface_node` 订阅 `PositionCommand`，经 `px4_frames` 换算到 NED 并配 `type_mask`，由
  `Px4Backend` 协议下发；`FakePx4Backend` 用于确定性脱机测试，`MavlinkPx4Backend` 为真实实现。
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
  控制器跑在 Companion 还是 PX4 **仍未定**，SITL 原型不构成架构定论。配置见
  [px4_interface.yaml](../companion/ros2_ws/src/boom_birds_nav/config/px4_interface.yaml)。
- Companion 端 **PX4 MAVLink IMU 上行 + 时间同步第一版代码完成**（脱机验证通过，真机未测）：`mavlink_imu_node` 接收 `HIGHRES_IMU` 并发布契约话题 `/boom_birds/imu`；`mavlink_clock` 用 `TIMESYNC` 往返估计「PX4 启动时钟 − Companion 单调时钟」偏移；`timebase` 把单调时钟映射到 ROS 时间域。`stereo_source` 的 V4L2/回放模式已接入 `camera_timestamp` 的采集时间戳判定与拼接帧切分，并发布左右图；同帧左右共享时间戳，时域不可核实时拒发。编译期 `offsetof()` 测试核对 `v4l2_buffer` 布局。**真实曝光时刻及相机与 IMU 的真机同步尚未验证。**串口/波特率/sysid-compid/流频率/话题全部配置化；无可靠映射、字段或时间校验失败时拒绝发布并给出诊断。运行与参数见 [boom_birds_nav README](../companion/ros2_ws/src/boom_birds_nav/README.md)，配置见 [mavlink_imu.yaml](../companion/ros2_ws/src/boom_birds_nav/config/mavlink_imu.yaml)。
- EGO 地图跳过无效深度观测，完成融合/膨胀且达到占据阈值后才放行规划。规划器对整段轨迹做动态约束与保守碰撞校验；拒绝时向 `traj_server` 发送失效消息并停发 PositionCommand。停发命令不等于 PX4 悬停或安全接管。
- **PX4 SIH 前台运动仿真（TEST-ONLY）**：以 PX4 自身局部位置/姿态回读作为仿真真值，驱动合成双目随相机平移和旋转，复用 `depth_node`、EGO 与 `px4_interface_node`。2026-09-24 在仅回环 `-i 0` 的 SIH 中，起飞并切 OFFBOARD 后，实际回读位置从约 `x=0.04 m` 到 `x=0.97 m`，目标 `x=1.0 m`，最后 `commander land` 自动 Disarmed；RViz2 前台窗口显示深度及占据点云。运行说明见 [导航包](../companion/ros2_ws/src/boom_birds_nav/README.md#前台运动仿真test-only)。该链使用仿真真值与测试 IMU，绕过 OpenVINS，不构成真机同步、VIO 或一般绕障证据。
- 默认标定 `stereo_depth/calibration/live_20260916_210120_642136/candidate.npz` 随工程保存；2026-09-16/17 的 Pi 5 旧记录仅证明当时的几何校验，真实米制距离精度仍须独立尺测。

## 当前验收范围和结果

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
| 真实双目采集→ROS 发布链（代码/回放） | PASS（脱机）：`boom_birds_nav` 全量 502 项通过（本轮复跑 1 次，60.43 s）；其中回放链用**真实 `depth_node` 进程**订阅左右图与 CameraInfo 并实际收到；V4L2 缓冲布局、拼接帧切分、单入口语义、时域不可用即拒发均有断言 |
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
| PX4 SITL：offboard 失联后的 failsafe | NOT RUN：未主动制造 Offboard 指令断流并核对 PX4 对应动作；旧的 `gcs_connection_lost` 不能替代该测试 |
| PX4 custom_mode 位域解码 | PASS（SITL 实证）：SITL 实收 `0x03040000`=AUTO/LOITER(3) 曾把工程 `>>8`/`>>16` 的错误暴露出来（解成 main=0/sub=4、OFFBOARD 恒判假）；已按 PX4 `px4_custom_mode.h` 改为 main=bit16-23/sub=bit24-31 并补真实值回归单测 |
| ARM64 构建 / Pi 5 性能 | NOT RUN（原因已核实）：本包零编译扩展（`*.so`=0、`setup.py` 无 `ext_modules`），故无「本包自身的交叉构建」；本机缺 `aarch64-linux-gnu-gcc`/`qemu-aarch64` 且禁止联网安装。已知架构相关点仅 `camera_timestamp` 的 64 位 `v4l2_buffer` 布局，仓库自带 gcc+`offsetof()` 探针可在 aarch64 上判定。ARM64 通过也不等于 Pi 5 实时性/驱动验收通过 |
| MAVLink IMU 真机（频率/带宽/同步误差/OpenVINS 初始化） | NOT RUN：未连接飞控与相机 |
| 相机曝光时间戳 / 图像-IMU 真机同一时间域发布 | NOT RUN：V4L2 采集时间戳判定与左右图 ROS 发布路径已脱机验证；真实曝光时刻、V4L2 时间戳与曝光的偏差以及与飞控 IMU 的真机同步均未测。记录帧的 `capture.json` 明确「host save time, not exposure time」 |
| 真机距离精度、端到端时延、ARM64/Pi 5、PX4 闭环与飞行 | NOT RUN |

以上 PASS 覆盖 WSL 合成/文件输入的软件行为，以及单次 SIH 短距离运动仿真；完整 VINS–深度–规划链、一般绕障和真机安全性**尚未通过总体验收**。测试 JSON、日志和过程记录均保留本机并由 Git 忽略，不随 GitHub clone 分发。

## 下一步

1. 联机验收本次新增链路：核对串口设备/波特率/heartbeat，记录实际 `imu_rate_hz`、`interval_max_s`、`gaps` 与 TIMESYNC `rtt_median_s`/`error_bound_s`（115200 是否够用由实测决定）；用 `camera_timestamp_probe` 核验相机帧时间戳时域，再标定 `camera_imu_offset_s`（符号 = `t_cam_ros − t_imu_ros`）；最后用同步的真实双目 + 飞控 IMU 验收 OpenVINS 初始化、输出频率、重置和漂移。当前无实机飞控连接，不把 TEST-ONLY 合成 IMU 或 MAVLink 回放结果充作真实数据。
2. 核验相机独立距离精度、端到端延迟和长期性能；位姿插值上限、队列容量及门控次数按实测重新定值。
3. 后续单独处理正后方目标曲线优化，并在更广场景验证可达性。当前安全拒绝不证明一般绕障能力。
4. Pi 5/ARM64 构建、PX4 控制接口和安全接管需另立硬件证据门槛；协调重启仅用于地面流程，不能用于飞行中连续控制。
5. **用位置 setpoint 控制实机之前必须先完成坐标系对齐核实**（当前**未通过**）：需要
   ①实测 VIO 与 PX4 的航向残差；②原点/平移的证据（PX4 侧融合外部视觉使 EKF 原点等于 VIO 原点，
   或实测标定 `frame_alignment_translation_m`），并把证据记入验收。当前默认 `frame_alignment=none`，
   两项证据都不采信，因此只会拦住位置指令。PX4 重启后必须重新核实两项证据并重启接口节点；
   联机时还需确认如何识别不伴随重启的 EKF 原点重置。
   **脱机换算与订阅通过不等于实机可控制。**

飞控/动力硬件验收入口为 [PX4 清单](../px4/README.md#fc-001-实板验收清单)。OpenVINS 和 EGO 为独立 Git 子模块；需要发布新的子模块提交时，应先推送子模块，再推送引用这些提交的母仓库。
