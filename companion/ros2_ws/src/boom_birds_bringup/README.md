# Companion 姿态控制实机验证入口

模式名 `companion_attitude`。OpenVINS 提供机体位姿和世界系速度，EGO 提供轨迹；Companion 执行位置/速度 PD 与加速度前馈，经 `/mavros/setpoint_raw/attitude` 发送 ENU/FLU 四元数和归一化推力。PX4 执行姿态与角速度闭环。

`px4_position` 保留原有位置 setpoint 路线，两个出口互斥。此实现参考 Fast-Drone-250 的控制分工，没有移植其 ROS 1 px4ctrl、增益、推力参数或自动解锁逻辑。

运行部署只安装本包的正式 launch 与配置；SIH/SITL/Gazebo 启动和参数生成器归 [仿真包](../boom_birds_sim/README.md)，构建边界见[工作空间](../../README.md#运行与开发隔离)。

## 定位、坐标与重置

- 控制位置和规划地图共用 VIO `global` 原点，速度为世界系；机体 `body` 使用 FLU。不向 PX4 回传外部视觉，也不要求 PX4 位置原点等于 VIO 原点。
- OpenVINS 接收项目 FRD IMU；`T_I_B` 必须把 FLU 机体系变到该 IMU 系，不能把单位矩阵当作默认实机外参。
- 飞控姿态按 PX4 NED/FRD 转到 MAVROS ENU/FLU。MAVROS 同步偏移将 ATTITUDE 的 `time_boot_ms` 映射到 ROS 时间；姿态历史与 VIO 采样时间配对，容差取 RuntimeConfig。未同步或没有对应样本时不输出。
- 解锁前连续静态样本建立并冻结水平旋转。起飞后不追随偏航跳变；VIO 时钟回退、位置跳变、姿态参考跳变、飞控重启或世界/机体帧错误均撤销参考并闭锁。必须落地后重新检查并重启节点，旧会话与轨迹不得续用。
- 正常起降由 VIO 闭环完成。接地附近且速度连续满足稳定窗口后，逐步降低推力；实际 disarm 仍要求 PX4 落地回读。失去有效 VIO 后停止外部输出并请求 PX4 原生 Land；这条后备链依赖 PX4 自己的估计器、遥控与 failsafe 配置，必须单独验证。停止发送不等于飞机已降落。

## 接入前准备

真实参数应存入独立目录，例如 `/home/waterc/boombirds-hardware/<airframe>/`，保留标定原文件与测量记录。

| 文件 / 信息 | 要求 |
| --- | --- |
| 双目 NPZ | `K1/K2/D1/D2/R/T/image_size`；米制基线，与采集尺寸一致 |
| OpenVINS estimator_config.yaml 及两条 Kalibr chain | 双目 estimator 设置 max_cameras=2、use_stereo=true；相机 chain 含 cam0/cam1，分别订阅 /boom_birds/stereo/left_raw 与 /boom_birds/stereo/right_raw，IMU 为 /boom_birds/imu。两目内参、畸变、分辨率和方向须与 NPZ 一致，等比例缩图同步缩放内参；T_imu_cam 为相机到 IMU，Kalibr T_cam_imu 须取逆；两目使用同一有限时间偏移，右目时间假设仍须独立验证 |
| extrinsics.yaml | 明确的 `T_I_C0`、`T_I_B` 和实测 `source`；与 OpenVINS cam0 外参相同 |
| attitude.yaml | 从 `boom_birds_control/config/attitude_hardware.yaml` 复制；实测悬停推力、推力上下限、增益与倾角限制，不使用 SIH 参数作为实机证据 |
| 飞控记录 | FCU system/component ID 均为 1（当前后端目标）；PX4 固件版本、机架、IMU安装方向、输出协议、端口/波特率、姿态/角速度调参、EKF来源及 failsafe 参数导出 |

首轮只接受 pinhole/radtan、固定标定；四项畸变须与 NPZ 一致。五项模型的第五项非零时拒绝。采集时间与飞控 IMU 时间须完成实际测量，包括相机—IMU偏移，不能把接收时间当曝光时间。IMU 默认请求 50 Hz，是否足以支持实际 OpenVINS 和机载算力，须按实测数据判断。

`hardware_verified: false` 和空 `evidence` 只允许 dry-run。实测后填写 `hardware_verified: true` 与记录路径；这只解除软件配置闸门，不替代脱桨和系留验收。程序不自动写 PX4 EKF、解锁检查或 failsafe 参数。

### 配置准备与选择

端口、PX4 参数、标定、机体安装外参、悬停推力与控制参数必须有各自证据；缺项保持待填，不用单位矩阵或 SIH 值补齐。用户指定原点重合时仅令杆臂为零；旋转按输入/输出坐标轴确定。当前 /boom_birds/imu 最终为 FRD，控制 body 为 FLU，因此中心假设候选使用 R_I_B=diag(1,-1,-1)，不能填单位旋转。保持 hardware_verified=false，仅供 dry-run 配置审查，不能据此开启 live 控制。当前候选与验证范围见 [STATUS](../../../../docs/STATUS.md#实现默认入口部署与验收)。

已选 VIO 开发配置的加载方法见[感知包](../boom_birds_sensing/README.md#已选开发配置)，当前路径和质量限制见 [STATUS](../../../../docs/STATUS.md#当前配置与标定)。配置部署完成后仍须核对实际节点参数；它不能补齐 T_I_B，也不能解除控制/飞行闸门。

相机/深度性能测试只启动对应感知节点，不用完整实机 launch 代替分项测试。

## 双目 VIO 与启动配置

双目 OpenVINS＋飞控 IMU 与稠密深度分别计算。实现、默认入口、Pi 部署和验收只维护在 [STATUS 状态表](../../../../docs/STATUS.md#实现默认入口部署与验收)。下表说明配置行为，不能只替换 VIO 文件：

| 用途 | `compute_profile` / `shared_decode` | VIO 配置与限制 |
| --- | --- | --- |
| 当前双目主线在现有 launch 中的配置入口 | `calibrated` / false | 显式提供 `max_cameras=2`、`use_stereo=true` 的 estimator 与双目 chain；不应用单目算力覆盖；正式共载未验收 |
| 旧单目算力对照 | `mono_budget` / auto | 仅接受 `max_cameras=1`、`use_stereo=false`；应用下节的跟踪、地图与规划覆盖 |
| 近期 Pi 双目隔离测试 | 独立脚本，见日期化记录 | 共享采集/深度与逐步追加控制/EGO；不能把该测试结果当作 `calibrated` launch 的结果 |

`calibrated` 不会自动套用 `mono_budget.yaml` 的共享解码、64 视差、17 Hz 深度上限、地图抽样或单候选参数；`shared_decode=true` 与 `calibrated` 组合被拒绝。性能须按实际加载参数重新测量。

环境选择是否已切换见 [STATUS 状态表](../../../../docs/STATUS.md#实现默认入口部署与验收)。测试双目时显式设置 `BB_VIO_CONFIG`，核对 estimator 引用的两条 chain；目录名和环境加载成功不代表已启用双目。10-21 的隔离 `stereo-config` 是旧方向，不能直接与 10-22 转正输入混用。图像、双目 NPZ、两目相机—IMU外参、时间偏移及 `pose_adapter` 的 `T_I_C0` 必须属于同一方向版本。当前外部配置与 Pi 正式入口的区别见 STATUS。

构建与 dry-run 章节的示例使用显式选择的双目配置；未提供有效 T_I_B 时配置核验拒绝。独立候选中的用户假设只能作为明确标注的 dry-run 输入，不是实测确认。

## 旧单目算力对照

硬件保持当前 Pi 5、USB 双目及下视相机。`compute_profile=mono_budget` 是保留的单目对照配置；其旧测试不覆盖当前双目全链预算。默认 dry-run、禁止解锁、标定一致性和故障闭锁继续生效。

10-15 单目分项测试候选为共享解码＋64 视差，见 [分项实测](../../../../docs/history/STATUS_2026-10.md#2026-10-15-单目优化-pi-分项测试无飞控)。相机短测通过，真实 VIO/地图/规划共载未验收。VIO 配置选择与代码部署分别核对，当前选择见 [STATUS](../../../../docs/STATUS.md#当前配置与标定)。

| 环节 | mono_budget |
| --- | --- |
| 采集 | 保持 2560×960 MJPEG / 60 FPS；发布上限 20 Hz |
| VIO | 左目 640×480 + IMU；100 跟踪点、6 克隆状态、0 长期地标、每次最多 20 个 MSCKF 特征、OpenCV 1 线程 |
| 原图 | 直接减量解码；左右目分别按订阅需求发布；不发布拼接原图 |
| 深度 | 与采集共进程共享解码数组；320×240 双向 SGBM，2 线程，最新帧、17 Hz 上限；点云无人订阅时只计算 Z |
| 地图 | 0.2 m 体素，像素步长 2；地图边界、未知区域处理和净空约束不变 |
| 规划 | 单候选轨迹；原计算超时、动力学与碰撞检查保留 |

20 Hz 在图像源限制；OpenVINS `track_frequency=30` 留采样抖动余量，避免第二次限频。表中是配置，不是 Pi 实测输出频率。地图像素抽样可能漏掉小障碍，关闭长期地标和多候选优化也可能降低定位/绕行质量。

从已有完整 OpenVINS 配置生成独立单目目录，不填写虚构外参或覆盖原文件：

```bash
python3 companion/ros2_ws/tools/prepare_mono_vio.py \
  --source "$BB_HW/estimator_config.yaml" --out "$BB_HW/mono-budget" \
  --width 640 --height 480
```

生成器保留 cam0 外参、IMU chain 和时间偏移，只缩放 cam0 内参并删除 cam1；输入文件 SHA256 写入 `source_manifest.json`。合成输入标记随输出保留，实机入口拒绝使用。采集节点按新 chain 自动选择原图缩放和 JPEG 解码倍率；OpenVINS 禁止再做一次缩图。

单目对照命令使用 `compute_profile:=mono_budget vio_config_file:="$BB_HW/mono-budget/estimator_config.yaml"`。`compute_profile:=calibrated` 可按提供的单目或双目配置运行，不应用算力覆盖；默认不再接受双目配置作为 mono_budget 输入。

`shared_decode=auto` 在 `mono_budget` 中启动 `shared_stereo_depth`，采集与深度保留两个 ROS 节点名，共用一个进程。深度工作线程只保留一组待处理数组引用，左右目 Image/CameraInfo 接口和时间戳不变；不再为深度发送 MJPEG。OpenCV 线程数为进程全局值，采集/深度共用 2 线程，OpenVINS 仍为独立进程的 1 线程。

`shared_decode:=false` 恢复原 MJPEG 分进程输入作对照，仍应用按需 Z 和视差范围配置。`calibrated` 的 auto 保持分进程；完整旧匹配范围可将深度配置设为 `num_disparities: 96`。

`mono_budget.yaml` 固定 `num_disparities: 64`，保留 `required_min_depth_m: 0.2` 的启动检查。历史测试标定下理论最近深度约 0.18552 m；此数值不适用于所有标定。启动检查按本次 NPZ 的 Q、输出尺寸与视差数计算，换标定后须重新核对；范围不足时拒绝启动。需要更近观测时，可在独立参数文件中改为 96 并重新验证。设置为 0 则按同一 Q 自动选择满足量程的 16 倍数，自动档不超过原 96。0.2 m 是本配置的搜索范围要求，不是实测量程或避障安全距离；任务需要更近观测时须调低该值并重测。独立深度节点默认仍为 96/不启用量程检查。

点云订阅在计算开始时采样，无需求时不生成 XYZ；订阅到来后从下一次计算恢复。主深度仍为 `32FC1` 米制/NaN，左右一致性检查保留。

感知参数在 `config/mono_budget.yaml`，VIO 覆盖在 `boom_birds_bringup/compute_profile.py`。脱机验证可运行：

```bash
python3 companion/ros2_ws/tools/check_mono_vio.py --out /tmp/bb-mono-check-new
```

该命令核对真实 OpenVINS 进程仅订阅左目和 IMU，再以合成特征执行滤波；后者由真值初始化、绕过 KLT，不证明真实初始化、前端性能、Pi 预算或飞行质量。

## 构建与静态核验

在 WSL 正式工程根运行；新消息字段要求重建接口及使用者，不能混用旧 install。

```bash
cd /home/waterc/workspace/Boom_Birds
bash companion/ros2_ws/tools/build_runtime.sh
source companion/ros2_ws/tools/activate_python_env.sh
source "$HOME/bb_build/runtime/install/setup.bash"
python3 companion/ros2_ws/tools/check_runtime_install.py --install "$HOME/bb_build/runtime/install"
```

命令核验文件存在、内参/基线/相机—IMU外参一致和控制参数范围，输出文件 SHA256。`--live` 额外要求机体参数已确认。文件一致不证明标定准确。

## 脱桨接入与 dry-run

确认同一 ROS 域内没有 SIH、回放或另一套控制节点。实机入口不启动合成源；缺必需文件直接失败。

```bash
ros2 launch boom_birds_bringup attitude_hardware.launch.py \
  fcu_url:="$BB_FCU" allow_non_loopback:=true \
  calibration_file:="$BB_HW/stereo.npz" extrinsics_file:="$BB_HW/extrinsics.yaml" \
  vio_config_file:="$BB_VIO_CONFIG" attitude_config_file:="$BB_HW/attitude.yaml" \
  compute_profile:=calibrated shared_decode:=false \
  camera_device:=/dev/v4l/by-id/actual-camera-video-index0 \
  capture_width:=2560 capture_height:=960 capture_fps:=60

# 另一终端加载相同环境；只订阅，不发模式/解锁/控制命令。
python3 companion/ros2_ws/tools/monitor_attitude_hardware.py \
  --seconds 15 --out "$BB_HW/evidence/bench-01.json"
```

dry-run 仍请求只读遥测采样率、配置并回读 MAVROS 时间与推力缩放；不发控制目标、模式或解锁命令。

默认 `dry_run=true`、`allow_arming=false`、`allow_hardware_actions=false`。报告 `READY_FOR_BENCH_REVIEW` 仅表示连接、单一输入源、采样年龄和参考就绪。手持机体沿三个轴平移和旋转，核对 VIO 米制尺度、方向、机体系外参、深度与地图位置、姿态参考残差；保存 rosbag 与日志。

可在 dry-run 下调用 START 检查地面 HOLD 的姿态/推力计算；随后的 Offboard 请求会被后端拒绝，不能将这次任务记为飞行通过。闭锁后重启任务链重新检查。

## 输出与飞行验证顺序

1. 脱桨确认 PX4 接收到的是姿态目标，位置/速度控制标志未启用；核验推力缩放为 1、四元数方向、遥控退出 Offboard、断流、飞控/VIO重启与取消。不得用解锁 ACK 代替 armed 回读。
2. 实机机体参数和手动姿态控制验收后，显式指定 `dry_run:=false allow_hardware_actions:=true hardware_validation_note:=<本次验收记录>`；解锁仍默认关闭。确认输出观察结果，再由操作员决定是否增加 `allow_arming:=true`。
3. 系留验证起飞、保持、降落和落地确认后的 disarm；再做短距离目标。预留的安全接管路径必须已经验证，VIO故障时不能依赖相同的 VIO 完成安全降落。

任务默认总预算 45 s，从发送解锁请求计时；剩余 14 s 时撤销规划并开始下降，超时请求 PX4 Land。起飞高度取 RuntimeConfig，默认 1.5 m；首轮需保证场地/系留与此高度相容。此预算不是能源预测；低电压、电流与续航仍由实机验收。

```bash
# 目标使用当前 VIO 世界系，先核对原点和自由空间；示例值不能直接照搬。
ros2 service call /boom_birds/mission boom_birds_interfaces/srv/Mission \
  '{action: 1, goal: {x: 1.5, y: 0.0, z: 1.5}}'
# 正常 VIO 闭环降落
ros2 service call /boom_birds/mission boom_birds_interfaces/srv/Mission '{action: 3}'
# 人工取消：撤销外部控制并请求后备 Land，仍须监看 PX4 实际行为。
ros2 service call /boom_birds/mission boom_birds_interfaces/srv/Mission '{action: 2}'
```

记录 `/boom_birds/control/status`、`/boom_birds/control/execution_status`、`/boom_birds/control/command`、`/boom_birds/mission/status`、VIO、IMU、双目/深度、MAVROS状态和原始回读；同步保存 PX4 ULog、参数、软件版本及各标定 SHA256。分别记录到目标、闭锁原因、最终模式、落地/Disarmed 和总时长。

## 本机 SIH 回归

```bash
INSTALL_BASE=/home/waterc/bb_build/ego-single/install \
BB_SIH_EVID=/home/waterc/bb_build/ego-single/evidence/attitude-new-run \
bash companion/ros2_ws/tools/run_sih_mission.sh --control-mode companion_attitude \
  --scenario attitude-new-run --scene local --goal 1.0 0 1.5 --timeout 80
```

SIH 使用真值里程计和合成双目，不运行真实 OpenVINS。为了在解锁前建立输入许可，姿态模式显式关闭合成图像的起飞高度门控；位置模式维持旧行为。实际检验结果与限制见项目 `docs/STATUS.md`。

SIH 锁步时钟与主机墙钟不能作为实机 TIMESYNC 证据。仅经 `verifySihProcess` 核验的本机 SIH PID 允许真值测试使用 router 接收时间；状态标记 `TEST_ONLY_SIH_router_receipt`；该路径按回调接收时刻构造真值，不验收传输采样年龄。实机入口不传 SIH PID，仍要求 PX4 boot 时间映射。SIH 不验收真实 VIO 或采样时间同步。

## 下视平台末段降落

可选平台交接默认关闭，配置、图像/rosbag 入口、状态与脱机/SIH 命令见 [控制包](../boom_birds_control/README.md)。当前真实入口拒绝执行；原点起飞和往返导航仍保留 VIO。

隔离诊断可在新的 OpenVINS 程序命令中显式加 `--ros-args -p camera_diagnostics:=true`，字段语义见[感知状态接口](../boom_birds_sensing/README.md#openvins-初始化与视觉更新状态)。正式 launch 未透传此参数，默认保持关闭；旧程序/旧动态库不具备该诊断，须核对实际加载来源。

## VIO 原点与地图 Z 范围

`attitude_hardware.launch.py` 向 EGO 传递 `map_min_z`（默认 -0.1 m）、`map_size_z`（6 m）、`virtual_ceil_z`（2.9 m）。`map_min_z` 对应原生 `grid_map/ground_height`，定义世界系中的地图下界，不是实测地面高度。两层入口均在启动节点前检查有限数值及 `map_min_z < virtual_ceil_z <= map_min_z + map_size_z`。

VIO 初始位置作为原点时，返回后的 Z 可能为负。脱机对照可显式传 `map_min_z:=-0.5`；该值只规定测试地图范围，实际任务须按工作区与坐标基准配置。相机、机体里程计、障碍和目标保持同一世界系，不能只平移里程计。地图外仍拒绝通行，不清空障碍或放宽碰撞检查。直接运行 `ego_planner/boom_birds_offline.launch.py` 时使用同名参数。

脱桨计算共载可用独立 EGO 入口 `require_session:=false position_cmd_topic:=/bench/ego/position_cmd`；该输出不接控制。正式入口保留 `require_session:=true` 及 `offboard_confirmed` 条件，独立规划不构成 OFFBOARD 或任务链验收。独立模式不发布会话用的 `PlannerStatus`，其计数为 0 不能用来判断地图是否就绪。

## 位置路线生命周期

本节只描述 px4_position。lifecycle.py 接收 Observation 并返回动作名，不做 ROS/I/O；lifecycle_node.py 执行动作。解锁、起飞和模式切换只经 VehicleAction；最终 setpoint 许可由控制包的 failsafe 判定。companion_attitude 使用 attitude_lifecycle.py，不沿用本节的 SIH 自动恢复。

### 状态迁移

| 迁移 | 条件（源码） |
| --- | --- |
| `IDLE`/`COMPLETE` → `PRECHECK` | 会话非空、goal 三分量有限；由 `Mission.START` 先请求 `VehicleAction.OPEN_SESSION`，服务返回的 `session_id` 交给 `fsm.start()` |
| `PRECHECK` → `TAKEOFF` | 连接 + `status_age ≤ status_timeout_s` + 未解锁 + `landed == 1` + `pose_age ≤ pose_timeout_s` + 对齐有效 + 位置有限；记录 `groundZ` 与 `boot_epoch`，输出 `arm` |
| `PRECHECK` 超时 → 闭锁 | `precheck_timeout_s`（缺省 10 s） |
| `TAKEOFF` → `HOLD_READY` | 解锁确认后发一次 `takeoff`；`landed == 2` 且 `StableWindow` 成立 → 锁点 = 当前观测位置 |
| `TAKEOFF` 闭锁 | 未解锁超过 `mode_timeout_s`；或超过 `takeoff_timeout_s` |
| `HOLD_READY` → `OFFBOARD_PENDING` | `sending ∧ sensors_ready ∧ map_ready` 且在本状态已停留 `stable_duration_s`，输出 `offboard` |
| `HOLD_READY` 闭锁 | 对齐失效 → `frame_reset`；或超过 `hold_ready_timeout_s` → `map_or_stream_not_ready` |
| `OFFBOARD_PENDING` → `EXECUTING` | **模式回读**确认 OFFBOARD（`offboardObserved`），输出 `enable_planner` |
| `OFFBOARD_PENDING` 闭锁 | 模式回读超 `mode_timeout_s` → `offboard_not_confirmed` |
| `EXECUTING` → `LANDING` | 到点判定：`pose_age ≤ pose_timeout_s` ∧ 距 goal ≤ `goal_tolerance_m` ∧ 速度 ≤ `stable_speed_m_s`，且持续 `stable_duration_s` |
| `LANDING` → `COMPLETE` | 连接 + 状态新鲜 + 已上锁 + `landed == 1` |
| 任意阶段 → `FAULT_LATCHED` | 时钟回退、状态超 `mode_timeout_s`、`boot_epoch` 变化（飞控重启）、会话变化、命中 `REVOKING_FAULTS`、飞行中 `landed == 1` 或上锁 |

`RECOVERABLE_FAULTS`（允许自动恢复）与 `REVOKING_FAULTS`（永久撤销本次恢复资格）是两个不同的集合；
`INTERRUPTIBLE_AUTO_MODES` 只识别 `auto:land` / `auto:rtl` 两个可被**有界**中断的
AUTO 模式。自动中断还需新鲜实际/意图模式一致，以及当前 SIH PX4 原生 failsafe 原因匹配；人工模式动作或未知原因闭锁。

### 恢复窗口与重规划语义

- **缺省不可达**：`RuntimeConfig.recovery_enabled` 缺省 `False`，只有 TEST-ONLY 的 SIH 入口把它显式打开。
- **恢复资格**：`recoveryAvailable()` = 开关 ∧ 未被撤销 ∧ 故障属于 `RECOVERABLE_FAULTS` ∧ 预算未耗尽。
- **进入前置条件**（fail-closed：观测缺失/过期即拒绝）：未人工取消、仍解锁、明确在空中（`MAV_LANDED_STATE ∈ {2,3}`，`lifecycle.py`）、
  `status_age ≤ status_timeout_s`、`pose_age ≤ pose_timeout_s`、位置与速度有限。
- **`recovery_attempts`**：进入 `RECOVERING` 即消耗一次，缺省预算 `2`；耗尽后闭锁
  `detail="exhausted"`，**不循环争抢、不自动重新解锁**；恢复后连续 1 s 健康 EXECUTING 才关闭本次故障并重置事件预算，任务累计次数另行记录；窗口内再次失效共用预算。
- **一次尝试的窗口**：`mode_timeout_s`（缺省 3 s）同时约束"等条件就绪"与"等模式回读"两段；超时且预算还有就重试。
- **连续有效窗口**：`recovery_valid_duration_s`（缺省 1 s）要求 `sensors_ready`、`map_ready`、`alignment`、
  两条 age 与有限位置**连续**成立，任一失效即清零并把原因记进 `recoveryWindowReset`。
- **接管放行**：还需 `Observation.sending`（即 `px4_failsafe` 的放行结果）、已记录 `groundZ`、相对高度 ≥ `recovery_min_altitude_agl_m`
  （1.0）、速度 ≤ `recovery_max_speed_m_s`（1.0）。
- **请求 OFFBOARD 只发一次并要求回读确认**：`recoveryRequestAt` 置位后返回 `offboard` 动作，随后靠 `offboardObserved` 确认，
  **命令被接受不算确认**。
- **确认后重新规划**：`_confirmRecovery()` 把 hold 移到故障瞬间位置、转 `EXECUTING`，输出 `replan` + `enable_planner`。`replan`/`recover` 都调 `_retireTrajectory()`：把当前 `trajectory_id` 记入 `ControlIngress.retired`
  并 +1，同 id 命令此后一律被 `retired_trajectory` 拒绝。
- **planner 重新使能**：正常路径是 `EXECUTING` 期内**限频重发**使能请求，间隔 `planner_enable_retry_s`（缺省 0.5 s，
  `runtime_config.py`），上限 `planner_activate_timeout_s`（缺省 30 s）后闭锁 `planner_timeout`。重发是必要的：EGO 的 `projectRequest` 在
  offboard/odom/地图未就绪时**静默丢弃**。
- **规划器单次作废 ≠ 任务故障**：收到 `CANCEL` 型规划命令只退役旧轨迹并转入"规划流静默"，由上面的有界等待处理。
- **规划流停止后的 hold 点**：若距 goal 已在 `handoff_max_distance_m`（0.5 m）以内，hold 点放到 **goal**，让飞控收敛最后一段；
  否则放在**当前位置**，不在没有规划的情况下继续飞（`lifecycle_node.py`，理由见 `lifecycle_node.py`）。
- **永久撤销**：人工取消 / 人工模式干预 / 未知故障 / 落地 / 重启 / 坐标系重置 / 几何变化。

## 独立 AprilTag 飞行任务

前视双目＋飞控 IMU 提供 VIO，下视相机检测指定板。三个入口各运行一个任务，不加载 EGO、地图或原 lifecycle_node。前视 depth_node 输出米制深度，不生成 XYZ/兼容输出。起飞、导航、前飞、绕行和返航保留 companion_attitude 与 VIO；末段在完整解码确认、连续样本对准后按 D-059 完成控制权交接，确认速度控制与回读后才释放 VIO/姿态适配器计算，随后由 Tag 相对速度引导、测距和 PX4 反馈继续到近地，最终由 PX4 Land 接管。交接关闭或硬件速度确认适配器未核验时保留旧 VIO 降落路径；真实入口在当前证据下拒绝开启交接。标定、动力参数、VIO 时效和故障闭锁仍须满足。

| 入口 | 任务文件 | 顺序 |
| --- | --- | --- |
| tag_takeoff_land.launch.py | [tag_takeoff_land.yaml](config/flight_tasks/tag_takeoff_land.yaml) | 起飞、悬停并确认原点 tag、对准降落 |
| tag_return_turn.launch.py | [tag_return_turn.yaml](config/flight_tasks/tag_return_turn.yaml) | 起飞、确认原点 tag、前飞 1 m、停稳、转向 +180°、返回原点、对准降落 |
| tag_forward_land.launch.py | [tag_forward_land.yaml](config/flight_tasks/tag_forward_land.yaml) | 起飞、前飞 1 m、捕获目标 tag、对准降落 |

每个 YAML 保存完整参数和步骤，可单独复制修改。forward_m 是相对起飞点、沿起飞航向的绝对偏移；yaw_deg 是相对起飞航向的角度。返航 forward_m=0 不随转向重定义。降落保持最后一段任务航向，任务 2 保持起飞航向 +180°；此入口不使用板配置的 target_yaw_rad。移动和转向采用速度、加速度受限的五次曲线，达到目标且连续稳定后才进入下一步。前视深度用于下述单次绕行和回撤检查。

开发默认：高度 0.6 m、水平峰值 0.3 m/s、偏航峰值 0.8 rad/s；任务自身同时将起飞高度和锁点最低高度设为 height_m，不修改原导航配置。总预算和下降预留取 attitude_config_file 的 mission，当前模板为 45/14 s，从解锁请求计时。实际延迟可能耗尽预算；预算到点请求后备 Land，不继续未完成航段。

### 输入与启动

启动前须有以下输入。downward_input:=v4l2 可启动独立下视采集；默认 external。range_input:=mavros 默认启动测距桥，但须显式填写实测 range_sensor_id；默认 -1 不出测距。参数见[感知包输入说明](../boom_birds_sensing/README.md#下视与测距输入)。约定 A4 单码黑框 70 mm，打印见[感知包](../boom_birds_sensing/README.md#实机待测打印板)；几何片段必须合入实测完整 profile。

| 输入 | 要求 |
| --- | --- |
| /boom_birds/downward/image | sensor_msgs/Image；尺寸与 tag_config_file 相同；原始采样时间。可用 downward_image_topic 改名 |
| /boom_birds/platform/range | sensor_msgs/Range；frame_id=range_sensor；原始采样时间、有效 min_range/max_range；距离为传感器斜距 |
| tag_config_file | 沿用[板配置](../boom_birds_control/README.md#配置)：实测黑框尺寸、ID、下视内参/安装外参、测距安装外参与证据 |
| 前视双目、IMU、VIO、深度 | 默认由入口启动；start_sensing:=false 时须由外部提供 MAVROS、IMU、双目图像、机体里程计、camera_pose、深度和 CameraInfo |
| obstacle_config_file | [tag_obstacle_guard.yaml](config/tag_obstacle_guard.yaml) 是未实测模板；实机须测量机体包络、制动距离、深度几何与时延，并填写 evidence |

CSI IMX219 可显式设置 downward_pixel_format:=pRAA、downward_media_device、downward_sensor_entity。downward_camera_config_file 非空时作为图像源工作相机配置；空值沿用 tag_config_file。downward_native_config_file 用于核对原生采集尺寸与缩放内参，downward_raw10_native_library 显式选择本地解包库。工作图尺寸/K/D 必须与 Tag 观测的 camera 一致；所有真实外参和 evidence 检查保留。参数语义见[感知输入](../boom_birds_sensing/README.md#下视与测距输入)。

所有进程使用同一 ROS 时域。Tag 与 VIO、测距的最近采样差各不得超过 30 ms；保留图像原时间，不以收到时间续期。VIO 使用 global/body，机体位姿包含完整旋转。完整解码确定身份及平面点对应；局部特征多点 PnP 与测距/姿态约束共同确定 VIO 世界系落点，再限制在航点周围 max_tag_correction_m 内。检查正深度、法向、倾角、ID、重投影、米制漂移及 yaw；不直接取平面 PnP 的第一个候选解。

三个入口参数相同。以下路径与 FCU 设备需替换为本机已核验值；每次只运行一个任务，不与旧控制/任务节点共存：

```bash
# 在仓库根运行，先激活项目 Python 环境。
source companion/ros2_ws/tools/activate_python_env.sh
source /absolute/path/to/install/setup.bash
BB_HW=/absolute/path/to/measured-hardware
BB_FCU=serial:///dev/serial/by-id/REPLACE_WITH_ACTUAL_DEVICE:921600
taskArgs=(
  "fcu_url:=$BB_FCU"
  "calibration_file:=$BB_HW/candidate.npz"
  "extrinsics_file:=$BB_HW/extrinsics.yaml"
  "vio_config_file:=$BB_HW/estimator_config.yaml"
  "attitude_config_file:=$BB_HW/attitude_hardware.yaml"
  "tag_config_file:=$BB_HW/platform.yaml"
  "obstacle_config_file:=$BB_HW/tag_obstacle_guard.yaml"
  dry_run:=true
)

# 分别选择一个入口；默认 start:=false，只启动节点。
ros2 launch boom_birds_bringup tag_takeoff_land.launch.py "${taskArgs[@]}"
ros2 launch boom_birds_bringup tag_return_turn.launch.py "${taskArgs[@]}"
ros2 launch boom_birds_bringup tag_forward_land.launch.py "${taskArgs[@]}"
```

START 前检查下视图像、Range、板观测各有唯一发布者，且采样新鲜；旧 VIO 降落路径还检查局部特征输入。交接路径不以局部特征诊断的新鲜度或发布者数量作为准入条件；地面可无有效 Tag，但输入链必须正常。交接开启时还要求 `/boom_birds/compute/release` 服务存在，否则拒绝 START；本机 SIH 工具直接构造节点并持有自有计算进程，不能用于实飞。PRECHECK 及解锁前继续检查，飞行中断流撤销运动/下降；有可用定位与前视深度时回撤，无法回撤时请求后备 Land。

交接相关参数（三个入口同名）：`handover:=true|false`（默认 true，仅 dry-run/SIH 允许；关闭后使用旧 VIO 降落路径）、`platform_status_timeout_s`（默认 0.5）、`compute_log_directory`（自有计算进程日志）、`platform_trace_directory`（控制包分段 trace）。交接开启且 `start_sensing:=true` 时，VIO 与姿态适配器由 `compute_supervisor` 以子进程方式启动并由任务持有；`start_sensing:=false` 时拒绝交接（没有可核验的自有进程句柄）。

`feature_tracking_diagnostics` 默认 false。`handover=true` 默认只处理完整解码；显式开启该参数可保留 LK/PnP 诊断，诊断不参与交接输入闸门。`handover=false` 始终启用局部特征和旧 VIO 降落输入检查。

`tag_compute_profile` 默认 `configured`，沿用所选 VIO 文件及原采集/深度参数。显式选择 `balanced` 时：前视发布 10 Hz、深度上限 8 Hz、下视发布 10 Hz；VIO 为 80 点、4 克隆、每次最多 10 个 MSCKF 特征，`use_multi_threading_subs=false`。它不改双目几何或标定；要求 `start_sensing=true`。本轮 Pi 运动观察使用这些降载参数并保留特征诊断，复现时加 `tag_compute_profile:=balanced feature_tracking_diagnostics:=true`；这不是精度、实物 Tag 或飞行验收。

准备好输入后，通过既有 Mission 服务启动；START 的 goal 被忽略，航点由任务文件生成。也可启动时显式传 start:=true，每次进程生命周期只自动请求一次。

```bash
ros2 service call /boom_birds/mission boom_birds_interfaces/srv/Mission "{action: 1}"
ros2 topic echo /boom_birds/tag_task/status
# CANCEL；飞行中闭锁并请求后备 Land，人工已切模式时不争抢模式。
ros2 service call /boom_birds/mission boom_birds_interfaces/srv/Mission "{action: 2}"
```

服务名以 RuntimeConfig 为准。真实启动还需现有 allow_arming、allow_non_loopback、allow_hardware_actions 和 hardware_validation_note；只改 dry_run 不能解除未验证参数的限制。独立 launch 的合成板只允许 tag_test_only:=true 且 dry_run:=true；SIH 工具直接构造节点并提供已核验的本机进程证明，不能用于实飞。

### Tag 捕获、控制交接与 VIO 释放

三个任务共用同一交接流程，参数在任务文件和控制器/任务节点中显式选择：

1. **捕获证据**（VIO 仍运行）：完整解码给出身份与板平面；任务端要求 Tag 身份/质量/新鲜度/连续样本检查通过，至少 4 个不同采样、连续对准且 `yaw_valid` 成立后，才请求平台接管。单帧检测、局部特征定位或仅质量边缘的观测都不触发交接或下降。
2. **撤销导航输出**：任务闸门发布同会话平台请求并撤销导航 setpoint；控制器平台层先只发零速度。ONLY-OFFBOARD 或本地发送成功不算交接成功。
3. **速度确认**：需同一飞控 epoch、新鲜 PX4 速度估计、与已发送目标相关的连续回读（SIH 读 offboard_control_mode / trajectory_setpoint / vehicle_control_mode / vehicle_local_position），且姿态发布者必须关闭。回读不符、过期或 `handoff_timeout_s` 超时则闭锁并请求 PX4 Land，**不得下降、不得释放计算**。
4. **释放计算**：确认后才请求 `/boom_birds/compute/release`。该服务只管理任务自己在 launch 中持有的 VIO/姿态适配器子进程（`compute_supervisor`），按自有进程组停止并回读退出，不以进程名匹配或误停深度/通信/测距节点。启动器先退出也必须检查同组工作进程；SIGTERM 超时后 SIGKILL。僵尸进程不占计算资源，不计作仍运行。释放失败保持诊断并如实报告，不声称已释放。
5. **交接后引导**：只用下视板相对位姿、`/boom_birds/platform/range` 和 PX4 姿态/速度/接地反馈；不再使用 odom_body 世界系位置或偏航。近地到达 `touchdown_height_m`（本机 70 mm 测试板 0.35 m，为 ±8 cm 对准容差和 Tag 视场边界留出余量）且连续对准后**主动请求 PX4 Land**，由 PX4 完成接地；完成要求 PX4 接地与 armed=false 回读，不用视觉高度判定接地。

边界（必须按此理解，不得静默改语义）：

- 交接前失败（捕获超时、交接超时、回读不符、VIO/深度失效）仍保留 VIO，按原路线回撤或请求 PX4 Land；只有 VIO 有效且未进入末段交接时，绕行失败才承诺沿实际飞过路径返回。
- 交接确认后 VIO 已释放时**不能承诺原路返航**；Tag 短暂丢失会立即停止下降并有限时间重捕获（`loss_land_timeout_s`），持续丢失、测距/速度回执失效或飞控 epoch 变化则交 PX4 Land 并记任务失败，不切换到没有定位依据的返航。
- 局部特征/LK/PnP 的米制解依赖 odom_body，因此在 VIO 释放后**不作为下降依据**；完整解码丢失即按上条处理。观测节点仍继续运行并发布诊断。
- 真实速度确认适配器尚未实现/核验，真实入口继续拒绝交接（`tag_handover_requires_dry_run_until_hardware_verified`、`hardware_velocity_confirmation_adapter_not_verified`）。本轮的通过证据只覆盖本机 SIH 与既有脱机回归。

### 下降与结束判定

任务 1/2 先在起飞位置上方取得有效板观测才允许外出；任务 3 不要求起点有 tag。末端须取得至少 4 个不同采样的完整解码结果并连续对准目标。交接关闭时可用原身份下的局部特征继续 VIO 引导下降；交接开启时只以完整解码进入平台速度交接（见上节）。

有效 Tag 与测距确认高度接近近地门限后，交接路径由平台层主动请求 PX4 Land（测试板配置 0.35 m，高于旧 VIO 路径的 `final_land_height_m` 0.18 m，确保 Tag 仍在视场内）；旧路径先固定高度、速度收敛并持续 `final_land_confirm_s` 再请求 PX4 Land。两条路径都需 auto:land 回读，切换超时、状态过期或总预算耗尽均闭锁。最后接地由 PX4 完成，不用 Tag 裁切或测距盲区推断接地，局部像素跟踪不提供接地证明。PX4 连续接地回读后才请求 disarm，接地且 armed=false 才 complete=true。交接任务还要求 `release_verified=true` 且自有计算进程已退出，否则即使落地也不报告成功。平台接管后仍检查从解锁请求起算的总飞行预算，超时请求 PX4 Land 并记失败。完成时间在接地且 disarm 的 COMPLETE 回读时记录。FAULT_LATCHED 后即使后备 Land 落地，也不报告本任务成功。route_reached 只表示已到末端并通过下降前对准。

控制接口的 camera_health_topic 在这三个入口中显式选择前视左目原图；空值仍沿用 depth_topic。只替换相机新鲜度输入，不伪造深度、地图或规划就绪，也不放宽 VIO/IMU 时效。

当前实现、Pi 部署与验收范围见 [STATUS](../../../../docs/STATUS.md#实现默认入口部署与验收)。

### 前向避障与原路返回

配置见 [tag_obstacle_guard.yaml](config/tag_obstacle_guard.yaml)。按深度采样时间，将 CameraInfo、camera_pose、odom_body 配对（各 30 ms）；检查停止距离内膨胀机体截面的所有近障点，投影走廊内存在 NaN、零值、越界深度或视场不足时返回 UNKNOWN。未收到新深度也不准移动。地面剔除使用起飞原点和已测机体离地高度。

MOVE 遇障先按当前速度制动，确认停稳后最多尝试一次左侧折线路径（首段前向上限 0.2 m、侧向 0.4 m）；绕行拐点使用任务到达容差的一半，默认 4 cm。每段先转向，使前视相机朝向移动方向；转向后必须重新取得连续 CLEAR 深度才移动，途中继续检查。候选超出任务半径、时间不足、深度不明或再次遇障则放弃绕行，倒序跟随实际飞过的位置记录返回。记录不删除起点，共线压缩误差上限 25 mm，最多 256 点。

回撤每段仍须转向并重新检查深度。任务 1/2 返回后使用原点 Tag 降落；任务 3 起点没有 Tag，返回起点上方后请求 PX4 Land。回撤受阻、定位失效、制动无法确认或预算不足时请求后备 Land；这些路径均标记任务失败。回撤优先不意味着盲目穿过新障碍。

保护范围限于当前前视截面；单相机不能证明机体侧后方、转向扫掠区域或机腹下面无障碍。模板的包络和制动参数尚未实测，hardware_verified=false 阻止实机使用。

### 本机 PX4 SIH 验证

在仓库根激活 platform_env.sh 和可用的 ROS RMW 环境后运行：

```bash
source companion/ros2_ws/tools/platform_env.sh
# 本次采用私有 Cyclone；路径是本机验证环境，不是运行包依赖。
source /home/waterc/bb_build/navigation-fix-20261024/cyclone-rmw/activate.sh
python3 companion/ros2_ws/tools/run_tag_flight_sih_matrix.py \
  --out /absolute/path/to/new-evidence-directory \
  --with-safety --allow-simulated-arming
```

矩阵运行：`python3 companion/ros2_ws/tools/run_tag_flight_sih_matrix.py --out <新目录> --with-safety --allow-simulated-arming --compute-load light`。顺序运行三个 nominal 与交接/故障场景：交接超时、回读不符、确认后反馈中断、Tag 短暂丢失、持续丢失、观测过期、错误 ID、人工取消、绕行成功、绕行失败回撤、回撤再次受阻、深度断流、VIO 断流，另保留一次交接关闭的 Tag 边框消失/LK 续跟对照。绕行相关案例显式使用 90 s 测试预算；不修改任务默认 45 s。单独运行一例：`run_tag_flight_sih.py --task tag_takeoff_land --scenario nominal --out 新目录 --allow-simulated-arming`。

脚本只允许经核验的本机 SIH 和回环 MAVROS，拒绝已有 PX4 实例；不连接串口、不改 PX4 参数。实际链路为任务节点 → 位置闭环 → MAVROS 姿态/推力 → PX4 SIH 动力学。HIL_STATE_QUATERNION 真值生成模拟 VIO、深度射线、下视 Tag 图像和测距；图像经过实际检测/PnP，不直接注入有效 Tag。速度回执由独立 `sih_feedback_probe.py` 进程读取 uORB 后写 JSON，控制进程只读文件，避免单机 SIH 内轮询拖慢控制/任务周期（`BB_SIH_FEEDBACK_FILE`）。`run_tag_flight_sih.py` / 矩阵可显式选择 `--feedback-source mavlink`，验证 MAVROS 回读候选；默认 `uorb`。候选尚未取得实机交接许可。探针并行读取四个主题，`stamp` 为文件生成时间，`sample_stamps` / `sample_ends` 为各次 CLI 读取的起止时间。读取端拒绝缺失、未来或超过 0.2 s 的时间戳；新鲜度按读取起点计入经过时间，目标关联使用读取结束上界，重复文件不会刷新证据。自有计算进程是两个合成 CPU 负载（`--compute-load idle|light|spin`），交接后必须退出；另建无关进程验证清理不误停。测试另查旋转机体包络与障碍盒相交；障碍不参与 SIH 接触动力学。

每例保存 report.json、trace.jsonl、输入 YAML、SIH 参数回读、进程日志、sih-feedback.json 及 px4.ulg，矩阵输出 matrix.json；`plot_tag_flight_sih.py matrix.json` 生成每例轨迹图、控制/平台状态与 VIO 释放时间线、矩阵 summary.md。仿真没有运行 OpenVINS、双目匹配或真实相机驱动；不证明真实输入连续性、标定、Pi 性能或实飞接管。


脱机回归使用 run_tag_flight_checks.py：无参数时分别运行 bringup/control/sensing 三个测试目录，也可透传单个 pytest 路径和选项。必须使用相互独立的进程，避免不同包中的同名测试模块冲突。设置 BB_TAG_TEST_RESULTS 可保存分包 JUnit XML。轨迹/时间线/summary 绘图命令：`python3 companion/ros2_ws/tools/plot_tag_flight_sih.py 证据目录/matrix.json`。

HIL 地理坐标真值的水平分辨率约 1.1/0.75 cm，高度 1 mm；报告中的零水平误差只表示落入同一量化坐标，不代表实物精度。


## 特征失效后 EKF dry-run 候选

`feature_ekf_candidate` 在单进程执行器中连接任务、EKF 读取与姿态控制。配置为 schema=1 的 YAML，包含 `control`、`task` 和 `max_accuracy_m`（0 < 值 ≤ 0.2）。控制必须为 `backend=mavros`、`control_mode=companion_attitude`、`dry_run=true`、`allow_arming=false`、`allow_hardware_actions=false`、`require_session=true`；任务必须为 `dry_run=true`、`start=false`、`handover_enabled=false`、`platform_landing_enabled=false`、`feature_tracking_enabled=true`。配置路径须存在，不接受 SIH PID 或平台配置覆盖。

```bash
ros2 run boom_birds_bringup feature_ekf_candidate --config /absolute/path/candidate.yaml --check-only
```

`--check-only` 只验证配置和导入，不初始化 ROS。去掉该选项才创建节点；MAVROS、VIO、下视图像、深度、测距需外部提供，本入口不启动它们、不自动开始任务。状态主题为 `/boom_birds/feature_ekf_candidate/status`。启用读取后请求 ODOMETRY/ESTIMATOR_STATUS 各 20 Hz；串口负载需另行验证。

特征有效时更新视觉与 EKF 的配对参考，失效后锁定 EKF XY；测距失效后使用配对的 EKF Z 位移。没有固定切换高度，也不注入仿真故障。EKF 无效、过期、重置或跳变时拒绝参考；VIO 姿态仍须新鲜。每次求参考和输出反馈都检查 dry-run 条件，修改 YAML 不能开放真实控制。该入口不释放 VIO。

Pi 正式入口统一为 `bash /home/gmaster/boombirds/current/run.sh --check-only`；删除 `--check-only` 才创建 dry-run 节点。安装位置、依赖环境及验收状态见 [STATUS](../../../../docs/STATUS.md#实现默认入口部署与验收)。当前硬件模板未标定，不能据此开放实飞。
