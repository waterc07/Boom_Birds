# Companion 姿态控制实机验证入口

模式名 `companion_attitude`。OpenVINS 提供机体位姿和世界系速度，EGO 提供轨迹；Companion 执行位置/速度 PD 与加速度前馈，经 `/mavros/setpoint_raw/attitude` 发送 ENU/FLU 四元数和归一化推力。PX4 执行姿态与角速度闭环。

`px4_position` 保留原有位置 setpoint 路线，两个出口互斥。此实现参考 Fast-Drone-250 的控制分工，没有移植其 ROS 1 px4ctrl、增益、推力参数或自动解锁逻辑。

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

端口、PX4 参数、标定、机体安装外参、悬停推力与控制参数必须有各自证据；缺项保持待填，不用单位矩阵或 SIH 值补齐。

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

构建与 dry-run 章节的示例使用显式选择的双目配置；T_I_B 未测时仍不能通过完整控制入口的配置核验。

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
export BUILD_BASE=/home/waterc/bb_build/ego-single/build
export INSTALL_BASE=/home/waterc/bb_build/ego-single/install
export LOG_BASE=/home/waterc/bb_build/ego-single/log
bash companion/ros2_ws/tools/build_all.sh --packages-up-to ego_planner boom_birds_nav
BUILD_BASE=/home/waterc/bb_build/ov/build \
INSTALL_BASE=/home/waterc/bb_build/ov/install \
LOG_BASE=/home/waterc/bb_build/ov/log \
bash companion/ros2_ws/tools/build_all.sh --packages-up-to ov_msckf
source companion/ros2_ws/tools/activate_python_env.sh
source "$INSTALL_BASE/setup.bash"
source /home/waterc/bb_build/ov/install/local_setup.bash

# 按实机修改路径与端口；/dev/serial/by-id 优先于可能变化的 ttyUSB 编号。
export BB_HW=/home/waterc/boombirds-hardware/my-airframe
export BB_FCU=serial:///dev/serial/by-id/actual-device:921600
# 已准备并核对的双目配置；示例路径必须替换，不能直接沿用 calibration.env 的单目选择。
export BB_VIO_CONFIG=/absolute/path/to/reviewed-stereo/estimator_config.yaml
python3 companion/ros2_ws/tools/check_attitude_hardware.py \
  --fcu-url "$BB_FCU" --calibration-file "$BB_HW/stereo.npz" \
  --extrinsics-file "$BB_HW/extrinsics.yaml" --vio-config-file "$BB_VIO_CONFIG" \
  --attitude-config-file "$BB_HW/attitude.yaml"
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

任务默认总预算 30 s，从发送解锁请求计时；剩余 14 s 时撤销规划并开始下降，超时请求 PX4 Land。起飞高度取 RuntimeConfig，默认 1.5 m；首轮需保证场地/系留与此高度相容。此预算不是能源预测；低电压、电流与续航仍由实机验收。

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

SIH 锁步时钟与主机墙钟不能作为实机 TIMESYNC 证据。仅经 `verify_sih_process` 核验的本机 SIH PID 允许真值测试使用 router 接收时间；状态标记 `TEST_ONLY_SIH_router_receipt`；该路径按回调接收时刻构造真值，不验收传输采样年龄。实机入口不传 SIH PID，仍要求 PX4 boot 时间映射。SIH 不验收真实 VIO 或采样时间同步。

## 下视平台末段降落

可选平台交接默认关闭，配置、图像/rosbag 入口、状态与脱机/SIH 命令见 [控制包](../boom_birds_control/README.md)。当前真实入口拒绝执行；原点起飞和往返导航仍保留 VIO。

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
| `PRECHECK` → `TAKEOFF` | 连接 + `status_age ≤ status_timeout_s` + 未解锁 + `landed == 1` + `pose_age ≤ pose_timeout_s` + 对齐有效 + 位置有限；记录 `ground_z` 与 `boot_epoch`，输出 `arm` |
| `PRECHECK` 超时 → 闭锁 | `precheck_timeout_s`（缺省 10 s） |
| `TAKEOFF` → `HOLD_READY` | 解锁确认后发一次 `takeoff`；`landed == 2` 且 `StableWindow` 成立 → 锁点 = 当前观测位置 |
| `TAKEOFF` 闭锁 | 未解锁超过 `mode_timeout_s`；或超过 `takeoff_timeout_s` |
| `HOLD_READY` → `OFFBOARD_PENDING` | `sending ∧ sensors_ready ∧ map_ready` 且在本状态已停留 `stable_duration_s`，输出 `offboard` |
| `HOLD_READY` 闭锁 | 对齐失效 → `frame_reset`；或超过 `hold_ready_timeout_s` → `map_or_stream_not_ready` |
| `OFFBOARD_PENDING` → `EXECUTING` | **模式回读**确认 OFFBOARD（`offboard_observed`），输出 `enable_planner` |
| `OFFBOARD_PENDING` 闭锁 | 模式回读超 `mode_timeout_s` → `offboard_not_confirmed` |
| `EXECUTING` → `LANDING` | 到点判定：`pose_age ≤ pose_timeout_s` ∧ 距 goal ≤ `goal_tolerance_m` ∧ 速度 ≤ `stable_speed_m_s`，且持续 `stable_duration_s` |
| `LANDING` → `COMPLETE` | 连接 + 状态新鲜 + 已上锁 + `landed == 1` |
| 任意阶段 → `FAULT_LATCHED` | 时钟回退、状态超 `mode_timeout_s`、`boot_epoch` 变化（飞控重启）、会话变化、命中 `REVOKING_FAULTS`、飞行中 `landed == 1` 或上锁 |

`RECOVERABLE_FAULTS`（允许自动恢复）与 `REVOKING_FAULTS`（永久撤销本次恢复资格）是两个不同的集合；
`INTERRUPTIBLE_AUTO_MODES` 只识别 `auto:land` / `auto:rtl` 两个可被**有界**中断的
AUTO 模式。自动中断还需新鲜实际/意图模式一致，以及当前 SIH PX4 原生 failsafe 原因匹配；人工模式动作或未知原因闭锁。

### 恢复窗口与重规划语义

- **缺省不可达**：`RuntimeConfig.recovery_enabled` 缺省 `False`，只有 TEST-ONLY 的 SIH 入口把它显式打开。
- **恢复资格**：`recovery_available()` = 开关 ∧ 未被撤销 ∧ 故障属于 `RECOVERABLE_FAULTS` ∧ 预算未耗尽。
- **进入前置条件**（fail-closed：观测缺失/过期即拒绝）：未人工取消、仍解锁、明确在空中（`MAV_LANDED_STATE ∈ {2,3}`，`lifecycle.py`）、
  `status_age ≤ status_timeout_s`、`pose_age ≤ pose_timeout_s`、位置与速度有限。
- **`recovery_attempts`**：进入 `RECOVERING` 即消耗一次，缺省预算 `2`；耗尽后闭锁
  `detail="exhausted"`，**不循环争抢、不自动重新解锁**；恢复后连续 1 s 健康 EXECUTING 才关闭本次故障并重置事件预算，任务累计次数另行记录；窗口内再次失效共用预算。
- **一次尝试的窗口**：`mode_timeout_s`（缺省 3 s）同时约束"等条件就绪"与"等模式回读"两段；超时且预算还有就重试。
- **连续有效窗口**：`recovery_valid_duration_s`（缺省 1 s）要求 `sensors_ready`、`map_ready`、`alignment`、
  两条 age 与有限位置**连续**成立，任一失效即清零并把原因记进 `recovery_window_reset`。
- **接管放行**：还需 `Observation.sending`（即 `px4_failsafe` 的放行结果）、已记录 `ground_z`、相对高度 ≥ `recovery_min_altitude_agl_m`
  （1.0）、速度 ≤ `recovery_max_speed_m_s`（1.0）。
- **请求 OFFBOARD 只发一次并要求回读确认**：`recovery_request_at` 置位后返回 `offboard` 动作，随后靠 `offboard_observed` 确认，
  **命令被接受不算确认**。
- **确认后重新规划**：`_confirm_recovery()` 把 hold 移到故障瞬间位置、转 `EXECUTING`，输出 `replan` + `enable_planner`。`replan`/`recover` 都调 `_retire_trajectory()`：把当前 `trajectory_id` 记入 `ControlIngress.retired`
  并 +1，同 id 命令此后一律被 `retired_trajectory` 拒绝。
- **planner 重新使能**：正常路径是 `EXECUTING` 期内**限频重发**使能请求，间隔 `planner_enable_retry_s`（缺省 0.5 s，
  `runtime_config.py`），上限 `planner_activate_timeout_s`（缺省 30 s）后闭锁 `planner_timeout`。重发是必要的：EGO 的 `projectRequest` 在
  offboard/odom/地图未就绪时**静默丢弃**。
- **规划器单次作废 ≠ 任务故障**：收到 `CANCEL` 型规划命令只退役旧轨迹并转入"规划流静默"，由上面的有界等待处理。
- **规划流停止后的 hold 点**：若距 goal 已在 `handoff_max_distance_m`（0.5 m）以内，hold 点放到 **goal**，让飞控收敛最后一段；
  否则放在**当前位置**，不在没有规划的情况下继续飞（`lifecycle_node.py`，理由见 `lifecycle_node.py`）。
- **永久撤销**：人工取消 / 人工模式干预 / 未知故障 / 落地 / 重启 / 坐标系重置 / 几何变化。
