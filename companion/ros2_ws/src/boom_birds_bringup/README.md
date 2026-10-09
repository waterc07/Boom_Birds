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
| OpenVINS estimator_config.yaml 及两条 Kalibr chain | 单目 cam0 使用左目内参、畸变与相机—IMU 标定；等比例缩图同步缩放内参；OpenVINS chain 使用 `T_imu_cam`（相机到 IMU），原始 Kalibr `T_cam_imu` 须取逆；IMU topic `/boom_birds/imu`，单目 topic `/boom_birds/stereo/left_raw`；双目深度继续使用同一采集帧的两目 |
| extrinsics.yaml | 明确的 `T_I_C0`、`T_I_B` 和实测 `source`；与 OpenVINS cam0 外参相同 |
| attitude.yaml | 从 `boom_birds_control/config/attitude_hardware.yaml` 复制；实测悬停推力、推力上下限、增益与倾角限制，不使用 SIH 参数作为实机证据 |
| 飞控记录 | FCU system/component ID 均为 1（当前后端目标）；PX4 固件版本、机架、IMU安装方向、输出协议、端口/波特率、姿态/角速度调参、EKF来源及 failsafe 参数导出 |

首轮只接受 pinhole/radtan、固定标定；四项畸变须与 NPZ 一致。五项模型的第五项非零时拒绝。采集时间与飞控 IMU 时间须完成实际测量，包括相机—IMU偏移，不能把接收时间当曝光时间。IMU 默认请求 50 Hz，是否足以支持实际 OpenVINS 和机载算力，须按实测数据判断。

`hardware_verified: false` 和空 `evidence` 只允许 dry-run。实测后填写 `hardware_verified: true` 与记录路径；这只解除软件配置闸门，不替代脱桨和系留验收。程序不自动写 PX4 EKF、解锁检查或 failsafe 参数。

### 飞控到手前

可以准备双目 NPZ、未验证的 `attitude_hardware.yaml` 副本和待填信息表。IMU 来源仍为飞控；端口/波特率、PX4 参数、相机—IMU外参和时间偏移、OpenVINS chain、悬停推力与控制参数没有实测前保持待填，不用单位矩阵或 SIH 值补齐。

本轮 Pi 感知证据和准备文件路径见 [STATUS](../../../../docs/STATUS.md)。只启动相机和深度节点，不用完整实机 launch 做相机性能测试。距离精度和地图输入验证本轮暂不执行。

## 单目算力配置

硬件保持当前 Pi 5、USB 双目及下视相机。默认 `compute_profile=mono_budget`；先验证全链算力预算，VIO 漂移、距离精度和导航质量后续验收。默认 dry-run、禁止解锁、标定一致性和故障闭锁继续生效。

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

实机命令使用 `vio_config_file:="$BB_HW/mono-budget/estimator_config.yaml"`。`compute_profile:=calibrated` 可按提供的单目或历史双目配置运行，不应用算力覆盖；默认不再接受双目配置作为 mono_budget 输入。

`shared_decode=auto` 在 `mono_budget` 中启动 `shared_stereo_depth`，采集与深度保留两个 ROS 节点名，共用一个进程。深度工作线程只保留一组待处理数组引用，左目 Image/CameraInfo 接口和时间戳不变；不再为深度发送 MJPEG。OpenCV 线程数为进程全局值，采集/深度共用 2 线程，OpenVINS 仍为独立进程的 1 线程。

`shared_decode:=false` 恢复原 MJPEG 分进程输入作对照，仍应用按需 Z 和视差范围配置。`calibrated` 的 auto 保持分进程；完整旧匹配范围可将深度配置设为 `num_disparities: 96`。

深度参数 `num_disparities: 0` 按同一标定 Q 和 `required_min_depth_m: 0.2` 选择最小可覆盖范围，保留 `d < N-1` 门限并向上取 16 的倍数，自动档不超过原 96。当前随仓库保存的标定选择 64，理论最近深度约 0.18552 m；48 无法覆盖 0.2 m，启动拒绝。0.2 m 是本配置的搜索范围要求，不是实测量程或避障安全距离；任务需要更近观测时须调低该值并重测。独立深度节点默认仍为 96/不启用量程检查。

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
python3 companion/ros2_ws/tools/prepare_mono_vio.py \
  --source "$BB_HW/estimator_config.yaml" --out "$BB_HW/mono-budget"
python3 companion/ros2_ws/tools/check_attitude_hardware.py \
  --fcu-url "$BB_FCU" --calibration-file "$BB_HW/stereo.npz" \
  --extrinsics-file "$BB_HW/extrinsics.yaml" --vio-config-file "$BB_HW/mono-budget/estimator_config.yaml" \
  --attitude-config-file "$BB_HW/attitude.yaml"
```

命令核验文件存在、内参/基线/相机—IMU外参一致和控制参数范围，输出文件 SHA256。`--live` 额外要求机体参数已确认。文件一致不证明标定准确。

## 脱桨接入与 dry-run

确认同一 ROS 域内没有 SIH、回放或另一套控制节点。实机入口不启动合成源；缺必需文件直接失败。

```bash
ros2 launch boom_birds_bringup attitude_hardware.launch.py \
  fcu_url:="$BB_FCU" allow_non_loopback:=true \
  calibration_file:="$BB_HW/stereo.npz" extrinsics_file:="$BB_HW/extrinsics.yaml" \
  vio_config_file:="$BB_HW/mono-budget/estimator_config.yaml" attitude_config_file:="$BB_HW/attitude.yaml" \
  camera_device:=/dev/video0 capture_width:=2560 capture_height:=960 capture_fps:=60

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
