# 感知包

导航使用左目图像与飞控 IMU；深度使用同帧左右目。硬件入口、单目标定配置生成及算力参数见 [bringup](../boom_birds_bringup/README.md)。算法与标定见 [stereo_depth](../stereo_depth/README.md)。

## 入口与实现

| 文件 | 职责 |
| --- | --- |
| `stereo_capture.py`、`stereo_source.py` | 采集/回放、单次相机打开、同帧拆分、时间戳与按订阅需求发布 |
| `shared_stereo_depth.py` | 同进程采集与深度入口；解码后的左右数组直接交付深度线程 |
| `depth_core.py`、`depth_node.py` | 算法适配与 ROS 发布；无点云需求时只生成 Z |
| `camera_geometry.py` | 标定与 CameraInfo 几何校验、内参缩放、原图尺寸与缩放 |
| `frame_worker.py` | 单槽最新帧、限频与退出；平台观测异常后继续，深度异常后停止并上报 |
| `mavros_imu_node.py`、`pose_adapter.py` | 飞控 IMU 门控上行与 VIO 位姿适配 |
| `platform_observation.py`、`platform_observation_node.py` | 下视板检测与 ROS 观测 |
| `platform_replay.py` | 图像清单/rosbag 回放及 TEST-ONLY 板图渲染 |
| `vio_calibration*.py` | 棋盘格相机—IMU采集、受限遥测入口、筛查与 Kalibr/OpenVINS 候选转换 |

`image_scaling.py`、`platform_worker.py`、`platform_synthetic.py` 保留旧导入入口，不保留第二份实现。新代码直接导入对应实现模块。

## 已选开发配置

当前标定选择、部署记录和未验收项只维护在 [STATUS](../../../../docs/STATUS.md#当前配置与标定)。WSL 与 Pi 的路径不同，不复制另一台机器的绝对路径。

~~~bash
# WSL：配置选择入口
source /home/waterc/boombirds-hardware/current-vio/calibration.env

# Pi：先加载对应 release 的 ROS/安装环境，再加载选择
# source /home/gmaster/boombirds/current/activate.sh
# source /home/gmaster/boombirds/hardware/current-vio/calibration.env

printf '%s\n' "$BB_VIO_CONFIG" "$BB_CALIBRATION"
readlink -f "$BB_VIO_CONFIG"
readlink -f "$BB_CALIBRATION"
sha256sum "$BB_VIO_CONFIG" "$BB_CALIBRATION"
~~~

`BB_VIO_CONFIG` 与 `BB_CALIBRATION` 必须来自同一选定批次。独立深度 CLI 未显式传参时使用模块默认 NPZ，不自动读取这两个环境变量；显式使用 `--calibration "$BB_CALIBRATION"` 才选择该批次。

环境读取和 hash 一致只证明配置文件选择。启动节点后还须核对实际加载的 estimator/chain、内参缩放、时间偏移以及深度 NPZ；各入口的参数见 [bringup](../boom_birds_bringup/README.md)。无 T_I_B 时不能用 pending 文件作为完整控制外参。

## OpenVINS 初始化与视觉更新状态

以下接口由 ROS2 OpenVINS 发布，默认命名空间为 /ov_msckf：

| 话题 | 条件与用途 |
| --- | --- |
| state_poseimu、state_odomimu | 状态初始化后输出估计值和协方差，包括静止零速更新与纯惯性传播；只用于诊断，不接地图、规划或控制 |
| poseimu、odomimu、pathimu | 至少有实际通过 MSCKF/SLAM 更新的特征，且最近更新未过期；原 pose_adapter 仍订阅 odomimu |
| status | IMU 回调驱动，最多 5 Hz；JSON 含 state_initialized、zero_velocity_update、visual_tracking_ready、visual_update_count、last_visual_feature_count、last_visual_update_stamp_s、stamp_ns |

ROS 参数 visual_update_timeout_s 默认 0.5 s，须为正有限值。零特征更新不刷新门限；没有 IMU 时 status 也停止，接收端还须检查 stamp_ns/话题接收年龄。visual_tracking_ready 只说明近期有视觉更新，不证明米制尺度、漂移、外参、导航或飞行许可。

ROS2 run_subscribe_msckf 默认 graceful_shutdown=true：同步处理回调并关闭后台图像发布，退出时先释放发布者再关闭 ROS context。处理时延和 Pi 共载需在该模式下实测；显式关闭此模式的异步退出尚未验收。

传播与零速更新要求真实 IMU 覆盖请求区间的起止边界；缺少边界时记录原因并拒绝更新，保留状态时间、协方差与克隆。高频预测同样拒绝未覆盖区间，不向最后一条采样之外外推；终点恰好等于最后一条真实采样时正常使用该采样。此处理不补齐丢失的 IMU，也不解除视觉更新和输入年龄门控。

## 连续 VIO 数据采集准备

本节用于下一轮传感器验证，不启动控制或解锁。真实设备开始前重新核对部署版本、动态库、选定标定和设备路径；10-19 隔离脚本固定了旧证据目录和静止条件，不能直接复用为运动采集入口，也不覆盖旧会话。

- 操作员确认已拆桨、飞控 USB 供电且未解锁；固定相机与飞控的相对安装，线缆留出活动余量。相机朝向有纹理、照明均匀的静态场景，避开反光、纯白墙和人员遮挡。
- 地面标出 A/B 两点，量取并记录距离与机体朝向；现场具备受控移动条件时，等待输入连续、初始化和采集就绪提示，再缓慢平移、短暂停留并返回 A 点。运动段与静止段分别记时，不通过只移动棋盘格代替整机运动。仅能静止时不执行移动。
- 每轮新建会话，先核算 RAM 可用容量并设置容量/时长退出条件。图像、CameraInfo、IMU、时间同步诊断、VIO status、导航输出和诊断输出分别记录；原始 header 与接收时间同时保留，保存配置、源码/运行库散列及运动事件。
- 观察实际视觉更新计数、导航输出、有效 IMU 连续性和消息年龄。零视觉更新的静止状态输出不能作为有效 VIO，也不能作为全链算力验收负载。沿用输入中断、时间同步与写盘错误门控，不为完成采集放宽阈值。
- 结束先停止输入，等待录盘关闭和遥测间隔恢复回读；再把 RAM 数据复制到持久存储，逐文件 SHA256 对比、SQLite integrity 检查并完成落盘。确认复制校验通过前保留 RAM 原件，完成前不关机或拔电；记录最终未解锁、设备释放与进程退出结果。

第一轮只验证输入连续性和有效视觉更新；独立尺度、方向、漂移与回零误差在全链预算通过后评价。1800 s 共载长测采用另行核算的记录策略，不能把短时 RAM 全量录盘方案直接延长。

## 飞控 IMU 时间门控

`mavros_imu_node` 只使用目标飞控的完整 `HIGHRES_IMU` 采样，时间戳由 PX4 boot 时间和 MAVROS 偏移映射。频率请求收到 ACK 后仍须检查实际采样频率。

- IMU 入口默认 `hold_last_sync_on_rtt=true`：单次 RTT 超过 20 ms 时丢弃该同步样本，已收敛映射最多保留到最后一次合格同步后的 1 s；异常样本不更新偏移或延长有效期。失联、飞控重启、ROS 时钟跳变、过期或偏移未收敛仍阻断输出；映射过期后须重新收敛。控制后端保留默认的严格 RTT 重置策略。
- `max_future_wait_s` 默认为 0.02，允许范围 0～0.02 s。映射时间略晚于接收时刻的 IMU 可在最多 64 条的队列中等待，由 5 ms 定时器重检；等待预算另加ROS 回调调度延迟；不改采样时间戳。到期仍在未来的样本按原门控拒绝，0 可关闭等待。该等待不证明时钟映射精度或相机—IMU时间标定通过。
- `/boom_birds/imu/mavlink_status` 包含 `imu_received`、`imu_sensor_rate_hz`、`imu_published`、`imu_rejection_counts`、`sync_rejection_counts`、`last_imu_age_s`、`last_good_sync_age_s`、`imu_future_buffered` 和 `imu_pending`。原始有效采样的频率按最近最多 200 条 PX4 采样时间计算；同步、心跳准入和接收节点调度仍须单独核验。


## 棋盘格相机—IMU标定程序

入口 `vio_calibration`，也可用 `python3 -m boom_birds_sensing.vio_calibration_cli`。本程序不连接完整导航 launch，不启动规划、任务或控制节点。

默认沿用原 A4 棋盘格：12×9 方格、11×8 内角点、20 mm 格长。配置在 `config/vio_calibration.yaml`。棋盘格固定，手持整机平顺移动；避免将棋盘格翻转、急甩或让图像持续模糊。原图保存方向不变，网页预览旋转 180°。

| 命令 | 操作 |
| --- | --- |
| `prepare` | 新建会话，保存原 NPZ、SHA256、棋盘规格、动作计划与内参模型检查 |
| `capture` | 独占 UART，临时精简遥测，启动 MAVROS/时钟/IMU/双目原图/采集界面；退出时恢复间隔并回读 |
| `record` | 仅订阅已有的双目、IMU、飞控状态、电池和同步诊断；供已有受限输入环境使用 |
| `inspect` | 检查已落盘数据、同帧配对、频率、缺口、时间重叠、棋盘可见比例和运动变化 |
| `bag` | 将原始整数纳秒时间戳导出为 Kalibr 可读的 ROS1 bag |
| `solve` | 调用 Kalibr，保存命令、日志和全部结果；不修改部署 |
| `candidate` | 转换为单目 OpenVINS chain、stereo.npz 和外参候选，保持未验收状态 |
| `doctor` | 检查软件依赖，不连接设备 |

采集依赖 ROS 2 Jazzy、本包、OpenCV、MAVROS 和 pymavlink。WSL 离线 bag 导出另需 rosbags。安装附加依赖到隔离环境，保留系统 OpenCV/NumPy：

```bash
source companion/ros2_ws/tools/activate_python_env.sh
PYTHONNOUSERSITE=1 python3 -m pip --python "$BOOM_BIRDS_PYTHON" install \
  -r companion/ros2_ws/requirements-vio-calibration.txt
python3 -m boom_birds_sensing.vio_calibration_cli doctor
```

在 WSL 主工程根准备新会话；不覆盖已有目录。会话可整体复制到 Pi：

```bash
export BB_SESSION=/home/waterc/bb_data/vio-calibration/session-01
python3 -m boom_birds_sensing.vio_calibration_cli prepare \
  --out "$BB_SESSION" \
  --calibration companion/ros2_ws/src/stereo_depth/calibration/live_20261017_141337_524478/candidate.npz \
  --config companion/ros2_ws/src/boom_birds_sensing/config/vio_calibration.yaml
```

在已加载 ROS、本包及最新 IMU 门控的 Pi 环境中，使用复制后的会话路径执行。当前入口只验证过 /dev/ttyAMA0、115200，不自动改波特率。默认使用既有 USB 双目 by-id；不打开下视相机。

```bash
export ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1
export BB_SESSION=/home/gmaster/boombirds/evidence/vio-calibration-session-01
python3 -m boom_birds_sensing.vio_calibration_cli capture \
  --session "$BB_SESSION" --device /dev/ttyAMA0 --port 8084
```

飞控由 USB 供电时，在 capture 命令添加 `--usb-powered`。该会话不等待或检查电池数据，不因低电量/无效电池遥测中止；供电模式写入 session.json。未解锁、心跳和时间同步门控保留。

串口出口仅允许 TIMESYNC、白名单消息的间隔查询/设置和版本读取；拒绝心跳、解锁、模式、执行器、setpoint 和参数写入。MAVROS 插件仅加载 sys_status/sys_time/command/imu。精简遥测档仅用于未解锁标定，不能作为正式导航通信配置；不停止已有串口使用者。

串口原始字节与发送审计由独立线程写盘，每秒 fsync；转发线程只入队。队列最多 4096 项，满队列、写盘失败或退出排空超时使采集 ABORTED，仍执行遥测恢复。`link/report.json` 的 `journal` 记录完整性、最大队列和最大同步耗时。

界面只绑定 Pi 的 127.0.0.1。在 Windows 建立转发后打开 http://127.0.0.1:8084：

```powershell
ssh -N -L 8084:127.0.0.1:8084 gmaster@192.168.137.200
```

- 默认启动要求未解锁、新鲜电量至少 30%；采集中电量低于或等于 20% 时停止。`--usb-powered` 仅免除这些电池条件；心跳/时间映射过期、时间戳回退、输入进程退出或写盘失败仍停止。
- 双目和 IMU 达到配置门限、识别到棋盘格、磁盘可用至少 2 GiB 后，界面才允许开始。默认最长等待和采集共 180 s。
- 点击开始后依次提示：静止 5 s、覆盖画面 15 s、平移 15 s、旋转 15 s、组合运动 15 s、静止 5 s，共 70 s。结束自动保存。只移动棋盘格不能代替整机运动。
- 图像以原始方向 mono8 PNG、IMU 以六轴值写入 capture.sqlite3，保留各自 header 时间戳和接收时间。SQLite WAL/FULL 同步、约 0.25 s 提交；断电仍可能丢失最后未提交批次。未完成或中止会话在 inspect 中拒绝，不标为有效标定。断电时不能执行遥测间隔恢复；下次上电须先核对配置。
- 当前门限允许已测的约 50 Hz IMU，界面显示实际频率；Kalibr 通常使用更高 IMU 频率。低频数据的求解结果仍须检查，不能插值补点冒充高频采样。
- 摄像机与飞控 IMU 的时间差交给 Kalibr 估计；通过采集筛查不证明曝光同步、可观性或 VIO 精度。
- 棋盘检测独立线程只处理最新帧，不阻塞原始图像/IMU写盘；OpenCV 线程限制为 1。状态文件交给写盘线程保存，ROS 回调不执行 fsync。状态中的 timing 记录回调最大间隔、各流接收年龄及写盘队列。完成或中止后先关闭输入并恢复遥测，再保留只读结果页 600 s；再次采集需新建会话。

复制完整会话回 WSL 后，先检查再离线求解：

```bash
python3 -m boom_birds_sensing.vio_calibration_cli inspect --session "$BB_SESSION"
python3 -m boom_birds_sensing.vio_calibration_cli bag \
  --session "$BB_SESSION" --out "$BB_SESSION/capture.bag"
python3 -m boom_birds_sensing.vio_calibration_cli solve \
  --session "$BB_SESSION" --imu-noise /absolute/path/to/measured-imu-noise.yaml \
  --out "$BB_SESSION/solve-01"
```

用户明确采用提前中止的数据时，solve 可添加 `--allow-incomplete-capture`，只豁免 recording_completed、no_abort、duration。原始检查和中止事件保留；图像/IMU频率、间隔、时间戳、配对、棋盘和运动检查仍须通过，solver.json 记录豁免项。

solve 需要环境中可运行的 kalibr_calibrate_cameras、kalibr_calibrate_imu_camera；若工具仅通过 rosrun 暴露，传 `--kalibr-prefix "rosrun kalibr"`。本程序不安装或隐式下载 Kalibr。IMU 噪声须为同一采样路径的实测正数、实际频率和测量来源；prepare 生成的 imu-noise.required.yaml 保留空值，不能直接用于求解。

已选候选使用约 50 Hz 的既有数据，并单独记录随机游走工程假设；这是一次明确授权的离线求解，不能将其假设参数写成噪声实测结果。Kalibr 后端、样条调整与求解结果见[日期化记录](../../../../docs/history/STATUS_2026-10.md#2026-10-18-采用已有数据的相机imu-标定候选)，不把该次设置默认为所有采集的求解参数。

原 NPZ 的 k3 非零时，不能截断为 OpenVINS 四项模型。solve 会先用同批双目棋盘格图像拟合 pinhole-radtan，再求解相机—IMU外参与时间差；原 NPZ 保留。输出的 camchain/PDF 须检查棋盘姿态翻转、角点覆盖、重投影、IMU残差和偏置，失败数据不要通过手工改门限变成 PASS。

```bash
python3 -m boom_birds_sensing.vio_calibration_cli candidate \
  --session "$BB_SESSION" --chain "$BB_SESSION/solve-01/capture-camchain-imucam.yaml" \
  --imu-noise /absolute/path/to/measured-imu-noise.yaml \
  --template companion/ros2_ws/src/open_vins/config/euroc_mav/estimator_config.yaml \
  --out "$BB_SESSION/candidate-01"
```

这里只复用模板的估计器选项；相机、外参、时间偏移和 IMU 噪声全部来自本次输入。Kalibr T_cam_imu 取逆写为 T_imu_cam。导出的 NPZ 内参还原到原传感器尺寸，物理基线不缩放。IMU 使用 PX4 已校正的 HIGHRES_IMU；OpenVINS model=calibrated，不重复应用内参校正。单位内参矩阵不代表机体安装外参已测。后续按 prepare_mono_vio.py 生成算力配置。

若没有带测量来源的 T_I_B，不填写单位矩阵，生成 extrinsics.pending.yaml；可通过 `--body-extrinsics` 提供已测文件。所有候选保持 hardware_verified=false，不自动替换 Pi current、旧标定或飞控参数。真实采集、实际求解、独立回放和 VIO 验收分别记录。

数据格式和求解参数依据 [Kalibr bag 格式](https://github.com/ethz-asl/kalibr/wiki/bag-format)、[相机—IMU标定](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)。ROS/HTTP模拟与伪串口验证仅证明程序行为，不能代替实际安装的标定。

## 验证

在主工程根构建后执行 `bash companion/ros2_ws/tools/check_offline.sh`；非默认构建前缀须设置对应 `BUILD_BASE`、`INSTALL_BASE` 和 `OV_INSTALL`。该检查不连接设备，不验收真实 VIO、Pi 全链负载或飞行。当前结果见 [STATUS](../../../../docs/STATUS.md)。

深度实验参数（定点映射、缓冲、LUT/native、Census、位姿先验、ELAS/RKNN）和分档回放见 [深度实验与回放说明](../../tools/depth_benchmark/README.md)。实验参数默认关闭，实机主线配置未切换。


## 静止 IMU 噪声

飞控 USB 供电，整机固定，不打开相机。使用同一飞控的 HIGHRES_IMU 六轴 FRD 值及原始 time_usec；采集器只允许既有受限遥测，未解锁/心跳门控保留，结束恢复临时消息间隔。

```bash
python3 -m boom_birds_sensing.imu_noise_capture --out /absolute/path/to/new-noise-session --seconds 600
python3 -m boom_birds_sensing.imu_noise_analysis --csv /absolute/path/to/new-noise-session/imu.csv --out /absolute/path/to/new-analysis
```

分析不插值，不跨采样缺口积分；取最长连续段并排除开头 30 s。analysis.json 记录静止筛查、逐轴拟合区间和前后半段系数差异；allan.csv 保存六轴曲线。白噪声用 -1/2 斜率，偏置随机游走用 +1/2 斜率；未找到可识别区间的参数保持 null。十分钟记录仅作短记录评估，不能保证四项都可识别。四项齐全时输出 imu-noise.candidate.yaml，否则输出 imu-noise.partial.yaml；均不自动替换部署配置。模型与单位依据 [Kalibr IMU Noise Model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)。
