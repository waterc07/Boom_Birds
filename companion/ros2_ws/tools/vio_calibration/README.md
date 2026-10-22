# 相机—IMU标定与噪声工具

入口 `python3 -m tools.vio_calibration.vio_calibration_cli`；先在仓库根加载 `source companion/ros2_ws/tools/activate_development_env.sh`。本程序不连接完整导航 launch，不启动规划、任务或控制节点。

默认沿用原 A4 棋盘格：12×9 方格、11×8 内角点、20 mm 格长。配置在本工具的 `config/vio_calibration.yaml`。棋盘格固定，手持整机平顺移动；避免将棋盘格翻转、急甩或让图像持续模糊。采集、保存与网页使用同一转正方向，约定见[运行库](../../src/stereo_depth/README.md#数据接口)。

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

采集依赖 ROS 2 Jazzy、感知运行包、OpenCV、MAVROS 和 pymavlink。WSL 离线 bag 导出另需 rosbags。安装附加依赖到隔离环境，保留系统 OpenCV/NumPy：

```bash
source companion/ros2_ws/tools/activate_development_env.sh
PYTHONNOUSERSITE=1 python3 -m pip --python "$BOOM_BIRDS_PYTHON" install \
  -r companion/ros2_ws/tools/vio_calibration/requirements.txt
python3 -m tools.vio_calibration.vio_calibration_cli doctor
```

在 WSL 主工程根准备新会话；不覆盖已有目录。会话可整体复制到 Pi：

```bash
export BB_SESSION=/home/waterc/bb_data/vio-calibration/session-01
python3 -m tools.vio_calibration.vio_calibration_cli prepare \
  --out "$BB_SESSION" \
  --calibration companion/ros2_ws/src/stereo_depth/calibration/upright_20261022/candidate.npz \
  --config companion/ros2_ws/tools/vio_calibration/config/vio_calibration.yaml
```

在已加载 ROS、本包及最新 IMU 门控的 Pi 环境中，使用复制后的会话路径执行。当前入口只验证过 /dev/ttyAMA0、115200，不自动改波特率。默认使用既有 USB 双目 by-id；不打开下视相机。

```bash
export ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1
export BB_SESSION=/home/gmaster/boombirds/evidence/vio-calibration-session-01
python3 -m tools.vio_calibration.vio_calibration_cli capture \
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
python3 -m tools.vio_calibration.vio_calibration_cli inspect --session "$BB_SESSION"
python3 -m tools.vio_calibration.vio_calibration_cli bag \
  --session "$BB_SESSION" --out "$BB_SESSION/capture.bag"
python3 -m tools.vio_calibration.vio_calibration_cli solve \
  --session "$BB_SESSION" --imu-noise /absolute/path/to/measured-imu-noise.yaml \
  --out "$BB_SESSION/solve-01"
```

用户明确采用提前中止的数据时，solve 可添加 `--allow-incomplete-capture`，只豁免 recording_completed、no_abort、duration。原始检查和中止事件保留；图像/IMU频率、间隔、时间戳、配对、棋盘和运动检查仍须通过，solver.json 记录豁免项。

solve 需要环境中可运行的 kalibr_calibrate_cameras、kalibr_calibrate_imu_camera；若工具仅通过 rosrun 暴露，传 `--kalibr-prefix "rosrun kalibr"`。本程序不安装或隐式下载 Kalibr。IMU 噪声须为同一采样路径的实测正数、实际频率和测量来源；prepare 生成的 imu-noise.required.yaml 保留空值，不能直接用于求解。

已选候选使用约 50 Hz 的既有数据，并单独记录随机游走工程假设；这是一次明确授权的离线求解，不能将其假设参数写成噪声实测结果。Kalibr 后端、样条调整与求解结果见[日期化记录](../../../../docs/history/STATUS_2026-10.md#2026-10-18-采用已有数据的相机imu-标定候选)，不把该次设置默认为所有采集的求解参数。

原 NPZ 的 k3 非零时，不能截断为 OpenVINS 四项模型。solve 会先用同批双目棋盘格图像拟合 pinhole-radtan，再求解相机—IMU外参与时间差；原 NPZ 保留。输出的 camchain/PDF 须检查棋盘姿态翻转、角点覆盖、重投影、IMU残差和偏置，失败数据不要通过手工改门限变成 PASS。

```bash
python3 -m tools.vio_calibration.vio_calibration_cli candidate \
  --session "$BB_SESSION" --chain "$BB_SESSION/solve-01/capture-camchain-imucam.yaml" \
  --imu-noise /absolute/path/to/measured-imu-noise.yaml \
  --template companion/ros2_ws/src/open_vins/config/euroc_mav/estimator_config.yaml \
  --out "$BB_SESSION/candidate-01"
```

这里只复用模板的估计器选项；相机、外参、时间偏移和 IMU 噪声全部来自本次输入。Kalibr T_cam_imu 取逆写为 T_imu_cam。导出的 NPZ 内参还原到原传感器尺寸，物理基线不缩放。IMU 使用 PX4 已校正的 HIGHRES_IMU；OpenVINS model=calibrated，不重复应用内参校正。单位内参矩阵不代表机体安装外参已测。candidate 当前导出单目 estimator/chain；双目开发还需准备包含 cam0/cam1 的 chain，并设置 max_cameras=2、use_stereo=true。prepare_mono_vio.py 仅用于旧单目算力对照，不能用于生成双目配置。

若没有带测量来源的 T_I_B，不填写单位矩阵，生成 extrinsics.pending.yaml；可通过 `--body-extrinsics` 提供已测文件。所有候选保持 hardware_verified=false，不自动替换 Pi current、旧标定或飞控参数。真实采集、实际求解、独立回放和 VIO 验收分别记录。

数据格式和求解参数依据 [Kalibr bag 格式](https://github.com/ethz-asl/kalibr/wiki/bag-format)、[相机—IMU标定](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)。ROS/HTTP模拟与伪串口验证仅证明程序行为，不能代替实际安装的标定。

## 静止 IMU 噪声

飞控 USB 供电，整机固定，不打开相机。使用同一飞控的 HIGHRES_IMU 六轴 FRD 值及原始 time_usec；采集器只允许既有受限遥测，未解锁/心跳门控保留，结束恢复临时消息间隔。

```bash
python3 -m tools.vio_calibration.imu_noise_capture --out /absolute/path/to/new-noise-session --seconds 600
python3 -m tools.vio_calibration.imu_noise_analysis --csv /absolute/path/to/new-noise-session/imu.csv --out /absolute/path/to/new-analysis
```

分析不插值，不跨采样缺口积分；取最长连续段并排除开头 30 s。analysis.json 记录静止筛查、逐轴拟合区间和前后半段系数差异；allan.csv 保存六轴曲线。白噪声用 -1/2 斜率，偏置随机游走用 +1/2 斜率；未找到可识别区间的参数保持 null。十分钟记录仅作短记录评估，不能保证四项都可识别。四项齐全时输出 imu-noise.candidate.yaml，否则输出 imu-noise.partial.yaml；均不自动替换部署配置。模型与单位依据 [Kalibr IMU Noise Model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)。
