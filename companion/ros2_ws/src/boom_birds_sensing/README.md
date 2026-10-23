# 感知包

导航主线使用同帧左右目图像与飞控 IMU 输入双目 OpenVINS；稠密深度独立使用这对图像。硬件入口、双目配置要求及旧单目算力对照见 [bringup](../boom_birds_bringup/README.md)。算法与标定见 [stereo_depth](../stereo_depth/README.md)。

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

原图缩放使用 `camera_geometry.py`，图像线程使用 `frame_worker.py`；合成板图渲染位于独立回放工具。

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

默认配置选择、Pi 部署与验收分别见 [STATUS 状态表](../../../../docs/STATUS.md#实现默认入口部署与验收)；目录名含 stereo 不表示估计器已启用双目。双目开发须显式选择 max_cameras=2、use_stereo=true 的 estimator 和双目 chain，并核对两目方向、内参与时间偏移；不能直接复用 10-21 旧方向测试配置接入转正图像。

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

- IMU 入口默认 `hold_last_sync_on_rtt=true`：单次 RTT 超过 20 ms 时丢弃该同步样本，已收敛映射最多保留到最后一次合格同步后的 1 s；异常样本不更新偏移或延长有效期。失联、飞控重启、ROS 时钟跳变、过期或偏移未收敛仍阻断输出；映射过期后须重新收敛。控制姿态同步行为见[控制包](../boom_birds_control/README.md#配置)。
- `max_future_wait_s` 默认为 0.02，允许范围 0～0.02 s。映射时间略晚于接收时刻的 IMU 可在最多 64 条的队列中等待，由 5 ms 定时器重检；等待预算另加ROS 回调调度延迟；不改采样时间戳。到期仍在未来的样本按原门控拒绝，0 可关闭等待。该等待不证明时钟映射精度或相机—IMU时间标定通过。
- `/boom_birds/imu/mavlink_status` 包含 `imu_received`、`imu_sensor_rate_hz`、`imu_published`、`imu_rejection_counts`、`sync_rejection_counts`、`last_imu_age_s`、`last_good_sync_age_s`、`imu_future_buffered` 和 `imu_pending`。原始有效采样的频率按最近最多 200 条 PX4 采样时间计算；同步、心跳准入和接收节点调度仍须单独核验。


## 标定与回放工具

相机—IMU标定、静止 IMU 噪声采集与分析见 [VIO 标定工具](../../tools/vio_calibration/README.md)。图像清单、rosbag 与 MAVLink 离线回放见 [传感器回放工具](../../tools/sensor_replay/README.md)。这些工具不随感知运行包安装。

## 验证

在主工程根构建后执行 `bash companion/ros2_ws/tools/check_offline.sh`；非默认构建前缀须设置对应 `BUILD_BASE`、`INSTALL_BASE` 和 `OV_INSTALL`。该检查不连接设备，不验收真实 VIO、Pi 全链负载或飞行。当前结果见 [STATUS](../../../../docs/STATUS.md)。

深度实验参数（定点映射、缓冲、LUT/native、Census、位姿先验、ELAS/RKNN）和分档回放见 [深度实验与回放说明](../../tools/depth_benchmark/README.md)。实验参数默认关闭，实机主线配置未切换。


## 采集时间戳实现与边界

`stereo_source` 的 `v4l2`/`replay` 模式已将「取帧 + 驱动时间戳判定 + 同帧左右共享 +
ROS 时域映射」接入左右图与 CameraInfo 发布路径，脱机回放测试通过。
**真实曝光时刻及与飞控 IMU 的同步未经验证**：记录帧的 `capture.json` 写的是
`host save time, not exposure time`；驱动时间戳与曝光中点的偏差还需用真实数据标定。

真实相机采集时间戳的来源是 V4L2 `VIDIOC_DQBUF` 返回的 `v4l2_buffer.timestamp`。
`camera_timestamp.py` 提供：

- `CameraTimestampSource`：直接 ioctl 取帧，返回**驱动时域**时间戳；
  只接受可核实时域（`MONOTONIC` 直接用；`REALTIME` 需显式允许并检查偏移不确定度；
  其它一律 `CameraTimestampError`）。
- `StereoFrameClock` / `decodeStitched`：一个拼接帧只取一个时间戳；拼接格式沿用现有
  采集链（**一整幅拼接 MJPEG**，解码后按宽度对半切，与 `depth_preview.py` 一致），
  **左右共享同一个 `capture_ros_s`**；驱动时间戳或映射后时间不递增即报错。
- `probeV4l2()` / CLI：部署前核验设备与时间戳能力。

结构体布局按 64 位 Linux 的 `struct v4l2_buffer`（88 字节）实现，
字段偏移由编译期 `offsetof()` 在测试中核对（`test_camera_timestamp.py`），
不靠记忆写偏移。

```bash
# 只读探测设备能力（不排队缓冲）
python3 -m boom_birds_sensing.camera_timestamp --probe --device /dev/video0
# 抓几帧观察时间戳时域与 payload
python3 -m boom_birds_sensing.camera_timestamp --device /dev/video0 --size 2560x960 --frames 5
# 退出码 0 = TIMESTAMP_TRACEABLE，1 = 时域不可用/未完成，2 = 参数缺失
```

旧的独立 `depth_preview.py` 使用 `cv2.VideoCapture.read()`，不能提供本链路要求的
V4L2 采集时间戳；真实 ROS 发布应使用 `stereo_source` 的 `v4l2` 模式，并在设备上
先通过时间戳时域核验。硬件验收范围见 [STATUS](../../../../docs/STATUS.md)。



## 双目 ROS 发布与回放

stereo_source 或 shared_stereo_depth 统一持有相机；其他节点只订阅。同帧左右图与各自 CameraInfo 共用采集时间戳。旧混合 /boom_birds/stereo/camera_info 默认关闭，仅供显式启用的兼容订阅者使用。

内参随图像尺寸缩放，物理基线不缩放；右目 P[0][3] = -fx_scaled·B。缩放不是重新标定，跨分辨率复用须核对实际成像模式。原图方向与标定必须成套转换，接口见[运行库](../stereo_depth/README.md#数据接口)。

~~~bash
source companion/ros2_ws/tools/activate_python_env.sh
source /home/waterc/bb_build/main/install/setup.bash
ros2 launch boom_birds_bringup stereo_camera.launch.py mode:=replay
~~~

frames_dir、尺寸、标定及话题参数见 [stereo_camera.yaml](config/stereo_camera.yaml)。replay 不打开相机；真实 v4l2 模式须另行核对设备号、像素格式和时间戳时域。旧记录帧只有保存时间时，不作为曝光同步证据。OpenVINS 的默认话题与项目话题不同，须在 estimator/chain 中显式对应。

MAVROS 是唯一 FCU 连接；IMU 节点不接受旧 connection/baud，端口由 MAVROS fcu_url 管理。vio_source 与真实 IMU 节点不得同时发布 /boom_birds/imu。
