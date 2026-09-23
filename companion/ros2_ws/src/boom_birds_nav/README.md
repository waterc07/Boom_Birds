# boom_birds_nav

Boom_Birds 第一阶段导航链路的 Companion 侧节点集合，当前包含：

- **脱机链路**（文件/合成双目 → 米制深度与完整 XYZ → 位姿/里程计适配 → EGO 规划）：
  仅用于 WSL 脱机开发，不连接真实设备。
- **真实链路第一版**（PX4 MAVLink `HIGHRES_IMU` → `/boom_birds/imu`，含 TIMESYNC 时钟映射）：
  代码完成、脱机测试通过，**真机未验收**（见文末清单）。
- **真实双目采集 → ROS 发布链**（`stereo_source` 的 `v4l2` 模式）：代码完成、回放链脱机通过；
  真实相机曝光时间戳**未在真机核验**。
- **规划输出 → Px4Interface → PX4 高层控制接口**：代码完成、脱机通过；链路/状态读取/`msg 84`
  送达与类型掩码已在 **PX4 SITL（SIH）** 上实测，**位置响应与 offboard failsafe 未触发、未验证**；
  真机未测。

## 节点与模块

| 可执行 / 模块 | 职责 | 关键约定 |
| --- | --- | --- |
| `stereo_source` | **唯一采集源**（`mode`: `v4l2`/`replay`/`file`/`synth`）：发布左右原始图 + 各自 CameraInfo | 全链路只有一个进程能打开相机；同帧左右图共享同一 V4L2 采集时间戳 |
| `stereo_capture` | 帧源抽象：`V4L2FrameSource`（唯一 mmap/V4L2 打开点）与 `ReplayFrameSource`（已保存帧） | 两种帧源给出同样的 `StereoFrame`（含可验证时间戳），发布语义一致 |
| `px4_interface_node` | 规划输出 → 高层 setpoint（`SET_POSITION_TARGET_LOCAL_NED`）的唯一出口 | 只依赖 `Px4Backend` 协议；不发 PWM/DShot/电机指令；停发 ≠ 已悬停 |
| `px4_backend` | `Px4Backend` 协议 + `FakePx4Backend`（脱机确定性）+ `MavlinkPx4Backend`（pymavlink） | 通信后端与算法解耦（SW-001）；默认 `dry_run`、禁解锁、仅回环地址 |
| `px4_frames` | ROS 局部系（Z 上）→ PX4 NED 的坐标/偏航换算与 `type_mask` | 对齐后的轴变换含 `yaw_offset` 与位置平移；`yaw_NED=-(yaw_ROS+yaw_offset)`；掩码与模式必须一致 |
| `px4_failsafe` | 失效判定状态机（INIT/OK/DEGRADED/STOPPED）与迟滞恢复 | `allow_setpoint=False` **只表示本节点停发**，不代表飞控已悬停/接管 |
| `depth_node` | 复用 `stereo_depth` 的 `StereoProcessor` 计算深度与 XYZ | 深度 `32FC1` 米制，无效为 NaN；另发 `16UC1` 毫米/整数 0 兼容话题 |
| `pose_adapter` | OpenVINS 里程计 → 相机位姿 + 机体里程计 + EGO 专用里程计 | `T_W_Crect = T_W_I·T_I_C0·T_C0_Crect`；容差/超时见契约 |
| `vio_source` | **TEST-ONLY** 合成 VIO/IMU 源，供脱机测试 | 不是飞控数据，不得作为精度证据 |
| `mavlink_imu_node` | **真实链路**：接收 PX4 `HIGHRES_IMU` → 校验 → 时钟映射 → `/boom_birds/imu` | 时间戳只来自 `time_usec` 映射；无可靠映射即拒绝发布 |
| `camera_timestamp_probe` | V4L2 采集时间戳能力核验（只读探测，可抓帧观察） | 时域不可核实即报错；不用 OpenCV 取帧返回时刻 |
| `mavlink_imu_replay` | 回放记录 MAVLink 数据并自检（不接设备） | 结论是「回放通过」，不是真机验收 |
| `boom_birds_nav.mavlink_clock` | MAVLink `TIMESYNC` → 偏移估计（可单测） | 偏移 = 飞控启动时钟 − Companion 单调时钟 |
| `boom_birds_nav.timebase` | Companion 单调时钟 ↔ ROS 时间域映射 | 报告偏移与不确定度；检测 ROS 时钟被步进 |
| `boom_birds_nav.mavlink_imu_core` | 校验 / 时间戳映射 / 诊断（纯 Python，无 rclpy） | 拒绝即是拒绝，不做近似时间戳 |
| `boom_birds_nav.camera_timestamp` | V4L2 帧时间戳与同帧左右共享接口 | 曝光时刻 ≠ 取帧时刻 ≠ 发布时间 |

## 契约

话题、坐标系、时间与无效值约定的唯一来源是 [config/contract.yaml](config/contract.yaml)。
要点：

- `T_A_B` 表示「把 B 系坐标变换到 A 系」；`T_I_C0` 即 Kalibr/OpenVINS 的 `T_imu_cam` 字段。
- 深度数组层无效值为 `NaN`；`/boom_birds/depth/image` 保持 `NaN`；兼容话题使用整数 0。
- 位姿与深度配对容差 0.03 s；VIO 位姿超 0.15 s、深度超 1.0 s 未更新即停止发布。
- `/boom_birds/imu` 单位 m/s²、rad/s，机体系 FRD，含重力反作用（静止水平 z ≈ +9.81）；
  `orientation` 按 ROS 约定标为不可用（`orientation_covariance[0] = -1`），不使用飞控融合姿态。
- 四个「时间偏移」名字不同、含义不同，不能混用（契约 timing 节有展开）：
  `clock_offset_s`（TIMESYNC）、`ros_minus_mono_s`、`camera_imu_offset_s = t_cam_ros − t_imu_ros`、
  以及位姿配对容差（不是时钟量）。

## 真实链路：MAVLink IMU 上行（第一版）

```text
PX4 HIGHRES_IMU ──┐
                  ├─► mavlink_imu_core: 来源/字段/时间校验 ──► /boom_birds/imu
PX4 TIMESYNC ◄────┘        ▲
   ▲                       │
   └── TIMESYNC 请求 ──────┘
```

### 依赖

- 运行真实链路需要 `pymavlink`（含 `pyserial`）。本工作空间已把它加入隔离 venv 的搜索路径，
  见 `companion/ros2_ws/tools/setup_python_env.sh`；纯函数测试不需要它（缺失时自动跳过）。

### 运行

```bash
source companion/ros2_ws/tools/activate_python_env.sh
bash companion/ros2_ws/tools/build_all.sh --packages-select boom_birds_nav
source /home/waterc/bb_build/main/install/setup.bash

# 真机（示例值，必须按实际接线与端口核验后修改）
ros2 launch boom_birds_nav mavlink_imu_offline.launch.py \
    connection:=serial:/dev/ttyAMA0 baud:=115200

# 或直接跑节点 + 参数文件
ros2 run boom_birds_nav mavlink_imu_node --ros-args \
    --params-file $(ros2 pkg prefix boom_birds_nav)/share/boom_birds_nav/config/mavlink_imu.yaml

# 脱机联调（不接飞控）：UDP 被动监听，用测试脚本/回放工具发送 MAVLink
ros2 run boom_birds_nav mavlink_imu_node --ros-args -p connection:=udpin:127.0.0.1:14555
```

**不要**同时运行 `vio_source` 与 `mavlink_imu_node`：两者会争抢 `/boom_birds/imu`。

安装后的入口（`ros2 pkg executables boom_birds_nav`）：

| 入口 | 用途 |
| --- | --- |
| `mavlink_imu_node` | 真实 MAVLink IMU 接收节点（`ros2 run` / launch） |
| `camera_timestamp_probe` | V4L2 帧时间戳能力核验（只读探测） |
| `stereo_source` / `depth_node` / `pose_adapter` / `vio_source` | 脱机链路节点 |
| `python3 -m boom_birds_nav.mavlink_imu_replay` | 记录数据回放自检（JSON 报告） |
| `python3 -m boom_birds_nav.camera_timestamp` | 同 `camera_timestamp_probe`（便于传参调试） |
| `python3 -m boom_birds_nav.deep_checks` | 深度单帧自检 |

### 参数

全部参数见 [config/mavlink_imu.yaml](config/mavlink_imu.yaml)。要点：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `connection` | `serial:/dev/ttyAMA0` | `serial:<设备>` / `udpin:ip:port` / `udpout:` / `tcp:`（默认值只是可编辑起点，不是已核验现场值） |
| `baud` | 115200 | 仅 `serial:` 生效 |
| `target_system` / `target_component` | 1 / 1 | PX4 `MAV_SYS_ID` / `MAV_COMP_ID` 默认值 |
| `device_id` | -1 | `HIGHRES_IMU.id`；-1 = 不校验（单 IMU 板卡） |
| `stream_rate_hz` | 50 | 请求频率，**不等于**实际达成频率 |
| `timesync_rate_hz` | 2 | TIMESYNC 请求频率 |
| `max_rtt_s` / `sync_timeout_s` | 0.02 / 1.0 | RTT 上限、映射失效超时 |
| `camera_imu_offset_s` / `apply_camera_imu_offset` | 0.0 / false | 相机—IMU 时间偏移（未标定前必须 false） |
| `imu_topic` | `/boom_birds/imu` | 契约话题，不建议改 |
| `stats_topic` / `diagnostics_topic` | `/boom_birds/imu/mavlink_status` / `.../diagnostics` | JSON 与 diagnostic_msgs 诊断 |

### 时间戳来源与失效行为

1. `HIGHRES_IMU.time_usec` 是 **PX4 启动时钟微秒**（`streams/HIGHRES_IMU.hpp` 里取
   `imu.timestamp_sample`），不是 UNIX 时间，也不是收包时刻。
2. `TIMESYNC` 往返给出偏移 `clock_offset_s = t2 − (t1+t3)/2`，即「飞控启动时钟 − Companion
   单调时钟」；`t1`/`t3` 用同一个 Companion 单调时钟，因此不依赖飞控与 Companion 的绝对同步。
3. `t_mono = time_usec·1e-6 − clock_offset_s`，再经 `RosTimeBase` 映射到 ROS 时间域；
   **串口收包时刻从不参与时间戳**。
4. 以下任一情况**拒绝发布**并计数（不是近似、不是补 0）：
   未建立/已超时的时钟映射、`fields_updated` 缺加速度或角速度位、非有限值、
   `time_usec` 不是启动时钟量级、`time_usec` 或映射后时间戳倒退、时间戳重复、
   采样年龄过大或过分超前、ROS 时间基采样不稳定。
5. 飞控重启（boot 时间大幅倒退）或偏移跳变 → 立即失效并重置，等待重新收敛；
   下游单调性状态同时清空（`timeline_resets`）。
6. 只发 `TIMESYNC` 请求与可选的 `MAV_CMD_SET_MESSAGE_INTERVAL(105)`；
   **不发送控制指令、不做 Offboard/解锁、不回传外部视觉**。

### 诊断

`/boom_birds/imu/mavlink_status`（JSON）与 `/boom_birds/imu/diagnostics`（DiagnosticArray）
包含：`imu_rate_hz`、间隔均值/中位/最大、丢样计数、`time_sync.{locked, offset_s, rtt_last_s,
rtt_median_s, error_bound_s, sample_age_s, samples_accepted, filter_resets, immediate_resets,
px4_restarts}`、各类拒绝计数、`ros_timebase`、`sample_to_publish_delay_s`、heartbeat 状态。

### 115200 不是结论

默认 `baud=115200`、`stream_rate_hz=50` 只是起点。`HIGHRES_IMU` 一帧约 62 字节，加上
`TIMESYNC` 与其它流，115200 是否够用**必须实测**：看 `imu_rate_hz` 是否达到目标、
`interval_max_s` 与 `gaps` 是否可接受；不够时提高波特率或下调请求频率。

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
- `StereoFrameClock` / `decode_stitched`：一个拼接帧只取一个时间戳；拼接格式沿用现有
  采集链（**一整幅拼接 MJPEG**，解码后按宽度对半切，与 `depth_preview.py` 一致），
  **左右共享同一个 `capture_ros_s`**；驱动时间戳或映射后时间不递增即报错。
- `probe_v4l2()` / CLI：部署前核验设备与时间戳能力。

结构体布局按 64 位 Linux 的 `struct v4l2_buffer`（88 字节）实现，
字段偏移由编译期 `offsetof()` 在测试中核对（`test_camera_timestamp.py`），
不靠记忆写偏移。

```bash
# 只读探测设备能力（不排队缓冲）
python3 -m boom_birds_nav.camera_timestamp --probe --device /dev/video0
# 抓几帧观察时间戳时域与 payload
python3 -m boom_birds_nav.camera_timestamp --device /dev/video0 --size 2560x720 --frames 5
# 退出码 0 = TIMESTAMP_TRACEABLE，1 = 时域不可用/未完成，2 = 参数缺失
```

旧的独立 `depth_preview.py` 使用 `cv2.VideoCapture.read()`，不能提供本链路要求的
V4L2 采集时间戳；真实 ROS 发布应使用 `stereo_source` 的 `v4l2` 模式，并在设备上
先通过时间戳时域核验。硬件验收项见本文末清单。

## 运行（脱机链路）

```bash
source companion/ros2_ws/tools/activate_python_env.sh
bash companion/ros2_ws/tools/build_all.sh --packages-select stereo_depth boom_birds_nav
source /home/waterc/bb_build/main/install/setup.bash
ros2 launch boom_birds_nav synthetic_layer2.launch.py
```

EGO 侧由 `ego-planner-swarm` 的 `boom_birds_offline.launch.py` 接入上述话题。

## 测试

```bash
cd companion/ros2_ws/src/boom_birds_nav
python3 -m pytest test -v                          # 全部脱机测试
python3 -m boom_birds_nav.deep_checks --synth      # 深度单帧自检（JSON）
python3 -m boom_birds_nav.mavlink_imu_replay <记录文件> --out report.json   # MAVLink 回放自检
```

最近一次 `boom_birds_nav` 全量脱机测试为 **498 项通过**（见 [当前状态](../../../../docs/STATUS.md)）。下表列出 MAVLink/时间同步/相机时间戳的专项测试；这些测试使用构造的模拟消息、模拟 ioctl 与真实记录帧，不连接设备：

| 测试文件 | 覆盖 |
| --- | --- |
| `test_mavlink_clock.py` | 偏移收敛与符号、RTT 异常、回显错配、超时、飞控重启、偏移跳变确认、时间基 |
| `test_mavlink_imu.py` | 无同步拒发、时间戳来自 `time_usec`、字段/来源/量级校验、单调性、丢样统计、单位与坐标约定；TIMESYNC 配对：PX4 主动请求插入不丢样、只认最新请求的回显、pending 超时、来源校验开关、重复回包 |
| `test_camera_timestamp.py` | `v4l2_buffer` 布局与 `<linux/videodev2.h>` 的编译期 `offsetof()` 逐字段核对（gcc 探针，漂移即失败）、模拟 ioctl 取帧、时域判定、时间倒退拒绝、真实记录帧的拼接切分与 MJPEG 路径 |
| `test_v4l2_driver_sim.py` | 完整 `open → S_FMT → REQBUFS → QBUF → DQBUF → read_raw → close` 流程（假 ioctl + 匿名 fd）、缓冲归还、字段取自正确偏移 |
| `test_time_domain_integration.py` | **相机与 IMU 同一 ROS 时间域**（共用同一 `RosTimeBase`）、真实记录帧按宽度对半切且左右共享同一采集时间戳 |
| `test_mavlink_imu_node.py` | 真实节点构造 + UDP 假飞控：发布、诊断话题、无同步时不发布 |
| `test_mavlink_imu_replay.py` | 合成 `.tlog` 回放：有/无 TIMESYNC 的两种结局、CLI 报告 |
| `test_contract_alignment.py` | 契约话题/单位/QoS/时间参数与节点默认值一致 |

`test/recordings/` 保存真实记录帧的逐字节副本（来源与哈希见该目录 `PROVENANCE.md`）；
这些帧**没有曝光时间戳**（原始 `capture.json`：`host save time, not exposure time`），
因此相关测试只证明拼接格式与切分，不证明曝光时刻。

## 已实测的合成精度边界（2026-09-22，本机 x86_64）

合成场景（3 m 墙 + 1.8 m 障碍，合成标定 TEST-ONLY）：

- 有效像素比例约 0.42（含右边界 99 px 不可测区）；
- 深度误差受 StereoSGBM 的 1/16 px 整数视差量化限制：3 m 处中位误差约 0.078 m；
- 因此**不要求**毫米级，也不得把该结果当作真机距离精度。

真机标定（1280×960、基线 67.6718 mm）对应 320×240 深度图；实际内参由运行时同一标定的 P1 推导并发布为 CameraInfo，地图从该值读取。

## 树莓派/飞控联机后必须验收的项目（当前全部未做）

| # | 验收项 | 通过判据 |
| --- | --- | --- |
| 1 | 串口与飞控 MAVLink 实例 | 设备节点、波特率、电平与共地核验；heartbeat 与 `HIGHRES_IMU` 稳定到达 |
| 2 | 实际 IMU 频率与带宽 | `imu_rate_hz` 达到目标、`interval_max_s`/`gaps` 可接受；否则提高波特率或降频 |
| 3 | TIMESYNC 同步误差 | `rtt_median_s`/`error_bound_s` 记录在案；**不得**把 RTT/2 写成实测同步误差 |
| 4 | 真实相机曝光时间戳 | `camera_timestamp_probe` 报出 `TIMESTAMP_TRACEABLE`，且抓帧时间戳与外部触发/闪光或 IMU 相关峰可对齐 |
| 5 | 相机—IMU 时间偏移标定 | 用真实数据估计 `camera_imu_offset_s` 并写明符号；标定前保持 `apply_camera_imu_offset=false` |
| 6 | OpenVINS 初始化与漂移 | 用同步的真实双目 + 飞控 IMU 数据验收；合成 IMU 不算证据 |
| 7 | 长期运行 | 丢样、时间回退、映射失效计数在长跑中保持 0（或可解释） |


## 真实双目采集 → ROS 发布链

目标：把「真实采集」收敛成**一个入口**，让深度与 OpenVINS 共用同一份左右图，而不是各自打开相机。

```text
/dev/videoN ──► V4L2FrameSource ──┐
                                  ├──► stereo_source ──► /boom_birds/stereo/left_raw                  (sensor_msgs/Image)
已保存的拼接帧 ──► ReplayFrameSource ─┘                     /boom_birds/stereo/left_raw/camera_info    (sensor_msgs/CameraInfo)
                                                            /boom_birds/stereo/right_raw
                                                            /boom_birds/stereo/right_raw/camera_info
                                                            /boom_birds/stereo/stitched
                                                            /boom_birds/stereo/source_status          (std_msgs/String, JSON)
```

**左右 CameraInfo 各自与图像配对**（ROS 惯例：一个相机一个 `camera_info` 话题）。
旧版把左右轮流发在同一个 `/boom_birds/stereo/camera_info` 上，下游只能靠 `frame_id` 猜；
该话题现在**默认关闭**，仅在显式设置 `legacy_combined_camera_info_topic` 时才发布，
供旧订阅者过渡。话题名可由 `left_camera_info_topic`/`right_camera_info_topic` 覆盖，
或由 `camera_info_topic_suffix`（默认 `camera_info`）从图像话题派生。

- **唯一采集点**：只有 `stereo_capture.V4L2FrameSource` 会打开设备；`stereo_source` 是唯一发布者。
  下游（`depth_node`、OpenVINS）只订阅话题，不得再 `cv2.VideoCapture` 打开同一台相机。
- **时间戳**：来自 V4L2 缓冲的 `v4l2_buffer.timestamp`（`V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC`），
  经 `CameraTimestampSource`/`StereoFrameClock` 映射进 ROS 时间域，**同帧左右图共享同一个采集时间戳**。
  取不到可验证时域时**拒绝发布并给出诊断**，绝不回退到 OpenCV 取帧返回时刻或 ROS 发布时刻。
- **回放**：`mode:=replay` 用已保存帧，发布语义与真实采集一致（同一话题、同一左右共享时间戳规则）；
  但记录帧只有保存时间、**没有曝光时间戳**，因此回放**不能**用来证明相机-IMU 同步。
- **标定复用**：`raw_camera_info()` 默认 `raw_info_scale=auto`（= 采集宽 / 标定 `image_size[0]`），
  当前 1280×960 标定配 640×480 采集时 scale=0.5，会缩放 **fx/fy/cx/cy** 并**打印警告**。
  **米制物理基线 `B` 不缩放**：纯降采样不改变刚体量，因此右目 `P[0][3] = -fx_当前分辨率 · B`
  只随 `fx` 变一次。（曾错误地把 `B` 也乘 scale，使 `P[0][3]` 在 scale=0.5 时衰减到 1/4，
  下游三角化尺度整体错掉——已修正，并有 `test/test_camera_info_scaling.py` 锁死。）
  跨分辨率复用内参只在纯降采样下近似成立，真机必须重新标定。
- **不做重复算法**：本节点不含矫正/深度/特征提取；矫正由 `stereo_depth` 提供（`cam0_rect`/`cam1_rect`），
  深度由 `depth_node` 计算。

运行（脱机回放，最常用）：

```bash
source companion/ros2_ws/tools/activate_python_env.sh
bash companion/ros2_ws/tools/build_all.sh --packages-select boom_birds_nav
source /home/waterc/bb_build/main/install/setup.bash

# 回放已保存的真实帧（不打开相机）
ros2 launch boom_birds_nav stereo_camera.launch.py mode:=replay
# 真实采集（需要真机先确认设备号/分辨率/像素格式，切勿盲跑）
ros2 launch boom_birds_nav stereo_camera.launch.py mode:=v4l2 device:=/dev/video0
```

关键参数见 [config/stereo_camera.yaml](config/stereo_camera.yaml)：`mode`、`device`、`width`/`height`、
`fps`、`pixel_format`、`frames_dir`、`calibration_file`、左右/CameraInfo/状态话题名。
`raw_info_scale`、`raw_info_scale_warn` 控制内参缩放与告警。

**状态**：代码完成；回放链脱机通过——
- `test/test_stereo_publish_chain.py` 用**真实 `depth_node` 进程**订阅并实际收到左右图与各自 CameraInfo；
- `test/test_openvins_subscription.py` 用**真实 `run_subscribe_msckf` 进程**（OpenVINS，remap 到我们的
  契约话题）验证它确实订阅了 `/boom_birds/stereo/left_raw`、`/boom_birds/stereo/right_raw`、`/boom_birds/imu`。
  注意这**只证明订阅关系**，不证明 VIO 能初始化：回放帧没有曝光时间戳，也没有同步的真实 IMU。
- OpenVINS 默认话题是 `/cam0/image_raw`、`/cam1/image_raw`、`/imu0`，与契约不同，
  **必须显式 remap 或传参**，漏配时会静默收不到数据（该事实也在测试里锁定）。

真实相机曝光时间戳、V4L2 与曝光之间的偏差、VIO 初始化/漂移 **未在真机核验**。

## 规划输出 → Px4Interface → PX4 高层控制接口

```text
traj_server ──/position_cmd (100 Hz)──► px4_interface_node ──► Px4Backend ──► PX4
   (quadrotor_msgs/PositionCommand)         │  px4_frames: ROS 局部系 → NED + type_mask
                                            │  px4_failsafe: 新鲜度/迟滞/闭锁
                                            └──► /boom_birds/control/status (JSON)
```

- **契约**：输入是 `quadrotor_msgs/PositionCommand`（`frame_id` 必须是 `world`/`global`/`map`，否则按
  失效处理——不做隐式坐标假设）；输出是 PX4 `SET_POSITION_TARGET_LOCAL_NED`（`MAV_FRAME_LOCAL_NED`）。
  位置/速度/加速度单位 m、m/s、m/s²，偏航 `yaw_dot` → `yaw_rate`（字段名不同，**不可当同义词**）。
- **坐标系对齐是位置 setpoint 的前置条件（重要）**：EGO 的 `world` 与 PX4 局部 NED 是**两个**局部系。
  轴翻转只解决"哪个轴朝哪"；**原点与水平朝向不会自动一致**（`world` 的原点由首帧决定、航向任意；
  PX4 的原点/航向由 EKF 决定，视觉融合或 EKF 重置还会改变它）。
  放行位置需要**两项独立证据，缺一不可**：
  1. **水平朝向**：`YawAlignmentResidual` 被动核实 VIO 与 PX4 航向残差（要求接近水平、
     角速率小、样本足够且集中、姿态新鲜且两个消息的本机到达时刻相近），
     或显式 `frame_alignment_observed: true`；到达时差不能证明两个测量时刻同步；
  2. **原点/平移**：`frame_alignment_origin_evidence: true`——外部视觉回传把 EKF 原点定义在
     VIO 原点上，或已用实测数据标定 `frame_alignment_translation_m`。

  **航向核实 ≠ 完整对齐**：两个系可以朝向完全一致而原点相隔很远，因此
  `YawAlignmentResidual` 只暴露 `yaw_verified`，单独达标**不会**放行位置。
  任一项缺失即不下发位置 setpoint（原因码 `local_frame_not_aligned`，状态 `STOPPED`），
  速度/加速度同样不发——只发速度会让位置语义以另一种形式继续误导。
  默认 `frame_alignment: "none"` 时一项证据都不采信；唯一的越权开关
  `unverified_test_only` 只允许 Fake 后端或 MAVLink `dry_run=true`，启动即打 ERROR。
  `frame_alignment=identity` 表示两系同向同原点，若同时配了非零偏移/平移会**直接报错**（自相矛盾）。
  PX4 启动周期变化会使旧对齐证据失效并闭锁位置指令；重新核实航向与原点后需重启本节点。
  不伴随 PX4 重启的 EKF 原点重置目前无可靠上行事件可自动识别，仍须在联机验收中处理。
- **一次完成全部换算**：位置/速度/加速度/偏航/偏航角速率由
  `px4_frames.ros_local_to_ned_setpoint` 统一换算——前三者走同一个 `R(φ) = R_z(−φ)·diag(1,−1,−1)`
  （**平移只作用于位置**），偏航为 `−(yaw+φ)`，偏航角速率为 `−yaw_dot`。
  自洽性有测试锁定：把"以 yaw=ψ 飞行的速度"旋转后，其实际朝向必须等于偏航换算的结果。
- **只发高层指令**：`Px4Backend` 协议没有 PWM/DShot/电机/执行器面（测试对协议与实现都做了字面断言）。
  解锁默认禁止（`allow_arming=false`），且只允许回环地址，串口需显式二次开关。
- **失效处理**：空或未知 `PositionCommand.header.frame_id`、规划拒绝、轨迹失效、
  VIO/IMU/相机断流、MAVLink 链路超时、飞控重启都会停止下发，
  并把状态切到 `STOPPED`/`DEGRADED` 写入状态话题。恢复必须满足迟滞（`recovery_required_samples`），
  且飞控重启后必须看到**新的 boot_id / trajectory_id** 才算重建。
- **重要边界**：`allow_setpoint=false` 的语义只有一句——**本节点停止发送**。它既不代表 PX4 已经悬停，
  也不代表已进入安全接管；飞控侧实际反应取决于 `COM_OF_LOSS_T` / `COM_OBL_RC_ACT`，
  必须在 SITL 与实机分别验证。这句话同时写在 `STOP_NOTE`、日志和状态 JSON 的 `note` 字段里。
- **控制器位置未定**：本节点只做「规划输出 → 高层 setpoint」的适配，不决定控制器跑在 Companion 还是
  PX4 内部；SITL 原型不得被当作架构定论。

运行（默认安全档：假后端、dry_run、不解锁）：

```bash
ros2 launch boom_birds_nav px4_interface.launch.py
```

参数见 [config/px4_interface.yaml](config/px4_interface.yaml)：`backend`(`fake`|`mavlink`)、`connection`、
`dry_run`、`allow_arming`、`connect_on_start`、`control_rate_hz`、`yaw_mode`(`yaw`|`yaw_rate`)、
`send_acceleration`、各信号超时（`setpoint_timeout_s`/`vio_timeout_s`/`heartbeat_timeout_s`/…）、
`backend_heartbeat_timeout_s`、`recovery_required_samples`。

### 在 PX4 SITL 上验证（SITL ≠ 实机）

1. 启动一个**明确标识的 SITL 实例**，只监听回环：用内置 SIH 模型（无需 Gazebo）——
   `PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim PX4_SYS_AUTOSTART=10040`，实例号固定 `-i 0`。
2. **端口选择是有讲究的**：PX4 的 onboard link 会把第一个给它发包的 localhost 地址**锁定**，
   因此被动 `udpin:0.0.0.0:14540` 能收到数据；若用 `udpout:127.0.0.1:14580`，必须先自己发一帧
   才会开始收。改连接方式前先重启 SITL 清掉锁定状态。
3. 用 `backend:=mavlink`、`dry_run:=false`、`allow_arming:=false` 跑节点，观察
   `/boom_birds/control/status` 里的 `connected`/`heartbeat_age_s`/`custom_main_mode`/`mode_name`。
4. **本工程已实测到**：真实 HEARTBEAT 解析、`connect()/read_vehicle_state()`、`msg 84` 确实被 PX4 接收
   （ulog `offboard_control_mode` 与 `type_mask` 位一一对应）、缺 `type_mask` 被拒、`arm()` 被拒、
   心跳超时后 `is_connected()` 翻假、以及 PX4 侧 `custom_mode` 位域的正确解码。
5. **未实测**：位置/姿态响应、offboard failsafe（`COM_OF_LOSS_T=1.0`/`COM_OBL_RC_ACT=0` 的实际动作）、
   真机串口/供电/飞行安全。这些**只能**在明确授权下针对该 SITL 实例 arm/切 OFFBOARD 才能观察到。

### PX4 接口的验证口径

| 项目 | 状态 |
| --- | --- |
| 代码完成（模块 + 节点 + 参数 + launch） | 是 |
| 离线单测/集成（无 ROS 与进程级回环 MAVLink） | 通过（`test_px4_frames.py`、`test_px4_failsafe.py`、`test_px4_backend.py`、`test_px4_interface_integration.py`） |
| PX4 SITL：链路/状态读取/msg 84 送达/type_mask | 通过（SIH，仅回环） |
| PX4 SITL：位置响应、offboard failsafe | **未触发、未验证**（未 arm、未切 OFFBOARD） |
| 左右 CameraInfo 配对发布 | 通过：各自与图像配对的话题；不再靠 `frame_id` 猜左右 |
| 物理基线不随分辨率缩放 | 通过：`test/test_camera_info_scaling.py` 锁定 `P[0][3] = -fx_scaled·B` |
| OpenVINS 订阅契约话题 | 通过（仅订阅关系）：真实 `run_subscribe_msckf` 进程连接左右图与 IMU；**VIO 初始化未验证** |
| 坐标系对齐闸门 | 通过（脱机）：**航向**与**原点**两项证据缺一即拦，位置/速度一个都不发，原因码 `local_frame_not_aligned`，状态 `STOPPED` |
| 完整 setpoint 换算（含非零 yaw_offset） | 通过（脱机）：位置/速度/加速度/偏航/偏航角速率一致性 + 正反变换互逆 + 45 项用例 |
| 姿态缺失/过期不得充当合格样本 | 通过（脱机）：后端不报 roll/pitch/yaw_rate 时保持未核实 |
| 原点/航向的真实对齐 | **未测**：必须在真机或融合后的 EKF 状态上用实测数据确认；未确认前位置 setpoint 保持闭锁 |
| ARM64 构建 | 本环境无法完成（原因已核实）：本包零编译扩展（`*.so`=0、`setup.py` 无 `ext_modules`），故没有「本包自身的交叉构建」；`aarch64-linux-gnu-gcc`/`qemu-aarch64` 均缺失且不允许联网安装。ARM64 验证 = 在 aarch64 上装依赖并跑同一套测试，**必须**在 Pi 5 上做，且与 Pi 5 实时性/驱动行为**不是同一件事** |
| 真机 | **未测** |

### ARM64 / Pi 5 的边界（本环境实测结论）

- **本包是纯 Python 安装**：`companion/ros2_ws/src/boom_birds_nav` 下 `*.so` 数量为 0，
  `setup.py` 没有 `ext_modules`。因此并不存在"给这个包做 ARM64 交叉编译"这一步；
  真正要做的是**在 aarch64 上安装依赖并跑同一套测试**。
- **本环境做不了**：`aarch64-linux-gnu-gcc`、`aarch64-linux-gnu-cpp`、`qemu-aarch64(-static)`
  均不存在，且本任务不允许联网安装工具链/镜像。所以 STATUS 里 ARM64 一栏仍是 NOT RUN。
- **已知的架构相关点只有一处**：`camera_timestamp` 依赖 64 位 `struct v4l2_buffer` 的手写偏移。
  仓库自带一个用 `gcc` 编译、以 `offsetof()` 逐字段核对的测试
  （`test/test_camera_timestamp.py::test_v4l2_buffer_layout_matches_c_header`，无 gcc 时跳过）。
  在 64 位 aarch64 上可以直接用它判定；这**不能**替代 Pi 5 上的 V4L2 驱动行为、
  USB 带宽、时间戳时域与实时性验收。
- 因此：**ARM64 构建未通过 != 两条链路未完成**；反过来，x86_64 全绿也**不构成** Pi 5 结论。
