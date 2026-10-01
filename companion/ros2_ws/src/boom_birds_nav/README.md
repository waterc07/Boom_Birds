# boom_birds_nav

本包保留模块与 launch 兼容转发，无可执行文件。实现分别位于 sensing、control、bringup、sim；下表按功能索引。

- **脱机链路**（文件/合成双目 → 米制深度与完整 XYZ → 位姿/里程计适配 → EGO 规划）：
  仅用于 WSL 脱机开发，不连接真实设备。
- **真实链路第一版**（PX4 MAVLink `HIGHRES_IMU` → `/boom_birds/imu`，含 TIMESYNC 时钟映射）：
  代码完成、脱机测试通过，**真机未验收**（见文末清单）。
- **真实双目采集 → ROS 发布链**（`stereo_source` 的 `v4l2` 模式）：代码完成、回放链脱机通过；
  真实相机曝光时间戳**未在真机核验**。
- **规划输出 → Px4Interface → PX4 高层控制接口**：代码完成、脱机通过；链路/状态读取/`msg 84`
  送达与类型掩码已在 **PX4 SITL（SIH）** 上实测。测试用合成双目与 PX4 真值回读的短距离
  位置响应已验证；失败试飞中观察到 Offboard 信号丢失后进入 Land；本机故障矩阵结果见 [STATUS](../../../../docs/STATUS.md)；真机未验收。

## 节点与模块

| 可执行 / 模块 | 职责 | 关键约定 |
| --- | --- | --- |
| `stereo_source` | **唯一采集源**（`mode`: `v4l2`/`replay`/`file`）：发布左右原始图 + 各自 CameraInfo | 全链路只有一个进程能打开相机；同帧左右图共享同一 V4L2 采集时间戳 |
| `synthetic_stereo_source`（sim） | TEST-ONLY 合成双目；生产采集入口不接受 synth | 复用采集发布基类，渲染器仅在 sim 包 |
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

话题、坐标系、时间与无效值见 [config/contract.yaml](../boom_birds_interfaces/config/contract.yaml)：

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
ros2 run boom_birds_sensing mavlink_imu_node --ros-args \
    --params-file $(ros2 pkg prefix boom_birds_sensing)/share/boom_birds_sensing/config/mavlink_imu.yaml

# 脱机联调（不接飞控）：UDP 被动监听，用测试脚本/回放工具发送 MAVLink
ros2 run boom_birds_sensing mavlink_imu_node --ros-args -p connection:=udpin:127.0.0.1:14555
```

`vio_source` 与 `mavlink_imu_node` 都发布 `/boom_birds/imu`，不能同时运行。

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

全部参数见 [config/mavlink_imu.yaml](../boom_birds_sensing/config/mavlink_imu.yaml)。要点：

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

脱机测试结果见 [当前状态](../../../../docs/STATUS.md)。下表列出 MAVLink/时间同步/相机时间戳的专项测试；这些测试使用构造的模拟消息、模拟 ioctl 与真实记录帧，不连接设备：

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

真机标定（1280×960、基线 67.6718 mm）对应 320×240 深度图；实际内参由运行时同一标定的 P1 推导并发布为 CameraInfo，分段 SIH 启动入口读取该 CameraInfo，其他 EGO 启动入口须显式提供匹配的内参。

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

`stereo_source` 发布左右图，`depth_node` 与 OpenVINS 订阅同一组话题。

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

关键参数见 [config/stereo_camera.yaml](../boom_birds_sensing/config/stereo_camera.yaml)：`mode`、`device`、`width`/`height`、
`fps`、`pixel_format`、`frames_dir`、`calibration_file`、左右/CameraInfo/状态话题名。
`raw_info_scale`、`raw_info_scale_warn` 控制内参缩放与告警。

回放链脱机测试：

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

参数见 [config/px4_interface.yaml](../boom_birds_control/config/px4_interface.yaml)：`backend`(`fake`|`mavlink`)、`connection`、
`dry_run`、`allow_arming`、`connect_on_start`、`control_rate_hz`、`yaw_mode`(`yaw`|`yaw_rate`)、
`send_acceleration`、各信号超时（`setpoint_timeout_s`/`vio_timeout_s`/`heartbeat_timeout_s`/…）、
`backend_heartbeat_timeout_s`、`recovery_required_samples`。

### 在 PX4 SITL 上验证（SITL ≠ 实机）

#### 前台运动仿真（TEST-ONLY）

2026-09-24 新增独立的 SIH 运动演示：PX4 的 LOCAL_POSITION_NED/ATTITUDE 回读为
仿真真值；同一相机位姿驱动合成左右图的射线投影，真实 `depth_node` 算深度，
EGO 建图和规划，`px4_interface_node` 向 PX4 SIH 下发高层位置 setpoint。
RViz 配置同时显示彩色深度预览、占据点云、目标、规划 Marker、机体里程计和实际路径。
SIH 场景的 `/boom_birds/depth/image` 为 240×180 米制 `32FC1`，EGO 内参也按 0.75 倍同步缩放；
`/boom_birds/depth/color_preview` 是固定量程的 `bgr8` 伪彩色图，黑色表示无有效深度，仅供观看。
RViz 的 Image 请选彩色预览话题；直接看米制深度会显示灰度。
`/boom_birds/stereo/right_raw` 是合成的 `mono8` 匹配纹理，随机斑点用于 SGBM 视差计算，
不代表真实相机拍摄的森林画面。原始双目到深度图的计算链保持不变。
真值里程计与测试 IMU 是**仿真替身**，不代表 OpenVINS 初始化或真实传感器已通过。
本链只允许本机回环 `-i 0`；真机仍使用默认禁止解锁和未核实坐标系的配置。

按以下顺序在 **PowerShell** 打开可见的 Windows Terminal 标签；不要在已进入
Linux 的提示符里执行 `wsl`。路径中的脚本都在 WSL 主工程。

```powershell
wt new-tab --title "PX4 SIH" wsl -d Ubuntu-24.04 -- bash /home/waterc/workspace/Boom_Birds/companion/ros2_ws/tools/run_px4_sih_visible.sh
```

等 PX4 标签显示启动成功，再在 PowerShell 中打开可见的起飞参数标签；它从
`RuntimeConfig` 下发 PX4 起飞参数（`boom_birds_bringup.sih_params`）并显示回读值，
解锁/起飞/切模式都由编排器执行：

```powershell
wt new-tab --title "SIH 起飞与高度" wsl -d Ubuntu-24.04 -- bash /home/waterc/workspace/Boom_Birds/companion/ros2_ws/tools/run_px4_sitl_takeoff_visible.sh
```

然后在 PowerShell 中依次打开：

```powershell
wt new-tab --title "SITL OFFBOARD" wsl -d Ubuntu-24.04 -- bash /home/waterc/workspace/Boom_Birds/companion/ros2_ws/tools/run_px4_sitl_offboard_visible.sh
wt new-tab --title "Boom Birds ROS" wsl -d Ubuntu-24.04 -- bash /home/waterc/workspace/Boom_Birds/companion/ros2_ws/tools/run_px4_sitl_motion_visible.sh goal_x:=1.0 goal_y:=0.0 goal_z:=2.5
wt new-tab --title "Boom Birds RViz2" wsl -d Ubuntu-24.04 -- bash /home/waterc/workspace/Boom_Birds/companion/ros2_ws/tools/run_px4_sitl_rviz_visible.sh
```

`run_px4_sitl_offboard_visible.sh` 不再判高度、也不切模式：它只显示 PX4 回读，并等待
编排器（`lifecycle_node`）把任务推进到 `EXECUTING`；模式请求经 PX4 接口服务下发。
因此它要求编排链（`px4_sih_mission.launch.py` 或等价的任务栈）已在运行。最终仍要看
`commander status`、`/boom_birds/control/status` 与实际
`/boom_birds/sitl/odom`；仅发出命令不算飞行成功。

TEST-ONLY 统一入口（本轮新增，尚未跑完整 SIH）：先
`ros2 launch boom_birds_nav px4_sih_mission.launch.py scene:=local`，再
`python3 -m boom_birds_nav.lifecycle_cli start --goal X Y Z`；用
`python3 -m boom_birds_nav.lifecycle_cli status --wait-state HOLD_READY` 查看任务状态。
该 launch 只对 SIH 显式打开 `recovery_enabled`，实机入口保持 `RuntimeConfig` 默认关闭。

RViz 的 Fixed Frame 为 `global`；绿色线是 `/boom_birds/sitl/path`，
显示 PX4 回读的实际路径。演示结束先在 PX4 标签输入 `commander land`，
确认 Disarmed 后再用 Ctrl+C 停止 ROS/PX4。

此场景只核验短距离运动与路径显示。合成世界为一面墙和一个平面障碍；
广范围绕障、真实曝光时间、OpenVINS、真实米制精度和失联后的 PX4 动作
均未由该演示证明。

#### 使用 EGO mockamap 复杂场景，保留合成双目链

同一 SIH 链现在可用 EGO 仓库的 `mockamap` 随机场景（固定 seed=511、20×20×4 m、
0.2 m 点云间距、25 个障碍）。世界点云仅作为**测试场景真值**：随 PX4 回读的相机
位姿投影、遮挡取近点后生成合成左右图，仍由原有 `depth_node` 计算深度，EGO 仍从
`/boom_birds/depth/image` 与相机位姿建立占据地图。RViz 中蓝灰色“场景真值（仅仿真）”
与红色“膨胀障碍物”分别表示完整场景和 EGO 实际重建结果。

首次使用前在 WSL 主工程构建这两个包：

```bash
bash companion/ros2_ws/tools/build_all.sh --packages-select mockamap boom_birds_nav ego_planner
```

**严格按顺序**在 PowerShell 启动上文的 `PX4 SIH`，用 `SIH 起飞与高度` 标签从 `RuntimeConfig` 下发 PX4 参数（起飞高度、失效动作都在那里定义），再让编排链起飞并确认处于 Hold。长距离目标须使用分段启动：先运行双目/深度/悬停设定点链，再切 OFFBOARD（切模式入口已迁到编排器，不再由脚本调 `px4-commander`），最后启动 EGO 目标规划。这样 EGO 轨迹在 PX4 真正能够执行时才开始计时。三个运行窗口在结束后都要关闭。

```powershell
wt new-tab --title "Boom Birds mockamap hold" wsl -d Ubuntu-24.04 -- bash /home/waterc/workspace/Boom_Birds/companion/ros2_ws/tools/run_px4_sitl_mockamap_visible.sh bootstrap_only:=true
wt new-tab --title "Boom Birds RViz2" wsl -d Ubuntu-24.04 -- bash /home/waterc/workspace/Boom_Birds/companion/ros2_ws/tools/run_px4_sitl_rviz_visible.sh
wt new-tab --title "SITL OFFBOARD" wsl -d Ubuntu-24.04 -- bash /home/waterc/workspace/Boom_Birds/companion/ros2_ws/tools/run_px4_sitl_offboard_visible.sh
# 等 OFFBOARD 标签确认 navigation mode: Offboard，再启动目标规划
wt new-tab --title "Boom Birds EGO 5m" wsl -d Ubuntu-24.04 -- bash /home/waterc/workspace/Boom_Birds/companion/ros2_ws/tools/run_px4_sitl_mockamap_goal_visible.sh goal_x:=5.0 goal_y:=1.0 goal_z:=2.5
```

接管距离与速度连续性由编排器按 `handoff_max_distance_m` / `handoff_max_speed_m_s` 判定（`run_px4_sitl_offboard_visible.sh` 不再自带距离判定），拒绝跳入已播放过的轨迹。悬停中继只在 `bootstrap_only:=true` 的 TEST-ONLY SIH 链启用，EGO 首个指令靠近当前位置后才接管。此模式要求 SIH 回读相机高度至少达到 `RuntimeConfig.image_publish_min_altitude_agl_m`（由 launch 取 `synth_min_altitude_m` 默认值，不再由脚本写死）；低于门槛、`mockamap` 未到、位姿过期或地图格式无效时停止发布双目帧。

**2026-09-27 单次实测**：固定 `seed=511`、目标 `(5.0, 1.0, 2.5) m`，PX4 巡航轨迹对 mockamap 原始点云的最小距离 `0.719 m`，设定点 `0.671 m`，最大侧向绕行约 `0.906 m`，终点误差约 `0.063 m`；降落后 Disarmed。旧启动顺序的同一 5 m 目标仅 `0.064 m`，判为失败。证据保存在本机 Git 忽略目录 `companion/ros2_ws/log/mockamap_long_20260927/`。这些结果不证明其他场景或真机安全性。演示结束后在 PX4 SIH 中 `commander land`，确认 Disarmed，再关闭 EGO、双目链、RViz 与 PX4 窗口。

#### random_forest 完整链单次复测（TEST-ONLY）

沿上述分段顺序：PX4 SIH 起飞并稳定高于锁点高度后，先运行双目/深度/悬停链，切入 OFFBOARD 后再启动目标规划；RViz 使用同一 `run_px4_sitl_rviz_visible.sh`。森林链的启动命令为：

```powershell
wt new-tab --title "Boom Birds forest hold" wsl -d Ubuntu-24.04 -- bash /home/waterc/workspace/Boom_Birds/companion/ros2_ws/tools/run_px4_sitl_motion_visible.sh use_random_forest:=true synth_map_topic:=/boom_birds/sitl/map synth_map_resolution_m:=0.1 bootstrap_only:=true
wt new-tab --title "Boom Birds EGO forest" wsl -d Ubuntu-24.04 -- bash /home/waterc/workspace/Boom_Birds/companion/ros2_ws/tools/run_px4_sitl_mockamap_goal_visible.sh goal_x:=4.0 goal_y:=-3.0 goal_z:=2.5
```

`synth_min_altitude_m` 不再写在命令里：它默认取 `RuntimeConfig.image_publish_min_altitude_agl_m`。

第二条命令只在 PX4 确认 `navigation mode: Offboard` 后运行。2026-09-27 的一次本机复测中，原始森林点云约 299,231 点；PX4 巡航轨迹对该点云最近 `0.536 m`，同起点到目标的直线基准最近 `0.180 m`，目标最近误差 `0.014 m`；飞行期间无 failsafe 记录，最后降落 Disarmed。证据保存在 Git 忽略目录 `companion/ros2_ws/log/forest_sih_20260927/`。距离是机体中心到点云的最近距离，未扣除机体外廓、点云采样间隙与定位误差；这只证明固定森林参数下的一次仿真，不证明多 seed、真机 VIO 或控制安全。结束后关闭已退出的终端标签，避免窗口堆积。

#### random_forest 30 m 起终点（TEST-ONLY）

起点 `(-15,0,0.1) m` 是规划世界坐标：`ego_reference_scene:=true` 把 PX4 SIH 局部原点映射到该点，下行 setpoint 使用同一平移。终点 `(15,0,1.0) m`。2026-09-28 使用 `seed=3`、20 柱体、20 环体，仍走合成双目→`depth_node`→EGO→PX4 SIH。首次运行先在 WSL 主工程构建：

```bash
cd /home/waterc/workspace/Boom_Birds
bash companion/ros2_ws/tools/build_all.sh --packages-select map_generator boom_birds_nav ego_planner
```

随后在 PowerShell 按上文命令启动 `PX4 SIH` 和 `SIH 起飞与高度` 标签。起飞高度取自 `RuntimeConfig.takeoff_altitude_agl_m`，PX4 进入 Hold 后，再在可见终端运行双目/深度/悬停链及 RViz：

```powershell
wt new-tab --title "Boom Birds 30m hold" wsl -d Ubuntu-24.04 -- bash /home/waterc/workspace/Boom_Birds/companion/ros2_ws/tools/run_px4_sitl_motion_visible.sh use_random_forest:=true synth_map_topic:=/boom_birds/sitl/map synth_map_resolution_m:=0.1 bootstrap_only:=true ego_reference_scene:=true forest_seed:=3 forest_obs_num:=20 forest_circle_num:=20 forest_x_size:=40.0 forest_center_x:=0.0
wt new-tab --title "Boom Birds RViz" wsl -d Ubuntu-24.04 -- bash /home/waterc/workspace/Boom_Birds/companion/ros2_ws/tools/run_px4_sitl_rviz_visible.sh
```

核对 `/boom_birds/vio/odom_ego` 的悬停位置约为 `(-15,0,1.6) m`，再等编排器接管；场景原点由 `scene:=forest_30m`（launch/编排器参数）选择，脚本不再读 `BB_SITL_SCENE`：

```powershell
wt new-tab --title "Boom Birds OFFBOARD" wsl -d Ubuntu-24.04 -- bash -lc 'BB_SITL_CLOSE_ON_DONE=1 bash /home/waterc/workspace/Boom_Birds/companion/ros2_ws/tools/run_px4_sitl_offboard_visible.sh'
```

确认编排器已进入 `EXECUTING`（脚本会打印 PX4 回读）后再启动目标规划：

```powershell
wt new-tab --title "Boom Birds EGO 30m" wsl -d Ubuntu-24.04 -- bash /home/waterc/workspace/Boom_Birds/companion/ros2_ws/tools/run_px4_sitl_mockamap_goal_visible.sh goal_x:=15.0 goal_y:=0.0 goal_z:=1.0
```

合成图像最低发布高度取自 `RuntimeConfig.image_publish_min_altitude_agl_m`（当前 `0.3 m`）；目标高 `1.0 m`，沿用旧 `1.3 m` 门槛会在下降时断流。一次试飞的 PX4 巡航中心距原始森林点云最近 `0.508 m`，直线基准 `0.041 m`，目标最近误差 `0.004 m`，failsafe 0 次，降落 Disarmed。证据在本机 Git 忽略目录 `companion/ros2_ws/log/forest_30m_seed3_20260928/`。距离未扣机体外廓或点云采样误差；仅为固定 seed 的 SIH 仿真。降落并确认 Disarmed 后关闭 EGO、双目链、RViz、PX4 标签，运行结束的终端立即关闭。

#### PX4 SITL 接口验证记录

1. 启动一个**明确标识的 SITL 实例**，只监听回环：用内置 SIH 模型（无需 Gazebo）——
   `PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim PX4_SYS_AUTOSTART=10040`，实例号固定 `-i 0`。
2. **端口选择**：PX4 的 onboard link 会把第一个给它发包的 localhost 地址**锁定**，
   因此被动 `udpin:0.0.0.0:14540` 能收到数据；若用 `udpout:127.0.0.1:14580`，必须先自己发一帧
   才会开始收。改连接方式前先重启 SITL 清掉锁定状态。
3. 用 `backend:=mavlink`、`dry_run:=false`、`allow_arming:=false` 跑节点，观察
   `/boom_birds/control/status` 里的 `connected`/`heartbeat_age_s`/`custom_main_mode`/`mode_name`。
4. **SIH 实测**：真实 HEARTBEAT 解析、`connect()/read_vehicle_state()`、`msg 84` 确实被 PX4 接收
   （ulog `offboard_control_mode` 与 `type_mask` 位一一对应）、缺 `type_mask` 被拒、`arm()` 被拒、
   心跳超时后 `is_connected()` 翻假、以及 PX4 侧 `custom_mode` 位域的正确解码。
5. **SIH 运动实测**：已在本机 `-i 0` 解锁并切 OFFBOARD；用 PX4 回读位置确认
   从约 `x=0.04 m` 到 `x=0.97 m`（目标 `x=1.0 m`），落地后确认 Disarmed。
   这只覆盖测试用合成双目和 PX4 EKF 真值的短距离轨迹。
   失败试飞中观察到 Offboard 信号丢失后 Land；主动断流与恢复未验收。真机串口/供电/飞行安全未测。

### PX4 接口的验证口径

| 项目 | 状态 |
| --- | --- |
| 代码完成（模块 + 节点 + 参数 + launch） | 是 |
| 离线单测/集成（无 ROS 与进程级回环 MAVLink） | 通过（`test_px4_frames.py`、`test_px4_failsafe.py`、`test_px4_backend.py`、`test_px4_interface_integration.py`） |
| PX4 SITL：链路/状态读取/msg 84 送达/type_mask | 通过（SIH，仅回环） |
| PX4 SITL：短距离位置响应 | 通过（SIH 测试链，约 `x=0.04→0.97 m`；目标 `x=1.0 m`，仿真真值） |
| PX4 SITL：offboard 失联后的 failsafe | 失败试飞中观察到 Offboard 信号丢失后 Land；主动断流、恢复及重复接管仍未验收 |
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

## SIH 启动边界与参数来源

| 阶段 | 检查与动作 | 未满足时 |
| --- | --- | --- |
| 起飞参数 | 脚本经 `boom_birds_bringup.sih_params` 从 `RuntimeConfig` 下发 PX4 参数 | 参数二进制缺失或任一条失败即非零退出 |
| 起飞/锁点 | 编排器（`lifecycle_node`）判定高度与稳定窗口（`stable_*`），请求解锁/起飞 | 未达条件不进入后续；不证明稳定悬停 |
| 起点设定点 | 编排器按 `hold_lock_min_altitude_agl_m` 与稳定窗口锁定位置 | 未锁定前不发悬停点 |
| 切 OFFBOARD | 编排器经 PX4 接口服务请求并回读确认 | 未确认按 `mode_timeout_s` 闭锁 |
| 激活 EGO 目标 | EGO 可提前启动；编排器确认 Offboard 后才发 PlannerRequest | 缺匹配 CameraInfo、地图或 Offboard 确认时不规划 |
| 轨迹接管 | 编排器检查首条轨迹指令的距离与速度连续性（`handoff_max_distance_m` / `handoff_max_speed_m_s`） | 保持起点设定点，等待可接管指令 |
| 控制失效 | `px4_failsafe` 与坐标对齐闸门决定是否停发 | PX4 后续动作仍须单独验收 |

shell 不承担高度/模式判定：`run_px4_sitl_takeoff_visible.sh` 只下发经校验的 PX4 参数并显示
回读值，`run_px4_sitl_offboard_visible.sh` 只等编排器状态，两个脚本都不再用 `px4-commander`
切模式、也不用 NED z 判高度；接管距离判定由编排器与共享判定函数完成，避免两处各判一套。
解锁/起飞/切 OFFBOARD 的统一入口是 `px4_sih_mission.launch.py` + `/boom_birds/mission`
（`python3 -m boom_birds_nav.lifecycle_cli start --goal X Y Z`）。

`px4_sitl_motion.launch.py` 的 `output_scale` 默认 0.75；深度与内置 EGO 由同一标定和
实际输出尺寸生成内参。分段 `*_goal_visible.sh` 从 `/boom_birds/depth/camera_info` 获取
P 矩阵，不接受 `fx/fy/cx/cy` 覆盖；10 s 内没有新鲜有效消息则拒绝启动。
这是启动时快照，运行中换标定或改变尺寸必须重启规划链，不支持动态重配置。
独立调用 EGO 子模块原始 launch 时仍由调用方保证内参一致。

运行脚本按自身位置定位主工程；`INSTALL_BASE` 覆盖 ROS 安装根，`PX4_SOURCE` / `PX4_BUILD`
覆盖本机 SIH 源码/构建位置。坐标原点由 `runtime_config.SCENES` 提供，launch 使用
`scene:=local|forest_30m`，只适用于当前零航向偏移的 SIH 配置；
修改场景原点须同时核对真值源、控制对齐和脚本配置。

验收命令见 [工作空间入口](../../README.md#架构修整后的脱机回归入口)。

### 运行配置与深度算法边界

`boom_birds_control.runtime_config.RuntimeConfig` 是共享运行参数来源：`boom_birds_control/config/runtime.yaml`
是它的模板（值与代码默认值必须逐一相等），节点参数文件、launch 与脚本只能引用同一数值。
`test/test_config_single_source.py` 逐项比对（runtime.yaml ↔ 代码默认值、
节点默认值 ↔ `RuntimeConfig`（共享字段不在节点模板重复）、contract.yaml 只留语义、launch/tools 无硬编码高度、
package.xml 依赖齐全）。`boom_birds_interfaces/config/contract.yaml` 记录消息、单位、坐标系与无效值语义，
**不写数值**，也不作为 ROS 参数文件加载。

高度三个量各自独立、不能因数值相同而合并（参考系见 `ALTITUDE_REFERENCE`）：
`takeoff_altitude_agl_m`（PX4 起飞参数，参数名由 `px4_takeoff_param_name` 提供，经
`boom_birds_bringup.sih_params` 下发）、`hold_lock_min_altitude_agl_m`（起点锁定的相对高度下限）、
`image_publish_min_altitude_agl_m`（合成输入发布下限，`px4_sitl_motion.launch.py` 的
`synth_min_altitude_m` 默认取它）。这些是 SIH 初始值，不是实机阈值；自动恢复
`recovery_enabled` 实机入口保持 `false`，只有 `px4_sih_mission.launch.py` 显式打开。

深度算法在 `stereo_depth.core.StereoProcessor`；`process_image(stitched_bgr)` 返回校正左右图、视差、米制 XYZ、有效掩码和耗时。
`depth_preview.StereoProcessor` 兼容旧 CLI；ROS 不访问 SGBM matcher。

项目 EGO 入口默认 `use_camera_info:=true`，内参取同步 CameraInfo，禁止同时指定 fx/fy/cx/cy。
未取得匹配图像尺寸、光学帧及时间的 CameraInfo 时不融合、不规划；内参、尺寸或光学帧变化使地图和轨迹闭锁，需重启地图/规划进程。
独立静态模式须显式传 `use_camera_info:=false fx:=... fy:=... cx:=... cy:=...`，不会回退到静态内参。

### 包拆分进度（D）

| 包 | 内容 | 状态 |
| --- | --- | --- |
| `boom_birds_interfaces` | 控制与生命周期接口 | 已存在 |
| `boom_birds_control` | PX4 后端、坐标换算、执行许可、会话协议、接管判定、SIH 守卫 | **已拆出**（本批） |
| `boom_birds_sensing` | 双目采集与发布、时间同步、深度计算与发布、位姿适配、MAVLink IMU 上行 | **已拆出**（第三批） |
| `boom_birds_bringup` | 生命周期编排、生产集成启动 | 已迁移 |
| `boom_birds_sim` | SIH 真值源、TEST-ONLY 合成 VIO/IMU、悬停中继 | **已拆出**（第二批） |
| `boom_birds_nav` | **纯兼容转发**到上述各包（无可执行文件、无实现） | **完成** |


底座归属：`runtime_config` / `frames` / `control_protocol` 放在 `boom_birds_control`——
控制包仅依赖接口层，共享配置模块不反向依赖 bringup 或 sim。若放进 bringup 会形成 control⇄bringup 环
（control 取阈值默认值，bringup 取 `px4_frames.LocalFrameAlignment`）。依赖方向由
`boom_birds_bringup/test/test_package_layout.py` 钉住。

`synthetic.py`、`pointcloud_scene.py` 与 `deep_checks.py` 位于 sim。
生产 `stereo_source` 只处理 v4l2/replay/file；合成输入由 sim 的 `synthetic_stereo_source` 提供。
配置和 launch 随实现归属迁移，nav 仅保留 Include 转发。生产包不依赖 sim；XML 和实际 import 均受测试检查。

兼容转发约定：`boom_birds_nav/<模块>.py` 把实现的全部公共名与 `__doc__` 绑定过来，
并支持 `python -m boom_birds_nav.<模块>`；对象身份与实现一致（各包的 `*_layout` 组断言）。

模块转发使用实现模块对象；源码级结构断言按所有者路径读取，兼容导入的对象身份由测试核验。感知自有的这类测试已改为 `boom_birds_sensing.*`。
生产包不得依赖 sim 包，该约束同样由测试强制。

### 本机 SIH 入口（完整任务与故障矩阵）

```bash
# 单次完整任务（TEST-ONLY，只连本机 SIH）
bash companion/ros2_ws/tools/run_sih_mission.sh \
  --scenario <名字> --scene local --goal 5.0 1.0 2.5 \
  [--fault 规划取消|深度断流|里程计断流|setpoint中断|模式确认失败] \
  [--obl-action land|rtl] [--timeout 240]

# 故障/反向矩阵（新批次名，拒绝覆盖既有证据）
bash companion/ros2_ws/tools/run_sih_matrix.sh recheck-faults companion/ros2_ws/tools/sih_cases/faults.spec
```

每次运行都会：显式下发并**回读** PX4 参数基线（`COM_OF_LOSS_T` 固定为本地版本缺省，不放大）、
生成合成标定、启动本机 PX4 SIH 与整条链、记录 `mission/control/execution` 周期快照与故障/状态变化、逐条控制命令及
PX4 独立回读、结束时核对 **Disarmed**，并把逐场景证据留在独立目录（失败证据不被覆盖）。
汇总行由 `tools/sih_matrix_row.py` 生成，包含状态序列、最近距目标、setpoint 计数与最终 armed/landed。
仓库保存 [正常/进程会话用例](../../tools/sih_cases/session.spec)、[故障/反向用例](../../tools/sih_cases/faults.spec)、[密集森林反向用例](../../tools/sih_cases/forest.spec)、[30 m 参考森林 seed 1–5](../../tools/sih_cases/forest-reachable.spec)。
spec 的可选字段为 fault_at、forest_seed、inject_alt_below、when_ready、stable_s、transient_relaunch_s、inject_mode_detail、planner_suspend_s、forest_profile；顺序见脚本 read 参数。

森林可达性检查依赖系统包 `python3-scipy`（`sudo apt-get install python3-scipy`）；CI 与 sim package.xml 已声明。
森林任务显式选择 `--forest-profile reference_30m|dense`。默认参考布局为 20 根柱、20 个环、X 宽 40 m、X 中心 0；dense 保留旧矩阵的 250/250、26 m、15 m 参数。布局定义在 sim/config/forest_scenes.yaml，不改变控制阈值。例：

```bash
bash companion/ros2_ws/tools/run_sih_mission.sh --scenario forest-reference-seed3 \
  --scene forest_30m --forest-profile reference_30m --forest-seed 3 --goal 15 0 1 --timeout 300
bash companion/ros2_ws/tools/run_sih_matrix.sh forest-reference companion/ros2_ws/tools/sih_cases/forest-reachable.spec
```

森林入口在 Mission.START 前保存规划器实际参数与完整场景点云，并按地图分辨率、立方膨胀、边界和虚拟顶棚检查连通性。报告 scene_reachability.json 区分 6/26 邻接；仅 6 邻接连通、起终点未占据时允许开始，拒绝时退出 5 并核对 PX4 Disarmed。参数/点云错误同样阻止任务。参考检查不向 EGO 注入地图或路线，不证明传感器可见性、机体外廓、轨迹动力学或闭环任务通过；最终仍检查目标到达、Offboard/failsafe 与降落。

### 生命周期表（lifecycle.py / lifecycle_node.py）

| 状态 | 进入条件 | 输出动作 | 失败去向 |
| --- | --- | --- | --- |
| IDLE | 初始 | 无 | — |
| PRECHECK | `Mission.START` 且会话开启成功 | 记录地面 z 与启动周期，`arm` | `precheck_timeout` → FAULT_LATCHED |
| TAKEOFF | 解锁且落地下、位姿新鲜、对齐有效 | `takeoff`，等稳定窗 | `arming_not_confirmed` / `takeoff_timeout` |
| HOLD_READY | 达起飞高度 + 连续 1 s 速度 ≤0.15 m/s、位置波动 ≤0.10 m | `hold_setpoint` | `map_or_stream_not_ready` |
| OFFBOARD_PENDING | 执行许可开放 + 传感器与地图就绪 | `offboard`（经 VehicleAction） | `offboard_not_confirmed` |
| EXECUTING | 模式回读确认 `offboard` | 转发规划指令；有界重发 `enable_planner` | 故障 → RECOVERING |
| RECOVERING | 已识别且允许恢复的故障，且全部前置条件成立 | 保持/制动段 → `offboard` → 回读确认 → `replan` | 预算耗尽 → FAULT_LATCHED |
| LANDING | 到达目标 / 人工降落 / 闭锁后收尾 | `cancel` + `land` | 落地且 Disarmed → COMPLETE |
| COMPLETE | 落地且 `armed=false`、`landed=1` | 无 | 终态 |
| FAULT_LATCHED | 任意活动阶段命中故障/取消/重启/坐标系重置 | `disable_planner` + `cancel` | 终态，不自动解锁 |

自动恢复默认关闭（`recovery_enabled=false`），只有 SIH 入口显式打开；每次故障最多 2 次尝试、
每次模式确认超时 3 s，耗尽即闭锁。恢复后连续 1 s 健康 EXECUTING 才重置事件预算；任务累计次数单独记录。人工取消 / 人工模式干预 / 未知故障 / 落地 / 重启 / 坐标系重置
永久撤销本次恢复资格。

### 控制协议表（control_protocol.py + boom_birds_interfaces）

| 消息 | 字段要点 | 校验 | 无效值语义 |
| --- | --- | --- | --- |
| `ControlCommand` (`/boom_birds/control/command`) | `session_id`、`planner_session_id`、`executor_session_id`、`trajectory_id`、`sequence`、`stamp`、`valid_for`、`command_type`(EXECUTE/CANCEL/HOLD) | 会话一致、序号严格递增、轨迹号未退役、`0 < valid_for <= command_timeout_s`、`-0.03 <= ros_now-stamp < valid_for` | 任一不满足即**整条丢弃**；CANCEL 无需有效期，形成序号屏障 |
| `ExecutionStatus` (`/boom_birds/control/execution_status`) | `sending`、`offboard_confirmed`、`mode`、`mode_detail`、`custom_main_mode/sub_mode`、`landed_state`、`sensors_ready`、`alignment_valid`、`reasons[]` | 编排器还检查实际/意图模式、落地、重启、reset counter、实际对齐变换与 SIH failsafe 原因的新鲜度（`status_timeout_s`） | `mode_detail=unknown` 表示缺少模式观测，闭锁自动恢复；命令被接受 ≠ 模式已确认 |
| `PlannerRequest`/`PlannerStatus` (`/boom_birds/planner/*`) | `session_id`、`sequence`、`enabled`、`goal` | EGO 侧要求会话已建立、序号递增、时间戳新鲜、`offboard_confirmed`、odom 与地图就绪 | 条件不满足时 EGO **静默丢弃**，因此编排器必须在窗口内重发使能 |
| `VehicleAction` (`/boom_birds/control/action`) | `OPEN_SESSION`/`ARM`/`TAKEOFF`/`HOLD`/`OFFBOARD`/`LAND`/`RETURN`/`CANCEL` | 会话一致；`sih_pid` 进程守卫 | 只有 PX4 接口能发模式命令；编排器与 shell 都不直接切模式 |

### 参数来源（单一来源）

| 内容 | 唯一来源 | 说明 |
| --- | --- | --- |
| 话题、帧名、时间阈值、高度、接管距离与速度连续性、恢复参数、场景原点 | `boom_birds_control/runtime_config.py` + `boom_birds_control/config/runtime.yaml` | 文件与代码默认值必须逐一相等（`test_runtime_config.py` 强制） |
| 接口语义（话题类型、单位、坐标系、无效值） | `boom_birds_interfaces/config/contract.yaml` | **只写语义，不写数值** |
| 节点参数覆盖 | sensing 的 `mavlink_imu.yaml`、control 的 `px4_interface.yaml`、sim 的 `px4_sitl_motion.yaml` | 被覆盖项必须与本节点默认值一致（`test_config_single_source.py` 比对） |
| PX4 起飞参数 | `boom_birds_bringup.sih_params`（读 RuntimeConfig 生成） | shell 不写高度、不做高度/模式判定 |

### EGO fork 补丁索引（子模块 `ego-planner-swarm`，HEAD `e96a455d`）

| 文件 | 作用 |
| --- | --- |
| `plan_env/include/plan_env/grid_map.h`、`src/grid_map.cpp` | 严格校验深度 CameraInfo 的 P 矩阵（有限、无倾斜、`P[10]=1`、主点在图内、左目 `P[0][3]=0`）；几何变化闭锁并清空占据/膨胀/深度缓存/就绪标志；`geometry_reset` 解除闭锁并代次 +1 |
| `plan_env/test/bb_grid_map_test.cpp` | 地图行为回归（含内参变化、无内参未就绪、光学帧变化、重建后清空） |
| `plan_manage/include/ego_planner/ego_replan_fsm.h`、`src/ego_replan_fsm.cpp` | 会话订阅 `ExecutionStatus` 取 `session_id`/`offboard_confirmed`；`/boom_birds/planner/request` 触发目标；10 ms 回调也拦截几何闭锁，堵住 50 ms 安全定时器的竞态 |
| `plan_manage/src/traj_server.cpp` | 订阅 `SessionBspline` 与 `ExecutionStatus`，携带会话元数据转发 `ControlCommand` |
| `traj_utils/msg/SessionBspline.msg`（未跟踪） | 会话元数据封装（`session_id` / `producer_session_id` / `sequence` / 内层 Bspline） |
| `plan_manage/launch/boom_birds_offline.launch.py`、`CMakeLists.txt`、`package.xml` | 项目入口默认动态内参、依赖与消息生成 |

### 会话任务入口

`px4_sih_mission.launch.py` 启动感知、规划、执行接口和生命周期；`/boom_birds/mission` 的 `Mission.START` 才发起任务。
参数 `sih_pid` 指向本轮启动的 PX4 SIH 进程。模式/解锁服务会核对该 PID 的可执行文件和 SIH 环境，PID 缺失或不匹配时拒绝请求。
当前实测与未通过项只维护在 [STATUS](../../../../docs/STATUS.md)。`run_exit=0` 表示脚本完成及最终 Disarmed，不代表目标到达或恢复验收通过。

| 接口 | 语义 |
|---|---|
| `VehicleAction.OPEN_SESSION` | 接口生成 UUID；更换会话清除缓存，旧会话不再接受 |
| `ControlCommand` | 同一 RELIABLE 通道承载 EXECUTE/CANCEL/HOLD；序号递增，轨迹 ID 不重用 |
| `header.stamp` / `valid_for` | ROS 时间有效期，同时用单调时钟限制缓存寿命；转发不刷新旧时间戳 |
| `ExecutionStatus.accepted` | 命令通过协议检查；不表示飞控执行 |
| `ExecutionStatus.sending` | 本周期后端发送成功；不表示模式已切换 |
| `ExecutionStatus.offboard_confirmed` | 新鲜 PX4 HEARTBEAT 回读为 Offboard |
| `PlannerRequest` | 只有当前会话且 Offboard 已确认才激活目标；禁用时使旧轨迹失效 |
| `traj_utils/SessionBspline` | 项目元数据封装；内部 `Bspline` 算法消息保持不变 |

| 生命周期 | 退出条件 |
|---|---|
| IDLE → PRECHECK | 接受任务并取得接口会话 |
| PRECHECK → TAKEOFF | 定位/连接/对齐/落地状态满足前置条件，发解锁请求 |
| TAKEOFF → HOLD_READY | 明确在空中，达到锁点高度，连续 1 s 满足速度与位置波动限制 |
| HOLD_READY → OFFBOARD_PENDING | 地图/传感器有效，保持流已发出；请求 Offboard |
| OFFBOARD_PENDING → EXECUTING | PX4 回读确认；随后激活规划目标 |
| EXECUTING → LANDING | 目标附近连续稳定；停止规划并请求降落 |
| LANDING → COMPLETE | 回读落地且 Disarmed |
| 任意活动阶段 → FAULT_LATCHED | 人工取消、未知错误、重启、坐标变化或确认超时；不得自动解锁 |

控制出口仍由 `px4_failsafe` 判定最终发送许可。会话检查位于其前，编排器不直接打开 MAVLink 连接。

控制节点在 ROS 初始化前默认设置 `RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`，显式环境配置优先；可靠 QoS 与取消屏障保留。生命周期检查把消息传输时间计入观测年龄。此默认值用于避免本机 SIH 中已复现的 ExecutionStatus 同步发布阻塞，不构成硬实时保证；证据见 [STATUS](../../../../docs/STATUS.md)。
