# boom_birds_nav

Boom_Birds 第一阶段导航链路的 Companion 侧节点集合。分两部分：

- **脱机链路**（文件/合成双目 → 米制深度与完整 XYZ → 位姿/里程计适配 → EGO 规划）：
  仅用于 WSL 脱机开发，不连接真实设备。
- **真实链路第一版**（PX4 MAVLink `HIGHRES_IMU` → `/boom_birds/imu`，含 TIMESYNC 时钟映射
  与相机采集时间戳接口）：代码完成、脱机测试通过，**真机未验收**（见文末清单）。

## 节点与模块

| 可执行 / 模块 | 职责 | 关键约定 |
| --- | --- | --- |
| `stereo_source` | 唯一采集源语义：合成或文件回放，发布左右原始图与拼接图 | 同帧左右图共享时间戳 |
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

## 相机采集时间戳接口（**仅接口与脱机验证，未接入真实发布链**）

> 范围声明：本模块目前只提供「取帧 + 驱动时间戳 + 同帧左右共享 + ROS 时域映射」的
> 接口与判据，**没有**接入真实左右图 ROS 发布链，因此「相机与 IMU 在同一时间域发布」
> 尚未端到端实现。真实曝光时刻**未经验证**：`data/stereo_depth/.../capture.json`
> 里记录的是 `host save time, not exposure time`，驱动时间戳与曝光中点的偏差需要
> 外部触发/闪光或与 IMU 相关峰对齐来单独标定。

真实相机采集的**唯一可信来源**是 V4L2 `VIDIOC_DQBUF` 返回的 `v4l2_buffer.timestamp`。
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

**现状**：现有 `depth_preview.py` 用 `cv2.VideoCapture.read()`，只有「取帧返回时刻」，
因此真实相机→ROS 链路在拿到可核实时间戳前必须保持禁用。需要的改造与硬件验证见
[stereo_depth README](../stereo_depth/README.md) 与本文末清单。

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

全量脱机测试共 **141 项**（原有 35 项 + 本次新增 106 项）。MAVLink/时间同步/相机时间戳
相关测试全部使用**构造的模拟消息、模拟 ioctl 与真实记录帧**，不连接任何设备：

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
