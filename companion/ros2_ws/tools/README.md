# 开发与维护工具

运行部署使用 `build_runtime.sh`，入口和安装边界见[工作空间说明](../README.md#运行与开发隔离)。开发工具不随运行包安装；从仓库根加载 `activate_development_env.sh` 后使用 `python3 -m tools.<目录>.<模块>`。

| 内容 | 入口 |
| --- | --- |
| 开发构建与完整脱机检查 | `build_all.sh`、`check_offline.sh` |
| 临时隔离网络中的 ROS/UDP 回归 | `run_network_isolated.sh COMMAND [ARG ...]`；仅 lo，不改宿主路由 |
| USB/MIPI 相机采集与标定 | [camera_calibration](camera_calibration/README.md) |
| 相机—IMU标定、IMU 噪声 | [vio_calibration](vio_calibration/README.md) |
| 深度预览与保存 | [depth_preview](depth_preview/README.md) |
| 深度实验与性能回放 | [depth_benchmark](depth_benchmark/README.md) |
| EGO 失败几何日志分类 | [ego_diagnostics](ego_diagnostics/README.md) |
| 图像、MAVLink、rosbag 离线回放 | [sensor_replay](sensor_replay/README.md) |
| SIH、SITL、Gazebo 与故障注入 | `run_sih_*`、`run_px4_*`、`run_gazebo.sh`、`sih_*`；[仿真包](../src/boom_birds_sim/README.md) |
| 平台降落合成/回放验证 | `check_platform_*`、`run_platform_*`、`scan_platform_offline.py`、`soak_platform_offline.py` |
| 三项 0.8 m Tag 测试 | `test1_takeoff_land.sh`、`test2_return_turn.sh`、`test3_forward_land.sh`；[模式与配置](../src/boom_birds_bringup/README.md#三个实机测试脚本) |
| 实机接入前检查与监测 | `check_attitude_hardware.py`、`monitor_attitude_hardware.py`；[接入说明](../src/boom_birds_bringup/README.md) |
| IMU→机体→姿态控制轴向契约 | `check_body_frame_contract.py --extrinsics-file PATH --calibration-file PATH --out REPORT.json`；实际节点接收合成输入，不连接设备；需已加载配套运行环境 |

采集与硬件脚本仍需其文档列出的接线和许可条件。开发环境脚本只加载软件，不启动设备。过程数据写入仓库外目录，不放在 `src/`、`tools/` 或安装树内。

WSL 若出现“节点已发现但跨进程收不到数据”，可在临时网络命名空间中复核：

```bash
INSTALL_BASE=/home/waterc/bb_build/camelcase-20261023/install \
BUILD_BASE=/home/waterc/bb_build/ego-single/build \
EGO_INSTALL=/home/waterc/bb_build/ego-single/install \
  bash companion/ros2_ws/tools/run_network_isolated.sh \
  bash companion/ros2_ws/tools/check_offline.sh --out /home/waterc/bb_build/checks/isolated/report.json
```

构建前缀改为本次实际路径；Python 与 EGO 分开构建时，`BUILD_BASE` 指向 EGO build，`EGO_INSTALL` 指向配套 EGO install，检查报告记录两个安装前缀、实际 RMW 实现、相关环境设置与外部 DDS 配置文件 SHA256。证据目录须为空。脚本需要 Linux 允许非特权 user/network namespace；不满足时直接失败，不回退到宿主网络。仅适用于全部通信进程均由该命令启动的脱机回归，不能与外部进程或真实设备通信；设备文件未隔离。该检查不能证明 Pi 传输或实机导航通过。

## SIH 启动与失败记录

`run_sih_mission.sh` 直接启动 `boom_birds_sim/px4_sih_mission.launch.py`，不依赖 nav 兼容入口。其服务发现与图快照使用 `--no-daemon --spin-time 1`，直接查询本轮 ROS_DOMAIN_ID，避免依赖已有 CLI daemon；任务服务调用和导航验收判据不变。

SIH 启动前核对场景所需地图生成包：local/recovery_local 为 `mockamap`，forest_30m 为 `map_generator`；它们安装在开发前缀，不进入 14 包运行树。任务链提前退出或任务服务未发现时直接失败，证据保留在 simulation-packages.txt、launch.log 和 service-discovery.log。


## 遥测预算离线核算

`check_telemetry_budget.py` 不连接设备、不写 PX4 参数。控制器请求档来自 `boom_birds_control.telemetry_budget.observationIntervals`，另计 IMU 50 Hz、测距 20 Hz、TIMESYNC 10 Hz 及基础状态。按 MAVLink 2 完整载荷上界、可选签名和 UART 8N1 计算，默认预留 20%。其他客户端请求流须另计。

```bash
source companion/ros2_ws/tools/platform_env.sh
python3 companion/ros2_ws/tools/check_telemetry_budget.py --baud 115200 --mav-rate 8000
python3 companion/ros2_ws/tools/check_telemetry_budget.py --baud 230400 --mav-rate 16000
# 可附加已归档 RX，duration 为完整采集秒数，不打开串口：
# --rx-bin /absolute/path/serial-rx.bin --duration 87.94
```

退出码 2 表示预算不足，0 仅为 `BUDGET_OK_UNVERIFIED`。自动预算 `MAV_X_RATE=0` 拒绝当作已知容量；不能用工具默认值代替实际参数回读。RX 分析报告各流接收频率、源时间戳间隔和非递增次数，不作定位或时序验收。

设备阶段先确认地面、拆桨、未解锁，再回读对应 MAVLink 实例的模式、串口映射、`MAV_X_RATE` 与对应 `SER_*_BAUD`，保存旧值。波特率须两端一致，不能只改 Companion；修改后检查实际消息频率、姿态配对、重置计数和输入门控。当前实机配置及验证范围见 [STATUS](../../../docs/STATUS.md#实现默认入口部署与验收)，不能把示例值当作其他设备的配置。

开发工具类 `TelemetryLink(device, out, baud=230400)` 支持显式选择 115200/230400，不修改飞控波特率；默认值仍为 115200。使用前回读实际链路参数。旧标定/观察脚本未传 baud 时仍使用 115200，不能直接用于已切换到 230400 的链路。退出时保留 raw、无回显串口设置，不恢复 canonical/echo，避免空闲 UART 自动回显飞控数据。
