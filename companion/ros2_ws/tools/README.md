# 开发与维护工具

运行部署使用 `build_runtime.sh`，入口和安装边界见[工作空间说明](../README.md#运行与开发隔离)。开发工具不随运行包安装；从仓库根加载 `activate_development_env.sh` 后使用 `python3 -m tools.<目录>.<模块>`。

| 内容 | 入口 |
| --- | --- |
| 开发构建与完整脱机检查 | `build_all.sh`、`check_offline.sh` |
| USB/MIPI 相机采集与标定 | [camera_calibration](camera_calibration/README.md) |
| 相机—IMU标定、IMU 噪声 | [vio_calibration](vio_calibration/README.md) |
| 深度预览与保存 | [depth_preview](depth_preview/README.md) |
| 深度实验与性能回放 | [depth_benchmark](depth_benchmark/README.md) |
| 图像、MAVLink、rosbag 离线回放 | [sensor_replay](sensor_replay/README.md) |
| SIH、SITL、Gazebo 与故障注入 | `run_sih_*`、`run_px4_*`、`run_gazebo.sh`、`sih_*`；[仿真包](../src/boom_birds_sim/README.md) |
| 平台降落合成/回放验证 | `check_platform_*`、`run_platform_*`、`scan_platform_offline.py`、`soak_platform_offline.py` |
| 实机接入前检查与监测 | `check_attitude_hardware.py`、`monitor_attitude_hardware.py`；[接入说明](../src/boom_birds_bringup/README.md) |

采集与硬件脚本仍需其文档列出的接线和许可条件。开发环境脚本只加载软件，不启动设备。过程数据写入仓库外目录，不放在 `src/`、`tools/` 或安装树内。
