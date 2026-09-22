# 当前状态与下一步

更新：2026-09-22。以下环境和测试结论承接已有记录；本次仅重组文档，未重新执行算法、依赖或设备验证。

## 开发环境与源码

- 唯一主开发根：WSL Ubuntu-24.04 `/home/waterc/workspace/Boom_Birds`；Windows 目录仅保存资料与备份。
- WSL 已安装 ROS 2 Jazzy，入口 `/opt/ros/jazzy/setup.bash`；2026-09-22 既有检查中 `source` 后可定位 `ros2`，`colcon list` 识别两个子模块的 25 个 ROS 2 包。此前本机 talker/listener 通信检查通过；不代表算法构建通过。
- 当前 `python3 -c "import cv2"` 失败：用户目录中的 NumPy 2.5.2 与现有 OpenCV 的 NumPy 1.x ABI 不兼容。下一步需修复 Python 依赖环境。
- `stereo_depth` 属母仓库；OpenVINS `master` 和 EGO `ros2_version` 属个人 fork 子模块。精确版本以 gitlink 为准；Jazzy/ARM64 构建和回放均未验收。
- 当前 WSL 主工程没有 `.codegraph/`，跳过 CodeGraph；Windows 旧索引不适用。

## 已实现与已有证据

- 双目独立 Python 程序：USB 同帧拼接采集、校正、StereoSGBM、米制 XYZ/Z、无效值 NaN、网页预览和数据保存。
- 视频调焦/引导标定已实现。默认标定 `calibration/live_20260916_210120_642136/candidate.npz` 已随 Git 跟踪。
- 2026-09-16/17 Pi 5 记录：总图 2560×960、11×8 内角点、20 mm 棋盘、30 组实采；估计基线 67.6718 mm，双目 RMS 0.4018 px，6 组留出垂直 P95 0.4924 px；完整 XYZ/Z 输出检查通过。见[标定说明](../companion/ros2_ws/src/stereo_depth/LIVE_CALIBRATION.md)。独立距离精度仍待测量。
- PX4 独立源码位于 `/home/waterc/PX4-Autopilot`；主工程 [manifests](../px4/manifests/) 保留旧源码/固件来源记录。已有固件与历史 ULog 不证明当前实板 target、烧录版本和接线已完成对应。
- Windows 原始采集/深度输出保留在资料根 `data/stereo_depth/windows_snapshot_20260922/`；备份位置见[资料位置](../README.md#源码与资料位置)。

## 已确定路线与未完成项

双目图像 + 飞控 IMU → OpenVINS；自算深度/XYZ + 位姿 → 局部地图 → EGO-Planner → 轨迹执行 → 控制/飞控通信层 → PX4。详见 [系统架构](../README.md#系统架构)。

- 尚未实现或验收：深度 ROS 2 封装、共享采集/IMU 记录回放、算法集成、完整仿真闭环、自动部署与 ARM64 发布包。
- 尚未确定：飞控 IMU 来源与消息、时间同步与相机—IMU 标定、控制器位置、ROS 2 通信后端及外部视觉融合配置。
- 尚未验证：距离精度、端到端延迟、长期性能、实板 target 对应、安全接管与飞行。
- Pi 5 为当前验证平台，RK3576 为后续迁移方向；具体板卡及最终机载适用性未验收。树莓派的 ROS 2 安装状态需独立核验，不能由 WSL 安装结果推定。
- 两个历史日志工具仍按旧父目录找数据，WSL 路径适配未完成，见 [tools/README](../tools/README.md)。

## 下一步与完成条件

当前优先修复 WSL Python 依赖，再验证构建；以下为待办，不自动授权设备操作、刷写、部署或推送。

| 顺序 | 工作 | 完成条件 |
| --- | --- | --- |
| 1 | 隔离并修复 NumPy/OpenCV ABI 冲突 | 记录环境与依赖版本，`import cv2` 和深度 CLI 帮助成功，不影响其他环境 |
| 2 | 核对两个 fork 的版本、接口与 Jazzy 构建 | 保存具体 SHA、实际构建命令和日志，分别报告 x86_64 与 ARM64 的验证范围；定义 IMU/时间戳/坐标系/质量字段及控制通信边界 |
| 3 | 共享双目/飞控 IMU 记录回放与深度 ROS 2 封装 | 同一数据可重复回放，保留原图、标定、米制深度、完整 XYZ 与无效性；节点接口和启动方式有可运行说明 |
| 4 | OpenVINS、深度和通信分别验证 | 留存初始化/漂移/重置、独立距离测量与通信稳定性证据；补齐相机—IMU 时空标定 |
| 5 | 建图、规划、轨迹与控制仿真闭环 | 可先用仿真里程计；验证绕障、无可行路径、定位重置、数据及通信失效 |
| 6 | Pi 5 联合验证与后续迁移 | 可重复部署/健康检查/回退，全链路延迟、丢帧、资源和温度有记录；完成硬件前置证据后才进行无桨联调及受控飞行，RK3576 单独验收 |

硬件并行主线为 **FC-001：实板身份与 PX4 bring-up 证据包**。厂商资料已获得，实板执行待开展；步骤和完成条件见 [PX4 验收清单](../px4/README.md#fc-001-实板验收清单)。之后补齐动力台架、人工稳定飞行、独立光流/测距与安全接管证据。软件仿真不替代这些验收。

历史日志工具的数据根配置待适配，入口见 [tools](../tools/README.md)。
