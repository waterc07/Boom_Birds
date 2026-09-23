# 当前状态与下一步

更新：2026-09-23。唯一正式开发根为 WSL Ubuntu-24.04 的 `/home/waterc/workspace/Boom_Birds`；本节仅描述 WSL 脱机验证，不推定树莓派或飞控状态。详细架构见 [项目入口](../README.md)，运行方式见 [ROS 工作空间](../companion/ros2_ws/README.md)。

## 已完成的 WSL 脱机部分

- ROS 2 Jazzy/x86_64 已构建 `stereo_depth`、`boom_birds_nav`、EGO 依赖链和 `ego_planner`，以及 OpenVINS 的 `ov_core`、`ov_init`、`ov_msckf`。OpenVINS 已启动并订阅合成双目与 IMU；尚无有效 VIO 初始化/里程计输出证据。
- 工作空间隔离 Python 环境可导入 NumPy 1.26.4、OpenCV 4.6.0、rclpy、cv_bridge；深度 CLI 和既有 10 项标定单测通过。构建脚本对 colcon/CMake 限并发，并默认使用仓库外持久目录 `/home/waterc/bb_build/main/{build,install,log}`。
- `boom_birds_nav` 提供合成/文件双目源、`32FC1` 米制深度（无效 NaN）、`16UC1` 毫米兼容深度（无效整数 0）、完整 H×W XYZ 点云、CameraInfo、按采集时间缓存/插值的位姿适配。TEST-ONLY 合成 IMU/VIO 不能作为飞控或精度证据。话题、坐标系和时间参数以 [契约](../companion/ros2_ws/src/boom_birds_nav/config/contract.yaml) 为准。
- EGO 地图跳过无效深度观测，完成融合/膨胀且达到占据阈值后才放行规划。规划器对整段轨迹做动态约束与保守碰撞校验；拒绝时向 `traj_server` 发送失效消息并停发 PositionCommand。停发命令不等于 PX4 悬停或安全接管。
- 默认标定 `stereo_depth/calibration/live_20260916_210120_642136/candidate.npz` 随工程保存；2026-09-16/17 的 Pi 5 旧记录仅证明当时的几何校验，真实米制距离精度仍须独立尺测。

## 当前验收范围和结果

第一阶段规划范围暂定为侧向可达目标、占据目标拒绝、长期门控、轨迹失效、运动中重置；**正后方目标绕障暂不纳入本阶段**。旧的正后方测试在 25 s 内 42 次规划均被安全拒绝，记录留在本机 `companion/ros2_ws/log/review_fix/final_center/`，不把该用例写成 PASS。

| WSL 验证项 | 结果 |
| --- | --- |
| 导航 Python 单测 | PASS：35 项 |
| EGO 地图行为回归 | PASS：41 项；当前参数至少 6 次正命中后才放行 |
| C++ 轨迹校验 | PASS：完整区间导数上界、时间拉伸和保守扫掠碰撞 |
| 侧向目标 (2.5, 1.2, 1.2) m | PASS：3 条运动轨迹，速度/加速度/障碍碰撞违规 0/0/0；最小障碍距离 0.304 m，终点误差 0.058 m |
| 占据目标 (3.0, 1.2, 1.2) m | PASS（安全拒绝）：120 s 轨迹/位置指令 0 |
| 地图就绪门控关闭 | PASS：120 s 规划尝试/运动指令 0 |
| 连续拒绝及执行端失效 | PASS：120 s 失败重试最短间隔约 0.5 s、位置指令 0；60 s 执行端失效测试停发并可恢复 |
| 运动中重置 | PASS：闭锁、停旧链、新坐标系重建地图及重新规划；旧/新占据体素重叠 0 |
| OpenVINS 初始化/漂移 | NOT RUN：无同步真实双目与飞控 IMU 数据集 |
| 真机距离精度、端到端时延、ARM64/Pi 5、PX4 闭环与飞行 | NOT RUN |

以上 PASS 只覆盖 WSL 合成/文件输入的软件行为；完整 VINS–深度–规划链与真机安全性**尚未通过总体验收**。测试 JSON、日志和过程记录均保留本机并由 Git 忽略，不随 GitHub clone 分发。

## 下一步

1. 获取并核验同步双目与**飞控 IMU** 数据，确定消息接口、轴向、时间映射与相机—IMU 时空标定，再验收 OpenVINS 初始化、输出频率、重置和漂移。当前无飞控连接，不把 TEST-ONLY 合成 IMU 充作真实数据。
2. 核验相机独立距离精度、端到端延迟和长期性能；位姿插值上限、队列容量及门控次数按实测重新定值。
3. 后续单独处理正后方目标曲线优化，并在更广场景验证可达性。当前安全拒绝不证明一般绕障能力。
4. Pi 5/ARM64 构建、PX4 控制接口和安全接管需另立硬件证据门槛；协调重启仅用于地面流程，不能用于飞行中连续控制。

飞控/动力硬件验收入口为 [PX4 清单](../px4/README.md#fc-001-实板验收清单)。本地整理提交已快进合并到母仓库 `main`；OpenVINS 和 EGO 分别位于各自的 `boombirds-jazzy` 分支，母仓库 gitlink 固定两者新提交。三个工作树均干净，但尚未推送；发布时先推两个子模块，再推母仓库 `main`。远端 `main` 仍为旧版本；普通删除提交只清理新检出内容，不清除旧 GitHub 历史中的记录，若要清理历史需单独评估改写及对子模块/协作者的影响。
