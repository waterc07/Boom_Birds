# SIH、Gazebo 与回放基准

WSL Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic。入口只连接本机 SITL；数据与日志写入显式指定的新目录。

## 构建

依赖：已有 ROS/MAVROS、Gazebo Harmonic、ros-jazzy-ros-gz-bridge、ros-jazzy-rosbag2-py、ros-jazzy-rosbag2-transport。OpenVINS 与导航安装前缀沿用当前工程。

~~~bash
cd /home/waterc/workspace/Boom_Birds
bash companion/ros2_ws/tools/build_gazebo.sh
~~~

默认安装在 `~/bb_build/gazebo/install`；可用 `BB_GAZEBO_BUILD_ROOT` 覆盖。构建使用 `build_all.sh` 的锁和并发限制。Gazebo 插件只有 `BB_BUILD_GAZEBO_MOTION=ON` 才编译，常规 SIH/Pi 构建保持关闭。

## 阶段 0：PX4 与 MAVROS

~~~bash
bash companion/ros2_ws/tools/run_gazebo.sh baseline \
  --out /home/waterc/bb_build/gazebo/new-baseline
~~~

启动官方 X500、PX4 daemon 和 MAVROS，观察 40 s。baseline.json 检查连接、IMU、位置消息和全程未解锁；结束时关闭本轮进程组。官方模型使用模拟 GPS，此测试不属于无 GPS 导航验收。

## 阶段 1：双目、IMU、时钟

~~~bash
bash companion/ros2_ws/tools/run_gazebo.sh capture \
  --out /home/waterc/bb_build/gazebo/new-capture --pause-test
~~~

生成有纹理的室内场景、独立模拟标定与 ros_gz_bridge 配置；默认录制 45 s 仿真时间；`--duration` 只改变录制时长，清单保存实际请求值，不改变 5～40 s 的激励窗口。默认 range_3m，可用 --scene range_5m 切换远场。运动默认 nominal；--motion-profile gentle 选择弱激励。三维刚体由物理力/力矩驱动，重力关闭；IMU 比力仍包含重力项。运动时段为 5～40 s，末尾静止。此平台不包含本项目机体/动力模型。

- 双目：每目 640×480，20 Hz，基线 0.0676718 m。
- IMU：Gazebo 物理传感器，200 Hz、FLU；本阶段不经过飞控 MAVROS IMU 上行。
- 渲染数据用 reliable QoS；左右图仅按原始相同时间戳配对，不近似配对或重写时间戳。
- CameraInfo 对照 Gazebo 原生内参检查；左右投影矩阵保留基线。
- 图像、IMU、真值和 /clock 同为仿真时间。capture.json 给出采样频率、最大间隔、配对丢弃、墙钟时长和实际时间比。
- 为避免渲染丢帧，场景目标实时因子为 0.2；45 s 数据约需 225 s 墙钟时间，另有启动开销。这里的 Hz 按仿真时间计算。
- --reset-test 在约 2 s 重置世界；只有采集因时钟回跳拒绝继续才判测试通过。reset.json 是故障测试结果，capture.json 应为拒绝状态。
- --pause-test 在约 2 s 仿真时间暂停世界，核验时钟停止，再恢复；结果写入 pause.json。
- manifest.json、stereo.npz、OpenVINS YAML、sensor_world.sdf、bridge.yaml 与 inputs/ 一起保存。基准传感器未加噪声，滤波器噪声参数仅用于算法配置。
- ROS 域通过本机锁分配，Gazebo 使用独立 partition，避免不同录制相互串入。

## 阶段 2：图像 VIO 与双目深度

~~~bash
bash companion/ros2_ws/tools/run_gazebo.sh replay \
  --input /home/waterc/bb_build/gazebo/new-capture \
  --out /home/waterc/bb_build/gazebo/new-replay

bash companion/ros2_ws/tools/run_gazebo.sh evaluate \
  --input /home/waterc/bb_build/gazebo/new-capture \
  --out /home/waterc/bb_build/gazebo/new-replay
~~~

该 Gazebo 回放入口仍使用单目 VIO 对照配置，不代表当前实机双目开发主线；实机配置见[bringup](../boom_birds_bringup/README.md#双目-vio-与启动配置)。

回放只发布左右图、CameraInfo、IMU 和回放时钟；真值只在离线评估读取。OpenVINS 使用左目、KLT 图像前端、100 跟踪点/6 克隆/0 长期地标，从传感器数据初始化；双目继续走默认 SGBM，输出 320×240 米制 32FC1/NaN。未注入真值初始状态或预生成特征。

模拟无噪声配置启用起始 ZUPT，zupt_max_disparity=0.001 px；fi_max_baseline=200、fi_max_cond_number=100000 适配 6 克隆短窗口。只修改模拟配置，不覆盖生产 OpenVINS 参数。跟踪上限 21 Hz 避免 20 Hz 输入在浮点边界被额外抽帧。OpenVINS 使用可选 `graceful_shutdown=true` 回放模式：同步处理回调，释放发布对象后关闭 ROS context；当前 ROS 2 订阅入口默认也为 graceful_shutdown=true；显式关闭后的异步退出未验收。参数说明见 [OpenVINS 回放入口](../open_vins/ReadMe.md#项目-ros-2-回放退出模式)。

输出：outputs/、replay.json、processes.json、evaluation.json、trajectory.csv、state-estimate.txt、state-std.txt、图像样例。默认 DEBUG 日志记录视觉更新数；INFO 日志不能证明实际视觉更新。质量门限在 gazebo_replay.GATES 定义并保存为 gates.json；评估失败返回非零退出码。

- VIO：先按时间插值真值，再用首条姿态/位置对齐航向和平移；不拟合尺度或时间偏移。统计位置 RMSE、终点误差、姿态 P95、状态输出覆盖率。首 pose 时间是初始化完成的上界，不等于内部初始化时刻。
- 深度：只比较与模拟深度时间戳完全一致的输出；固定中心 ROI [40:200,60:280]，报告误差和有效率。模拟深度 10 Hz、双目 20 Hz，未匹配帧另计，不插值真值图。
- IMU：用三维真值位姿差分核验比力与角速度，避免仅修改运动速度导致模拟加速度错误。

### 距离与运动对照

四组分别录制 range_3m/range_5m × nominal/gentle。固定 seed=12、标定、20/200 Hz、45 s；两档场景也改变了障碍布局，属于近/远场场景对照，不是只平移同一面墙。

旧 bag 保留原配置。回放旧 bag 时用 --zupt-mode noiseless --feature-profile low-parallax 选择修正后的模拟配置；source 可复现原配置。imu、disparity-strict 是诊断对照。noiseless 的 0.001 px 阈值只针对无噪声传感器，不直接用于真实数据。

~~~bash
bash companion/ros2_ws/tools/run_gazebo.sh replay \
  --input /home/waterc/bb_build/gazebo/new-capture \
  --out /home/waterc/bb_build/gazebo/new-fullres-replay \
  --zupt-mode noiseless --feature-profile low-parallax --depth-profile fullres
~~~

默认 --depth-profile budget：320×240 匹配、5×5 窗口、发布 320×240。
fullres：640×480 匹配、3×3 窗口、64 视差，按最近邻缩回 320×240；CameraInfo 同步缩放。输出单位和 NaN 不变，未部署 Pi。64 视差在 fullres 下理论最近深度约 0.451 m，此测试不验证 0.2 m 近距覆盖。

evaluate 使用回放启动时保存的 gates.json；位置、深度阈值保持原值。新增 require_visual_updates：日志必须出现实际 MSCKF 特征更新，避免纯 IMU 积分的位置误差偶然很小而被判为 VIO PASS。诊断同时输出 5 s 误差窗口、状态最大间隔、最终偏置和按真实距离分档的深度误差。

~~~bash
bash companion/ros2_ws/tools/run_gazebo.sh matrix \
  --run /path/near-nominal /path/near-nominal-replay \
  --run /path/near-gentle /path/near-gentle-replay \
  --run /path/far-nominal /path/far-nominal-replay \
  --run /path/far-gentle /path/far-gentle-replay \
  --out /path/new-matrix.json
~~~

matrix 重新评估四组，核对输入标定、配置散列、统一门限和参数档位、真值未发布、三个子进程退出码。可以重复 --run 增加同一组的重复回放；同一 bag 重放不算独立场景或随机种子。

已有结果及失败记录见项目 [STATUS](../../../../docs/STATUS.md)。基础场景通过不包含跨场景 VIO、远场深度质量、EGO/Companion 飞行闭环、真实传感器同步、Pi 共载或飞行验收。


## SIH 完整任务与故障矩阵


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


## SIH 启动边界与参数来源

| 阶段 | 检查与动作 | 未满足时 |
| --- | --- | --- |
| 起飞参数 | 脚本经 `boom_birds_sim.sih_params` 从 `RuntimeConfig` 下发 PX4 参数 | 参数二进制缺失或任一条失败即非零退出 |
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



三类高度参数不互换：takeoff_altitude_agl_m 是 PX4 起飞参数，hold_lock_min_altitude_agl_m 是锁点下限，image_publish_min_altitude_agl_m 是合成输入发布下限。位置路线 recovery_enabled 仅 SIH 显式启用，不等同于姿态控制器的 recovery_mode。

分段可视化脚本与旧接口验证记录见[历史导航说明](../../../../docs/history/NAV_GUIDE_2026-09.md)。当前任务优先使用本节统一入口。

## 运行与开发边界

本包及 `config/attitude_sih.yaml`、`config/platform_landing*_test.yaml` 仅用于开发验证，不由 `build_runtime.sh` 构建。SIH 起飞参数生成器位于 `boom_birds_sim.sih_params`；参数值仍从 `boom_birds_control.runtime_config` 读取。SIH 进程身份、安全门控和反馈适配仍由控制接口执行，不随测试工具移除。

## 速度坐标契约

SITL 真值源的 body/IMU 反馈按消息姿态将世界系线速度旋转到 child_frame_id 的机体系；EGO 话题保留世界系线速度，使用独立消息。合成 VIO 源也把位置导数转入 IMU 局部系再写 twist。控制入口再将 body 线速度转回世界系，不依赖仿真航向为零。此约定与 pose_adapter 的 body/EGO 两路输出一致。
