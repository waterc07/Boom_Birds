# Gazebo 传感器与回放基准

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
