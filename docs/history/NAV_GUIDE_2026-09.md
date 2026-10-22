# 2026-09 导航仿真与接口历史记录

移自 boom_birds_nav/README.md 的旧演示、合成精度与接口验证段落。以下状态和命令属于当时快照；当前 SIH 操作见[仿真包](../../companion/ros2_ws/src/boom_birds_sim/README.md#sih-完整任务与故障矩阵)，不用于判断当前 Pi/VIO 验收。

## 已实测的合成精度边界（2026-09-22，本机 x86_64）

合成场景（3 m 墙 + 1.8 m 障碍，合成标定 TEST-ONLY）：

- 有效像素比例约 0.42（含右边界 99 px 不可测区）；
- 深度误差受 StereoSGBM 的 1/16 px 整数视差量化限制：3 m 处中位误差约 0.078 m；
- 因此**不要求**毫米级，也不得把该结果当作真机距离精度。

该批历史标定（1280×960、基线 67.6718 mm）对应 320×240 深度图；实际内参由运行时同一标定的 P1 推导并发布为 CameraInfo，分段 SIH 启动入口读取该 CameraInfo，其他 EGO 启动入口须显式提供匹配的内参。


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

TEST-ONLY 统一入口（执行结果按 STATUS 的日期化记录核对）：先
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

以下为迁移前 pymavlink backend 的历史记录；当前 MAVROS 验收见 [STATUS](STATUS_2026-10.md#2026-10-02-px4-通信迁移到-mavros)。

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

### PX4 接口的历史验证口径

下表为早期位置路线批次结果，不作为当前 Pi/完整机状态；后续 MAVROS、SIH 和实机证据见 STATUS。

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

### ARM64 / Pi 5 的验收边界

`boom_birds_nav` 是纯 Python 兼容包，无独立扩展需要交叉编译；原生依赖、OpenVINS/EGO 和 V4L2 驱动仍须在 aarch64 上验证。已有 Pi 原生构建、采集和深度性能证据见 [STATUS](../STATUS.md)，不再沿用旧 x86 环境缺交叉工具链的结论描述当前设备。

`camera_timestamp` 依赖 Linux `struct v4l2_buffer` 布局，测试用 gcc+`offsetof()` 核对。布局检查通过不能替代曝光时间戳、USB 带宽、正式遥测与全导航共载验收。
