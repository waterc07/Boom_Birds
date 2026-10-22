# 控制、会话与平台降落

平台降落默认不启用；仅允许测试配置、dry-run 或已核验的本机 SIH。真实平台入口仍缺硬件速度控制确认适配器。姿态控制实机准备见 [bringup](../boom_birds_bringup/README.md)。

往返导航沿用 OpenVINS（同帧双目＋飞控 IMU）与现有控制链。降落时检测下视板，由 Companion 生成 NED 速度和偏航角速度参考，PX4 执行速度闭环。原点起飞仍由现有 VIO 路线完成；原点降落和返航到板附近后的降落使用同一流程。新增引导器的可选 takeoff 路径保留 VIO，达到高度后停在 TAKEOFF_HOLD；尚未实现从该路径自动反向交接到导航。

## 实现位置

- `platform_model.py`：板观测与配置校验；`platform_landing.py`：引导状态机。
- `platform_executor.py`：输出所有权、计算释放、有界历史和分段记录。
- `platform_bridge.py`：ROS 输入与状态发布；`platform_sih_feedback.py`：本机 SIH 回读。
- `platform_runtime.py` 仅兼容旧导入，运行配置与记录实现位于 `platform_executor.py`。

## 配置

`px4_interface_node` 与 `lifecycle_node` 的 `diagnostic_rate_hz` 默认 5 Hz，只限制可读 JSON 状态；关键状态变化同周期发布。控制、任务状态机和结构化 `ExecutionStatus` 的周期不变。

MAVROS `companion_attitude` 请求 ATTITUDE 20 Hz、LOCAL_POSITION_NED 2 Hz，EXTENDED_SYS_STATE、CURRENT_MODE、ODOMETRY 各 1 Hz；HIGHRES_IMU 由感知节点另请求 50 Hz。`px4_position` 保留原请求频率。消息请求不等于实际接收频率；串口波特率和 PX4 `MAV_i_RATE` 必须另行核对，本节点不写 PX4 参数。

姿态路线拒绝 RTT >20 ms 的同步样本；已收敛映射可保留至最后一个合格样本后 1 s，超限样本不刷新偏移或有效期。映射过期、飞控重启、ROS 时钟跳变仍须重新收敛。映射后稍处于未来的姿态/位置观测最多等待 20 ms，再按原采样时间和年龄限制检查；不提前用于配对。等待队列为空时取消轮询定时器，有待处理样本时才唤醒。

测试配置：`config/platform_landing_test.yaml`，必须显式使用 `test_only=true` 或 CLI 的 `--test-only`。参数均为合成值，不作为实测值。现有 VIO 起飞高度仍在 [config/runtime.yaml](config/runtime.yaml)；新引导器和独立 SIH 起飞高度取本配置 takeoff_height_m。

| 配置 | 内容 |
| --- | --- |
| board | family、name、每个 Tag 的 ID/黑色外边框实测尺寸（m）/`T_platform_tag`、落点、目标朝向 |
| camera | 固定图像尺寸、K/D、`T_body_camera`；只接受 pinhole/radtan，4 或 5 项畸变 |
| range | `T_body_sensor`；本阶段只接受机体向下的传感器 +z 波束，计入安装平移和飞控倾角 |
| quality | 最少 Tag 数、最小边长（px）、最大重投影误差（px）、可区分两解的误差比 |
| guidance | P 增益、水平/上升/下降/flare 速度、变化率、新鲜度、位姿/测距跳变、对准窗、起飞高度、采集/交接/接地确认超时 |
| evidence | 内参、相机外参、板实测、采样时域、测距、PX4 速度估计及近地验证记录文件 |

改板布局、落点、朝向、起飞高度或场地门限只改配置。当前检测器只支持共面的 Tag，平台 z=0；布局中的 Tag 旋转只能绕平台 +z。尺寸量到印刷 Tag 的黑色外边框，不包含纸张白边。板内坐标按角点对应定义：Tag +x 指印刷图案右侧，+y 指上侧，+z 指板正面；实际打印图案的旋转须写入布局。

真实配置必须 `synthetic: false`、`hardware_verified: true` 且七类证据文件存在。随后控制入口仍会报 `hardware_velocity_confirmation_adapter_not_verified`，不会连接或开放平台控制。文件存在本身不证明标定正确；本阶段不填假标定解除闸门。

## 坐标与输入

`T_A_B` 把 B 系坐标变到 A 系。相机采用 OpenCV optical（右、下、前），body 为 FLU（前、左、上），平台 +z 指板正面，PX4 控制系为局部 NED。

检测器先求 `T_camera_platform`，再算：

```text
T_body_platform = T_body_camera * T_camera_platform
落点误差（body） = T_body_platform * landing_point_platform
v_NED = R_NED_FRD(飞控 roll/pitch/yaw) * diag(1,-1,-1) * v_FLU
```

引导水平分量使用落点误差转到 NED 后的 x/y，不依赖 VIO/PX4 原点相同。平台 +x 和配置目标朝向经同一旋转生成偏航误差。测距从传感器 +z 波束转换到 NED，计入安装平移；平台视觉高度与测距高度必须一致，板法向须满足倾角门限。range_min/range_max 针对原始斜距。

观测输出含板名、机体相对位姿、采样时间、valid/reason、有效 ID、最小边长、重投影误差、两解比及质量字段。控制器再次核对 ID、质量、帧、时间与位姿。平面 PnP 的两解明显不同而重投影误差接近时拒绝；几何等价的两解不当作歧义。算法采用 [OpenCV IPPE](https://docs.opencv.org/4.12.0/d5/d1f/calib3d_solvePnP.html)，本机检测 API 验证版本为 OpenCV 4.6.0。

| ROS 接口 | 类型与时间 |
| --- | --- |
| /boom_birds/downward/image | sensor_msgs/Image；固定配置尺寸；header.stamp 必须为图像采样时间 |
| /boom_birds/platform/observation | std_msgs/String，BoardObservation JSON；stamp 保持 ROS 采样时域 |
| /boom_birds/platform/range | sensor_msgs/Range；header.frame_id=range_sensor；stamp 为测距采样时间 |
| /boom_birds/platform/request | std_msgs/String，`{"action":"land","session":"当前会话"}`；取消为 `{"action":"cancel"}` |
| /boom_birds/platform/status | std_msgs/String，状态、原因、会话/token/序号、速度、下降许可、依赖/释放状态和测试标记 |

ROS 桥接保留采样年龄，将时间映射到主机单调时钟。回放使用原 header.stamp，不用 rosbag 写入时间或读取时间替代。驱动/录包时间可追溯不证明曝光时间、图像—IMU同步或传感器时延。

## 交接与近地处理

`NAVIGATION → ACQUIRE → PREPARE → ALIGN → DESCEND/FLARE → TOUCHDOWN → COMPLETE`。

- ACQUIRE 保留 VIO、IMU与双目依赖，连续不同采样的板/测距观测合格；导航保持当前位置与高度，停止转发规划轨迹。没有板或缺导航依赖则等待，采集超时转原生 Land。
- PREPARE 撤销旧导航指令，关闭姿态发布者后建立速度发布者。只发零速度。MAVROS 后端绑定交接 token，并拒绝旧姿态及非速度 mask 的目标。
- 确认要求同一飞控 epoch、新鲜速度估计、连续独立回读、有效序号及实际速度目标一致；OFFBOARD 回读或本地 send 成功均不足。本机 SIH 读取 offboard_control_mode、trajectory_setpoint、vehicle_control_mode 和 vehicle_local_position；位置入口、姿态入口须关闭，速度控制与 Offboard 须启用。PX4 的 yawspeed 会使 OCM.body_rate 为 true，不能把它误判为第二条姿态输出。
- 确认前 VIO 不得释放。确认后 release_compute 才可请求停止 VIO/双目；只有停止回读通过才置 released。失败保留诊断，不自动声称停止成功。ComputeResources 只管理调用者持有的子进程，不枚举或终止其它任务。
- 对准需要连续合格样本；失准、错误 ID、过期/未来观测、丢标、位姿/测距跳变或估计器失效立即撤销下降，零下降速度不受原下降变化率延迟。持续故障、失去速度确认、飞控 epoch 变化、退出 Offboard 或人工取消转原生 Land。计算释放后不自动回到 VIO。
- 近地没有有效 Tag 或落入测距盲区时停止平台下降，超时交给原生 Land；不凭“看不见”推断接地。低高度、低速度与 PX4 landed 的连续确认窗满足后才请求 disarm；观测中断重置窗口。无接地回读超时转原生 Land。COMPLETE 还要求 armed=false。
- 原生 Land 的请求成功不代表已落地。仿真报告另检查最终 landed/Disarmed；实机的光流/测距融合与后备 Land 能力仍待接机验证。

`Px4InterfaceNode` 参数 `platform_config_file` 默认为空，`platform_test_only=false`。任务编排参数 `platform_landing_enabled=false`。显式启用后，Mission LAND 和既有任务进入 LANDING 时改走平台采集；起飞/往返导航沿用原路线。任务编排收到平台接管状态后停止导航动作，状态丢失请求原生 Land。ROS 桥接、任务门控与发布者互斥已有脱机测试；受控导航输入加平台收尾的 ROS/MAVROS/SIH 已执行；完整 EGO/OpenVINS 任务尚未执行。

计算释放服务为 /boom_birds/compute/release（std_srvs/Trigger）。ComputeReleaseNode 由持有 VIO/双目子进程的调用者构造，接收同会话、有效 token 与新鲜的交接确认后才接受停止请求；未确认、错误会话或退出未完成均回报 false。该节点须在独立 executor 中运行，避免等待进程退出阻塞控制周期。既有 launch 未托管这些进程句柄，未注册管理节点时不会停止计算。

SIH 任务 launch 可传 platform_config_file；非空时向控制节点转发测试配置并开启任务平台门控。下视 Image/Range 输入须另行提供，不默认打开真实下视相机。默认空配置保留原流程。

生产传输仍为 MAVROS。独立 SIH 脚本使用仅回环的历史 MAVLink 测试后端，不作为生产传输迁移回 pymavlink。

## 执行脱机与回放

在仓库根：

```bash
source companion/ros2_ws/tools/platform_env.sh
BB_PLATFORM_PREFIX=/home/waterc/bb_build/platform-20261007/install \
  bash companion/ros2_ws/tools/check_platform_landing.sh /tmp/platform-offline-new

python3 -m boom_birds_sensing.platform_replay \
  --config companion/ros2_ws/src/boom_birds_control/config/platform_landing_test.yaml \
  --manifest /path/to/frames.jsonl --test-only --out /tmp/platform-observations.jsonl

# 或 --bag /path/to/rosbag --topic /boom_birds/downward/image
# 支持 sensor_msgs/Image 和 CompressedImage。
python3 companion/ros2_ws/tools/run_platform_fake.py \
  --config companion/ros2_ws/src/boom_birds_control/config/platform_landing_test.yaml \
  --out /tmp/platform-fake-new
```

frames.jsonl 每行仅包含 `{"stamp":123.45,"image":"frame.png"}`，图像路径相对清单目录。空输入、缺采样时间、配置不全和图像尺寸不符均不产生有效观测。回放输出为逐帧 JSONL，旁边的 summary.json 保存完整配置与配置 SHA256。

只启动图像观测节点：

```bash
ros2 launch boom_birds_bringup platform_observation.launch.py \
  config_file:=/absolute/path/platform.yaml test_only:=true
```

不启动相机、不启动飞控通信，也不发控制命令。图像由回放或已有下视采集节点提供。

## SIH

仅在无其它 PX4 实例时运行。入口核验启动环境与 PID，只连接 127.0.0.1；不写 PX4 参数、不连接硬件。

```bash
source companion/ros2_ws/tools/platform_env.sh
python3 companion/ros2_ws/tools/run_platform_sih.py \
  --config companion/ros2_ws/src/boom_birds_control/config/platform_landing_test.yaml \
  --scenario return --out /tmp/platform-sih-return-new --allow-simulated-arming

bash companion/ros2_ws/tools/run_platform_sih_matrix.sh /tmp/platform-sih-matrix-new
```

输出目录必须是新的。矩阵包含 origin、return、tag_loss、range_jump、handoff_failure。默认由 PX4 真值渲染下视 Tag 图像，经过实际检测/PnP；测距和导航输入仍是合成值。独立脚本用 PX4 位置目标完成仿真起飞/外出/返回，再交接到平台速度目标；不运行 OpenVINS/EGO，也不证明姿态导航到平台速度的整段运动链。测试用 release receipt 没有实际 VIO/相机进程；真正子进程停止由脱机资源测试覆盖。

证据分开存储：回放图像/CDR/观测、模拟飞控逐步输入/输出/进程状态、SIH 配置/源码哈希/逐步输入与输出/uORB原文/控制台及最终模式。结果位置与本轮 PASS/FAIL 见 [STATUS](../../../../docs/STATUS.md)。相机精度、Pi 实时性、实际传感器融合、近地可见范围和落点精度均未验收。

## 有界运行与联合验证

可选 runtime 配置：history_capacity（默认 1000）、release_retry_s（0.2 s）、release_timeout_s（2 s）、release_max_attempts（5）、trace_queue_capacity（256）、trace_segment_records（1000）。history 超限移除最旧记录；ROS 参数 platform_trace_directory 非空时开启分段 JSONL。控制周期只尝试入队，队列满时记 dropped，不等待磁盘。分段文件不限制目录总容量，长时间运行前须安排磁盘保留策略。

计算释放的服务请求在独立 executor 内执行。等待结果时不重复发送；失败按间隔重试，总等待和尝试次数均有上限。结果必须匹配当前 token，取消、飞控 epoch 变化和超时后的迟到结果不能确认释放。状态包含 release_state、release_attempts 和 release_verified。

平台状态带 producer、status_sequence、stamp_monotonic、fcu_epoch。任务门控拒绝倒序、过期和生产者重启；重复消息不刷新有效期。释放管理节点再次核验状态年龄、序号、会话、token 和 epoch。这些单调时间只适用于同一主机。

下视 ROS 节点默认 process_latest_only=true：一条待处理图像，新帧替换旧帧；检测过程和状态机互不等待。observation_metrics 发布替换次数、处理数量、错误和最近 1000 次耗时。离线回放仍按输入顺序逐帧处理。

在仓库根执行，输出目录须不存在：

```bash
source companion/ros2_ws/tools/platform_env.sh
python3 companion/ros2_ws/tools/run_platform_ros_sih.py \
  --config companion/ros2_ws/src/boom_birds_control/config/platform_landing_test.yaml \
  --scenario return --control-mode companion_attitude \
  --out /tmp/platform-ros-return-new --allow-simulated-arming
python3 companion/ros2_ws/tools/scan_platform_offline.py \
  --config companion/ros2_ws/src/boom_birds_control/config/platform_landing_test.yaml \
  --out /tmp/platform-scan-new
python3 companion/ros2_ws/tools/soak_platform_offline.py \
  --config companion/ros2_ws/src/boom_birds_control/config/platform_landing_test.yaml \
  --out /tmp/platform-soak-new
python3 companion/ros2_ws/tools/check_platform_feature_track.py \
  --config companion/ros2_ws/src/boom_birds_control/config/platform_landing_test.yaml \
  --out /tmp/platform-track-new
```

联合脚本运行实际 Px4InterfaceNode、LifecycleNode 的 Mission.LAND、图像观测节点、MAVROS 和 ComputeReleaseNode。本机 PX4 产生反馈与 ULog；VIO/IMU/depth 是两个实际受管进程提供的合成输入，导航指令为受控 HOLD。停止这些进程证明资源交接流程，不能证明 OpenVINS 停止或真实相机释放。

scenario 支持 origin、return、service_late、tag_loss、range_jump、old_session。记录输入、配置、源码哈希、控制状态、MAVROS 输出、子进程状态、uORB 原文和 ULog；脚本不写 PX4 参数。未配置真实标定时仍拒绝真实运行。

[局部特征跟踪](../../../../docs/PLATFORM_FEATURE_TRACKING.md)采用 0.5～1.5 m 完整 Tag 捕获，再延续中心附近纹理。当前原型只输出像素目标，未接入下降许可。

A4 候选使用 config/platform_landing_a4_test.yaml（tag36h11 ID7，黑框 150 mm）；原 platform_landing_test.yaml 的 300 mm 板保留为既有回归基线。打印尺寸与白边见 [局部跟踪说明](../../../../docs/PLATFORM_FEATURE_TRACKING.md#板尺寸)。

70 mm 候选使用 config/platform_landing_70mm_test.yaml；ID 解码和位姿有效性分别检查，不能用解出 ID 单独开放下降。

## VIO 与飞控姿态配对

`companion_attitude` 保留最多 16 条待配对里程计，按输入顺序处理；新位姿到达和现有控制周期会重试。队列保留原始采样时间，仍执行 150 ms 位姿年龄、30 ms 配对差和 300 ms 已接受反馈间隔检查。不增加重试定时器，不允许外推飞控姿态。

队列过期不刷新旧反馈；时钟倒退、VIO 发布者变化、飞控重启或参考重置会清队列并闭锁。状态 `attitude_controller.pending` 与 `feedback_counters` 报告积压、接受、过期、重复、溢出和终止拒绝原因。等待配对期间旧反馈照常按采样年龄失效。

## VIO 空档恢复（工作区实现）

以下参数属于当前未提交的控制包改动；本次文档提交不包含实现。部署与验收状态统一见 [STATUS 状态表](../../../../docs/STATUS.md#实现默认入口部署与验收)。

| 参数 | 默认值 | 语义 |
| --- | --- | --- |
| recovery_mode | off | off 在姿态参考建立后，保持已接受反馈间隔超过 300 ms 后闭锁；test_dry_run 与 flight_bounded 在恢复边界内保留参考 |
| max_resume_gap_s | 1.0 s | 可恢复的相邻已接受 VIO 采样戳间隔上限 |
| max_resume_position_jump_m | 0.3 m | 空档前后位置变化上限 |
| max_resume_attitude_jump_rad | 0.2 rad | 空档前后四元数夹角上限 |

非关闭模式使用相同的恢复判断，保留冻结 offset、不重新对齐；已有闭锁不由该路径解除。test_dry_run 的名称不自动强制 dry_run，调用者仍须独立设置 dry_run、禁解锁和禁止硬件动作。flight_bounded 是实现中的模式名，不表示经过飞行验证。

参数在 px4_interface_node 声明并传入控制器，正式 attitude_hardware.launch.py 尚未增加同名参数透传。状态 attitude_controller 增加 recovery_mode、recovery_count、last_gap_s 与 last_recovery。夜间试验使用的 30 s/1 m/0.5 rad 不是默认值或飞行安全边界；测试结果见[夜间记录](../../../../docs/history/STATUS_2026-10.md#2026-10-22-超频降载与夜间长测交付)。

## 控制协议（EGO ↔ 控制）

两个 EGO 侧的入口由同一个 launch 参数 `require_session` 切换（EGO 子模块内 `src/planner/plan_manage/launch/boom_birds_offline.launch.py`、
`ego_replan_fsm.cpp`、`traj_server.cpp`）：`false` 走上游的 `planning/bspline` + `/position_cmd` 直连；
`true` 走下面这张会话化接口表。SIH 任务入口固定 `require_session=true`。

| 方向 | 话题/服务 | 类型 | QoS / 深度 | 谁发布 → 谁订阅 |
| --- | --- | --- | --- | --- |
| 编排 → EGO | `/boom_birds/planner/request` | `boom_birds_interfaces/PlannerRequest` | depth 50 | `lifecycle_node.py` → `ego_replan_fsm.cpp` |
| 编排 → 控制 | `/boom_birds/control/command` | `boom_birds_interfaces/ControlCommand` | depth 50（发布）/ reliable depth 10（订阅） | `lifecycle_node.py` → `px4_interface_node.py` |
| EGO → 编排 | `/boom_birds/planner/command` | `boom_birds_interfaces/ControlCommand` | depth 50 | EGO `traj_server.cpp` → `lifecycle_node.py` |
| 执行器 → 编排 | `/boom_birds/planner/executor_status` | `boom_birds_interfaces/PlannerStatus` | depth 10 | `traj_server.cpp` → `lifecycle_node.py` |
| EGO → 编排 | `/boom_birds/planner/status` | `boom_birds_interfaces/PlannerStatus` | depth 10 | `ego_replan_fsm.cpp` → `lifecycle_node.py` |
| 控制 → 编排/EGO | `/boom_birds/control/execution_status` | `boom_birds_interfaces/ExecutionStatus` | 发布 reliable depth 10（`QOS_RELIABLE`）/ 订阅 depth 50 ×3 | `px4_interface_node.py` → `lifecycle_node.py`、`ego_replan_fsm.cpp`、`traj_server.cpp` |
| 编排 → 控制 | `/boom_birds/control/action` | `boom_birds_interfaces/srv/VehicleAction` | 服务 | 客户端 `lifecycle_node.py`；服务端 `px4_interface_node.py` |
| 外部 → 编排 | `/boom_birds/mission` | `boom_birds_interfaces/srv/Mission` | 服务 | 服务端 `lifecycle_node.py` |
| 编排 → 外部 | `/boom_birds/mission/status` | `std_msgs/String`（JSON） | depth 10 | `lifecycle_node.py`（含 `hold_ready_gate` 与 `recovery` 诊断） |
| 控制 → 外部 | `/boom_birds/control/status` | `std_msgs/String`（JSON） | depth 10 | `px4_interface_node.py` |
| EGO 内部（会话模式） | `planning/session_bspline` | `traj_utils/SessionBspline` | depth 50 | `ego_replan_fsm.cpp` → `traj_server.cpp` |
| EGO 内部（上游模式） | `planning/bspline` | `traj_utils/Bspline` | depth 10 | `ego_replan_fsm.cpp` → `traj_server.cpp`；launch 重映射到 `/boom_birds/ego/planning/bspline` |
| 直连模式专用 | `/position_cmd` | `quadrotor_msgs/PositionCommand` | depth 50（发布）/ reliable depth 10（订阅） | `traj_server.cpp` → `px4_interface_node.py`；`frame_id` 必须在允许集合内，无会话、无序号 |

QoS 常量：`QOS_SENSOR` = best_effort / depth 5，`QOS_RELIABLE` = reliable / depth 10。

### session 语义

任务会话由 PX4 接口生成；EGO 与轨迹执行器另各自生成随机进程 ID。
`PlannerStatus.producer_session_id` 在规划器状态与 `/boom_birds/planner/executor_status` 中注册生产者。
编排器绑定本任务的两个进程 ID；执行命令、取消与内部 `SessionBspline` 均携带对应 ID。
活动任务中进程 ID 改变即 `session_changed` 闭锁，旧进程的序号或轨迹号不能重新解释为新命令。
EXECUTE/CANCEL/HOLD 共用可靠、有序的控制话题；取消形成退役屏障，后续旧轨迹不再发送。

`ExecutionStatus` 分开报告 accepted、sending 与模式回读。恢复还需新鲜 CURRENT_MODE 的实际/意图模式、
ODOMETRY reset counter、落地状态及 SIH PX4 原生 failsafe 原因；缺少任一必要观测拒绝恢复。
对齐 yaw/translation 由控制后端回读，编排器按同一变换换算位置和速度；运行中变化闭锁。

- **开会话**：`VehicleAction.OPEN_SESSION` 生成 `uuid4` 会话并把闸门上限绑定到当时的飞控启动周期。开会话**不是**规划拒绝，它清掉上一轮的 `PLANNING_REJECTED` 闭锁。
- **消息字段**：`ControlCommand.msg` 的字段是 `header / session_id / planner_session_id / executor_session_id / trajectory_id / sequence / valid_for / command_type / position /
  velocity / acceleration / yaw / yaw_rate`——**没有 `frame_id` 字段**；坐标系由 `std_msgs/Header.header.frame_id` 携带，`control_protocol.Command.frame` 即取自它。`command_type` 常量
  `EXECUTE=1 / CANCEL=2 / HOLD=3`。
- **接收校验（顺序即拒绝优先级）**：会话一致 → `sequence` 严格递增 → `trajectory_id > 0` → `command_type` 合法 →
  `CANCEL` 形成屏障（退役并取消，不需延续旧 setpoint）→ 未退役/未回退的轨迹 → 全量有限 → `frame` 非空且 `0 < valid_for ≤ max_ttl`
  （`max_ttl = command_timeout_s`，0.2 s）→ `-0.03 ≤ ros_now - stamp < valid_for` → 时钟回退即 `clock_reset`。
- **有效期**：`deadline = mono_now + min(valid_for, stamp + valid_for - ros_now)`，`active()` 每周期复核，过期即 `cancel("expired")`。`traj_server` 发出的 `ControlCommand.valid_for` 取 `command_timeout_s`（当前 200 ms）。
- **trajectory id**：`traj_server` 每接受一条新的 `SessionBspline` 序号即 `++project_trajectory_id_` 并写入消息；编排器侧在轨迹号变化时再 `+1`，`recover`/`replan` 时也 `+1`。
- **retire / `clear_planning_rejected`**：`PLANNING_REJECTED` 是**外部事件闭锁**，只能由**新** `trajectory_id` 清除——同一 id 继续出现
  只说明规划器还在拒绝同一个目标。
  `TRAJECTORY_INVALIDATED`（`trajectory_flag != READY`、空 Bspline、`order == 0`）由"带**不同** id 的 setpoint 到达"清除。
- **EGO 侧门限**：`traj_server` 只接受"会话一致 ∧ `offboard_confirmed` ∧ 授权时间戳 0.2 s 内"的样条；`cmdCallback` 每周期复核授权，
  失配即清空轨迹并停发。`ego_replan_fsm` 在 `projectRequest` 里对 `offboard_confirmed`、
  odom、`mapReady()` 三项不满足时**直接返回**，并在 `execFSMCallback` 里同样按 0.2 s 窗口门控。

门限数值取 RuntimeConfig，见[参数来源](../../README.md#参数来源)。accepted 表示协议接收，sending 表示后端发送，均不等于 PX4 已进入 Offboard；须检查新鲜模式回读。停发 setpoint 后的 PX4 接管仍需独立验收。

## 位置 setpoint 兼容入口

px4_position 与 companion_attitude 互斥。直连模式接收 quadrotor_msgs/PositionCommand；位置、速度、加速度和偏航统一经过局部系到 NED 的变换及新鲜度检查。航向与原点对齐证据不完整时拒绝输出；飞控重启或坐标系重置后不得续用旧轨迹。

默认假后端、dry-run、禁解锁：

~~~bash
ros2 launch boom_birds_nav px4_interface.launch.py
~~~

参数见 [px4_interface.yaml](config/px4_interface.yaml)。MAVROS 是唯一 FCU 连接；算法不直接打开串口，不输出 PWM/DShot。allow_setpoint=false 仅表示停止外部发送，不代表 PX4 已悬停或接管。
