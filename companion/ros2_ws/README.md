# 机载 ROS 2 工作空间

开发目标：Ubuntu 24.04 / ROS 2 Jazzy。正式项目根为 `/home/waterc/workspace/Boom_Birds`。

## 目录与状态

- `src/stereo_depth/`：自研双目深度程序及 ROS 2 算法包；默认标定随包安装，深度发布节点位于 `boom_birds_sensing`。
- `src/open_vins/`：个人 fork `https://github.com/waterc07/open_vins`，初始检出 master。
- `src/ego-planner-swarm/`：个人单机 EGO fork，维护分支 `boombirds-jazzy`；目录名保留，精确版本由 gitlink 固定。已删除多机通信、协作和专用消息。

- 两个依赖使用 Git submodule，父仓库记录精确提交；分支只是维护方向。Jazzy/x86_64 已构建，ARM64 未验证。OpenVINS fork 的上游真值/评估表不再随当前版本跟踪，仿真所需 `ov_data/sim/` 仍保留；旧提交历史中的数据不会自动消失。
- `src/stereo_depth/` 同时作为 ROS 2 包安装（`ament_cmake`）：算法模块进 site-packages，默认标定进 share，供脱机链路复用同一套几何运算，不复制第二份实现。
- `src/boom_birds_{interfaces,sensing,control,bringup,sim}/`：接口、感知、控制、编排与仿真；职责见项目 README。
- `src/boom_birds_nav/`：兼容模块与 launch 转发；操作说明与集成测试保留在此，契约位于 `boom_birds_interfaces/config/contract.yaml`。
- PX4 通信由 ROS 2 MAVROS 统一持有 FCU 连接；控制、SIH 真值与 IMU 通过 ROS 话题/服务接入。`mavros_imu_node` 使用 MAVROS router 的 HIGHRES_IMU 与 sys_time 同步偏移；旧 `mavlink_imu_node` 可执行名转发到该入口。`pymavlink`/`pyserial` 仅用于历史脱机回归与测试对端。

- `tools/setup_python_env.sh`、`tools/activate_python_env.sh`、`tools/build_all.sh`：隔离环境与受限构建入口（见下节）。
- `build/`、`install/`、`log/` 和本机测试证据已忽略；构建默认放在仓库外的持久目录，不复制到树莓派。

## 单机 EGO 安装前缀

EGO 单机化改变了 `traj_utils/Bspline` 消息定义，旧构建前缀不能混用。本次安装前缀为 `/home/waterc/bb_build/ego-single/install`；运行工具前显式指定：

```bash
export BUILD_BASE=/home/waterc/bb_build/ego-single/build
export INSTALL_BASE=/home/waterc/bb_build/ego-single/install
export LOG_BASE=/home/waterc/bb_build/ego-single/log
bash companion/ros2_ws/tools/build_all.sh --packages-up-to ego_planner boom_birds_nav map_generator mockamap --symlink-install --cmake-args -DBB_BUILD_GRIDMAP_TESTS=ON
source "$INSTALL_BASE/local_setup.bash"
```

## PX4 通信（MAVROS）

安装 MAVROS 与其 GeographicLib 数据：

```bash
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-msgs
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
```

单独启动连接和 IMU（SIH motion launch 已包含连接，不要重复启动）：

```bash
# 终端 1，在项目根目录执行；安装前缀须对应本次构建。
source companion/ros2_ws/tools/activate_python_env.sh
source "${INSTALL_BASE:-/home/waterc/bb_build/ego-single/install}/local_setup.bash"
ros2 launch boom_birds_bringup mavros.launch.py

# 终端 2，同样加载以上环境后执行。
ros2 run boom_birds_sensing mavros_imu_node
```

SIH 启停步骤见 [集成操作说明](src/boom_birds_nav/README.md)。当前验收结果见 [STATUS](../../docs/STATUS.md#2026-10-02-px4-通信迁移到-mavros)。

默认 `fcu_url=udp://127.0.0.1:14540@127.0.0.1:14580`。控制 launch 选择 `backend:=mavros`，默认 `dry_run:=true`、`allow_arming:=false`；dry_run 抑制项目模式/解锁/setpoint，MAVROS 自身仍有心跳和时间同步流量。

## 获取完整源码

在网络及 Git 认证可用的环境中：

```bash
git clone --recurse-submodules https://github.com/waterc07/Boom_Birds.git
# 已克隆且包含本次目录调整提交时：
git submodule update --init --recursive
git submodule status
```

源码版本以母仓库提交及子模块 gitlink 为准。
不要使用 `git submodule update --remote` 做日常部署，以免绕过父仓库固定版本。

## 现有深度程序

从本目录进入 `src/stereo_depth`，按模块 README 安装 Python 依赖，再运行 `python3 depth_preview.py --help`。树莓派当前部署路径尚未迁移，不因本地重排自动改变。

环境验证与当前边界见 [STATUS](../../docs/STATUS.md)。深度节点封装已完成；真机共享采集与飞控 IMU 接口待验证。

## 脱机开发环境与构建（WSL）

本机 `~/.local` 下的 numpy 2.5.2 会遮蔽系统 numpy 1.26.4，导致系统 OpenCV 4.6（NumPy 1.x ABI）
与 rclpy 导入失败。使用隔离 venv 和系统 dist-packages，设置 PYTHONNOUSERSITE=1，
不改动用户目录与系统 Python：

```bash
export BOOM_BIRDS_VENV=$HOME/bb_build/architecture/venv
bash companion/ros2_ws/tools/setup_python_env.sh --install  # 创建环境并安装声明依赖
source companion/ros2_ws/tools/activate_python_env.sh  # ROS jazzy + venv
python3 -c "import numpy, cv2, rclpy, cv_bridge; print('ok')"
```

构建统一走 `tools/build_all.sh`：它加锁串行、限制包内并发与单进程地址空间。
2026-09-22 曾因两处重度构建并行编译把 WSL 发行版拖到无响应（`Wsl/Service/0x8007274c`），
因此**不要**绕过该脚本自行并发构建。

```bash
BUILD_BASE=/home/waterc/bb_build/main/build INSTALL_BASE=/home/waterc/bb_build/main/install LOG_BASE=/home/waterc/bb_build/main/log   bash companion/ros2_ws/tools/build_all.sh --packages-up-to boom_birds_nav
source /home/waterc/bb_build/main/install/setup.bash
ros2 pkg executables boom_birds_sensing
```

构建产物默认放 `/home/waterc/bb_build/main/{build,install,log}`，在仓库外持久保存。WSL x86_64 的构建结果不推定 ARM64 / 树莓派可用。

脱机分层验证（层 2：合成双目 → 深度 → 位姿适配）：

```bash
# 1) 先生成合成标定（TEST-ONLY），否则节点会按契约显式报错退出
python3 -m boom_birds_sim.synthetic --write-calibration /tmp/boom_birds_synth/synthetic_candidate.npz
# 2) 启动链路
ros2 launch boom_birds_nav synthetic_layer2.launch.py
# 3) 另一个终端检查
ros2 topic list | grep boom_birds
ros2 topic hz /boom_birds/depth/image
```

真实相机采集仍按 `src/stereo_depth/README.md` 在树莓派上运行；WSL 侧只做文件与合成输入。

## 环境与部署

WSL 与 Pi 5 分别验证，不由本机结果推定设备已安装或构建成功。WSL 非交互终端显式加载：

```bash
source /opt/ros/jazzy/setup.bash
command -v ros2
```

| 项目 | 已有记录与限制 |
| --- | --- |
| 平台 | Raspberry Pi 5 / Ubuntu Server 24.04；2026-09-16 有 aarch64 运行记录，内存、内核及精确镜像需重新核验 |
| SSH | `gmaster@192.168.137.200`，历史连接地址，使用前核验 |
| 当前部署路径 | `/home/gmaster/boom_birds_ws/stereo_depth` |
| 历史 Python 依赖 | OpenCV 4.6.0、NumPy 1.26.4；不推定设备环境始终不变 |
| ROS 2 | 开发基线 Jazzy；设备安装状态尚未独立确认 |
| 后续平台 | RK3576；具体板卡、系统和联合性能待验证，未锁定最终机载板 |

设备凭据仅保存在 Windows 私有资料区，不写入版本控制。部署前比对现场改动，保持测量数据。设备不在线时不把历史 IP 或进程记录视为当前状态。

需要连接核验时可执行：

```bash
hostnamectl
id
cat /etc/os-release
dpkg --print-architecture
uname -a
ip -brief address
ls /opt/ros
# 确认安装 Jazzy 后：
source /opt/ros/jazzy/setup.bash
command -v ros2
printenv ROS_DISTRO
```

相机模式、标定与运行命令见 [深度模块](src/stereo_depth/README.md)。硬件同帧同步不证明全局快门或曝光时序精度；相机—IMU 外参与时间同步、运动适用性仍需验证。

部署前比对现场源码和配置，保留标定与测量数据；当前按模块说明维护固定部署目录，自动部署/版本化回退尚未实现。设备密钥和飞控校准独立管理，WSL x86_64 产物不能作为 ARM64 包。PX4 与独立 MAVLink 位置见 [项目入口](../../README.md#源码与资料位置)。

## Git 与版本管理

以下命令在 `/home/waterc/workspace/Boom_Birds` 执行；子模块改动在对应子模块提交。

```bash
git status --short --branch
# 母仓库本轮直接在 main 集成；修改并检查后明确指定路径
git add <文件路径>
git diff --cached --check
git diff --cached
git commit -m "更新项目文档"
```

母仓库当前在 `main` 集成；并行修改使用独立 worktree。
`main` 只代表代码集成基线，不代表飞行认证。设备差异通过配置管理；提交消息使用中文。

### 子模块

- OpenVINS 维护基线 `master`；EGO-Planner 维护基线 `ros2_version`，均为 `waterc07` 个人 fork。
- 两个子模块的本轮工作分支均为 `boombirds-jazzy`（分别位于各自仓库）。
- 本地可先提交子模块并更新母仓库 gitlink；对外推送时必须先推两个子模块提交，再推母仓库，确保他人可获取对应 SHA。
- 初次获取或部署执行 `git submodule update --init --recursive`；不使用 `--remote` 绕过固定提交。
- 子模块按固定 SHA 检出后处于 detached HEAD 属正常状态；需要修改时先切到明确的开发分支。
- `boombirds-jazzy` 已建立，仅记录 Jazzy/WSL 脱机适配，不宣称真实 VIO 或 ARM64 验收。

### 跟踪范围与环境

只跟踪源码、测试、配置、当前文档、BOM 和运行必需的默认/回归标定。运行日志、性能测量、过程清单、原始 captures/depth_outputs、固件二进制、缓存、凭据和构建目录均留在本机，不纳入新提交。历史 Git 对象不会因取消跟踪自动消失。

Linux `core.filemode=true` 保留执行位，换行规则由 `.gitattributes` 管理。其余 Git 配置先用 `git config --show-origin --get <key>` 核验，不假定旧 Windows 仓库的本地配置会随 clone 迁移。

```bash
git status --short --branch
git submodule status
git rev-parse HEAD origin/main
# 需要确认远端实际状态时（会访问网络）：
git ls-remote origin refs/heads/main
```

`origin/main` 是本地缓存，不等于实时远端。Git 提交不替代外部原始资料备份。新增公开发布范围时检查资料权限与可达历史；现有推送状态不依赖旧文档中的 ahead/behind 数字。

## 独立审查修复的复现入口

构建默认使用持久目录 ~/bb_build/main/{build,install,log}，从任意工作目录调用脚本均可。
只运行 WSL 合成数据；ROS_DOMAIN_ID 隔离其他任务，不使用全局 pkill。

```bash
cd /home/waterc/workspace/Boom_Birds
bash companion/ros2_ws/tools/build_all.sh --packages-up-to ego_planner stereo_depth boom_birds_nav
source companion/ros2_ws/tools/activate_python_env.sh
source /home/waterc/bb_build/main/install/setup.bash
ctest --test-dir /home/waterc/bb_build/main/build/ego_planner -R '^trajectory_validation$' --output-on-failure
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 python3 -m pytest companion/ros2_ws/src/boom_birds_nav/test -q
ROS_DOMAIN_ID=175 ROS_LOCALHOST_ONLY=1 python3 companion/ros2_ws/src/ego-planner-swarm/src/planner/plan_manage/scripts/test_executor_invalidation.py --out companion/ros2_ws/log/review_fix/executor_60s.json --rejection-duration 60
ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1 python3 companion/ros2_ws/src/ego-planner-swarm/src/planner/plan_manage/scripts/run_review_integration.py --out-dir companion/ros2_ws/log/review_fix/accepted --duration 40
ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1 python3 companion/ros2_ws/src/ego-planner-swarm/src/planner/plan_manage/scripts/run_review_integration.py --out-dir companion/ros2_ws/log/review_fix/gated --duration 120 --expect-gated
ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1 python3 companion/ros2_ws/src/ego-planner-swarm/src/planner/plan_manage/scripts/run_review_integration.py --out-dir companion/ros2_ws/log/review_fix/rejected --duration 120 --expect-rejected
ROS_DOMAIN_ID=182 ROS_LOCALHOST_ONLY=1 python3 companion/ros2_ws/src/ego-planner-swarm/src/planner/plan_manage/scripts/test_motion_reset.py --out-dir companion/ros2_ws/log/review_fix/reset
```

MAVLink IMU 上行/时间同步的脱机测试（全部使用构造的模拟 MAVLink 消息与合成 `.tlog`，
不连接设备；节点级测试在 UDP 回环上构造真实节点）：

```bash
cd /home/waterc/workspace/Boom_Birds
source companion/ros2_ws/tools/activate_python_env.sh
bash companion/ros2_ws/tools/build_all.sh --packages-up-to ego_planner boom_birds_nav --cmake-args -DBB_BUILD_GRIDMAP_TESTS=ON
source /home/waterc/bb_build/main/install/setup.bash
ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 python3 -m pytest \
    companion/ros2_ws/src/boom_birds_nav/test -q
# 记录数据回放自检（无记录文件时跳过；报告为 JSON）
python3 -m boom_birds_nav.mavlink_imu_replay <record.tlog> --out /tmp/replay_report.json
# 相机帧时间戳能力核验（只读；退出码 0 = 时域可核实）
python3 -m boom_birds_nav.camera_timestamp --probe --device /dev/video0
```

回放与合成数据只能证明算法与失效路径，**不能**当作真机 IMU 频率、时间同步误差或
相机曝光时间戳的证据。

发布前约束使用导数控制点范数的凸包保守上界，覆盖整个有效时间区间；
上界可能比曲线真实峰值大，因此可能保守拒绝。最多修正三次，每次检查优化成功状态，
包含第三次修正后的结果。最终路径使用速度上界包围每个时间段，检查相交的膨胀地图体素，
不将有限采样最大值称为严格极值。就绪门控至少等待地图占据概率达到阈值所需的正命中次数；
轨迹优化在本脱机配置中检查完整路径。正后方目标 (2.5, 0, 1.2) m 暂不纳入本阶段规划验收；已有测试显示当前 EGO 优化器持续拒绝，证据保留在 log/review_fix/final_center/。默认合成目标 (2.5, 1.2, 1.2) m 位于 x=3 m 背景墙前并绕开前方障碍，
占据目标 x=3 m 用于拒绝测试。静止起点的速度修正只拉伸时间，不改变避障路径；
运动边界保留再优化并进行最终全路径校验。
合成重置中 origin_offset_m 仅平移位姿，scene_x_offset_m 可独立改变相机相对场景；
旧链路停止、新坐标系重启后必须重新建图和规划。
未知区域的总体探索策略仍按 EGO 原有行为，本补丁不提供未知区域安全保证。

本工程 EGO/traj_server 增加本地失效约定：同一 planning/bspline 可靠话题上，
order=0 且 pos_pts 为空表示旧轨迹失效。规划拒绝或未就绪时发送该消息，
执行端停止 PositionCommand，直到下一条有效轨迹到达。
这不是物理悬停或飞控接管保证；未来控制接口必须处理命令失效。
其他 Bspline 消费者也须识别失效消息，不得当普通轨迹解析。
紧急停止轨迹沿用原有单独路径，不等于已经通过本次正常轨迹验证。

历史 20260922/traj_analysis_final.json 为 FAIL（一次碰撞），
traj_analysis_gated.json 为另一场门控测试 PASS，不覆盖前者。
本地验证记录位于 `log/review_fix/`。历史文件不能代替当前测试。

## 架构修整后的脱机回归入口

```bash
bash companion/ros2_ws/tools/build_all.sh --packages-up-to ego_planner boom_birds_nav --cmake-args -DBB_BUILD_GRIDMAP_TESTS=ON
bash companion/ros2_ws/tools/check_offline.sh
```

入口运行可移植行为、导航、配置、包归属、双目标定、接口、launch 可执行文件、地图 C++ 行为及 trajectory_validation 共 13 组；不会启动 SIH 或连接设备。
结果写入 `companion/ros2_ws/log/architecture_checks/report.json`，包含命令、Git 状态、耗时和日志路径；
pytest 的跳过项记为 PARTIAL，不计全量通过。SIH 与硬件另行验收。
`INSTALL_BASE`、`BUILD_BASE` 覆盖主安装/构建目录，`OV_INSTALL` 覆盖独立 OpenVINS 安装目录；
验收入口加载其 `local_setup.bash`，避免找到可执行文件却缺动态库。

MAVLink 的 WSL 已验证 Python 版本列在 `requirements-mavlink.txt`。
新环境安装到工作空间 venv；不要用 pip 替换 ROS 所用的系统 NumPy/OpenCV：

```bash
python3 -m venv companion/ros2_ws/.venv
bash companion/ros2_ws/tools/setup_python_env.sh
companion/ros2_ws/.venv/bin/python -m pip install -r companion/ros2_ws/requirements-mavlink.txt
```

环境脚本已移除用户目录 `.pth` 注入；`PYTHONNOUSERSITE=1`，缺依赖明确失败。
既有环境保留，独立前缀验证不替换 ROS 系统 NumPy/OpenCV，也不代表新操作系统镜像复现。

### 独立验收环境

`BOOM_BIRDS_VENV` 选择 Python 环境，`BUILD_BASE`、`INSTALL_BASE`、`LOG_BASE` 选择仓库外构建目录。
MAVLink 依赖来自 `requirements-mavlink.txt`；NumPy/OpenCV 使用系统包，不读取 `~/.local`。

```bash
export BOOM_BIRDS_VENV=$HOME/bb_build/architecture/venv
export BUILD_BASE=$HOME/bb_build/architecture/build
export INSTALL_BASE=$HOME/bb_build/architecture/install
export LOG_BASE=$HOME/bb_build/architecture/log
bash companion/ros2_ws/tools/setup_python_env.sh --install
bash companion/ros2_ws/tools/build_all.sh --packages-up-to ego_planner boom_birds_nav map_generator mockamap --cmake-args -DBB_BUILD_GRIDMAP_TESTS=ON
bash companion/ros2_ws/tools/check_offline.sh --out "$HOME/bb_build/architecture/evidence/current/report.json"
```

验收报告记录母仓库/子模块 SHA、工作区文件散列、依赖版本、配置、命令、跳过项及退出状态；
每个测试组有 600 s 超时，结束后清理该组进程。缺少测试二进制或 OpenVINS 安装不能计为全量 PASS。
OpenVINS 订阅测试使用 `OV_INSTALL`（默认 `$HOME/bb_build/ov/install`），上面的构建命令不重新构建它。
这是当前 WSL 内的新 venv/构建目录验收，不是新操作系统镜像复现。CI 构建项目 Python 包和接口，执行同一入口的 10 组；显式排除依赖 EGO/OpenVINS 的 navigation、map_behavior、trajectory_validation，并记录在报告。

## Companion 姿态控制实机入口

`ros2 launch boom_birds_bringup attitude_hardware.launch.py` 启动真实采集、MAVROS IMU、OpenVINS、深度、EGO、位置闭环和编排；必须显式提供设备 URL、双目标定、VIO 标定、机体外参和控制参数。默认 dry-run/禁解锁，缺参数拒绝启动。命令与分阶段验收见 [bringup README](src/boom_birds_bringup/README.md)。
