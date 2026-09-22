# 机载 ROS 2 工作空间

开发目标：Ubuntu 24.04 / ROS 2 Jazzy。正式项目根为 `/home/waterc/workspace/Boom_Birds`。

## 目录与状态

- `src/stereo_depth/`：已迁入原有 Python 双目深度程序，保留算法、页面、标定与证据；尚未封装为 ROS 2 包，不会因为放入 src 自动生成节点。
- `src/open_vins/`：个人 fork `https://github.com/waterc07/open_vins`，初始检出 master。
- `src/ego-planner-swarm/`：个人 fork `https://github.com/waterc07/ego-planner-swarm`，初始检出 ros2_version。
- 两个依赖使用 Git submodule，父仓库记录精确提交；分支只是维护方向，不代表 Jazzy 或 ARM64 兼容性已验证。
- `build/`、`install/`、`log/` 由构建工具生成，已忽略，不复制到树莓派。适配包待接口设计后创建，不建立空壳冒充实现。

## 获取完整源码

在网络及 Git 认证可用的环境中：

```bash
git clone --recurse-submodules https://github.com/waterc07/Boom_Birds.git
# 已克隆且包含本次目录调整提交时：
git submodule update --init --recursive
git submodule status
```

实际源码版本以检出的母仓库及子模块 gitlink 为准。
不要使用 `git submodule update --remote` 做日常部署，以免绕过父仓库固定版本。

## 现有深度程序

从本目录进入 `src/stereo_depth`，按模块 README 安装 Python 依赖，再运行 `python3 depth_preview.py --help`。树莓派当前部署路径尚未迁移，不因本地重排自动改变。

环境验证和依赖阻塞统一见 [STATUS](../../docs/STATUS.md)。深度节点封装与共享采集仍待实现。

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

以下命令在主工程根 `/home/waterc/workspace/Boom_Birds` 执行；涉及子模块改动时再进入对应子模块。

```bash
git status --short --branch
git switch -c docs/topic
# 修改并检查后，明确指定路径
git add <文件路径>
git diff --cached --check
git diff --cached
git commit -m "docs: 说明本次修改"
```

使用 `feat/`、`fix/`、`docs/` 短期分支；main 为集成基线，不代表飞行认证。设备差异通过配置管理。需要隔离同时进行的任务时使用独立 worktree。

### 子模块

- OpenVINS 维护基线 `master`；EGO-Planner 维护基线 `ros2_version`，均为 `waterc07` 个人 fork。
- 先在子模块内提交和推送依赖改动，再在母仓库更新 gitlink 并提交，确保他人可获取对应 SHA。
- 初次获取或部署执行 `git submodule update --init --recursive`；不使用 `--remote` 绕过固定提交。
- 子模块按固定 SHA 检出后处于 detached HEAD 属正常状态；需要修改时先切到明确的开发分支。
- 后续 Jazzy 适配可分别建立 `boombirds-jazzy` 分支；目前只是建议，尚未创建。

### 跟踪范围与环境

跟踪源码、中文文档、BOM、默认标定和小型验证/来源清单。不跟踪原始 captures/depth_outputs、固件二进制、缓存、凭据、构建目录和本机进程记录。

Linux `core.filemode=true` 保留执行位，换行规则由 `.gitattributes` 管理。其余 Git 配置先用 `git config --show-origin --get <key>` 核验，不假定旧 Windows 仓库的本地配置会随 clone 迁移。

```bash
git status --short --branch
git submodule status
git rev-parse HEAD origin/main
# 需要确认远端实际状态时（会访问网络）：
git ls-remote origin refs/heads/main
```

`origin/main` 是本地缓存，不等于实时远端。Git 提交不替代外部原始资料备份。新增公开发布范围时检查资料权限与可达历史；现有推送状态不依赖旧文档中的 ahead/behind 数字。
