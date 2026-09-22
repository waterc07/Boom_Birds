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

这些命令要求目录调整已提交并同步至目标版本。是否已经获取该布局，以检出的父仓库提交为准。
不要使用 `git submodule update --remote` 做日常部署，以免绕过父仓库固定版本。

## 现有深度程序

从本目录进入 `src/stereo_depth`，按模块 README 安装 Python 依赖，再运行 `python3 depth_preview.py --help`。树莓派当前部署路径尚未迁移，不因本地重排自动改变。

当前 WSL 未发现 `/opt/ros`，且 `import cv2` 报缺少模块。ROS 2 尚未安装/构建验收；后续补齐 stereo_depth 的 package.xml、安装入口、节点接口与 launch。统一采集输出分别供 OpenVINS 与深度分支，IMU 来自飞控。

## 项目记忆与外部仓库

新任务从根 AGENTS.md、docs/CURRENT_STATUS.md、docs/PROJECT_CONTEXT.md 与 docs/NEXT_TASK.md 恢复项目知识。Windows 个人记忆/会话数据库不随 Git 克隆，未在此复制或改写。
PX4 仍在 `/home/waterc/PX4-Autopilot`，独立 MAVLink 仍在 `/home/waterc/mavlink`。
