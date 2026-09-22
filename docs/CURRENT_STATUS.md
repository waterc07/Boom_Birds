# 当前状态：AI 快速入口

快照日期：2026-09-19（事实截至 2026-09-17）。开始新任务时核验相关易变状态；本文件不是持续在线监控。

## 已确认路线（2026-09-21，尚未集成验收）

用户确认 Ubuntu 24.04 + ROS 2（Jazzy 基线）、双目图像 + 飞控 IMU 的 OpenVINS 定位、自算双目深度/XYZ 建图，以及个人规划仓库 https://github.com/waterc07/ego-planner-swarm 。已核实该 fork 存在 ROS 2 分支，具体分支与提交仍待兼容验证后锁定。完整边界见 PROJECT_CONTEXT 第4节与 DECISIONS D-024；本次仅确定路线，不新增构建、台架或飞行证据。以下保留原有日期的实现快照。

## WSL 目录迁移（2026-09-21）

主开发副本改为 `/home/waterc/workspace/Boom_Birds`，原深度目录迁入 `companion/ros2_ws/src/stereo_depth`，36 个文件移动前后 SHA256 一致。2026-09-22 已将用户普通克隆的两个个人 fork 登记为 submodule：OpenVINS master `6948812`、EGO ros2_version `a3e14dd`，工作区均干净且 Git 对象连通性检查通过。见 `tasks/2026-09-21-wsl-layout.md`。Windows 旧代码副本归档后清理，原始采集/深度输出保留于外层 `data/stereo_depth/windows_snapshot_20260922/`；树莓派部署、既有 PX4/MAVLink 均未移动；项目文档随仓库保留，个人记忆/任务历史未迁移。当前 WSL 未发现 `/opt/ros` 安装，未进行 ROS 2 或 ARM64 构建，stereo_depth 仍为独立 Python 程序。

## Windows 副本退出开发（2026-09-22）

用户授权提交推送 WSL 迁移并清理 Windows 代码副本。清理前确认 Windows Git 工作区干净、无 stash 或独有分支；545 个文件已完整 ZIP 备份并逐项 SHA256 校验。305 个原始采集/深度输出文件另存于 Windows 外层 `data/stereo_depth/windows_snapshot_20260922/`，复制后哈希一致。备份与资料均不上传 Git。Windows 外层入口改指向 WSL；旧内层代码目录在远端 SHA 核验后移入回收站。详见 `tasks/2026-09-22-windows-retirement.md`。下方 Git 与 CodeGraph 描述属于旧日期快照，不代表当前状态。

## 已完成

- 2026-09-16 新增双目实时调焦/引导标定页面，默认 2560×960 同帧原图、11×8 内角点、20 mm。
  独立预览、自动角点采样、覆盖引导、单目/双目求解、留出验证及校正视图已实现；
  深度程序支持 `--calibration`，采集模式匹配标定尺寸。见 `companion/ros2_ws/src/stereo_depth/LIVE_CALIBRATION.md`。
  树莓派合成几何与流程测试通过，高分辨率实机取帧及网页原图已验证；已完成 30 组真实棋盘标定并设为默认：基线 67.6718 mm，双目 RMS 0.4018 px，6 组留出垂直 P95 0.4924 px；完整 XYZ/Z 实机输出检查通过，独立距离精度仍未验证。
  部署沿用固定目录直接更新，不保存旧工程副本；测量记录仍保留。
  同日预览改为逐帧取最新图像、减少调焦后台解码负载，10 项测试通过；代码补全/AI 后台暂时暂停，
  恢复命令及缓存清理记录见模块 LIVE_CALIBRATION.md。真实端到端延迟仍未测量。

- 本地主目录 Git 已初始化，分支 main，忽略规则和换行配置已完成；远端 `origin` 已配置（github.com/waterc07/Boom_Birds）。2026-09-19 提交 `f1c301f` 后本地 `main` 领先 `origin/main`（`9e3e376`）1 个提交，尚未推送；首次发布的公开权限审查尚未记录，见 DECISIONS D-023 与 workflows/GIT_WORKFLOW.md。
- 09-16/09-17 的双目标定、标定页面、预览改动及 09-19 文档一致性修复已随 `f1c301f` 提交（29 个文件），工作区干净；标定数据 `calibration/live_20260916_210120_642136/` 已纳入版本控制，见 `docs/tasks/2026-09-16-live-stereo-calibration.md` 与 `docs/tasks/2026-09-19-docs-consistency-sync.md`。

- 开发目录为内层 Boom_Birds；外层资料按根 README 分类。
- 深度项目已导入 companion/ros2_ws/src/stereo_depth，导入时 334 个非缓存文件 SHA256 一致，见 `workflows/stereo_import_20260915.json`。
- 已有 StereoSGBM、XYZ/深度、预览与保存实现，旧 65 mm 标定、新 20 mm 棋盘标定、原始样例和历史性能记录；不代表距离精度验收通过。
- WSL PX4 独立仓库已存在；当日 HEAD 为 ff5b9484369b714763db9638517c08df0c242237。两个已有固件副本及校验见 `../px4/manifests/wsl_inventory_20260915.json`；本次没有重新构建或刷写。
- 内层 CodeGraph 已更新：2026-09-19 核验为 7 个 Python 文件、163 节点、412 关系，7 个文件内容哈希与索引一致（无待同步）；后续以 `codegraph status` 为准。
- 外层 data/px4_logs 已有历史 ULog，不是“没有日志”；尚不能据此认定已与当前实板 revision、烧录版本完整对应。

## 未完成 / 不可推定

- 已配置远端并于 2026-09-21 推送 `83859b3`；尚无 CI 或发布标签验收记录。
- 无自动部署、回滚、新机初始化、视频回放 CLI 或已验证 ARM64 发布包。
- 当前程序不是已验收 ROS 2 节点；Jazzy 是基线，安装状态需核验。
- 距离精度、长期实机性能、飞控 target 与实板匹配、安全接管和飞行仍需对应证据。
- Pi 5 为验证平台；RK3576 迁移未验证，最终机载计算板未由此锁定。
- 默认标定 `calibration/live_20260916_210120_642136/candidate.npz` 已跟踪并随目录迁移保留；CodeGraph 索引和大型原始采集不随 clone 携带。

用户当前任务优先；后续候选任务见 NEXT_TASK。未来步骤清单不自动授权执行硬件操作。
