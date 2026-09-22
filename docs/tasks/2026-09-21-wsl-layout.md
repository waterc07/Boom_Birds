# WSL 源码布局迁移

更新：2026-09-22。

- 主目录：`/home/waterc/workspace/Boom_Birds`，基于 `83859b3`；Windows 旧工作副本完整归档于外层 `.local/backups/windows-retirement-20260922/`，不再作为开发目录。
- `companion/stereo_depth` 已移至 `companion/ros2_ws/src/stereo_depth`。
- 迁移清单：`../workflows/stereo_move_20260921.json`；36 个文件 SHA256 全部一致，5 个 Python 文件 AST 解析通过。
- `git check-ignore` 验证新位置的 captures、depth_outputs 和 ROS build/install/log 被忽略。
- `core.filemode=true`，临时文件 chmod 执行位检查通过，探针已删除。
- ROS 2 未发现安装目录，cv2 导入失败；未运行算法或构建，不宣称 ROS 2/ARM64 兼容通过。
- 2026-09-22 已复用用户普通克隆，执行 `git submodule add -b <branch> <fork-url> <path>` 与 `git submodule absorbgitdirs`，保留独立历史。
- OpenVINS：`https://github.com/waterc07/open_vins.git`，master，`69488123ed9362dd44b6f28e7f4680abbff1442b`。
- EGO：`https://github.com/waterc07/ego-planner-swarm.git`，由干净 master 切换 ros2_version，`a3e14dd1ec3dbcec4619ccc9049b888bbcdcee6d`。
- `git submodule status` 与索引 gitlink 对应；两个子模块 `git status --porcelain` 为空，`git fsck --connectivity-only` 返回 0。
- 主仓库目录改动以重命名审查；`git diff --cached --check` 通过。2026-09-22 用户授权提交推送，最终 SHA 以 Git 日志为准。
- PX4、独立 MAVLink、树莓派和个人记忆/会话目录未改动。
- 深度源码仍为独立 Python 程序，ROS 2 封装待下一步实现。
