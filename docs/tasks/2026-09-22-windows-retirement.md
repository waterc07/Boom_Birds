# Windows 重复代码副本清理与 WSL 发布

日期：2026-09-22。用户授权：整理并提交推送，同时清理 Windows 代码工程部分。

- 唯一主开发目录：WSL Ubuntu-24.04 `/home/waterc/workspace/Boom_Birds`。
- 发布前检查：迁移 36 文件哈希不变，5 个 Python 文件 AST 通过，子模块工作区干净、对象连通性通过，差异格式检查通过。未安装 ROS 2/OpenCV，未进行运行或硬件验证。
- Windows 原目录：外层资料目录下的 `Boom_Birds/`。Git main 与远端基线 `83859b3` 一致，无工作区改动、stash 或独有分支。
- 完整备份：外层 `.local/backups/windows-retirement-20260922/windows-code-before-cleanup.zip`，545 文件，99,149,936 字节；同目录 manifest.json 记录每文件 SHA256 与 ZIP SHA256，压缩包读取及逐项校验通过。
- 原始测量：外层 `data/stereo_depth/windows_snapshot_20260922/{captures,depth_outputs}`，305 文件复制前后 SHA256 一致。
- 清理顺序：先提交推送 WSL，核对远端 SHA，再将明确的旧内层目录送入回收站，更新 Windows 外层 README、AGENTS 与 VS Code 入口；不永久删除备份。
- 历史资料、外层 `.local`、个人记忆/会话、WSL PX4/MAVLink、树莓派均保留。
- WSL 独立 clone 不包含 Windows 外层历史原件；需要日志/规则原件时从 `/mnt/d/Users/Admin/Desktop/G-Master/Boom_Birds` 访问，不按 WSL 仓库上一级寻找。
