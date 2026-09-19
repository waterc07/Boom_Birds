# 2026-09-19 文档与仓库一致性修复

- 目标与完成条件：把文档描述与仓库实际状态对齐（Git 远端、CodeGraph 计数、标定证据链、BOM 重复行、依赖声明、脚本入口），完成条件为每项都有可复核证据且不改变任何硬件结论。
- 工作目录 / 设备：内层 `Boom_Birds`；未连接树莓派，未启动相机、未刷写、未推送、未改远端设置。
- 版本：基于提交 `9e3e376` 的工作区；本次改动随 `f1c301f` 一并提交（29 个文件，含 09-16/09-17 未提交实现），本地 `main` 领先 `origin/main` 1 个提交，未推送。

## 已完成文件

- DECISIONS：追加 D-022（20 mm 棋盘标定成为深度基线）、D-023（远端 origin 状态记录），表头版本 0.5→0.6、最近更新 2026-09-19。
- CURRENT_STATUS：快照日期改为 2026-09-19（事实截至 2026-09-17）；补 Git 远端与未提交清单、CodeGraph 7 文件/163 节点/412 关系、默认标定不在版本控制内。
- REQUIREMENTS：版本 0.6→0.7，基线日期 2026-09-16（规则依据仍为 2026-09-09 手册）。
- NEXT_TASK：远端状态与待办；任务编号 `P0-FC-001` 统一为 `FC-001（P0 优先级）`。
- README、GIT_WORKFLOW、PROJECT_LAYOUT：远端 origin 事实、公开权限审查待办、CodeGraph 现状、目录职责表补充、路径基准说明。
- COMPANION_DEV_ENV：第 4 节“标定”行由 TBD 更新为 2026-09-16 实测结果，并加 2026-09-19 状态补充。
- AGENTS：更新日期、CodeGraph 计数、Git 表述（本仓库已有 Git，验证须给出实际命令与输出）。
- hardware/bom/bom.csv：第 17 行与 `CAM-STEREO-DEV-001` 交叉引用；新增 `hardware/bom/README.md` 说明列含义与两行关系。
- 新增 `tools/README.md`、根 `requirements.txt`；模块 README 补依赖版本、默认标定未纳管说明与常见问题。
- 代码：`depth_preview.py` 缺标定文件时给出可操作错误；`live_calibration.py` 补 height 偶数校验；`height_timeline.py` 加 docstring、`main()` 与缺文件提示；`resume_dev_backgrounds.py` 加 docstring、入口保护与缺记录提示。

## 验证

- 事实核对：`git remote -v` / `git rev-parse HEAD origin/main` / `git status --short`；CodeGraph 库内 `files` 表逐条 SHA256 比对；`candidate.npz` SHA256 与任务记录一致（d27f24de…）；已跟踪文件名扫描无凭据类条目。
- 语法：改动后的 Python 文件通过 `python -m py_compile`。
- 链接：文档内 Markdown 相对链接全部可解析。
- 未执行：单元测试（本机无 OpenCV；设备端 10 项通过记录见 `docs/tasks/2026-09-16-live-stereo-calibration.md`）、任何设备与硬件动作。

## 未验证 / 下一步

- 未验证：米制距离精度、端到端延迟、ROS 2 安装状态、RK3576 迁移、实板 revision 与规则合规；均不在本次范围。
- 下一步：完成首次发布的公开权限审查并在 GIT_WORKFLOW 记录证据，再决定是否推送 `f1c301f`。
- 限制：保留未提交与未跟踪工作，未改写规则原件、历史快照与旧决策。
