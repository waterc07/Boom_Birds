# Boom Birds

AI 工作先读 [当前状态](docs/CURRENT_STATUS.md) 和 [交接约定](docs/AI_WORKFLOW.md)；开发行为以根 AGENTS.md 为准。

本目录为正式开发根目录（2026-09-15 从外层资料目录迁入）。历史日志、手册与规则原件位于上一级；源码与标定保持原模块布局。

## 开发入口（2026-09-15）

- [目录与开发工作流](docs/workflows/PROJECT_LAYOUT.md)：本地、树莓派与 WSL 的分工。
- [双目深度与 XYZ 程序](companion/stereo_depth/README.md)：已从树莓派复制完整开发副本及标定、样例。
- [PX4 源码与固件索引](px4/README.md)：源码保留 WSL，已有固件副本与来源清单收录到本项目。
- VS Code 直接打开本目录；当前未发现独立 `.code-workspace` 文件。

当前深度代码为实验版本，距离准确性与飞行应用未验收；本次目录整理没有重新构建、刷写或进行飞行测试。下文原阶段规划保留，硬件进度以对应证据为准。

RoboMaster 2027 微型空间智能无人机预研工作区。

当前阶段目标是完成一台可稳定飞行、日志完整、支持 Offboard、便于扩展的全包围四旋翼 V0 验证机。2026-09-09官方书面规则前瞻为当前首要规则依据；在官方后续规则发布前，新文明确项优先，未提及或未写明项沿用原规范。详见 `docs/RULE_BASELINE.md` 和 `docs/RULE_REVIEW_2026-09-09.md`。

## 当前文档

- `docs/PROJECT_CONTEXT.md`：项目上下文、当前工程基线与历史方案边界。
- `docs/DECISIONS.md`：版本化决策记录及其失效条件。
- `docs/REQUIREMENTS.md`：可追踪需求、验收方法和开放项。
- `docs/RULE_BASELINE.md`：当前强制规则转录、工程解释和缺失定义。
- `hardware/bom/bom.csv`：V0 硬件 BOM 与核验状态。

## 当前阶段

先完成飞控、动力和基础定位闭环，再引入 Companion、自主算法和超级电容研究。任何新器件进入 CURRENT BASELINE 或 LOCKED 前，必须核对尺寸、重量、功耗、接口和 PX4 兼容性，并在 `docs/DECISIONS.md` 留下证据。

## Git 开发

本地 main 分支已建立；远端 origin 已配置，2026-09-19 提交 f1c301f 后本地领先 origin/main 1 个提交、尚未推送（见 DECISIONS D-023），首次发布的公开权限审查待记录。分支、提交、忽略范围及环境差异见 [Git 工作流](docs/workflows/GIT_WORKFLOW.md)。
