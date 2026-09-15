# 项目目录与开发入口

AI 工作先读 [当前状态](../CURRENT_STATUS.md) 和 [交接约定](../AI_WORKFLOW.md)；开发行为以根 AGENTS.md 为准。

更新：2026-09-15。正式开发根目录为外层资料目录中的 `Boom_Birds/`。已完成迁移，尚未实现自动部署。

## 目录职责

| 目录 | 用途与管理方式 |
| --- | --- |
| `companion/stereo_depth/` | 从树莓派复制的完整双目深度开发副本；代码、标定与历史记录保留原布局 |
| `px4/manifests/` | WSL PX4 提交、子模块状态及固件来源清单 |
| `px4/firmware/` | 已有固件的本地副本；忽略二进制，只跟踪清单 |
| `docs/` | 规则、需求、决策、工作流及验证索引 |
| `hardware/` | BOM、接线及硬件证据 |
| `tools/` | 现有日志分析工具 |
| `../data/px4_logs/` | 现有 ULog，保留位置，避免破坏工具路径；不进入普通 Git |
| `../references/manuals/`、`../references/rules/` | 历史厂商与规则原件，保留位置和现有引用 |
| `../reports/flight_analysis/`、`../artifacts/calibration/` | 历史分析和导出结果，本次保留 |
| `../archive/temporary/`、`../.local/` | 临时内容、本机配置与备份，不进入 Git |

尚无机库代码，不建立无实现的模块骨架。后续实际开发时增加 `hangar/`、`deploy/`、`config/`。

## 两个开发入口

- 主工程：电脑上的内层 Boom_Birds；VS Code 直接打开内层根目录；当前未发现独立工作区文件。
- PX4：WSL `Ubuntu-24.04` 中 `/home/waterc/PX4-Autopilot`，保持独立上游 Git 仓库。不要把整个 PX4 源码搬到 Windows 或嵌套提交到主仓库。
- 树莓派现场副本：`gmaster@192.168.137.200:/home/gmaster/boom_birds_ws/stereo_depth`。地址可能变化。本次只读取和复制，未更改远程目录或停止服务。

当前主工程已初始化本地 Git，使用 main 分支；已审查初始文件清单并配置忽略和换行规则。日常操作见 GIT_WORKFLOW.md，尚未配置远端。PX4 保留其已有 Git 历史。每项功能单独分支，多任务并行使用独立 worktree；设备差异使用配置，不为每架飞机建立分支。

## 深度程序使用

详细参数、依赖与证据限制见 [深度模块说明](../../companion/stereo_depth/README.md)。本地副本支持离线阅读和修改，不代表已实现命令行视频回放模式，也不代表 Windows 相机采集可用。

在已安装 NumPy、OpenCV 的 Linux 环境中，从主项目根目录运行：

```bash
cd companion/stereo_depth
python3 depth_preview.py --help
python3 depth_preview.py
```

后一条会尝试打开真实相机。不要在现有采集程序运行时启动第二实例。复制来的 README 中旧远程路径仍是有效现场路径，历史数据里的绝对路径保留作为来源证据。

标定与源码一起保留；`captures/`、`depth_outputs/` 完整复制到本地但由 Git 忽略。本次不是距离精度、实时性能或飞行能力验收。

## 修改与交接

1. 日常在电脑开发副本修改；离线测试后记录提交或文件校验值。
2. 连接树莓派后，先比对源码，防止覆盖设备上更新的工作。
3. 需要远程临时修改时，先取回差异，再形成统一版本；当前没有自动双向同步。
4. 后续部署脚本应传输版本包到独立目录，完成检查再切换，保留前一版本；本次不声称这些脚本已实现。
5. 新设备可复用同平台程序，但设备编号、密钥、相机标定和飞控校准须独立配置。

## PX4 固件使用边界

已有 `micoair_h743-lite_default` 与 `micoair_h743-v2_default` 两个固件副本。名称和当前源码提交不足以证明实板 target 正确，也不能单凭源码 HEAD 推断旧产物来源。本次保存固件内部元数据和 SHA256，不重新构建、不刷写、不更改参数。查看 [PX4 入口](../../px4/README.md)。

## 恢复

整理前的根 README、AGENTS 与忽略文件保存在 `../.local/backups/organization-20260915/`。树莓派原始目录和 WSL PX4 均保留，可用导入清单重新比对。历史资料现已分类移入外层 data、references、reports、artifacts、archive；原文件保留，标定数据不变。

## 内外层关系

开发内容 companion、docs、hardware、px4、tools 和根配置已迁入内层。外层 README 为入口。历史数据不属于可独立克隆的源码内容；单独复制内层时需另外提供外层规则原件和日志。两个日志工具按脚本位置定位外层目录，与终端当前目录无关。外层 .codegraph 为旧索引，未迁入或重建。

迁移前 360 个文件的路径与 SHA256 清单位于外层 .local/backups/organization-20260915/nesting-before.json；移动完成后全部一致，随后仅更新目录说明和两个日志工具路径。

## 外层资料分类（2026-09-15）

当前外层 data/px4_logs 保存原始 ULog；references/manuals 与 references/rules 保存原始手册和规则；reports/flight_analysis 保存分析结果；artifacts/calibration/pdf 保存棋盘格 PDF 及其生成脚本；archive/temporary 保存旧临时渲染资料。生成脚本与 PDF 一起保留，路径基于脚本自身目录。旧决策中的路径是当时记录，以此处映射为准。

.local 保留凭据与备份；.codegraph 因后台占用留在原处，不能用于内层源码定位。没有删除文件或停止服务。

## CodeGraph 当前状态

2026-09-15 已在内层开发根目录执行 codegraph init，索引 4 个源码文件、81 个节点和 200 条边。后续在内层执行 codegraph sync；需要全量重建时执行 codegraph index --force。外层旧索引不使用，WSL PX4 不包含在本索引中。
