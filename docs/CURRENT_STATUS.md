# 当前状态：AI 快速入口

快照日期：2026-09-15。开始新任务时核验相关易变状态；本文件不是持续在线监控。

## 已完成

- 本地主目录 Git 已初始化，分支 main，忽略规则和换行配置已完成；工作流见 workflows/GIT_WORKFLOW.md。

- 开发目录为内层 Boom_Birds；外层资料按根 README 分类。
- 深度项目已导入 companion/stereo_depth，导入时 334 个非缓存文件 SHA256 一致，见 `workflows/stereo_import_20260915.json`。
- 已有 StereoSGBM、XYZ/深度、预览与保存实现，65 mm 标定、原始样例和历史性能记录；不代表距离精度验收通过。
- WSL PX4 独立仓库已存在；当日 HEAD 为 ff5b9484369b714763db9638517c08df0c242237。两个已有固件副本及校验见 `../px4/manifests/wsl_inventory_20260915.json`；本次没有重新构建或刷写。
- 内层 CodeGraph 已生成，快照为 4 个 Python 文件、81 节点、200 关系；后续以 codegraph status 为准。
- 外层 data/px4_logs 已有历史 ULog，不是“没有日志”；尚不能据此认定已与当前实板 revision、烧录版本完整对应。

## 未完成 / 不可推定

- 尚未配置远端、CI 或发布标签。
- 无自动部署、回滚、新机初始化、视频回放 CLI 或已验证 ARM64 发布包。
- 当前程序不是已验收 ROS 2 节点；Jazzy 是基线，安装状态需核验。
- 距离精度、长期实机性能、飞控 target 与实板匹配、安全接管和飞行仍需对应证据。
- Pi 5 为验证平台；RK3576 迁移未验证，最终机载计算板未由此锁定。

用户当前任务优先；后续候选任务见 NEXT_TASK。未来步骤清单不自动授权执行硬件操作。
