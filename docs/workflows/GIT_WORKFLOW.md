# Git 工作流

2026-09-15 建立本地仓库；默认分支 `main`。提交身份继承电脑已有 Git 配置；没有创建远端仓库、推送或发布标签。

## 日常开发

在内层开发根目录执行：

```bash
git status --short
git switch -c feat/stereo-replay
# 完成修改和相关检查后，明确指定要提交的路径
git add <文件路径>
git diff --cached --check
git diff --cached
git commit -m "功能：增加双目离线回放"
```

使用 `feat/`、`fix/`、`docs/` 加主题的短期分支。合并前审查差异并验证，main 表示集成基线，不等于飞行认证。设备差异放配置，不为每架飞机建立分支。暂不需要长期 develop 分支。

并行任务需要时，在同级独立目录创建 worktree；勿让多个任务同时修改同一工作副本。新 worktree 不自动包含被忽略的样例与固件，按导入清单补充所需数据。

## 版本控制范围

跟踪源码、中文文档、BOM、标定、小型历史 benchmark、固件来源和校验清单。不跟踪缓存、凭据、原始 captures/depth_outputs、固件二进制或本机进程记录。外层历史资料、WSL PX4 和树莓派目录不属于本仓库；clone 不会带上它们，单独获取并核验。

忽略仅避免误提交，不删除文件或替代凭据审查。首次远端发布前另行检查整个可达历史和资料公开权限。

## 仓库本地配置

- `.gitattributes` 控制换行；core.autocrlf=false，safecrlf=warn。
- core.quotepath=false：中文路径直接显示。
- pull.ff=only：拉取时拒绝自动产生分叉合并，先检查差异。
- fetch.prune=true：获取时清理已删除的远端跟踪引用。
- merge.conflictStyle=zdiff3：冲突显示共同基线。
- push.default=simple：推送使用当前同名上游分支。

这些设置仅存在本仓库 `.git/config`，不会修改全局或 WSL 仓库，也不会随 clone 自动传递。远端地址尚未指定；实际接入远端时再配置 origin 与上游。Git 提交不是额外的数据备份，外层原始资料需要独立备份。
