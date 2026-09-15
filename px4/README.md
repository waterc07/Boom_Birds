# PX4 开发与固件索引

源码保留在 WSL Ubuntu-24.04：`/home/waterc/PX4-Autopilot`。

在 WSL 终端中进入源码：

```bash
cd /home/waterc/PX4-Autopilot
git status --short
git describe --always --tags
```

`manifests/` 保存 2026-09-15 读取的源码和固件信息；`firmware/` 为当日复制的已有构建产物，不是本次新构建。两个 target 均保留，不能据此直接决定烧录哪个。

固件内部 `git_identity`、board 信息和 SHA256 见清单；实际飞控 revision、当前烧录版本及硬件验证结果仍需单独核验。主仓库不管理上游整份源码或构建缓存。
