# PX4 开发与固件索引

源码保留在 WSL Ubuntu-24.04：`/home/waterc/PX4-Autopilot`。

在 WSL 终端中进入源码：

```bash
cd /home/waterc/PX4-Autopilot
git status --short
git describe --always --tags
```

2026-09-15 的 `manifests/` 来源清单与 `firmware/` 历史构建产物仅在原 WSL 工作区保留，不再纳入 GitHub 新提交；它们均不是本次新构建。旧清单中记录了两个 target 的 `git_identity`、board 信息和 SHA256，但不能据此决定实际烧录目标。实板 revision、当前烧录版本及硬件结果仍须单独核验；主仓库不管理上游整份源码或构建缓存。

## FC-001 实板验收清单

以下是待执行方案，不自动授权刷写或硬件动作；项目现状见 [STATUS](../docs/STATUS.md)。清单中的仓库相对路径均从主工程根解析。

任务编号：FC-001（P0 优先级）  
状态：READY，厂商资料已获得，等待实板执行  
范围：确认已购入 MicoAir743v2-AIO-45A 的 PX4 target、刷写和安全上电路径；不接桨、不开始 Offboard、不改 PX4 控制算法。

### 为什么先做这个

当前已归档两个既有固件和历史飞行日志，但实板 revision、当前烧录版本、参数与接线之间的可追溯对应仍未完成。飞控 target、实物 revision 和引脚一旦认错，会同时阻塞 DShot、UART、MTF-02P、日志和 Companion 接口，因此应先形成唯一可信的实板基线。

### 输入

必须至少获得以下之一，最好全部获得：

1. 已确认的采购型号 MicoAir743v2-AIO-45A 和厂商用户手册；
2. 到货 PCB 正反面清晰照片和丝印版本；
3. 厂商 pin map、固件说明和 PX4 target 资料；
4. 45A AIO 的 ESC 固件/版本说明；
5. 实板、数据线、限流电源或开发电池、烟雾阻断器。

### 执行清单

1. 在 `hardware/pinout/` 保存原始资料及来源、获取日期和硬件版本。
2. 在 PX4 源码/官方资料中确认准确 board target；不得仅按 MCU 型号猜测。
3. 建立引脚表：USB、供电、4 路电机、接收机、至少两个可用 UART、I2C、调试口、电压/电流采样。
4. 明确 4-in-1 ESC 固件、DShot600 支持、双向 DShot/eRPM 能力和电机输出映射。
5. 无桨、限流上电，完成固件刷写、重启和 USB/串口连接。
6. 记录 PX4 版本、board target、bootloader、默认参数差异和首次启动日志。
7. 校验 IMU、气压计、电压/电流采样、microSD/日志和四路 actuator test；执行器测试期间不得安装桨叶。
8. 将证据写入 `px4/notes/MICOAIR743V2_BRINGUP.md`，并同步更新 BOM、需求状态和决策记录。

### 完成条件

- 准确硬件 revision 与 PX4 board target 有可追溯证据；
- 固件可重复刷写和启动，传感器健康状态明确；
- 四路电机输出、UART、I2C、电源采样和日志接口均有明确映射；
- DShot600 在无桨条件下完成四路输出验证；
- 未确认能力仍明确标为 TBD，不以“能刷机”替代运行验证；
- `FC-001` 有足够证据更新为 PASS，或以具体失败证据更新为 FAIL。

### 安全边界

- 全过程拆除桨叶，首次上电使用限流/烟雾阻断措施。
- 在 UART 映射、5 V 供电能力和公共地确认前，不连接 MTF-02P 或 Companion。
- 刷写成功只证明固件链路可用，不证明传感器、DShot、日志或飞行状态正常。

### 规则核对补充

当前任务安全边界不变。后续比赛合规需增加RULE-011～019验证；优先复核全包围保护罩（机体不得外露）和柔性桨材料，并在无桨条件验证紧急停桨遥控链路。人工飞行仅属开发验证，比赛要求全自动；返库后回传、10 s悬停及沿用30 s架次限制见 [规则基线](../docs/RULE_BASELINE.md)。本次规则阅读不更新任何硬件实测结果。
