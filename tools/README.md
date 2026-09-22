# 历史日志分析工具

两个脚本分析原有 `log_0..2_UnknownDate.ulg`；依赖 `numpy` 和 `pyulog`，见根 requirements.txt。

| 脚本 | 用途 |
| --- | --- |
| `analyze_height_logs.py` | 高度、测距、估计器切换与执行器指标；写出 metrics.json 与 log_summary.json |
| `height_timeline.py` | 按 1 秒步长打印高度、设定值、对地距离与油门 |

**WSL 目录适配尚未完成。** 脚本仍用 `Path(__file__).resolve().parents[2]` 定位旧外层，在当前 WSL 布局下会指向 `/home/waterc/workspace`；真实资料根为 `/mnt/d/Users/Admin/Desktop/G-Master/Boom_Birds`。因此不能按旧 README 直接运行并声称找到原日志。

后续应为脚本增加明确的数据根/输出目录配置，分别指向资料根的 `data/px4_logs/` 与 `reports/flight_analysis/`；不要搬动原始日志来迎合旧路径。本轮仅更新此说明，未修改或执行脚本。

历史日志再分析不证明当前实板 revision、烧录版本或飞行能力，不替代台架和飞行验收。
