# 日志分析工具

两个脚本都读取**外层** `data/px4_logs/` 的历史 ULog（按脚本自身位置解析路径：`tools/../..`），与终端当前目录无关；输出写到外层 `reports/flight_analysis/`。需要换日志目录时只改脚本内的 `ROOT`/`LOGDIR`，不要移动原始日志。

依赖：`numpy` 与 `pyulog`（见仓库根 `requirements.txt`；pyulog 版本未在仓库内记录）。用法：

```bash
python3 tools/analyze_height_logs.py   # 写出 metrics.json 与 log_summary.json
python3 tools/height_timeline.py       # 打印 z / 设定值 / 对地距离 / 油门时间线
```

| 脚本 | 用途 | 输出 |
| --- | --- | --- |
| `analyze_height_logs.py` | 分析 3 个历史 ULog 的高度、测距、估计器切换与执行器指标 | `reports/flight_analysis/height_hold_2026-08-27/metrics.json`、`log_summary.json` |
| `height_timeline.py` | 按 1 秒步长打印同一批日志的关键量，仅供人工查看 | 终端输出 |

两个脚本默认只处理具体文件名（`log_0..2_UnknownDate.ulg`）。外层日志目录还有其它架次，需要分析别的日志时改脚本内的文件名列表，并在交接记录中写明所用日志。

证据边界：这些脚本产出的是历史日志的**再分析结果**，不能证明当前实板 revision、烧录版本或飞行能力，也不替代新的台架或飞行测试。
