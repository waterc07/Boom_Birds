# ULog 离线分析

依赖见根目录 `requirements.txt`，其中 pyulog 固定为本次核验的 1.2.4。使用独立 Python 环境安装，不把 pip OpenCV 装入 ROS 环境。显式传入日志文件，输出使用新目录：

```bash
python3 tools/analyze_height_logs.py /absolute/path/log_27.ulg --out /absolute/path/new-analysis
python3 tools/height_timeline.py /absolute/path/log_27.ulg
```

`analyze_height_logs.py` 输出 `metrics.json` 与 `log_summary.json`；`height_timeline.py` 打印秒级时间线。数据集字段须符合脚本读取的 PX4 日志版本，结果不代替原始日志复核。

相机预览、标定及回放见 [ROS 开发工具](../companion/ros2_ws/tools/README.md)。旧的显示旋转截图脚本及 `camera_orientation.json` 已移除；算法方向由标定的 `image_orientation` 定义。
