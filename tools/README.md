# 工具

## 历史日志分析

两个脚本分析原有 `log_0..2_UnknownDate.ulg`；依赖 `numpy` 和 `pyulog`，见根 requirements.txt。

| 脚本 | 用途 |
| --- | --- |
| `analyze_height_logs.py` | 高度、测距、估计器切换与执行器指标；写出 metrics.json 与 log_summary.json |
| `height_timeline.py` | 按 1 秒步长打印高度、设定值、对地距离与油门 |

**WSL 路径尚未适配。** 脚本仍用 `Path(__file__).resolve().parents[2]` 定位旧外层，在当前 WSL 布局下会指向 `/home/waterc/workspace`；真实资料根为 `/mnt/d/Users/Admin/Desktop/G-Master/Boom_Birds`。因此不能按旧 README 直接运行并声称找到原日志。

脚本需要数据根/输出目录配置，分别指向资料根的 `data/px4_logs/` 与 `reports/flight_analysis/`；不要搬动原始日志来迎合旧路径。脚本尚未修改或重新运行。

历史日志不能证明当前实板 revision、烧录版本或飞行能力。

## 装机相机预览

2026-10-15 确认安装方向：USB 双目每目旋转 180°；下视原图右侧对应机头，预览逆时针旋转 90°。配置在 `camera_orientation.json`。双目保持原拼接通道顺序，物理左右对应尚未遮挡核验。

Pi 上后续取图使用固定入口，输出目录必须不存在。工具依赖 OpenCV、NumPy、`v4l2-ctl`、`media-ctl` 和 Pi 已有的 `mipi_raw.py`；默认路径为 `/home/gmaster/boombirds/tools/mipi-calibration-20261008`，可通过 `--mipi-tools` 指定。配置默认读取脚本同目录的 `camera_orientation.json`：

```bash
/home/gmaster/boombirds/capture-cameras.sh --output /tmp/camera-view-NEW
```

`stereo.jpg`、`downward.jpg` 已校正方向；`*-raw.jpg` 保留原图，`capture.json` 保存方向及设备信息。只取一路可加 `--camera stereo` 或 `--camera downward`。Pi 配置在 `/home/gmaster/boombirds/tools/camera-orientation/camera_orientation.json`，不依赖 current release。

旋转仅用于预览和图片导出；现有 ROS、深度、VIO、标定工具不自动套用。不能直接把旋转图交给旧 CameraInfo、双目校正或相机—IMU外参。方向记录不代表这些几何关系已验收。
