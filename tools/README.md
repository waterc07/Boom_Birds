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

## 历史装机截图工具

tools/capture_camera_views.py 与 camera_orientation.json 保留 10-15 的截图约定：双目每目旋转 180°，保留原 A/B 拼接顺序；下视逆时针旋转 90°。这份配置只用于旧截图导出，没有随 10-22 主链切换物理左右身份。

当前 USB 双目主链按原 B 转正作新左目、原 A 转正作新右目，ROS 图像、CameraInfo、NPZ、VIO chain 与角点同步转换。当前预览和左右身份检查使用[深度预览工具](../companion/ros2_ws/tools/depth_preview/README.md)，方向契约见[运行库](../companion/ros2_ws/src/stereo_depth/README.md#数据接口)，Pi 部署与用户确认见 [STATUS](../docs/STATUS.md#当前配置与标定)。旧 A/B 截图不能直接交给新标定。

旧 Pi 截图入口记录如下；使用前核对脚本版本和输出方向，输出目录必须不存在：

```bash
/home/gmaster/boombirds/capture-cameras.sh --output /tmp/camera-view-NEW
```

工具依赖 OpenCV、NumPy、v4l2-ctl、media-ctl 和 Pi 的 mipi_raw.py；默认 MIPI 工具目录为 /home/gmaster/boombirds/tools/mipi-calibration-20261008，可通过 --mipi-tools 指定。默认读取脚本同目录的 camera_orientation.json；旧 Pi 配置位于 /home/gmaster/boombirds/tools/camera-orientation/camera_orientation.json，不依赖 current release。

stereo.jpg、downward.jpg 是按旧配置旋转后的导出，*-raw.jpg 保留原输入，capture.json 保存方向及设备信息。只取一路可加 --camera stereo 或 --camera downward。下视原图右侧对应机头的记录不等同于实测安装外参。
