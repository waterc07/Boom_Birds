# 双目 XYZ 与深度预览

新增：[双目视频流调焦与引导标定](LIVE_CALIBRATION.md)。使用 `live_calibration.py`，
原图为 `2560×960`、每目 `1280×960`，棋盘默认 11×8 内角点、20 mm；自动收集角点，无需保存多张照片。
当前默认使用本次 20 mm 棋盘的新标定。其他标定可通过 `depth_preview.py --calibration <candidate.npz>` 显式加载，
采集尺寸随所选标定匹配；不会自动替换旧标定，也不自动认定距离精度通过。

从一帧 USB 左右拼接图像生成米制深度及 XYZ 坐标，并通过浏览器实时预览、保存数据。当前程序为 `depth_preview.py`；当前源码与默认标定在 WSL 主工程；历史设备采集和保存数据的位置以当时记录为准。

**当前状态：试验验证阶段。** 新标定覆盖检查及留出几何验证通过，距离尚未独立尺测验证；黑色无效区域不能理解为空闲空间。代码运行和标定残差不代表飞行避障已验收。

## 1. 环境与当前参数

| 项目 | 当前值 |
| --- | --- |
| 主机 | Raspberry Pi 5 / Ubuntu 24.04 ARM64 |
| SSH | `gmaster@192.168.137.200`，网络变化后需确认地址 |
| 项目目录 | `/home/gmaster/boom_birds_ws/stereo_depth` |
| 依赖 | 树莓派实测 Python 3 / OpenCV 4.6.0 / NumPy 1.26.4；HTTP 使用 Python 标准库；依赖清单见仓库根 requirements.txt |
| 输入 | `/dev/video0`，MJPEG `2560×960`，同帧 A/B 左右拼接 |
| 每目原标定尺寸 | `1280×960` |
| 深度和 XYZ 尺寸 | `320×240` |
| 基线 | 20 mm 棋盘估计 67.6718 mm；未强制缩放 |
| 标定文件 | `calibration/live_20260916_210120_642136/candidate.npz` |
| 匹配 | StereoSGBM，96 视差、5 像素块、左右一致性检查 |
| 并行 | OpenCV 4 线程，不主动限制输出帧率 |
| 网络服务 | 树莓派 `127.0.0.1:8081`，经 SSH 转发访问 |

相机配置请求 60 FPS，不代表深度能达到 60 FPS。FPS 受场景、后台负载、温度和解码方式影响，以当前画面与 `/health` 为准。历史 Pi 5 短时记录约 23–24 FPS；旧低分辨率版本的约 31 FPS 不代表当前设备或 WSL 模式的性能。

默认标定 `calibration/live_20260916_210120_642136/candidate.npz` 纳入 Git；标定过程与验证记录仅在本机保留。正常 clone 会包含运行所需的标定。标定只适用于对应相机与安装几何；换设备后应重新标定。原始 captures/depth_outputs 不随 Git 分发。

## 2. 开发与设备运行

日常开发在 WSL 主工程 `companion/ros2_ws/src/stereo_depth` 完成；WSL 依赖状态见[项目状态](../../../../docs/STATUS.md)。以下 Remote-SSH 用于设备运行、调试和查看数据；现场改动应先取回主工程，不维护另一套独立源码。

### 设备 Remote-SSH

1. Windows VS Code 安装 **Remote - SSH** 扩展。
2. `Ctrl+Shift+P` → `Remote-SSH: Connect to Host...` → `gmaster@192.168.137.200`。
3. 选择“文件 → 打开文件夹”，打开 `/home/gmaster/boom_birds_ws/stereo_depth`。
4. 检查左下角显示 SSH 主机，再新建终端。这里编辑的文件和运行的命令都在树莓派上。
5. 执行：

```bash
cd ~/boom_birds_ws/stereo_depth
python3 depth_preview.py
```

6. 在 VS Code **端口 / Ports** 面板转发 `8081`，点击浏览器图标访问。以面板显示的本地端口为准。
7. 修改代码后按 `Ctrl+S` 保存，在运行终端按 `Ctrl+C`，再运行启动命令。仅保存不会热更新程序。

若不用 VS Code 转发，可在 Windows PowerShell 中单独运行：

```powershell
ssh -N -L 18081:127.0.0.1:8081 gmaster@192.168.137.200
```

然后访问 `http://127.0.0.1:18081/`。保持该 SSH 窗口运行；不要重复占用同一本地端口。浏览器关闭不会停止树莓派上的程序。

## 3. 安装与相机权限

已有环境不必重复安装。新环境执行：

```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy v4l-utils
sudo usermod -aG video gmaster
```

重新建立登录会话后检查：

```bash
id
v4l2-ctl -d /dev/video0 --list-formats-ext
python3 -c "import cv2; print(cv2.__version__)"
```

旧 VS Code 后台可能继续继承旧组权限；可在当前终端**单独执行** `newgrp video`，等新提示符出现，再运行程序。不要用 `sudo python3` 作为长期解决办法。

## 4. 启动选项

```bash
python3 depth_preview.py --help
python3 depth_preview.py --threads 4 --port 8081 --device /dev/video0
python3 depth_preview.py --full-decode
```

默认使用半分辨率 JPEG 解码。`--full-decode` 使用完整解码再缩小，便于效果对照；保留兼容环境变量 `STEREO_FULL_DECODE=1`。两种解码方式的像素可能略有差异。

分辨率和标定文件不作为随意可调选项：更换相机、焦距、安装几何或输入裁剪后，需要重新核对或标定。不能只改显示尺寸就声称标定适配。

## 5. 如何阅读代码

| 组件 | 责任 |
| --- | --- |
| `Config` / 尺寸常量 | 集中保存设备、线程、端口、路径和解码参数 |
| `CapturedFrame` | 一次采集的 JPEG、序号及主机接收时间 |
| `DepthFrame` | 同一帧的视差、XYZ、有效掩码、预览和计时 |
| `StereoProcessor.__init__` | 加载标定，缩放内参，生成校正映射和匹配器 |
| `rectify` / `rectify_image` → `reconstruct` → `process` | 解码校正、双向匹配、有效性筛选、重投影和可视化；`rectify_image` 接收已解码的拼接 BGR 图，供 ROS 2 节点复用，不重新编码 JPEG |
| `PreviewApp.capture_loop` | 读取压缩帧，覆盖旧帧，避免队列积压 |
| `PreviewApp.processing_loop` | 每个选中的新帧计算一次，编码并发布结果 |
| `save_frame` | 保存一份完整、对应同一帧的结果 |
| `RequestHandler` | 网页、视频流、状态和保存请求 |
| `main` | 配置、启动线程、关闭与释放资源 |

处理链：

```text
USB 同帧 JPEG
  → 最新帧槽（覆盖旧帧，不排队）
  → 解码、拆分 A/B、去畸变及极线校正
  → A→B 和 B→A StereoSGBM
  → 视差除以 16，恢复像素单位
  → 左右一致性与有效性检查
  → Q 重投影为 XYZ；Z 通道即深度
  → 预览编码 / HTTP 发布 / 按需保存
```

锁只保护共享引用和状态，不包住匹配、编码或磁盘写入。结果发布后不再修改数组，因此保存线程取得引用后，可保存同一帧的完整数据。不要从外部修改已发布结果。

## 5.1 ROS 2 复用与资源位置

本模块同时作为 ROS 2 包（`ament_cmake`）安装，供脱机链路复用同一套几何运算，
不复制第二份算法：

| 内容 | 安装位置 | 用途 |
| --- | --- | --- |
| `depth_preview` 模块 | `<prefix>/lib/python3.12/site-packages/depth_preview.py` | `boom_birds_nav.depth_node` 导入 `StereoProcessor` |
| 默认标定 | `<prefix>/share/stereo_depth/calibration/live_20260916_210120_642136/candidate.npz` | 脱机链路默认标定 |
| 说明文档 | `<prefix>/share/stereo_depth/{README.md,LIVE_CALIBRATION.md}` | 随包分发 |

- 复用入口：`StereoProcessor.rectify_image(bgr)`（已解码拼接图）与既有的
  `reconstruct` / `process`；两者的缩放与校正路径完全相同。
- 资源定位：`ROOT` 优先取源码目录（仓库内直接运行 `python3 depth_preview.py` 行为不变），
  目录内没有 `calibration/` 时回退到 `ament_index_python.get_package_share_directory("stereo_depth")`，
  因此安装后不依赖当前工作目录。
- 320×240 深度图的等效内参由同一标定的 `stereoRectify(alpha=0)` 输出 `P1` 推导
  （真机标定实测：fx=fy=172.5980149526744、cx=156.8623504638672、cy=116.15113067626953、
  基线 0.06767180410552348 m），ROS 侧据此发布 `CameraInfo`，地图与控制不得另算一套。
- 本模块**不启动相机、不发布话题**：脱机输入与 ROS 2 节点在
  `companion/ros2_ws/src/boom_birds_nav`；真实采集仍由 `depth_preview.py` 独立程序负责，
  后续把采集线程抽象成同一输入接口即可复用。

## 6. 保存数据及坐标含义

点击 **保存 XYZ、深度与原图**，会在 `depth_outputs/<时间>/` 生成：

| 文件 | 内容 |
| --- | --- |
| `raw.jpg` | 相机原始压缩帧 |
| `raw.png` | 原始帧完整解码，`2560×960` |
| `rectified.png` | 校正后的 A 目，`320×240` |
| `preview.png` | A 目与深度伪彩色并排图 |
| `xyz_m.npy` | `float32 (240,320,3)`，XYZ，单位米 |
| `depth_m.npy` | `float32 (240,320)`，与 XYZ 第三通道一致 |
| `disparity_px.npy` | 像素视差，包含无效位置，使用时结合 `valid.png` |
| `valid.png` | 有效像素 255，无效像素 0 |
| `Q.npy` | 当前 `320×240` 对应的重投影矩阵 |
| `metadata.json` | 标定路径、单位、坐标系、计时与采集序号；最后写入 |

XYZ 在**校正 A 目相机光学坐标系**中：X 向图像右、Y 向图像下、Z 向前。还没有转换到无人机机体或世界坐标系。Z 是光轴深度；直线距离为 `sqrt(X²+Y²+Z²)`。

无效像素 XYZ 三个通道均为 `NaN`。预览色标固定为 0.15–3 m，超界颜色饱和；数值以数组为准。

```python
from pathlib import Path
import numpy as np

# 改成实际保存目录。
folder = Path("depth_outputs/你的保存时间目录")
xyz = np.load(folder / "xyz_m.npy")
u, v = 160, 120  # u 为列，v 为行，位于 320×240 深度图上。
point = xyz[v, u]
if np.isfinite(point).all():
    print("XYZ（米）：", point)
    print("直线距离（米）：", np.linalg.norm(point))
else:
    print("该像素无有效三维坐标")
```

## 7. 监控与接口

在另一个树莓派终端运行 `btop`，或：

```bash
watch -n 1 'curl -s http://127.0.0.1:8081/health'
```

| 字段 | 含义 |
| --- | --- |
| `fps` | 最近最多 30 帧的深度生成速率，不是浏览器呈现 FPS |
| `prep_ms` | JPEG 解码、缩放、校正、灰度转换 |
| `match_ms` | 双向 SGBM |
| `post_ms` | 一致性检查、XYZ 重投影、伪彩色生成 |
| `compute_ms` | 上述三段之和 |
| `encode_ms` | 状态文字绘制和预览 JPEG 编码 |
| `sequence` | 已发布的结果帧数 |
| `error` | `null` 表示没有报告错误 |

这些耗时不包含完整曝光、驱动队列、网络和浏览器呈现，不是端到端控制延迟。时间戳是主机接收/保存时间，不是传感器曝光时间。

HTTP：`GET /` 网页，`GET /stream` MJPEG，`GET /frame.jpg` 最近帧，`GET /health` 状态，`POST /capture` 保存。服务只绑定树莓派回环地址，通过 SSH 转发访问。

## 8. 常见问题

- **相机打不开**：检查 `id` 是否包含 `video`、设备是否存在，以及 `fuser /dev/video0` 是否显示其他采集程序。
- **端口被占用**：检查 `ss -ltnp | grep 8081`；不要再启动第二个实例。
- **帧率下降**：先看分段耗时、`btop`、温度和 `vcgencmd get_throttled`，不要直接删除缓存或终止系统更新。
- **黑区较多**：检查纹理、照明、遮挡、测量距离和标定质量；黑区不是无障碍证明。
- **提示标定文件不存在**：检查 clone 是否完整、默认标定是否被本地删除；换相机时运行 `python3 live_calibration.py` 重新标定，或用 `--calibration` 指定匹配该设备的标定。
- **相机断开**：程序报告错误并停止发布新结果；重新接好相机后重启程序。

默认标定与回归标定随源码跟踪；原始 `captures/`、`depth_outputs/` 和历史 JSON 测试记录仅在本机保留。
