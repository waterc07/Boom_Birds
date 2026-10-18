# 双目 XYZ 与深度预览

调焦与标定步骤见 [LIVE_CALIBRATION.md](LIVE_CALIBRATION.md)，使用 `live_calibration.py`。
原图为 `2560×960`、每目 `1280×960`，棋盘默认 11×8 内角点、20 mm；自动收集角点，无需保存多张照片。
当前默认使用 2026-10-17 的 20 mm 棋盘标定。其他标定可通过 `depth_preview.py --calibration <candidate.npz>` 显式加载，
采集尺寸随所选标定匹配；不会自动替换旧标定，也不自动认定距离精度通过。

`depth_preview.py` 读取 USB 左右拼接帧，输出米制深度和 XYZ，并提供浏览器预览与数据保存。源码和默认标定在 WSL 主工程。

USB 双目倒装：所有双目查看入口逐目旋转 180°，保持 A/B 和图像/深度槽位顺序。`--preview-rotation 0` 可查看传感器方向。
原始 JPEG、ROS 原图/CameraInfo、NPZ、XYZ/Z/Q 保留标定光学坐标；旋转查看图不作为算法输入。

新标定通过覆盖检查及留出几何验证；距离尚未独立尺测验证。黑色无效区域不能理解为空闲空间。代码运行和标定残差不代表飞行避障已验收。

## 1. 环境与当前参数

| 项目 | 当前值 |
| --- | --- |
| 主机 | Raspberry Pi 5 / Ubuntu 24.04 ARM64 |
| SSH | `gmaster@192.168.137.200`，网络变化后需确认地址 |
| 独立预览旧入口 | `/home/gmaster/boom_birds_ws/stereo_depth`；ROS 生产部署入口见[工作空间](../../README.md#环境与部署) |
| 依赖 | 树莓派实测 Python 3 / OpenCV 4.6.0 / NumPy 1.26.4；HTTP 使用 Python 标准库；依赖清单见仓库根 requirements.txt |
| 输入 | `/dev/video0`，MJPEG `2560×960`，同帧 A/B 左右拼接 |
| 每目原标定尺寸 | `1280×960` |
| 深度和 XYZ 尺寸 | `320×240` |
| 基线 | 20 mm 棋盘估计 67.7368 mm；未强制缩放 |
| 标定文件 | `calibration/live_20261017_141337_524478/candidate.npz` |
| 匹配 | StereoSGBM，96 视差、5 像素块、左右一致性检查 |
| 并行 | OpenCV 4 线程，不主动限制输出帧率 |
| 网络服务 | 树莓派 `127.0.0.1:8081`，经 SSH 转发访问 |

相机配置请求 60 FPS，不代表深度能达到 60 FPS。FPS 受场景、后台负载、温度和解码方式影响，以当前画面与 `/health` 为准。历史 Pi 5 短时记录约 23–24 FPS；旧低分辨率版本的约 31 FPS 不代表当前设备或 WSL 模式的性能。

默认标定随源码交付，正常 clone 包含运行必需的 NPZ。标定过程与验证记录留在本机。标定只适用于对应相机与安装几何；换设备后应重新标定。原始 captures/depth_outputs 不随 Git 分发。

独立 CLI 默认 NPZ 与已选 VIO 开发批次分别管理；选择入口和状态见 [STATUS](../../../../docs/STATUS.md#当前配置与标定)。测试整条 VIO/深度链时使用同一批内参和基线，不能混用两批标定。

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

本模块通过 `ament_cmake` 安装为 ROS 2 包；脱机链路复用 `StereoProcessor`：

| 内容 | 安装位置 | 用途 |
| --- | --- | --- |
| `depth_preview` 模块 | `<prefix>/lib/python3.12/site-packages/depth_preview.py` | `boom_birds_nav.depth_node` 导入 `StereoProcessor` |
| 默认标定 | `<prefix>/share/stereo_depth/calibration/live_20261017_141337_524478/candidate.npz` | 脱机链路默认标定 |
| 说明文档 | `<prefix>/share/stereo_depth/{README.md,LIVE_CALIBRATION.md}` | 随包分发 |

- 复用入口：`StereoProcessor.rectify_image(bgr)`（已解码拼接图）与既有的
  `reconstruct` / `process`；两者的缩放与校正路径完全相同。
- 资源定位：`ROOT` 优先取源码目录（仓库内直接运行 `python3 depth_preview.py` 行为不变），
  目录内没有 `calibration/` 时回退到 `ament_index_python.get_package_share_directory("stereo_depth")`，
  因此安装后不依赖当前工作目录。
- 320×240 深度图的等效内参由同一标定的 `stereoRectify(alpha=0)` 输出 `P1` 推导
  （真机标定实测：fx=fy=181.5728695833146、cx=157.81851196289062、cy=118.38076400756836、
  基线 0.06773675534195574 m），ROS 侧据此发布 `CameraInfo`，地图与控制不得另算一套。
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
| `raw.view.png` / `rectified.view.png` / `valid.view.png` | 旋转后的查看图，不与原始数值像素混用 |
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

## 9. IMX219 单目内参标定

原 A4 棋盘继续使用：11×8 内角点，20 mm 方格，外格 12×9、240×180 mm。先实测连续 10 格为 200 mm，贴在平板上；采集期间不转动镜头、不改变安装。

Pi 5 Ubuntu 的当前 RAW 管线使用 CAM/DISP 1、IMX219、1640×1232、pRAA。传感器裁剪回读须为 (8,8)/3280×2464；入口拒绝尺寸、Bayer 格式或裁剪不符。依赖系统 Python 的 OpenCV、NumPy，以及 media-ctl、v4l2-ctl、fuser。设备编号会因启动/接入顺序变化，运行前用 media-ctl / v4l2-ctl 核对；不要用本节命令打开 USB 双目。

在独立工具目录中运行：

```bash
python3 live_mipi_calibration.py   --device /dev/video0 --media /dev/media2 --subdev /dev/v4l-subdev2   --exposure 1600 --gain 192   --output /home/gmaster/boombirds/evidence/mipi-calibration-20261008
```

Windows 上转发后打开 http://127.0.0.1:8083：

```powershell
ssh -N -L 127.0.0.1:8083:127.0.0.1:8083 gmaster@192.168.137.200
```

曝光单位为传感器行，增益为 analogue_gain 寄存器值，不是 ISO。网页可在检查阶段调整曝光 4..1600、模拟增益 0..232；数字增益固定 256（1×）。采集线程持续排空 RAW 管线，转图线程只处理最新一帧。网页预览缩小为 820×616，点击显示 320×240 原像素局部；角点检测和样本保存保持 1640×1232。RAW 解包计入行尾 padding，固定黑电平 64 / gamma 2.2 后 RGGB 去马赛克，不经过 ISP，不做缩放、旋转或翻转。若画面偏暗，先补光再调整增益；白格不能过曝。开始采集后设置锁定，暂停也不能改；需要改动时重置一轮，旧文件保留。

操作顺序与双目页相同：检查清晰度并确认棋盘尺寸 → 自动收样 → 覆盖达标后计算 → 新姿态检查。每个姿态静止约 1 秒；至少 24 组，需覆盖画面位置、远近、平面转角和两方向倾斜。网页点击图像可查看原像素局部；候选结果提供去畸变和实时重投影检查，不提供双目基线或极线误差。

每轮独立 mono_<时间>/ 保存 observations.npz（单目角点、采集序号、主机接收时间）、sample_*.png（检测使用的无损全分辨率图）、sample_*.raw（对应 RAW）、source.json（管线与控制回读）。求解仅使用 80% 姿态，20% 留出检查，写入 candidate.npz 的 K/D/image_size 和 report.json。训练 RMS >1 px 或留出 RMS >1.5 px 标为 CANDIDATE_NEEDS_REVIEW；这些门限是原分辨率的几何筛查。

时间是 RAW 管道读完后的主机 monotonic 时间，不是曝光时间；网页图像年龄不代表传感器端到端时延。内参只对应本次传感器模式、裁剪、镜头和安装；改用其他模式或 ISP 几何处理前需核对映射或重标定。候选结果不自动覆盖原双目文件，也不自动写入平台降落配置。实际距离精度、相机到机体外参、同步和飞行仍需独立验证。

软件检查：

```bash
PYTHONNOUSERSITE=1 python3 -m pytest -q test_live_calibration.py test_mipi_calibration.py
```


## 10. 深度实验参数与分档回放

`StereoProcessor(Config(...))`、`depth_node` 和 `shared_stereo_depth` 共用这些参数。默认保持浮点校正图、SGBM、NumPy 后处理；实验不改写 `mono_budget` 默认值。

| 参数 | 默认 | 对照范围 |
| --- | --- | --- |
| `fixed_maps` | `false` | 用 `convertMaps` 生成 CV_16SC2/CV_16UC1 校正图 |
| `reuse_buffers` | `false` | 复用缩放、校正、灰度和匹配内部缓冲 |
| `postprocess` | `numpy` | `numpy` / `lut` / `native`；后两项只支持水平 Q |
| `matcher` | `sgbm` | `sgbm` / `census` / `coarse_census` / `elas` |
| `experimental` | `false` | Census、ELAS、VIO 先验或 RKNN 必须显式启用 |
| `native_library` | 空 | 空值从已安装包加载 `lib/stereo_depth/libstereo_depth_kernels.so`；源码回放可指定绝对路径 |
| `elas_library` | 空 | ELAS 包装库绝对路径；不自动下载、构建或回退 |
| `rknn_model` / `rknn_manifest` | 空 | RKNN 模型与 JSON 合约文件，必须同时提供 |
| `vio_prior_topic` | 空 | 仅 ROS 节点：已转换到深度校正光学系的 `nav_msgs/Odometry` |

SGBM 定点视差保留 1/16 px。`lut` 用同一 Q 生成 Z 表；`native` 在一次 C++ 遍历中完成左右采样、一致性检查和米制 Z/XYZ。输出仍是米、NaN、H×W 像素布局；不会把匹配无效区填成自由空间。

缓冲复用只作用于处理器内部。`process_*` / `match_rectified` 返回独立帧快照；开启复用后，单独调用 `rectify*` 返回的内部图像仅在下一次处理前有效。一个处理器只能由一个计算线程调用。

Census 为 5×5、24 bit 描述子、5×5 代价聚合、整数视差和唯一性筛查，保留双向匹配与左右一致性检查；不是 SGBM 的等价实现。AArch64 编译时启用 NEON Hamming 计数，其他平台走标量核。没有 H×W×D 代价体，但每次匹配仍分配 Census/积分图工作区。

`coarse_census` 先在半尺寸图匹配，再围绕上采样视差限制当前帧搜索。缺失先验区域全范围搜索，搜索边界命中或不确定结果回到全范围，每五帧强制全范围匹配。当前实现仍遍历视差生成聚合代价，先验主要限制候选选择；不能据此声称运算量或障碍漏检已达标。

VIO 先验输入必须提供 `T_W_Crect`，不能直接订阅 OpenVINS 的 IMU 位姿。转换链是 `T_W_I · T_I_C0 · T_C0_Crect`，时间戳使用图像曝光时域并与 VIO 对齐。ROS 输入的 `child_frame_id` 必须等于深度 `frame_id`，`header.frame_id` 是固定世界系；协方差对角须有限、非负且不超过 0.1（位置 m²、旋转 rad²；实验筛查值）。位姿缓存最多 32 个样本，图像与位姿相差不得超过 20 ms，相邻深度不得超过 150 ms。世界系变化、时间回退、单帧平移超过 0.5 m 或旋转超过 30° 会清除旧先验。先验只用于搜索，不直接发布传播深度；新一帧仍必须通过双向匹配。真实 VIO 坐标转换、同步和跳变门限需实测。

离线输入是按文件名排序的 NPZ，每帧有同尺寸 `uint8` 灰度 `left` / `right`。先验比较另需秒制 `stamp` 和 4×4 `T_w_crect`；两者必须来自该帧对应的真实位姿，不能用单位矩阵冒充 VIO。工具不打开相机，不连接飞控。

```bash
python -m stereo_depth.benchmark \
  --frames /绝对路径/双目NPZ序列 \
  --calibration /绝对路径/candidate.npz \
  --output /绝对路径/benchmark.json \
  --profiles baseline,fixed,reuse,lut,native,combined,census,coarse \
  --threads 1,2,3 --disparities 64 --repeats 10
# 需要独立 XYZ 对照时追加 --xyz；有同步相机位姿时追加 prior 档。
```

`baseline/fixed/reuse/lut/native/combined` 逐项及组合对照；`prior`、`elas`、`rknn` 按条件显式加入。工具随机排列各档顺序，记录输入/标定 SHA256、OpenCV 构建信息、先验准备、校正＋匹配＋后处理 P50/P95、单核 CPU、有效率、与 SGBM 的掩码差异和共同有效区域深度差。RSS 是同一进程累计高水位，不能用于各档独立内存比较；CPU 不包含采集、JPEG 解码、ROS、VIO、规划共载。`PASS` 仅表示该档完成回放，不表示精度或算力验收。缺依赖写 `BLOCKED` 并返回 2，运行失败写 `FAIL` 并返回 1，不静默跳过。实机另测整机 CPU、消息年龄、功耗/降频、真实深度误差、1800 s 和 10 次启停。

ELAS 外部构建入口：`colcon build --packages-select stereo_depth --cmake-args -DELAS_SOURCE_DIR=/绝对路径/libelas`，生成 `libstereo_depth_elas.so`，用 `elas_library` 指定该文件。[上游 libelas](https://www.cvlibs.net/software/libelas/) 含 SSE 实现；ARM 测试需提供经过核验的兼容源码，不能把 x86 包装接口视为 ARM 移植完成。库不随本仓库分发。

RKNN 当前是残差模型适配器，不包含训练模型。需要目标设备的 [RKNN Toolkit2/Lite2](https://github.com/rockchip-linux/rknn-toolkit2)、模型和清单。合约固定：输入 NHWC float32 `[1,240,320,3]`，通道为 `L/255,R/255,d/num_disparities`；单输出 `[1,240,320,2]`，通道为视差残差（px）与置信度（0..1）。左右目各推理一次，再检查左右一致性。清单示例：

```json
{
  "contract": "stereo-residual-nhwc-v1",
  "shape": [1, 240, 320, 3],
  "sha256": "实际模型文件的SHA256",
  "max_delta_px": 1.0,
  "min_confidence": 0.8
}
```

超出残差限幅、置信度不足或非有限输出保留原视差；原匹配无效像素不由模型补齐。加载、输出形状或推理失败显式报错。Pi 5 无此 RKNN 实验路径；RK3576 的模型转换、运行时、量化精度与共载测试未完成。

### Gazebo 远场分辨率对照

Config.matching_size 默认 (320,240)。仅 experimental=true 可选择 (640,480)；内参、校正映射、Q 和有效掩码按同一匹配尺度生成，基线仍用米。sgbm_block_size 默认 5，可在实验模式选择 3/7，P1/P2 同步按窗口面积设置。

ROS depth_node 参数：matching_size:=[640,480]、sgbm_block_size:=3、experimental:=true、output_scale:=0.5。匹配在 640×480 进行，发布仍为 320×240，CameraInfo 同步缩放；32FC1、米制、NaN 契约保留。默认路径仍为 320×240/5×5；此对照未部署 Pi/RK，不作为原算力预算通过。
