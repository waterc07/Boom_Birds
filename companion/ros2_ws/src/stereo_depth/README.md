# 双目深度运行库

提供 `stereo_depth.core.Config`、`StereoProcessor` 和原生计算核，不打开相机、不启动 HTTP 或 ROS 节点。ROS 2 采集与发布节点在 `boom_birds_sensing`。

| 内容 | 位置 |
| --- | --- |
| Python 算法与共享显示函数 | `stereo_depth/` |
| C++ 计算核 | `native/` |
| 自动测试与旧标定夹具 | `test/` |
| 默认标定 | `calibration/upright_20261022/candidate.npz` |
| USB 双目与 MIPI 标定工具 | [tools/camera_calibration](../../tools/camera_calibration/README.md) |
| 深度预览、HTTP 和数据保存 | [tools/depth_preview](../../tools/depth_preview/README.md) |
| 分档回放与性能测量 | [tools/depth_benchmark](../../tools/depth_benchmark/README.md) |

## 构建与导入

在仓库根目录执行：

```bash
bash companion/ros2_ws/tools/build_all.sh --packages-select stereo_depth
source companion/ros2_ws/tools/activate_python_env.sh
source /home/waterc/bb_build/main/install/setup.bash
python3 -c 'from stereo_depth.core import Config, StereoProcessor; print(Config.calibration)'
```

构建与安装前缀可由 `BUILD_BASE`、`INSTALL_BASE`、`LOG_BASE` 覆盖，见[工作空间](../../README.md#脱机开发环境与构建wsl)。构建产物留在仓库外。

只安装 `stereo_depth` Python 包、原生库、默认 NPZ 和本说明。预览、标定、benchmark、测试和过程记录不装进运行包；旧的 `import depth_preview`、`import stereo_view` 和 `python -m stereo_depth.benchmark` 入口已移除。

## 标定与输出路径

源码运行优先从包目录查找默认标定，安装后从 ament `share/stereo_depth` 查找，不依赖当前工作目录。
默认由 2026-10-17 批次转换为转正 B/A 标定：原图 `2560×960`、每目 `1280×960`、11×8 内角点、20 mm 方格，估计基线 67.7368 mm。独立距离尺测未完成。

其他标定通过 `Config(calibration=Path(...))` 或工具 `--calibration` 显式选择，不自动发现最新文件。
整条 VIO/深度链使用同批内参与基线；当前选择见 [STATUS](../../../../docs/STATUS.md#当前配置与标定)。

- 标定会话默认写入 `~/boombirds-data/stereo_depth/calibration/`。
- 深度预览默认写入 `~/boombirds-data/stereo_depth/depth_outputs/`。
- 工具通过 `--output-dir` 覆盖保存目录，不写入包源码或安装资源目录。
- `test/fixtures/calibration/` 保存旧 NPZ，只供回归，不随运行包安装。

## 数据接口

同一拼接帧拆为左右目，按标定校正、匹配并重投影。默认计算尺寸 `320×240`。
`StereoProcessor.rectifyImage(bgr)` 接受已解码拼接图；`processImage` / `processPair` 提供数值处理结果。
XYZ/Z 单位米，位于校正左目光学坐标系（原 B 目转正）；无效值为 `NaN`，黑色无效区不是自由空间。
`CameraInfo` 使用同一标定缩放并校正后的内参，不能另设不匹配的基线。

USB 倒装输入统一为新左目 = 原 B 目旋转 180°、新右目 = 原 A 目旋转 180°。标定包含 `imageOrientation=upright_ba_rot180`。

- `rectify(packet)` 接收设备方向 JPEG，解码后转换一次。`rectifyImage`、`processImage`、`processPair` 接收方向已与标定一致的数组。
- ROS 采集在共享深度线程和左右 Image 发布前转换。CompressedImage 同样转正，`format` 附带 `boombirds_orientation`；深度节点按标记处理，避免再次旋转。
- `input_orientation` 默认 `sensor_ab`；回放已转正的图像设为 `upright_ba_rot180`。设备原始 JPEG 可保留为采集证据，不作为已转正图发布。
- 派生标定含 `intrinsics_scale_offset=1`。缩放主点用 `c' = s*(c+1)-1`，与“先按原约定缩放传感器内参，再旋转像素”一致；不能直接乘 s。
- 内参、切向畸变、双目外参、相机—IMU外参随换序和旋转一起变换；`R1/R2/P1/P2/Q` 重新生成。VIO 的 cam0 及时间偏移来自原 cam1；IMU 坐标与 `T_I_B` 不因此改变。
- 无方向字段的旧 NPZ 仍按 `sensor_ab` 读取，供历史回放与合成测试使用，不自动套用转正标定。新标定工具采集转正图、保存转正角点，并写入方向字段。

坐标转换保留原标定的精度及限制，不代替独立测距、VIO 方向/尺度或机体安装外参验收。

实验参数与限制见[分档回放](../../tools/depth_benchmark/README.md)。默认保持 SGBM、NumPy 后处理和 320×240，不因目录拆分更改。

## 脱机测试

在 `companion/ros2_ws` 下加载开发环境和对应安装前缀后执行：

```bash
python3 -m pytest src/stereo_depth/test tools/camera_calibration/test -q
```

测试包含合成输入、几何、显示方向、HTTP 和失效路径，不作为 Pi 性能、真实测距或飞行验收。

## 安装边界

运行安装只含 `stereo_depth` 库、原生计算核和转正默认 NPZ。旧方向标定留在 `test/fixtures/calibration/`；采集、预览与 benchmark 位于工作空间 `tools/`。可选实验匹配后端仍由显式参数启用，默认关闭，不能按目录名当作失效内容删除。


### IMX219 RAW10 解包

libstereo_depth_kernels.so 增加 bb_unpack_raw10，按行步长将 pRAA 转为 8 位 Bayer，使用调用方提供的 1024 项 LUT。检查输入长度、尺寸与步长；不做相机配置、缩放或时间戳转换。NativeKernels.unpackRaw10 提供 ctypes 包装；下视源显式选择该库后才启用，不影响默认双目匹配路径。与 NumPy 参考的像素一致性测试位于 boom_birds_sensing/test/test_imx219_capture.py。
