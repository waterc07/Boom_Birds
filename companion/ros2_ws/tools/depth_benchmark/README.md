# 深度实验参数与分档回放

在 `companion/ros2_ws` 下加载 `tools/activate_python_env.sh` 后执行。原生库需先构建，并加载对应安装环境或显式指定库路径。

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

缓冲复用只作用于处理器内部。`process_*` / `matchRectified` 返回独立帧快照；开启复用后，单独调用 `rectify*` 返回的内部图像仅在下一次处理前有效。一个处理器只能由一个计算线程调用。

Census 为 5×5、24 bit 描述子、5×5 代价聚合、整数视差和唯一性筛查，保留双向匹配与左右一致性检查；不是 SGBM 的等价实现。AArch64 编译时启用 NEON Hamming 计数，其他平台走标量核。没有 H×W×D 代价体，但每次匹配仍分配 Census/积分图工作区。

`coarse_census` 先在半尺寸图匹配，再围绕上采样视差限制当前帧搜索。缺失先验区域全范围搜索，搜索边界命中或不确定结果回到全范围，每五帧强制全范围匹配。当前实现仍遍历视差生成聚合代价，先验主要限制候选选择；不能据此声称运算量或障碍漏检已达标。

VIO 先验输入必须提供 `T_W_Crect`，不能直接订阅 OpenVINS 的 IMU 位姿。转换链是 `T_W_I · T_I_C0 · T_C0_Crect`，时间戳使用图像曝光时域并与 VIO 对齐。ROS 输入的 `child_frame_id` 必须等于深度 `frame_id`，`header.frame_id` 是固定世界系；协方差对角须有限、非负且不超过 0.1（位置 m²、旋转 rad²；实验筛查值）。位姿缓存最多 32 个样本，图像与位姿相差不得超过 20 ms，相邻深度不得超过 150 ms。世界系变化、时间回退、单帧平移超过 0.5 m 或旋转超过 30° 会清除旧先验。先验只用于搜索，不直接发布传播深度；新一帧仍必须通过双向匹配。真实 VIO 坐标转换、同步和跳变门限需实测。

离线输入是按文件名排序的 NPZ，每帧有同尺寸 `uint8` 灰度 `left` / `right`。先验比较另需秒制 `stamp` 和 4×4 `T_w_crect`；两者必须来自该帧对应的真实位姿，不能用单位矩阵冒充 VIO。工具不打开相机，不连接飞控。

```bash
PYTHONPATH="src/stereo_depth${PYTHONPATH:+:$PYTHONPATH}" python3 -m tools.depth_benchmark.benchmark \
  --frames /绝对路径/双目NPZ序列 \
  --calibration /绝对路径/candidate.npz \
  --output /绝对路径/benchmark.json \
  --profiles baseline,fixed,reuse,lut,native,combined,census,coarse \
  --threads 1,2,3 --disparities 64 --repeats 10
# 需要独立 XYZ 对照时追加 --xyz；有同步相机位姿时追加 prior 档。
```

`baseline/fixed/reuse/lut/native/combined` 逐项及组合对照；`prior`、`elas`、`rknn` 按条件显式加入。工具随机排列各档顺序，记录输入/标定 SHA256、OpenCV 构建信息、先验准备、校正＋匹配＋后处理 P50/P95、单核 CPU、有效率、与 SGBM 的掩码差异和共同有效区域深度差。RSS 是同一进程累计高水位，不能用于各档独立内存比较；CPU 不包含采集、JPEG 解码、ROS、VIO、规划共载。`PASS` 仅表示该档完成回放，不表示精度或算力验收。缺依赖写 `BLOCKED` 并返回 2，运行失败写 `FAIL` 并返回 1，不静默跳过。实机另测整机 CPU、消息年龄、功耗/降频、真实深度误差、1800 s 和 10 次启停。

ELAS 外部构建入口：`bash tools/build_all.sh --packages-select stereo_depth --cmake-args -DELAS_SOURCE_DIR=/绝对路径/libelas`，生成 `libstereo_depth_elas.so`，用 `elas_library` 指定该文件。[上游 libelas](https://www.cvlibs.net/software/libelas/) 含 SSE 实现；ARM 测试需提供经过核验的兼容源码，不能把 x86 包装接口视为 ARM 移植完成。库不随本仓库分发。

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

`--input-orientation` 默认 `sensor_ab`（旧设备记录）；已转正图像对设 `upright_ba_rot180`。带 `T_w_crect` 的记录若与标定方向不一致会拒绝运行，必须连同位姿转换，不能只旋转图像。
