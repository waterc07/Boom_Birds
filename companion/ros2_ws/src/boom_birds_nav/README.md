# boom_birds_nav

Boom_Birds 第一阶段**脱机**导航链路：文件/合成双目输入 → 米制深度与完整 XYZ →
位姿/里程计适配 → EGO 局部地图与规划。仅用于 WSL 脱机开发，
**不连接真实设备**，不代表真机验收。

## 节点

| 可执行 | 职责 | 关键约定 |
| --- | --- | --- |
| `stereo_source` | 唯一采集源语义：合成或文件回放，发布左右原始图与拼接图 | 同帧左右图共享时间戳 |
| `depth_node` | 复用 `stereo_depth` 的 `StereoProcessor` 计算深度与 XYZ | 深度 `32FC1` 米制，无效为 NaN；另发 `16UC1` 毫米/整数 0 兼容话题，主点云保留 H×W 与 NaN |
| `pose_adapter` | OpenVINS 里程计 → 相机位姿 + 机体里程计 + EGO 专用里程计 | `T_W_Crect = T_W_I·T_I_C0·T_C0_Crect`；容差/超时见契约 |
| `vio_source` | **TEST-ONLY** 合成 VIO/IMU 源，供脱机测试 | 不是飞控数据，不得作为精度证据 |

## 契约

话题、坐标系、时间与无效值约定的唯一来源是 [config/contract.yaml](config/contract.yaml)。要点：

- `T_A_B` 表示「把 B 系坐标变换到 A 系」；`T_I_C0` 即 Kalibr/OpenVINS 的 `T_imu_cam` 字段。
- 深度数组层无效值为 `NaN`；发布到 `/boom_birds/depth/image` 时仍为 `NaN`；兼容话题使用整数 0。
- 主 XYZ 点云保持 240×320 像素对应关系、无效点三分量 NaN、`is_dense=false`；
  `/boom_birds/depth/xyz_valid` 只是附加的紧凑点云。
- 位姿与深度配对容差 0.03 s；VIO 位姿超 0.15 s、深度超 1.0 s 未更新即停止发布。
- 标准机体里程计（`/boom_birds/vio/odom_body`）twist 表达在 `body` 系；
  EGO 专用输出（`/boom_birds/vio/odom_ego`）twist 为世界系速度，两者不互相冒充。

## 运行（脱机）

```bash
# 环境（隔离 venv，见 companion/ros2_ws/tools/）
source companion/ros2_ws/tools/activate_python_env.sh

# 构建（加锁串行 + 并发/内存上限）
bash companion/ros2_ws/tools/build_all.sh --packages-select stereo_depth boom_birds_nav
source /home/waterc/bb_build/main/install/setup.bash

# 链路本体（合成输入 + 合成 VIO）
ros2 launch boom_birds_nav synthetic_layer2.launch.py
```

EGO 侧由 `ego-planner-swarm` 的 `boom_birds_offline.launch.py` 接入上述话题。

## 测试

```bash
cd companion/ros2_ws/src/boom_birds_nav
python3 -m pytest test -v          # 纯函数 + 深度契约 + 外参 + 适配节点行为
python3 -m boom_birds_nav.deep_checks --synth   # 单帧自检（JSON 报告）
```

## 已实测的合成精度边界（2026-09-22，本机 x86_64）

合成场景（3 m 墙 + 1.8 m 障碍，合成标定 TEST-ONLY）：

- 有效像素比例约 0.42（含右边界 99 px 不可测区）；
- 深度误差受 StereoSGBM 的 1/16 px 整数视差量化限制：3 m 处中位误差约 0.078 m；
- 因此**不要求**毫米级，也不得把该结果当作真机距离精度。

真机标定（1280×960、基线 67.6718 mm）对应 320×240 深度图；实际内参由运行时同一标定的 P1 推导并发布为 CameraInfo，地图从该值读取。
