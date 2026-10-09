# 感知包

导航使用左目图像与飞控 IMU；深度使用同帧左右目。硬件入口、单目标定配置生成及算力参数见 [bringup](../boom_birds_bringup/README.md)。算法与标定见 [stereo_depth](../stereo_depth/README.md)。

## 入口与实现

| 文件 | 职责 |
| --- | --- |
| `stereo_capture.py`、`stereo_source.py` | 采集/回放、单次相机打开、同帧拆分、时间戳与按订阅需求发布 |
| `shared_stereo_depth.py` | 同进程采集与深度入口；解码后的左右数组直接交付深度线程 |
| `depth_core.py`、`depth_node.py` | 算法适配与 ROS 发布；无点云需求时只生成 Z |
| `camera_geometry.py` | 标定与 CameraInfo 几何校验、内参缩放、原图尺寸与缩放 |
| `frame_worker.py` | 单槽最新帧、限频与退出；平台观测异常后继续，深度异常后停止并上报 |
| `mavros_imu_node.py`、`pose_adapter.py` | 飞控 IMU 门控上行与 VIO 位姿适配 |
| `platform_observation.py`、`platform_observation_node.py` | 下视板检测与 ROS 观测 |
| `platform_replay.py` | 图像清单/rosbag 回放及 TEST-ONLY 板图渲染 |

`image_scaling.py`、`platform_worker.py`、`platform_synthetic.py` 保留旧导入入口，不保留第二份实现。新代码直接导入对应实现模块。

## 验证

在主工程根构建后执行 `bash companion/ros2_ws/tools/check_offline.sh`；非默认构建前缀须设置对应 `BUILD_BASE`、`INSTALL_BASE` 和 `OV_INSTALL`。该检查不连接设备，不验收真实 VIO、Pi 全链负载或飞行。当前结果见 [STATUS](../../../../docs/STATUS.md)。
