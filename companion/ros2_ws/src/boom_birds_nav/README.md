# boom_birds_nav

本包只保留 Python 模块与 launch 兼容转发，不安装可执行文件。实现位于 sensing、control、bringup、sim；新代码直接依赖所有者包。

## 功能入口

| 内容 | 所有者与说明 |
| --- | --- |
| 相机、CameraInfo、时间戳、飞控 IMU、VIO 适配与标定 | [感知包](../boom_birds_sensing/README.md) |
| PX4 后端、坐标转换、会话、取消屏障与控制出口 | [控制包](../boom_birds_control/README.md) |
| 实机姿态路线、配置核验与任务编排 | [bringup](../boom_birds_bringup/README.md) |
| 合成输入、SIH、Gazebo 与回放 | [仿真包](../boom_birds_sim/README.md) |
| 话题类型、单位、坐标系与无效值 | [contract.yaml](../boom_birds_interfaces/config/contract.yaml) |
| 构建、依赖、安装前缀与共享参数 | [ROS 工作空间](../../README.md) |
| 当前实现、默认值、Pi 部署与验收 | [STATUS](../../../../docs/STATUS.md) |

## 兼容入口

模块转发绑定所有者模块的公共名与 __doc__，对象身份保持一致；支持 python -m。生产包不依赖 sim，合成帧源不进入生产 stereo_source。

| 旧入口 | 当前归属 |
| --- | --- |
| stereo_source、stereo_capture、depth_node、pose_adapter、camera_timestamp | boom_birds_sensing |
| px4_backend、mavros_backend、px4_frames、px4_failsafe、control_protocol、runtime_config | boom_birds_control |
| lifecycle、lifecycle_node、lifecycle_cli | boom_birds_bringup |
| synthetic、pointcloud_scene、deep_checks、vio_source | boom_birds_sim |
| mavlink_imu_node / mavlink_imu_offline.launch.py | 转发至 MAVROS IMU；不再接受 connection/baud |

可执行文件使用 ros2 run 的所有者包名查询。旧 launch 名称保留 Include 转发；它们没有另一份配置或运行实现。

## 脱机链路与回归

在 WSL 项目根执行并加载与本次构建对应的安装前缀：

~~~bash
source companion/ros2_ws/tools/activate_development_env.sh
bash companion/ros2_ws/tools/build_all.sh --packages-up-to boom_birds_nav
source /home/waterc/bb_build/main/install/setup.bash
ros2 launch boom_birds_nav synthetic_layer2.launch.py
~~~

合成输入为 TEST-ONLY。回放与专题检查：

~~~bash
python3 -m pytest companion/ros2_ws/src/boom_birds_nav/test -q
python3 -m boom_birds_nav.deep_checks --synth
python3 -m boom_birds_nav.mavlink_imu_replay /path/to/record.tlog --out /path/to/new-report.json
~~~

完整脱机入口见[工作空间](../../README.md#架构修整后的脱机回归入口)，相机时间戳核验见[感知包](../boom_birds_sensing/README.md#采集时间戳实现与边界)。订阅关系、回放、仿真、Pi 实时性与飞行分别验收。

## 历史记录

9 月的分段 SIH 演示、旧 pymavlink 接口结果及合成精度见[历史导航说明](../../../../docs/history/NAV_GUIDE_2026-09.md)。旧 x86 环境缺少交叉工具链的结论不描述当前 Pi；本包为纯 Python，原生依赖与实际驱动仍须在 aarch64 上验证。
