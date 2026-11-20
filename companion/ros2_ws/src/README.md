# ROS 2 源码模块导航

各包 README 介绍职责、内部结构和操作；实现与验收状态见 [STATUS](../../../docs/STATUS.md#实现默认入口部署与验收)。

| 模块 | 用途 |
| --- | --- |
| [stereo_depth](stereo_depth/README.md) | 双目匹配、深度重建及原生内核 |
| [boom_birds_interfaces](boom_birds_interfaces/README.md) | 消息、服务与数据契约 |
| [boom_birds_sensing](boom_birds_sensing/README.md) | 相机、IMU、深度发布及 VIO 适配 |
| [boom_birds_control](boom_birds_control/README.md) | 控制会话、引导和 MAVROS 接口 |
| [boom_birds_bringup](boom_birds_bringup/README.md) | 启动、任务编排和资源管理 |
| [boom_birds_sim](boom_birds_sim/README.md) | 仿真、合成输入、回放与评估 |
| [boom_birds_nav](boom_birds_nav/README.md) | 旧模块和 launch 兼容转发 |
| [open_vins](open_vins/ReadMe.md) | Git 子模块：视觉惯性里程计；`ov_core` 为基础算法，`ov_init` 为初始化，`ov_msckf` 为估计器，`ov_data` 和 `ov_eval` 为数据及评估工具。 |
| [ego-planner-swarm](ego-planner-swarm/Readme.md) | Git 子模块：轨迹规划；`src/planner/` 包含环境、搜索、优化及规划管理，`src/uav_simulator/` 包含仿真相关组件。 |

典型数据关系：双目与 IMU → sensing → OpenVINS 位姿；双目 → stereo_depth → 深度；位姿与深度 → EGO 规划；任务编排与规划结果 → control → MAVROS/PX4。仿真可为验证提供输入；这张关系图不表示每条路径均已完成部署或验收。第三方子模块保留各自原有 README 和独立版本管理。
