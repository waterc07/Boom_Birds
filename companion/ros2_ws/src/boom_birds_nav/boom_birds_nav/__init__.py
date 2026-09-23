"""Boom_Birds 脱机导航链路的可导入模块集合。

模块划分（可离线单测，不依赖 ROS 运行时）：
- frames        : 位姿、四元数、齐次变换与协方差的纯函数工具
- depth_core    : 复用 stereo_depth 的 StereoProcessor，负责深度/XYZ 与发布前契约转换
- synthetic     : 确定性合成双目/VIO/IMU 数据源（TEST-ONLY）
- stereo_source : 唯一采集源语义的 ROS 节点（文件/合成 → 左右原始图）
- depth_node    : 深度与完整 XYZ 的 ROS 节点
- pose_adapter  : OpenVINS 位姿 → 机体里程计与校正相机位姿的 ROS 节点
- deep_checks   : 单帧自检入口（缺标定/缺外参必须显式失败）

坐标与话题契约见 config/contract.yaml 与 README.md。
"""

__all__ = ["frames", "depth_core", "synthetic", "deep_checks"]
