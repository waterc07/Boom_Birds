# ******************************************************************************
# @file           : lifecycle_node.py
# @brief          : 将任务状态机接入 ROS 2，协调规划和 PX4 接口服务。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

"""兼容转发：实现已迁至 boom_birds_bringup.lifecycle_node。"""
from boom_birds_nav._compat import forward as _forward

_forward(globals(), "boom_birds_bringup.lifecycle_node")
del _forward
