# ******************************************************************************
# @file           : px4_interface_node.py
# @brief          : 接收规划或任务设定点，管理控制会话并通过后端向 PX4 发送控制指令。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

"""兼容转发：实现已迁至 boom_birds_control.px4_interface_node。"""
from boom_birds_nav._compat import forward as _forward

_forward(globals(), "boom_birds_control.px4_interface_node")
del _forward
