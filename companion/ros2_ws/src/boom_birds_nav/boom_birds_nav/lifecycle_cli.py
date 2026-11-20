# ******************************************************************************
# @file           : lifecycle_cli.py
# @brief          : 从命令行调用 ROS 2 任务服务。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

"""兼容转发：实现已迁至 boom_birds_bringup.lifecycle_cli。"""
from boom_birds_nav._compat import forward as _forward

_forward(globals(), "boom_birds_bringup.lifecycle_cli")
del _forward
