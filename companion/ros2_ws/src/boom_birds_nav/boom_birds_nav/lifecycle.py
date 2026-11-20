# ******************************************************************************
# @file           : lifecycle.py
# @brief          : 根据已观测的飞行状态推进任务，并返回交给 PX4 接口的动作。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

"""兼容转发：实现已迁至 boom_birds_bringup.lifecycle。"""
from boom_birds_nav._compat import forward as _forward

_forward(globals(), "boom_birds_bringup.lifecycle")
del _forward
