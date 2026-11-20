# ******************************************************************************
# @file           : px4_backend.py
# @brief          : 定义控制算法调用飞控通信后端所需的接口。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

"""兼容转发：实现已迁至 boom_birds_control.px4_backend。"""
from boom_birds_nav._compat import forward as _forward

_forward(globals(), "boom_birds_control.px4_backend")
del _forward
