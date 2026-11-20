# ******************************************************************************
# @file           : control_protocol.py
# @brief          : 将旧导航模块转发至 boom_birds_control.control_protocol。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

"""兼容转发：实现已迁至 boom_birds_control.control_protocol。"""
from boom_birds_nav._compat import forward as _forward

_forward(globals(), "boom_birds_control.control_protocol")
del _forward
