# ******************************************************************************
# @file           : sih_guard.py
# @brief          : 将旧导航模块转发至 boom_birds_control.sih_guard。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

"""兼容转发：实现已迁至 boom_birds_control.sih_guard。"""
from boom_birds_nav._compat import forward as _forward

_forward(globals(), "boom_birds_control.sih_guard")
del _forward
