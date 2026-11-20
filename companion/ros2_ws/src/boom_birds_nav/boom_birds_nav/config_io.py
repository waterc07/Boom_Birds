# ******************************************************************************
# @file           : config_io.py
# @brief          : 将旧导航模块转发至 boom_birds_bringup.config_io。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

"""兼容转发：实现已迁至 boom_birds_bringup.config_io。"""
from boom_birds_nav._compat import forward as _forward

_forward(globals(), "boom_birds_bringup.config_io")
del _forward
