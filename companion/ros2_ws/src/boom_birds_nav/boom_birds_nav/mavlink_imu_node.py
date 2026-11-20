# ******************************************************************************
# @file           : mavlink_imu_node.py
# @brief          : 将旧导航模块转发至 tools.sensor_replay.mavlink_imu_node。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

"""兼容转发：实现已迁至 tools.sensor_replay.mavlink_imu_node。"""
from boom_birds_nav._compat import forward as _forward

_forward(globals(), "tools.sensor_replay.mavlink_imu_node")
del _forward
