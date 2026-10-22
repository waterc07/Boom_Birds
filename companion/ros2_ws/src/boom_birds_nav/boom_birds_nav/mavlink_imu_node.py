"""兼容转发：实现已迁至 tools.sensor_replay.mavlink_imu_node。"""
from boom_birds_nav._compat import forward as _forward

_forward(globals(), "tools.sensor_replay.mavlink_imu_node")
del _forward
