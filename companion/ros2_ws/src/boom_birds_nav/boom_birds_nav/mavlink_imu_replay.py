"""兼容转发：实现已迁至 boom_birds_sensing.mavlink_imu_replay。"""
from boom_birds_nav._compat import forward as _forward

_forward(globals(), "boom_birds_sensing.mavlink_imu_replay")
del _forward
