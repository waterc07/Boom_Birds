"""兼容转发：实现已迁至 boom_birds_bringup.lifecycle_cli。"""
from boom_birds_nav._compat import forward as _forward

_forward(globals(), "boom_birds_bringup.lifecycle_cli")
del _forward
