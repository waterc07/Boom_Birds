"""兼容转发：实现已迁至 boom_birds_sim.sih_params。"""
from boom_birds_nav._compat import forward as _forward

_forward(globals(), "boom_birds_sim.sih_params")
del _forward
