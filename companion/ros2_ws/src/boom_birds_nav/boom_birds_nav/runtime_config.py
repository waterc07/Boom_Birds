"""兼容转发：实现已迁至 boom_birds_control.runtime_config。

保留本模块只为不打断既有导入路径；这里**不复制第二份实现**。
"""
from boom_birds_control.runtime_config import *  # noqa: F401,F403
from boom_birds_control import runtime_config as _impl
import sys as _sys

_this = _sys.modules[__name__]
for _n in dir(_impl):
    if not _n.startswith("_"):
        setattr(_this, _n, getattr(_impl, _n))
__doc__ = _impl.__doc__
del _sys, _this, _n

if __name__ == "__main__":
    raise SystemExit(getattr(_impl, "main", lambda: 0)())
