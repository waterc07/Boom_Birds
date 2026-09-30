"""兼容转发：实现已迁至 boom_birds_sensing.stereo_source。

保留本模块只为不打断既有导入路径（launch/脚本/测试）；这里**不复制第二份实现**，
所有公共名与 __doc__ 都指向同一个对象。新代码请直接用 boom_birds_sensing。
"""
from boom_birds_sensing.stereo_source import *  # noqa: F401,F403
from boom_birds_sensing import stereo_source as _impl
import sys as _sys

_this = _sys.modules[__name__]
for _n in dir(_impl):
    if not _n.startswith("_"):
        setattr(_this, _n, getattr(_impl, _n))
__doc__ = _impl.__doc__
del _sys, _this, _n

if __name__ == "__main__":
    raise SystemExit(getattr(_impl, "main", lambda: 0)())
