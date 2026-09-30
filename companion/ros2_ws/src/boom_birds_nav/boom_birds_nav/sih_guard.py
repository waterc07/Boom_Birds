"""兼容转发：实现已迁至 boom_birds_control.sih_guard。

保留本模块只为不打断既有导入路径（launch/脚本/测试/外部订阅者）；这里
**不复制第二份实现**——所有公共名都指向同一个对象。新代码请直接用
boom_birds_control。

注意：不能只用 `from ... import *`。实现模块若定义了 `__all__`，星号导入会被
限制在 `__all__` 之内，于是"文档里写着转发、实际少了一半名字"，测试会在导入时
才炸。这里显式把实现的全部公共名绑定到本模块。
"""
from boom_birds_control.sih_guard import *  # noqa: F401,F403
from boom_birds_control import sih_guard as _impl
import sys as _sys

_this = _sys.modules[__name__]
for _n in dir(_impl):
    if not _n.startswith("_"):
        setattr(_this, _n, getattr(_impl, _n))
# 转发 docstring：模块级边界说明属于实现，兼容入口必须一并转发，
# 否则按 __doc__ 做断言的地方会误判。
__doc__ = _impl.__doc__
del _sys, _this, _n

# `python -m boom_birds_nav.<mod>` 兼容转发：入口点迁走后仍有脚本/测试用 -m 调旧路径。
if __name__ == "__main__":
    raise SystemExit(getattr(_impl, "main", lambda: 0)())
