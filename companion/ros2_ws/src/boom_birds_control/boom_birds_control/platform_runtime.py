"""兼容导入：运行配置与记录位于 platform_executor。"""
import sys
from . import platform_executor as _impl
sys.modules[__name__] = _impl
