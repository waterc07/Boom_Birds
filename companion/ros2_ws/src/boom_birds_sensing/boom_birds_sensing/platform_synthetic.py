"""兼容导入：TEST-ONLY 板图渲染位于 platform_replay。"""
import sys
from . import platform_replay as _impl
sys.modules[__name__] = _impl
