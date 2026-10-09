"""兼容导入：原图缩放位于 camera_geometry。"""
import sys
from . import camera_geometry as _impl
sys.modules[__name__] = _impl
