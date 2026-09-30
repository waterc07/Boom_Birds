"""兼容导入：外参读取实现位于 boom_birds_sensing.config_io。"""
import sys
from boom_birds_sensing import config_io as _impl
sys.modules[__name__] = _impl
