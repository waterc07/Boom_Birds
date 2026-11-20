# ******************************************************************************
# @file           : conftest.py
# @brief          : 为标定工具测试加入工作空间和双目深度库的源码导入路径。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

from pathlib import Path
import sys

WS = Path(__file__).resolve().parents[3]
for path in (WS, WS / "src/stereo_depth"):
    sys.path.insert(0, str(path))
