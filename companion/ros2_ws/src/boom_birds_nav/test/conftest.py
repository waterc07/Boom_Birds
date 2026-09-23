"""把 src 布局下的两个包加入导入路径，使 pytest 可直接运行（不依赖 colcon install）。"""

import pathlib
import sys

_WS_SRC = pathlib.Path(__file__).resolve().parents[2]      # .../ros2_ws/src
for _p in (_WS_SRC / "stereo_depth", _WS_SRC / "boom_birds_nav"):
    if _p.is_dir() and str(_p) not in sys.path:
        sys.path.insert(0, str(_p))
