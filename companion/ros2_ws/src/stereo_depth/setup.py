"""stereo_depth 算法模块的安装脚本。

仓库内直接运行（`python3 depth_preview.py`）不依赖本文件：Config 默认路径指向
源码目录。只有在 colcon build（BOOM_BIRDS_COLCON_BUILD=1，由 CMakeLists.txt 设置）
时才把 depth_preview 模块安装到 site-packages，使安装后的 ROS 2 节点可以
`import depth_preview`，无需拼接 sys.path。
"""
import os

from setuptools import setup
from setuptools.command.build_py import build_py
from setuptools.command.develop import develop


class _NoBuild(build_py):
    """非 colcon 构建时跳过安装，保持仓库内开发布局不受影响。"""

    def run(self):
        if os.environ.get("BOOM_BIRDS_COLCON_BUILD") == "1":
            super().run()
        else:
            print("[stereo_depth] 跳过安装（仅 colcon build 时安装算法模块）")


class _NoDevelop(develop):
    def run(self):
        if os.environ.get("BOOM_BIRDS_COLCON_BUILD") == "1":
            super().run()
        else:
            print("[stereo_depth] 跳过 develop 安装")


setup(
    name="stereo_depth",
    version="0.1.0",
    description="Boom_Birds 双目深度算法模块（StereoProcessor）",
    py_modules=["depth_preview"],
    cmdclass={"build_py": _NoBuild, "develop": _NoDevelop},
)
