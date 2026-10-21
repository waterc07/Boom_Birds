"""仅安装 stereo_depth 运行库；预览、标定和回放工具位于工作空间 tools。"""
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
    packages=["stereo_depth"],
    cmdclass={"build_py": _NoBuild, "develop": _NoDevelop},
)
