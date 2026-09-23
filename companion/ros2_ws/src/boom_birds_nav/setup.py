"""boom_birds_nav 安装脚本（由 colcon/ament 在安装阶段调用）。"""

from setuptools import setup

setup(
    name="boom_birds_nav",
    version="0.1.0",
    packages=["boom_birds_nav"],
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "stereo_source = boom_birds_nav.stereo_source:main",
            "depth_node = boom_birds_nav.depth_node:main",
            "pose_adapter = boom_birds_nav.pose_adapter:main",
            "vio_source = boom_birds_nav.vio_source:main",
        ]
    },
)
