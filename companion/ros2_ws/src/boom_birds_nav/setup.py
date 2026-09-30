"""boom_birds_nav 安装脚本（由 colcon/ament 在安装阶段调用）。"""

from setuptools import setup

setup(
    name="boom_birds_nav",
    version="0.1.0",
    packages=["boom_birds_nav"],
    zip_safe=False,
    entry_points={
        # 拆包后 nav 只是兼容转发层：入口点已随实现迁到 sensing/control/bringup/sim，
        # 这里**故意为空**（保留键是为了让安装脚本能明确报告"本包不提供可执行文件"）。
        "console_scripts": []
    },
)
