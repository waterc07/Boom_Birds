"""boom_birds_control 安装脚本（由 colcon/ament 在安装阶段调用）。"""

from setuptools import setup

setup(
    name="boom_birds_control",
    version="0.1.0",
    packages=["boom_birds_control"],
    zip_safe=False,
    entry_points={
        "console_scripts": [
            # 规划输出 → Px4Interface → PX4 的高层 setpoint 与失效处理
            "px4_interface_node = boom_birds_control.px4_interface_node:main",
        ]
    },
)
