"""boom_birds_bringup 安装脚本（由 colcon/ament 在安装阶段调用）。"""

from setuptools import setup

setup(
    name="boom_birds_bringup",
    version="0.1.0",
    packages=["boom_birds_bringup"],
    zip_safe=False,
    entry_points={
        "console_scripts": [
            # 任务编排（生命周期状态机）；模式命令只经 PX4 接口服务
            "lifecycle_node = boom_birds_bringup.lifecycle_node:main",
            # RuntimeConfig → PX4 起飞参数（SIH 脚本调用的唯一入口）
            "sih_params = boom_birds_bringup.sih_params:main",
        ]
    },
)
