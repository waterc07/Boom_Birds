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
            "tag_flight_task = boom_birds_bringup.tag_flight_node:main",
            "feature_ekf_candidate = boom_birds_bringup.feature_ekf_candidate_node:main",
            # 任务自有的 VIO/适配器子进程监管与计算释放服务
            "compute_supervisor = boom_birds_bringup.compute_supervisor:main",
        ]
    },
)
