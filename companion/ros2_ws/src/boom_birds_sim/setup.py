"""boom_birds_sim 安装脚本（由 colcon/ament 在安装阶段调用）。"""

from setuptools import setup

setup(
    name="boom_birds_sim",
    version="0.1.0",
    packages=["boom_birds_sim"],
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "synthetic_stereo_source = boom_birds_sim.synthetic_stereo_source:main",
            # SIH 真值源（只回环 14550，dry_run）
            "sitl_truth_source = boom_birds_sim.sitl_truth_source:main",
            # 旧分段启动的悬停中继（正式入口已用 lifecycle；保留兼容）
            "sitl_hold_relay = boom_birds_sim.sitl_hold_relay:main",
            # TEST-ONLY 合成 VIO/IMU 源
            "vio_source = boom_birds_sim.vio_source:main",
        ]
    },
)
