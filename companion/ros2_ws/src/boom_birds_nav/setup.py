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
            # 真实链路：PX4 MAVLink HIGHRES_IMU → /boom_birds/imu
            "mavlink_imu_node = boom_birds_nav.mavlink_imu_node:main",
            # 采集时间戳能力核验（只读探测）
            "camera_timestamp_probe = boom_birds_nav.camera_timestamp:main",
            # 规划输出 → Px4Interface → PX4 的高层 setpoint 与失效处理
            "px4_interface_node = boom_birds_nav.px4_interface_node:main",
            "sitl_truth_source = boom_birds_nav.sitl_truth_source:main",
        ]
    },
)
