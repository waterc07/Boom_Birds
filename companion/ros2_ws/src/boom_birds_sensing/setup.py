"""boom_birds_sensing 安装脚本（由 colcon/ament 在安装阶段调用）。"""

from setuptools import setup

setup(
    name="boom_birds_sensing",
    version="0.1.0",
    packages=["boom_birds_sensing"],
    zip_safe=False,
    entry_points={
        "console_scripts": [
            # 唯一采集与发布入口（v4l2 / replay / file），经 stereo_capture 打开设备
            "stereo_source = boom_birds_sensing.stereo_source:main",
            # 深度计算与发布（32FC1 米制 / NaN）
            "depth_node = boom_birds_sensing.depth_node:main",
            # 位姿适配（按采集时间配对/插值）
            "pose_adapter = boom_birds_sensing.pose_adapter:main",
            # 飞控 IMU 上行（PX4 MAVLink HIGHRES_IMU + TIMESYNC）
            "mavlink_imu_node = boom_birds_sensing.mavlink_imu_node:main",
            # 采集时间戳能力核验（只读探测）
            "camera_timestamp_probe = boom_birds_sensing.camera_timestamp:main",
        ]
    },
)
