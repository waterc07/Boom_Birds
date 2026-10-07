#!/usr/bin/env python3
"""TEST-ONLY：由本机 SIH 真值驱动的可托管导航/相机替身进程。"""
import argparse
import json
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import String
from boom_birds_control.runtime_config import DEFAULTS
from boom_birds_control.platform_landing import ned_from_flu
from boom_birds_control.frames import rot_to_quat

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kind", choices=("navigation", "camera"), required=True)
    args = ap.parse_args()
    rclpy.init()
    node = Node("platform_test_" + args.kind)
    if args.kind == "navigation":
        odom = node.create_publisher(Odometry, DEFAULTS.odom_topic, 10)
        imu = node.create_publisher(Imu, DEFAULTS.imu_topic, 10)
    else:
        camera = node.create_publisher(Image, DEFAULTS.depth_topic, 10)
    def truth(msg):
        data = json.loads(msg.data)
        stamp = node.get_clock().now().to_msg()
        if args.kind == "navigation":
            sample = Odometry()
            sample.header.stamp, sample.header.frame_id = stamp, DEFAULTS.world_frame
            sample.child_frame_id = "body"
            p, v = data["position"], data["velocity"]
            sample.pose.pose.position.x, sample.pose.pose.position.y, sample.pose.pose.position.z = p[0], -p[1], -p[2]
            sample.twist.twist.linear.x, sample.twist.twist.linear.y, sample.twist.twist.linear.z = v[0], -v[1], -v[2]
            rotation = np.diag([1., -1., -1.]) @ ned_from_flu(*data["attitude"])
            q = rot_to_quat(rotation)
            sample.pose.pose.orientation.x, sample.pose.pose.orientation.y, sample.pose.pose.orientation.z, sample.pose.pose.orientation.w = map(float, q)
            odom.publish(sample)
            m = Imu()
            m.header = sample.header
            m.orientation = sample.pose.pose.orientation
            imu.publish(m)
        else:
            sample = Image()
            sample.header.stamp = stamp
            sample.header.frame_id = DEFAULTS.camera_frame
            sample.width = sample.height = 16
            sample.encoding, sample.step = "32FC1", 64
            sample.data = np.full((16, 16), 2., np.float32).tobytes()
            camera.publish(sample)
    node.create_subscription(String, "/boom_birds/platform/test_truth", truth, 10)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
