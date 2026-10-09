"""ROS image subscriber; never opens a device."""
import json
from dataclasses import asdict
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from .frame_worker import LatestFrameWorker
from .platform_observation import BoardDetector


class PlatformObservationNode(Node):
    def __init__(self, **kwargs):
        super().__init__("platform_observation", **kwargs)
        self.declare_parameter("config_file", "")
        self.declare_parameter("test_only", False)
        self.declare_parameter("image_topic", "/boom_birds/downward/image")
        self.declare_parameter("observation_topic", "/boom_birds/platform/observation")
        self.detector = BoardDetector(yaml.safe_load(Path(
            self.get_parameter("config_file").value).read_text()),
            test_only=self.get_parameter("test_only").value)
        self.bridge = CvBridge()
        self.declare_parameter("process_latest_only", True)
        self.closing = False
        self.worker = LatestFrameWorker(self.process_image) if self.get_parameter("process_latest_only").value else None
        self.metrics = self.create_publisher(String, "/boom_birds/platform/observation_metrics", 10)
        self.create_timer(1., self.publish_metrics)
        self.pub = self.create_publisher(String,self.get_parameter("observation_topic").value,10)
        self.sub = self.create_subscription(Image,self.get_parameter("image_topic").value,
                                            self.on_image,qos_profile_sensor_data)

    def on_image(self, msg):
        if self.worker:
            self.worker.submit(msg)
        else:
            self.process_image(msg)

    def publish_metrics(self):
        if self.worker:
            self.metrics.publish(String(data=json.dumps(self.worker.snapshot())))

    def close(self):
        self.closing = True
        return self.worker.close() if self.worker else True

    def destroy_node(self):
        self.close()
        return super().destroy_node()

    def process_image(self,msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if stamp <= 0:
            self.get_logger().error("image sample stamp missing")
            return
        try:
            image=self.bridge.imgmsg_to_cv2(msg,desired_encoding="mono8")
            o=self.detector.observe(image,stamp,"ROS_Image")
            o.quality["processing_age_s"] = self.get_clock().now().nanoseconds*1e-9-stamp
            if not self.closing and self.context.ok():
                self.pub.publish(String(data=json.dumps(asdict(o),allow_nan=False)))
        except (ValueError,RuntimeError) as exc:
            self.get_logger().error(str(exc))


def main():
    rclpy.init()
    node=None
    try:
        node=PlatformObservationNode()
        rclpy.spin(node)
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=="__main__": main()
