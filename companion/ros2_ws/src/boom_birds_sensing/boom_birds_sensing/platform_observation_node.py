"""ROS image subscriber; never opens a device."""
import json
import cv2
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
        self.declare_parameter("image_frame_id","")
        self.declare_parameter("opencv_threads",2)
        cv2.setNumThreads(int(self.get_parameter("opencv_threads").value))
        self.declare_parameter("observation_topic", "/boom_birds/platform/observation")
        self.detector = BoardDetector(yaml.safe_load(Path(
            self.get_parameter("config_file").value).read_text()),
            test_only=self.get_parameter("test_only").value)
        self.declare_parameter("feature_tracking_enabled",False)
        self.features=None
        if self.get_parameter("feature_tracking_enabled").value:
            from .platform_feature_track import BoardFeatureTrack,TrackConfig
            config=self.detector.profile.get("feature_tracking",{})
            self.features=BoardFeatureTrack(self.detector,TrackConfig(**config))
        self.featurePub=self.create_publisher(String,"/boom_birds/platform/feature_observation",10)
        self.bridge = CvBridge()
        self.declare_parameter("process_latest_only", True)
        self.closing = False
        self.worker = LatestFrameWorker(self.processImage) if self.get_parameter("process_latest_only").value else None
        self.metrics = self.create_publisher(String, "/boom_birds/platform/observation_metrics", 10)
        self.create_timer(1., self.publishMetrics)
        self.pub = self.create_publisher(String,self.get_parameter("observation_topic").value,10)
        self.sub = self.create_subscription(Image,self.get_parameter("image_topic").value,
                                            self.onImage,qos_profile_sensor_data)

    def onImage(self, msg):
        if self.worker:
            self.worker.submit(msg)
        else:
            self.processImage(msg)

    def publishMetrics(self):
        if self.worker:
            self.metrics.publish(String(data=json.dumps(self.worker.snapshot())))

    def close(self):
        self.closing = True
        return self.worker.close() if self.worker else True

    def destroy_node(self):
        self.close()
        return super().destroy_node()

    def processImage(self,msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if stamp <= 0:
            self.get_logger().error("image sample stamp missing")
            return
        try:
            expected=str(self.get_parameter("image_frame_id").value)
            if expected and msg.header.frame_id!=expected:raise ValueError("downward_image_frame")
            image=self.bridge.imgmsg_to_cv2(msg,desired_encoding="mono8")
            if image.shape[:2]!=tuple(reversed(self.detector.camera["image_size"])):
                raise ValueError("image_size")
            detections,rejected=self.detector.findCorners(image)
            o=self.detector.estimate(detections,stamp,"ROS_Image",rejected)
            feature=None
            if self.features is not None:
                if detections and not any(i in self.detector.tags for i,_ in detections):
                    feature=self.features.invalid(stamp,"conflicting_tag_identity")
                else:
                    feature=self.features.seedDecoded(image,detections,stamp)
                    if feature is None:feature=self.features.update(image,stamp)

            o.quality["processing_age_s"] = self.get_clock().now().nanoseconds*1e-9-stamp
            if not self.closing and self.context.ok():
                self.pub.publish(String(data=json.dumps(asdict(o),allow_nan=False)))
                if feature is not None:
                    self.featurePub.publish(String(data=json.dumps(feature,allow_nan=False)))
        except (ValueError,TypeError,RuntimeError,cv2.error) as exc:
            self.get_logger().error(str(exc))
            if self.features is not None and not self.closing and self.context.ok():
                feature=self.features.invalid(stamp,"image_processing_failed")
                self.featurePub.publish(String(data=json.dumps(feature,allow_nan=False)))


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
