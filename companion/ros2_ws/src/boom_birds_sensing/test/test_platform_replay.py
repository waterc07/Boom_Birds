from dataclasses import asdict
from pathlib import Path
import json
import subprocess
import sys
import cv2
import numpy as np
import pytest
import yaml
from boom_birds_sensing.platform_observation import BoardDetector
from boom_birds_sensing.platform_replay import bag_frames,manifest_frames,main

CONFIG=Path(__file__).parents[2]/"boom_birds_control/config/platform_landing_test.yaml"


def fixture_image():
    detector=BoardDetector(yaml.safe_load(CONFIG.read_text()),test_only=True)
    image=np.full((480,640),255,np.uint8)
    image[180:300,260:380]=cv2.aruco.drawMarker(detector.dictionary,7,120)
    return image


def test_manifest_cli_retains_sampling_time_and_quality(tmp_path):
    image=fixture_image()
    cv2.imwrite(str(tmp_path/"tag.png"),image)
    blank=np.full_like(image,255)
    cv2.imwrite(str(tmp_path/"blank.png"),blank)
    manifest=tmp_path/"frames.jsonl"
    manifest.write_text(json.dumps(dict(stamp=123.1,image="tag.png"))+"\n"+
                        json.dumps(dict(stamp=123.2,image="blank.png"))+"\n")
    output=tmp_path/"observations.jsonl"
    main(["--config",str(CONFIG),"--manifest",str(manifest),"--test-only","--out",str(output)])
    rows=[json.loads(line) for line in output.read_text().splitlines()]
    assert len(rows)==2 and rows[0]["valid"] and not rows[1]["valid"]
    assert rows[0]["stamp"]==123.1 and rows[0]["tag_ids"]==[7]
    assert rows[0]["frame"]=="body_flu" and rows[0]["quality"]["synthetic"] is True
    assert json.loads(output.with_suffix(".summary.json").read_text())["kind"]=="REPLAY"


def test_rosbag_real_cdr_image_and_compressed_paths(tmp_path):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image,CompressedImage
    from cv_bridge import CvBridge
    image=fixture_image()
    bag=tmp_path/"bag"
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr","cdr"))
    for topic,kind in (("/down/image","sensor_msgs/msg/Image"),
                       ("/down/compressed","sensor_msgs/msg/CompressedImage")):
        writer.create_topic(rosbag2_py.TopicMetadata(id=0,name=topic,type=kind,serialization_format="cdr"))
    msg=CvBridge().cv2_to_imgmsg(image,encoding="mono8")
    msg.header.stamp.sec=321;msg.header.stamp.nanosec=10000000
    writer.write("/down/image",serialize_message(msg),999000000000)
    compressed=CompressedImage()
    compressed.header=msg.header;compressed.format="png"
    compressed.data=cv2.imencode(".png",image)[1].tobytes()
    writer.write("/down/compressed",serialize_message(compressed),999000000001)
    del writer
    for topic in ("/down/image","/down/compressed"):
        frames=list(bag_frames(bag,topic))
        assert len(frames)==1 and frames[0][0]==321.01
        assert np.array_equal(frames[0][1],image)
        output=tmp_path/(topic.split("/")[-1]+".jsonl")
        main(["--config",str(CONFIG),"--bag",str(bag),"--topic",topic,"--test-only","--out",str(output)])
        assert json.loads(output.read_text())["valid"]


def test_replay_refuses_real_entry_synthetic_and_wrong_image_size(tmp_path):
    cv2.imwrite(str(tmp_path/"small.png"),np.full((200,200),255,np.uint8))
    manifest=tmp_path/"frames.jsonl";manifest.write_text('{"stamp":1,"image":"small.png"}')
    with pytest.raises(ValueError,match="synthetic"):
        main(["--config",str(CONFIG),"--manifest",str(manifest),"--out",str(tmp_path/"live.jsonl")])
    main(["--config",str(CONFIG),"--manifest",str(manifest),"--test-only","--out",str(tmp_path/"test.jsonl")])
    assert json.loads((tmp_path/"test.jsonl").read_text())["reason"]=="image_size"


def test_ros_observation_node_constructs_without_camera(tmp_path):
    # A real ROS node callback computes and publishes a timestamped observation.
    import rclpy
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    from boom_birds_sensing.platform_observation_node import PlatformObservationNode
    context=rclpy.context.Context();rclpy.init(context=context)
    node=PlatformObservationNode(context=context,parameter_overrides=[
        Parameter("config_file",value=str(CONFIG)),Parameter("test_only",value=True)])
    from rclpy.executors import SingleThreadedExecutor
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    messages=[]
    sub=node.create_subscription(String,"/boom_birds/platform/observation",
                                 lambda msg:messages.append(json.loads(msg.data)),10)
    try:
        msg=CvBridge().cv2_to_imgmsg(fixture_image(),encoding="mono8")
        msg.header.stamp.sec=456
        import time
        discover=time.monotonic()+2.
        while time.monotonic()<discover and node.pub.get_subscription_count()==0:
            executor.spin_once(timeout_sec=.05)
        node.on_image(msg)
        deadline=time.monotonic()+2.
        while time.monotonic()<deadline and not messages:executor.spin_once(timeout_sec=.05)
        assert messages and messages[0]["stamp"]==456. and messages[0]["valid"]
    finally:
        executor.remove_node(node);executor.shutdown()
        node.destroy_node();rclpy.shutdown(context=context)
