"""Replay image manifests or rosbag2 Image/CompressedImage using recorded sample stamps."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
import yaml
from .platform_observation import BoardDetector


def manifest_frames(path):
    path = Path(path)
    for line_number, line in enumerate(path.read_text().splitlines(), 1):
        if not line.strip(): continue
        entry = json.loads(line)
        if set(entry) != {"stamp", "image"}: raise ValueError(f"manifest_fields:{line_number}")
        image_path = path.parent / entry["image"]
        image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if image is None: raise ValueError(f"image_unreadable:{image_path}")
        yield entry["stamp"], image, str(image_path)


def bag_frames(path, topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image, CompressedImage
    from cv_bridge import CvBridge
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    types = {x.name:x.type for x in reader.get_all_topics_and_types()}
    kind = types.get(topic)
    if kind not in ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"):
        raise ValueError("bag_image_topic_missing_or_type")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    bridge = CvBridge()
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, Image if kind.endswith("/Image") else CompressedImage)
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if stamp <= 0: raise ValueError("bag_sample_stamp_missing")
        if kind.endswith("/Image"):
            image = bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        else:
            image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_GRAYSCALE)
            if image is None: raise ValueError("compressed_image_invalid")
        yield stamp, image, str(path)+":"+topic


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument("--manifest")
    inputs.add_argument("--bag")
    parser.add_argument("--topic", default="/boom_birds/downward/image")
    parser.add_argument("--test-only", action="store_true")
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    profile = yaml.safe_load(Path(args.config).read_text())
    detector = BoardDetector(profile, test_only=args.test_only)
    frames = manifest_frames(args.manifest) if args.manifest else bag_frames(args.bag,args.topic)
    out = Path(args.out); out.parent.mkdir(parents=True, exist_ok=True)
    count=valid=0
    with out.open("w") as stream:
        for stamp, image, source in frames:
            observation = detector.observe(image,stamp,source)
            stream.write(json.dumps(asdict(observation), allow_nan=False)+"\n")
            count+=1; valid+=int(observation.valid)
    summary = dict(kind="REPLAY", test_only=args.test_only, synthetic=profile["synthetic"],
                   count=count, valid=valid, config=profile,
                   config_sha256=hashlib.sha256(Path(args.config).read_bytes()).hexdigest(),
                   input=args.manifest or args.bag, input_topic=args.topic if args.bag else None)
    out.with_suffix(".summary.json").write_text(json.dumps(summary,indent=2))
    if count==0: raise ValueError("empty_replay")

if __name__ == "__main__": main()
