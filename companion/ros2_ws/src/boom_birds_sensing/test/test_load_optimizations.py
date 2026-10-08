import array
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage

from boom_birds_sensing.depth_node import DepthNode
from boom_birds_sensing.image_scaling import resize_raw_pair, scaled_raw_size
from boom_birds_sensing.stereo_source import StereoSourceNode, raw_camera_info


class Publisher:
    def __init__(self, count=0):
        self.count = count
        self.messages = []

    def get_subscription_count(self):
        return self.count

    def publish(self, message):
        self.messages.append(message)


@pytest.mark.parametrize("scale,size", [(1., (1280, 960)), (.75, (960, 720)), (.5, (640, 480))])
def test_raw_scaling_preserves_pair_and_camera_rays(scale, size):
    left = np.full((960, 1280), 30, np.uint8)
    right = np.full_like(left, 200)
    a, b = resize_raw_pair(left, right, scale)
    assert a.shape == b.shape == size[::-1]
    assert np.all(a == 30) and np.all(b == 200)
    k = np.array([[650., 0., 630.], [0., 650., 470.], [0., 0., 1.]])
    cal = dict(image_size=np.array([1280, 960]), K1=k, K2=k,
               D1=np.zeros(5), D2=np.zeros(5), T=np.array([-.067, 0., 0.]))
    info, baseline, _ = raw_camera_info(cal, "right", "cam1", *size)
    assert baseline == pytest.approx(.067)
    assert info.width == size[0] and info.height == size[1]
    assert info.p[3] == pytest.approx(-650 * scale * baseline)
    pixel = np.array([800., 600., 1.])
    np.testing.assert_allclose(np.linalg.inv(np.array(info.k).reshape(3, 3)) @ (pixel * [scale, scale, 1]),
                               np.linalg.inv(k) @ pixel)
    if scale == 1:
        assert a is left and b is right


@pytest.mark.parametrize("scale", [0, -1, 2, float("nan"), float("inf"), .333])
def test_invalid_scale_rejected(scale):
    with pytest.raises(ValueError):
        scaled_raw_size(1280, 960, scale)


def test_aux_outputs_resume_when_subscriber_arrives_and_keep_depth_contract():
    node = DepthNode.__new__(DepthNode)
    node.output_scale = 1.
    node.min_depth = .2
    node.max_depth = 5.
    node.frame_id = "cam0_rect"
    node.bridge = CvBridge()
    node.publish_color_preview = node.publish_depth_compat = node.publish_xyz = node.publish_compact = True
    node.publish_aux_without_subscribers = False
    for name in ["pub_depth", "pub_info", "pub_preview", "pub_depth_compat", "pub_xyz", "pub_xyz_valid"]:
        setattr(node, name, Publisher())
    node.camera_info = dict(width=2, height=2, fx=100., fy=100., cx=.5, cy=.5)
    node.frames = 0
    node.stats = dict(valid_ratio_sum=0., compute_ms_sum=0., over_range=0)
    xyz = np.ones((2, 2, 3), np.float32)
    xyz[0, 1] = np.nan
    result = SimpleNamespace(depth=xyz[:, :, 2], xyz=xyz,
                             valid=np.array([[True, False], [True, True]]), timings={"compute_ms": 1.})
    stamp = Header().stamp
    aux = [node.pub_preview, node.pub_depth_compat, node.pub_xyz, node.pub_xyz_valid]
    node._publish_result(result, stamp)
    assert len(node.pub_depth.messages) == len(node.pub_info.messages) == 1
    assert all(not p.messages for p in aux)
    for p in aux:
        p.count = 1
    node._publish_result(result, stamp)
    assert all(len(p.messages) == 1 for p in aux)
    depth = node.bridge.imgmsg_to_cv2(node.pub_depth.messages[-1])
    np.testing.assert_array_equal(depth, result.depth)
    cloud = node.pub_xyz.messages[-1]
    assert cloud.header.stamp == stamp and not cloud.is_dense
    assert np.isnan(np.frombuffer(cloud.data, dtype="<f4").reshape(2, 2, 3)[0, 1]).all()
    for p in aux:
        p.count = 0
    node._publish_result(result, stamp)
    assert all(len(p.messages) == 1 for p in aux)
    node.publish_aux_without_subscribers = True
    node._publish_result(result, stamp)
    assert all(len(p.messages) == 2 for p in aux)


@pytest.mark.parametrize("divisor", [1, 2, 4])
def test_reduced_decode_preserves_stamp_and_stereo_halves(divisor, monkeypatch):
    import boom_birds_sensing.depth_node as module
    source = np.hstack([np.full((960, 1280), 40, np.uint8), np.full((960, 1280), 180, np.uint8)])
    ok, packet = cv2.imencode(".jpg", source)
    assert ok
    msg = CompressedImage()
    msg.format = "jpeg"
    msg.header.stamp.sec = 123
    msg.header.stamp.nanosec = 456
    msg.data = array.array("B", packet.tobytes())
    captured = []
    monkeypatch.setattr(module, "process_stitched", lambda p, image: image)
    node = DepthNode.__new__(DepthNode)
    node.mjpeg_decode_divisor = divisor
    node.processor = None
    node._publish_result = lambda result, stamp: captured.append((result, stamp))
    node.on_packet(msg)
    image, stamp = captured[0]
    assert image.shape == (960 // divisor, 2560 // divisor)
    assert np.all(image[:, :1280 // divisor] == 40)
    assert np.all(image[:, 1280 // divisor:] == 180)
    assert stamp == msg.header.stamp


def test_scaled_raw_keeps_original_mjpeg_and_pair_stamp():
    node = StereoSourceNode.__new__(StereoSourceNode)
    node.raw_output_scale = .75
    node.raw_decode_divisor = 1
    node.frame_left, node.frame_right, node.frame_stitched = "cam0", "cam1", "cam0"
    node.bridge = CvBridge()
    node.counters = {"frames_published": 0}
    params = dict(publish_mjpeg=True, publish_stitched=False, publish_raw_without_subscribers=False)
    node.get_parameter = lambda key: SimpleNamespace(value=params[key])
    frame = SimpleNamespace(capture_ros_s=123.25, stitched=b"unchanged JPEG payload",
                            stitched_width=2560, height=960)
    node._take_frame = lambda: frame
    left = np.full((960, 1280), 30, np.uint8)
    node.frame_source = SimpleNamespace(decode=lambda frame: (left, left))
    node.pub_left, node.pub_right, node.pub_mjpeg = Publisher(1), Publisher(1), Publisher()
    node.pub_stitched = Publisher()
    infos = []
    node._publish_camera_info = lambda stamp, w, h: infos.append((stamp, w, h))
    node._tick_captured()
    a, b = node.pub_left.messages[0], node.pub_right.messages[0]
    assert (a.width, a.height) == (b.width, b.height) == (960, 720)
    assert a.header.stamp == b.header.stamp == infos[0][0]
    assert infos[0][1:] == (960, 720)
    assert bytes(node.pub_mjpeg.messages[0].data) == frame.stitched


@pytest.mark.parametrize("divisor", [1, 2, 4])
def test_raw_reduced_decoder_checks_original_size_and_stereo_order(divisor):
    from boom_birds_sensing.camera_timestamp import decode_stitched, jpeg_dimensions, CameraTimestampError
    image = np.hstack([np.full((960, 1280), 40, np.uint8), np.full((960, 1280), 180, np.uint8)])
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    data = encoded.tobytes()
    assert jpeg_dimensions(data) == (2560, 960)
    a, b = decode_stitched(data, 2560, 960, decode_divisor=divisor)
    assert a.shape == b.shape == (960 // divisor, 1280 // divisor)
    assert np.all(a == 40) and np.all(b == 180)
    with pytest.raises(CameraTimestampError):
        decode_stitched(data, 2560, 964, decode_divisor=divisor)


@pytest.mark.parametrize("data", [b"", b"not JPEG", b"\xff\xd8\xff", b"\xff\xd8\xff\xe0\x00\x01", b"\xff\xd8\xff\xe0\xff\xff"])
def test_malformed_jpeg_header_rejected(data):
    from boom_birds_sensing.camera_timestamp import jpeg_dimensions, CameraTimestampError
    with pytest.raises(CameraTimestampError):
        jpeg_dimensions(data)


def test_depth_pacing_keeps_latest_pending_and_shutdown_interrupts_wait():
    import threading
    import time
    node = DepthNode.__new__(DepthNode)
    node._depth_condition = threading.Condition()
    node._depth_stop = threading.Event()
    node._pending_pair = None
    node._worker_error = None
    node.dropped_processing = 0
    node.processing_period = .08
    first = threading.Event()
    second = threading.Event()
    seen = []
    def process(left, right):
        seen.append((left, time.monotonic()))
        (first if len(seen) == 1 else second).set()
    node.on_pair = process
    node._depth_thread = threading.Thread(target=node._process_worker)
    node._depth_thread.start()
    try:
        node.queue_pair(1, 1)
        assert first.wait(1)
        node.processing_period = 10.
        node.queue_pair(2, 2)
        node.queue_pair(3, 3)
        assert second.wait(1)
        assert [x[0] for x in seen] == [1, 3]
        assert seen[1][1] - seen[0][1] >= .07
        node.queue_pair(4, 4)
        time.sleep(.02)
        assert [x[0] for x in seen] == [1, 3]
    finally:
        started = time.monotonic()
        node.shutdown()
        assert time.monotonic() - started < .5
    assert node._worker_error is None
