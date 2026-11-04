"""裁剪匹配、原图几何与执行包络回退。"""
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PolygonStamped, Point32
from boom_birds_sensing.depth_roi import DemandRoi, projectEnvelope
from boom_birds_sensing.depth_node import DepthNode
from stereo_depth.core import Config, StereoProcessor

K = np.array([[180., 0, 160.], [0, 180., 120.], [0, 0, 1.]])


def envelope(near=1.):
    return np.array([[x, y, z] for z in (near, 3.)
        for x in (-.2, .2) for y in (-.18, .18)])


def test_margin_expands_both_dimensions_from_their_center():
    points = envelope()
    a = projectEnvelope(points, K, (320, 240), .2)
    b = projectEnvelope(points, K, (320, 240), .5)
    assert a.reason == b.reason == "demand"
    x0, y0, x1, y1 = b.bounds
    # 0.4 x 0.36 m 的截面在 1 m 处投影，宽高各乘 1.5；含端点取整。
    assert x0 <= 106 and x1 >= 214 and y0 <= 71.4 and y1 >= 168.6
    assert x1-x0 > a.bounds[2]-a.bounds[0]
    assert y1-y0 > a.bounds[3]-a.bounds[1]


def test_near_volume_prevents_far_plane_only_crop():
    assert projectEnvelope(envelope(.2), K, (320, 240)).reason == "envelope_outside_fov"


@pytest.mark.parametrize("points", [[], [[1, 2, 3]], np.full((8, 3), np.nan),
    np.zeros((8, 3)), np.ones((8, 2)), np.ones((257, 3))])
def test_invalid_envelope_falls_back_to_full_frame(points):
    assert projectEnvelope(points, K, (320, 240)).bounds is None


def test_timestamps_frame_clock_rewind_and_wall_age():
    roi = DemandRoi("rect", .5, .1)
    assert roi.select(10., K, (320, 240), now=20.).reason == "demand_missing"
    roi.push(10., "rect", envelope(), received=20.)
    assert roi.select(10.01, K, (320, 240), now=20.01).bounds is not None
    for stamp, now in ((9.99, 20.01), (10.11, 20.01), (10.01, 20.11), (10.01, 19.99)):
        assert roi.select(stamp, K, (320, 240), now=now).bounds is None
    roi.push(9., "rect", envelope(), received=20.02)
    assert roi.select(9., K, (320, 240), now=20.02).bounds is None
    roi.push(11., "world", envelope(), received=20.03)
    assert roi.select(11., K, (320, 240), now=20.03).bounds is None
    roi.push(12., "rect", [], received=20.04)
    assert roi.select(12., K, (320, 240), now=20.04).bounds is None


@pytest.mark.parametrize("with_xyz", [True, False])
@pytest.mark.parametrize("reuse", [True, False])
def test_cropped_matching_retains_metric_geometry_and_unknown_outside(with_xyz, reuse):
    processor = StereoProcessor(replace(Config(), reuse_buffers=reuse))
    rng = np.random.default_rng(77)
    left = rng.integers(0, 256, (240, 320), dtype=np.uint8)
    right = np.zeros_like(left)
    right[:, :-12] = left[:, 12:]
    roi = (125, 75, 205, 165)
    a, b, disparity, output, valid, timings = processor.matchRectified(
        left, right, with_xyz=with_xyz, roi=roi)
    assert a.shape == b.shape == valid.shape == (240, 320)
    mask = np.zeros_like(valid); mask[75:165, 125:205] = True
    assert not valid[~mask].any()
    assert np.isnan(output[~mask]).all() and np.isnan(disparity[~mask]).all()
    assert valid[90:150, 135:195].mean() > .95
    depth = output[:, :, 2] if with_xyz else output
    expected = processor.q[2, 3] / (processor.q[3, 2]*12 + processor.q[3, 3])
    assert np.nanmedian(depth[90:150, 135:195]) == pytest.approx(expected, rel=.01)
    if with_xyz:
        rows, cols = np.where(valid)
        expectedX = (cols-processor.p1[0, 2])*depth[rows, cols]/processor.p1[0, 0]
        np.testing.assert_allclose(output[rows, cols, 0], expectedX, atol=1e-6)
    assert timings["roi_area_fraction"] == pytest.approx(80*90/(320*240))
    assert timings["roi_area_fraction"] < timings["match_area_fraction"] < 1
    snapshot = output.copy()
    processor.matchRectified(left, right, with_xyz=with_xyz)
    np.testing.assert_array_equal(snapshot, output)


def test_actual_matcher_receives_cropped_arrays_and_full_roi_is_identical():
    p = StereoProcessor(Config())
    rng = np.random.default_rng(5)
    a = rng.integers(0, 256, (240, 320), dtype=np.uint8)
    b = np.roll(a, -12, axis=1)
    full = p.matchRectified(a, b)
    explicit = p.matchRectified(a, b, roi=(0, 0, 320, 240))
    np.testing.assert_array_equal(full[4], explicit[4])
    np.testing.assert_allclose(full[3], explicit[3])
    shapes = []
    original = p.matcherA
    class Spy:
        def compute(self, left, right):
            shapes.append((left.shape, right.shape))
            return original.compute(left, right)
    p.matcherA = Spy()
    p.matchRectified(a, b, roi=(140, 100, 180, 140))
    assert shapes[0][0][0] < 240 and shapes[0][0][1] < 320


@pytest.mark.parametrize("bounds", [(-1, 0, 3, 4), (0, 0, 400, 240), (0, 0, 0, 10),
    (0., 0, 320, 240), (0, 0, 320)])
def test_bad_roi_rejected(bounds):
    p = StereoProcessor(Config())
    a = np.zeros((240, 320), dtype=np.uint8)
    with pytest.raises(ValueError, match="ROI"):
        p.matchRectified(a, a, roi=bounds)


def test_ros_envelope_ingress_and_frame_selection():
    node = DepthNode.__new__(DepthNode)
    node.roiDemand = DemandRoi("rect")
    node.processor = SimpleNamespace(p1=np.c_[K, np.zeros(3)], depth_size=(320, 240))
    msg = PolygonStamped()
    msg.header.stamp.sec = 10
    msg.header.frame_id = "rect"
    msg.polygon.points = [Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in envelope()]
    node._onRoiDemand(msg)
    assert node._roiForFrame(Time(sec=10, nanosec=10_000_000)) is not None
    msg.header.stamp.sec = 11
    msg.polygon.points = []
    node._onRoiDemand(msg)
    assert node._roiForFrame(Time(sec=11)) is None


def test_ros_depth_publishing_preserves_geometry_and_reports_crop():
    import json
    import rclpy
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    rclpy.init(args=["--ros-args", "-p", "calibration_file:="+str(Config.calibration),
        "-p", "roi_demand_topic:=/test/roi_envelope", "-p", "frame_id:=rect",
        "-p", "publish_xyz:=false", "-p", "publish_compact_xyz:=false"])
    node = None
    try:
        node = DepthNode()
        captured = {}
        # 真实 ROS 消息转换与节点处理；publisher 只替换为捕获器，不运行设备或 DDS 发现等待。
        class Capture:
            def __init__(self, key): self.key = key
            def publish(self, msg): captured[self.key] = msg
        node.pub_depth, node.pub_info, node.pubRoiStatus = [Capture(k) for k in ("depth", "info", "roi")]
        msg = PolygonStamped()
        msg.header.stamp.sec = 10
        msg.header.frame_id = "rect"
        msg.polygon.points = [Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in envelope()]
        node._onRoiDemand(msg)
        rng = np.random.default_rng(4)
        left = rng.integers(0, 256, (240, 320), dtype=np.uint8)
        right = np.roll(left, -12, axis=1)
        stamp = Time(sec=10, nanosec=1_000_000)
        node.onDecodedPair(left, right, stamp)
        assert isinstance(captured["depth"], Image) and isinstance(captured["info"], CameraInfo)
        status = json.loads(captured["roi"].data)
        assert status["reason"] == "demand" and status["match_area_fraction"] < 1.
        image = CvBridge().imgmsg_to_cv2(captured["depth"])
        x0, y0, x1, y1 = status["bounds"]
        mask = np.zeros((240, 320), dtype=bool); mask[y0:y1, x0:x1] = True
        assert np.isnan(image[~mask]).all()
        assert captured["info"].width == 320 and captured["info"].height == 240
        assert captured["info"].k[2] == node.processor.p1[0, 2]
        assert captured["depth"].header.stamp == captured["info"].header.stamp == stamp
        # 明确失效消息立即退回全幅，不保持旧裁剪范围。
        msg.header.stamp.sec = 11; msg.polygon.points = []
        node._onRoiDemand(msg)
        node.onDecodedPair(left, right, Time(sec=11))
        assert json.loads(captured["roi"].data)["roi_area_fraction"] == 1.
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()
