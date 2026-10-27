import struct
from types import SimpleNamespace as NS
import pytest
from boom_birds_control.mavros_ekf_reference import MavrosEkfReference
from boom_birds_control.landing_ekf_reference import LandingEkfReference, LandingReferenceSelector, VisualReference


def wire(mid, payload):
    data = payload+bytes((-len(payload)) % 8)
    return NS(msgid=mid, sysid=1, compid=1, framing_status=1, len=len(payload),
        header=NS(stamp=NS(sec=1000, nanosec=0)),
        payload64=struct.unpack("<"+"Q"*(len(data)//8), data))


def odometry(t=99990000, counter=0, frame=1, child=1, kind=8):
    data = bytearray(232)
    struct.pack_into("<Q", data, 0, t)
    struct.pack_into("<ff", data, 8, 10., 20.)
    struct.pack_into("<ff", data, 36, .1, -.2)
    struct.pack_into("<f", data, 60, .0025)
    struct.pack_into("<f", data, 84, .0036)
    data[228:232] = bytes([frame, child, counter, kind])
    return wire(331, bytes(data))


def status(t=99990000, flags=11, accuracy=.08, vertical_accuracy=0.):
    return wire(230, struct.pack("<Q8fH", t, 0., 0., 0., 0., 0., 0., accuracy, vertical_accuracy, flags))


def reader():
    r = MavrosEkfReference.__new__(MavrosEkfReference)
    r.maxAge, r.maxSkew = .2, .15
    r.samples, r.lastBoot, r.epoch, r.reason = {}, {}, None, ""
    state = NS(restart_epoch=0, frame_reset_epoch=0, connected=True)
    r.node = NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=1000_000_000_000)))
    r.backend = NS(_endpointVerified=True, target_system=1, target_component=1,
        _sihReceiptTime=False, _clock=lambda:100., readVehicleState=lambda:state,
        _timeGate=NS(lastSync=99.9, count=5, converge_samples=5,
                     sync_timeout_s=1., offsetNs=900_000_000_000))
    return r, state


def test_pairs_real_wire_odometry_reset_with_horizontal_estimator_status():
    r, _ = reader()
    r.onRaw(odometry(counter=4))
    r.onRaw(status())
    result, velocity = r.snapshot()
    assert result.stamp == pytest.approx(99.99)
    assert result.position_ned_xy == pytest.approx((10., 20.))
    assert velocity == pytest.approx((.1, -.2))
    assert result.position_accuracy_m == pytest.approx(.08)
    assert result.reset_counter == 4


@pytest.mark.parametrize("fault", ["missing", "invalid_flags", "constant", "unknown_accuracy",
    "wrong_source", "wrong_frame", "wrong_child", "unknown_estimator", "short",
    "timesync", "epoch", "stale", "covariance"])
def test_missing_or_invalid_estimator_evidence_cannot_be_used(fault):
    r, state = reader()
    o, s = odometry(), status()
    if fault == "missing": o = None
    elif fault == "invalid_flags": s = status(flags=3)
    elif fault == "constant": s = status(flags=139)
    elif fault == "unknown_accuracy": s = status(accuracy=0.)
    elif fault == "wrong_source": o.sysid = 2
    elif fault == "wrong_frame": o = odometry(frame=20)
    elif fault == "wrong_child": o = odometry(child=12)
    elif fault == "unknown_estimator": o = odometry(kind=0)
    elif fault == "short": o.len = 230
    elif fault == "timesync": r.backend._timeGate.count = 0
    elif fault == "covariance":
        raw = bytearray(232); struct.pack_into("<Q", raw, 0, 99990000)
        struct.pack_into("<f", raw, 60, float("nan")); raw[228:232] = bytes([1,1,0,8])
        o = wire(331, bytes(raw))
    if o is not None: r.onRaw(o)
    r.onRaw(s)
    if fault == "epoch": state.frame_reset_epoch = 1
    if fault == "stale": r.backend._clock = lambda:100.3
    assert r.snapshot() is None


def test_reset_counter_from_fcu_revokes_anchored_reference():
    r, _ = reader()
    r.onRaw(odometry(counter=4)); r.onRaw(status())
    sample, _ = r.snapshot()
    selector = LandingReferenceSelector(LandingEkfReference(max_age_s=.2, max_pair_skew_s=.1,
        max_accuracy_m=.2, max_duration_s=10., max_displacement_m=1., max_speed_m_s=2.))
    assert selector.step(100., VisualReference(99.99, (.1, 0.)), sample) == (.1, 0.)
    r.onRaw(odometry(t=99995000, counter=5))
    sample, _ = r.snapshot()
    assert selector.step(100., None, sample) is None
    assert selector.reason == "estimator_reset"


def test_replayed_wire_timestamp_does_not_refresh_position():
    r, _ = reader()
    o = odometry(); r.onRaw(o); r.onRaw(status())
    r.backend._clock = lambda:100.3
    r.onRaw(o)
    assert r.snapshot() is None


def test_ros_mavlink_subscription_receives_paired_ekf_observations():
    import time
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import SingleThreadedExecutor
    from mavros_msgs.msg import Mavlink
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node("test_ekf_reference_rx", context=context)
    state = NS(restart_epoch=0, frame_reset_epoch=0, connected=True)
    backend = NS(_endpointVerified=True, target_system=117, target_component=1,
        _sihReceiptTime=True, _clock=time.monotonic, readVehicleState=lambda:state)
    reader = MavrosEkfReference(node, backend)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    publisher = node.create_publisher(Mavlink, "/uas117/mavlink_source", qos_profile_sensor_data)
    def make(source):
        msg = Mavlink()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.msgid, msg.sysid, msg.compid = source.msgid, 117, 1
        msg.framing_status, msg.len = 1, source.len
        msg.payload64 = list(source.payload64)
        return msg
    try:
        deadline = time.monotonic()+3.
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert publisher.get_subscription_count() > 0
        publisher.publish(make(odometry()))
        publisher.publish(make(status()))
        while time.monotonic() < deadline and reader.snapshot() is None:
            executor.spin_once(timeout_sec=.05)
        result = reader.snapshot()
        assert result is not None
        assert result[0].position_ned_xy == pytest.approx((10.,20.))
    finally:
        reader.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)


def test_vertical_reference_requires_valid_flags_and_positive_accuracy():
    r, _ = reader()
    r.onRaw(odometry())
    r.onRaw(status(flags=47, vertical_accuracy=.12))
    sample, _ = r.snapshot()
    assert sample.vertical_position_valid
    assert sample.position_ned_z == 0.
    assert sample.velocity_ned_z == 0.
    assert sample.vertical_accuracy_m == pytest.approx(.12)
    r, _ = reader()
    r.onRaw(odometry())
    r.onRaw(status(flags=11))
    sample, _ = r.snapshot()
    assert not sample.vertical_position_valid
    assert sample.position_ned_z is None
