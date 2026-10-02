from types import SimpleNamespace as NS
import pytest
from boom_birds_sensing.mavros_imu import ImuGate


def header(t):
    return NS(stamp=NS(sec=int(t), nanosec=round((t-int(t))*1e9)))


def sync(remote, t=1000., rtt=2.):
    return NS(header=header(t), remote_timestamp_ns=remote,
        round_trip_time_ms=rtt, observed_offset_ns=900_000_000_000, estimated_offset_ns=900_000_000_000)


def imu(t=1000.):
    return NS(header=header(t), angular_velocity=NS(x=1., y=2., z=3.),
        linear_acceleration=NS(x=4., y=5., z=-9.81),
        angular_velocity_covariance=[0.]*9, linear_acceleration_covariance=[0.]*9)


def ready():
    g=ImuGate()
    for i in range(5): g.on_sync(sync(100_000_000_000+i), 100., 1000.)
    return g


def test_unsynchronized_and_axis_timestamp_contract():
    g=ImuGate()
    assert g.accept(imu(), now_mono=100., now_ros=1000., connected=True) is None
    g=ready()
    stamp, angular, accel=g.accept(imu(), now_mono=100., now_ros=1000., connected=True, camera_offset_s=.01)
    assert stamp == pytest.approx(1000.01)
    assert angular == (1., -2., -3.) and accel == (4., -5., 9.81)
    assert g.accept(imu(), now_mono=100., now_ros=1000., connected=True) is None


@pytest.mark.parametrize("kind", ["stale", "future", "unavailable", "nonfinite", "disconnect", "sync_timeout", "clock_jump"])
def test_invalid_observations_rejected(kind):
    g=ready();m=imu();mono=100.;ros=1000.;connected=True
    if kind=="stale": m.header=header(990.)
    if kind=="future":m.header=header(1001.)
    if kind=="unavailable":m.angular_velocity_covariance[0]=-1.
    if kind=="nonfinite":m.angular_velocity.x=float("nan")
    if kind=="disconnect":connected=False
    if kind=="sync_timeout":mono=102.;ros=1002.
    if kind=="clock_jump":ros=1000.1
    assert g.accept(m, now_mono=mono, now_ros=ros, connected=connected) is None


def test_restart_and_bad_sync_reconverge():
    g=ready()
    g.on_sync(sync(1_000_000_000), 100., 1000.)
    assert g.restarts==1 and g.count==1
    assert g.accept(imu(), now_mono=100., now_ros=1000., connected=True) is None
    g.on_sync(sync(2_000_000_000, rtt=200.), 100., 1000.)
    assert g.count==0
