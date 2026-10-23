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


def ready(**kwargs):
    g=ImuGate(**kwargs)
    for i in range(5): g.onSync(sync(100_000_000_000+i), 100., 1000.)
    return g


def test_unsynchronized_and_axis_timestamp_contract():
    g=ImuGate()
    assert g.accept(imu(), nowMono=100., nowRos=1000., connected=True) is None
    g=ready()
    stamp, angular, accel=g.accept(imu(), nowMono=100., nowRos=1000., connected=True, cameraOffsetS=.01)
    assert stamp == pytest.approx(1000.01)
    assert angular == (1., -2., -3.) and accel == (4., -5., 9.81)
    assert g.accept(imu(), nowMono=100., nowRos=1000., connected=True) is None


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
    assert g.accept(m, nowMono=mono, nowRos=ros, connected=connected) is None


def test_restart_and_bad_sync_reconverge():
    g=ready()
    g.onSync(sync(1_000_000_000), 100., 1000.)
    assert g.restarts==1 and g.count==1
    assert g.accept(imu(), nowMono=100., nowRos=1000., connected=True) is None
    g.onSync(sync(2_000_000_000, rtt=200.), 100., 1000.)
    assert g.count==0


def test_imu_holds_fresh_mapping_across_single_rtt_spike():
    g = ready(hold_last_sync_on_rtt=True)
    assert g.accept(imu(), nowMono=100., nowRos=1000., connected=True)
    oldOffset, oldSync = g.offsetNs, g.lastSync
    bad = sync(100_100_000_000, t=1000.1, rtt=40.)
    bad.estimated_offset_ns += 123_000_000
    assert not g.onSync(bad, 100.1, 1000.1)
    assert (g.offsetNs, g.lastSync) == (oldOffset, oldSync)
    assert g.count == g.converge_samples
    assert g.accept(imu(1000.09), nowMono=100.1, nowRos=1000.1, connected=True)
    assert g.syncRejections == {"timesync_rtt_rejected": 1}
    assert g.last_sync_rejection == "timesync_rtt_rejected"
    # 异常同步不应清掉已发布 IMU 的单调性屏障。
    assert g.accept(imu(1000.08), nowMono=100.1, nowRos=1000.1, connected=True) is None
    assert g.rejectionCounts == {"imu_nonmonotonic": 1}


def test_repeated_rtt_spikes_do_not_extend_mapping_lifetime():
    g = ready(hold_last_sync_on_rtt=True)
    for dt in (.2, .6, .9):
        assert not g.onSync(sync(int((100+dt)*1e9), t=1000+dt, rtt=40.), 100+dt, 1000+dt)
        assert g.lastSync == 100.
    assert g.accept(imu(1001.0), nowMono=101.01, nowRos=1001.01, connected=True) is None
    assert g.reason == "timesync_stale"
    assert g.onSync(sync(101_100_000_000, t=1001.1), 101.1, 1001.1)
    assert g.count == 1
    assert g.accept(imu(1001.09), nowMono=101.1, nowRos=1001.1, connected=True) is None
    assert g.reason == "no_clock_mapping"


def test_rtt_spike_during_initial_convergence_requires_restart():
    g = ImuGate(hold_last_sync_on_rtt=True)
    for i in range(4):
        assert g.onSync(sync(100_000_000_000+i), 100., 1000.)
    assert not g.onSync(sync(100_100_000_000, t=1000.1, rtt=40.), 100.1, 1000.1)
    assert g.count == 0 and g.lastSync is None


@pytest.mark.parametrize("kind", ["restart", "clock_jump", "stale", "offset_unsettled"])
def test_hold_policy_keeps_hard_invalidations(kind):
    g = ready(hold_last_sync_on_rtt=True)
    m = sync(100_100_000_000, t=1000.1, rtt=40.)
    ros = 1000.1
    if kind == "restart": m.remote_timestamp_ns = 1_000_000_000
    if kind == "clock_jump": ros = 1000.3
    if kind == "stale": m = sync(100_100_000_000, t=998.)
    if kind == "offset_unsettled":
        m.round_trip_time_ms = 2.
        m.observed_offset_ns += 100_000_000
    assert not g.onSync(m, 100.1, ros)
    assert g.count == 0 and g.lastSync is None
    assert g.accept(imu(ros-.01), nowMono=100.1, nowRos=ros, connected=True) is None


def test_fresh_sample_after_sync_gap_must_reconverge_even_without_hold():
    g = ready()
    assert g.onSync(sync(102_000_000_000, t=1002.), 102., 1002.)
    assert g.count == 1
    assert g.accept(imu(1001.99), nowMono=102., nowRos=1002., connected=True) is None


def test_future_wait_timer_runs_only_with_pending_samples(monkeypatch):
    from collections import deque
    from boom_birds_sensing import mavros_imu_node as module
    now = [100.]
    monkeypatch.setattr(module.time, "monotonic", lambda: now[0])
    class Timer:
        canceled=True
        def isCanceled(self): return self.canceled
        def reset(self): self.canceled=False
        def cancel(self): self.canceled=True
    published=[]
    node=NS(_pendingImu=deque(),_pendingTimer=Timer(),_futureBuffered=0,
            get_parameter=lambda _:NS(value=.02),
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=round((now[0]+900)*1e9))),
            _imu=published.append)
    node._drainPendingImu=lambda:module.MavrosImuNode._drainPendingImu(node)
    message=imu(1000.005)
    module.MavrosImuNode._bufferImu(node,message)
    assert not published and not node._pendingTimer.isCanceled()
    now[0]+=.006
    node._drainPendingImu()
    assert published==[message] and not node._pendingImu
    assert node._pendingTimer.isCanceled()
