from types import SimpleNamespace
import pytest
from rclpy.time import Time
from nav_msgs.msg import Path
from boom_birds_sim.sitl_truth_source import SitlTruthSource


def truth(**changes):
    state = dict(connected=True, restart_epoch=0, position_ned_m=(1., 2., -1.5),
                 velocity_ned_m_s=(0., 0., 0.), yaw_rad=0., roll_rad=0., pitch_rad=0.,
                 yaw_rate_rad_s=0., position_age_s=.12, attitude_age_s=.1,
                 position_received_mono_s=99.88, attitude_received_mono_s=99.9)
    state.update(changes)
    received = []
    pub = SimpleNamespace(publish=received.append)
    node = SimpleNamespace(first_epoch=0, _last_source_times=None, world_origin=(0., 0., .1),
        backend=SimpleNamespace(read_vehicle_state=lambda: SimpleNamespace(**state)),
        get_parameter=lambda name: SimpleNamespace(value=.25),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=100.)),
        get_logger=lambda: SimpleNamespace(warning=lambda *a, **k: None, error=lambda *a, **k: None),
        pub_odom=pub, pub_body=pub, pub_path=pub, pub_camera=pub, pub_ego=pub, pub_imu=pub,
        path=Path(), _path_tick=0)
    return node, received


def test_truth_pose_does_not_refresh_px4_measurement_age():
    node, received = truth()
    SitlTruthSource.tick(node)
    assert len(received) == 5
    stamp = received[0].header.stamp
    assert stamp.sec + stamp.nanosec / 1e9 == pytest.approx(99.88)
    assert received[0].pose.pose.position.z == pytest.approx(1.6)


@pytest.mark.parametrize("changes", [{"connected": False}, {"position_age_s": .3},
                                    {"attitude_age_s": .3}, {"restart_epoch": 1}])
def test_truth_does_not_publish_stale_or_previous_epoch_state(changes):
    node, received = truth(**changes)
    SitlTruthSource.tick(node)
    assert received == []


def test_truth_waits_for_both_sources_before_publishing_another_pose():
    node, received = truth()
    state = node.backend.read_vehicle_state()
    node.backend.read_vehicle_state = lambda: state
    SitlTruthSource.tick(node)
    state.attitude_received_mono_s = 99.95
    state.yaw_rad = .2
    SitlTruthSource.tick(node)
    assert len(received) == 5
    state.position_received_mono_s = 99.96
    state.attitude_received_mono_s = 99.98
    state.position_age_s, state.attitude_age_s = .04, .02
    SitlTruthSource.tick(node)
    assert len(received) == 10
    stamp = received[5].header.stamp
    assert stamp.sec + stamp.nanosec / 1e9 == pytest.approx(99.96)


@pytest.mark.parametrize("field", ["position_received_mono_s", "attitude_received_mono_s"])
def test_truth_refuses_unobserved_source_sample_times(field):
    node, received = truth(**{field: None})
    SitlTruthSource.tick(node)
    assert received == []
