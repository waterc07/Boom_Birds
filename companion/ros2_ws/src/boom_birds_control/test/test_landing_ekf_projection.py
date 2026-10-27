from dataclasses import replace
import math
import pytest
from boom_birds_control.landing_ekf_projection import EkfLandingProjection, FeatureWorldSample
from boom_birds_control.landing_ekf_reference import EkfReferenceSample, LandingEkfReference


def projection():
    return EkfLandingProjection(LandingEkfReference(max_age_s=.2, max_pair_skew_s=.05,
        max_accuracy_m=.2, max_duration_s=12., max_displacement_m=1., max_speed_m_s=2.))


def ekf(t=1., p=(10.,20.), reset=0):
    return EkfReferenceSample(t, p, True, False, .04, 1, reset)


@pytest.mark.parametrize("yaw, move_ned, expected_world", [
    (0., (.02,.03), (.03,.02)),
    (math.pi/2, (.02,.03), (.02,-.03)),
    (-math.pi/2, (.02,.03), (-.02,.03)),
])
def test_feature_to_ekf_uses_rotation_and_does_not_require_common_origins(yaw, move_ned, expected_world):
    p = projection()
    feature = FeatureWorldSample(1.,1.,(3.2,-1.1),(3.,-1.),yaw)
    first = p.step(1., feature, ekf(), (.1,.2))
    assert first.target_world_xy == pytest.approx((3.2,-1.1))
    result = p.step(1.1, None, ekf(1.1,(10.+move_ned[0],20.+move_ned[1])), (.1,.2))
    assert result.source == "ekf"
    assert result.position_world_xy == pytest.approx((3.+expected_world[0],-1.+expected_world[1]))
    assert result.target_world_xy == pytest.approx((3.2,-1.1))
    # Reappearing visual data cannot move the frozen target.
    returned = replace(feature, stamp=1.2, vehicle_stamp=1.2, target_world_xy=(30.,30.))
    result = p.step(1.2, returned, ekf(1.2,(10.+move_ned[0],20.+move_ned[1])), (.1,.2))
    assert result.source == "ekf" and result.target_world_xy == pytest.approx((3.2,-1.1))


def test_ekf_reset_revokes_projection():
    p = projection()
    feature = FeatureWorldSample(1.,1.,(0.,0.),(0.,0.),0.)
    assert p.step(1., feature, ekf(), (0.,0.))
    assert p.step(1.1, None, ekf(1.1,reset=1), (0.,0.)) is None
    assert p.reason == "estimator_reset"
    assert p.step(1.2, replace(feature,stamp=1.2,vehicle_stamp=1.2), ekf(1.2), (0.,0.)) is None


def test_unsynchronized_or_missing_evidence_never_seeds():
    p = projection()
    feature = FeatureWorldSample(1.,.9,(0.,0.),(0.,0.),0.)
    assert p.step(1., feature, ekf(), (0.,0.)) is None
    assert p.reason == "feature_pose_pair_invalid"
    p.reset()
    assert p.step(1., replace(feature,vehicle_stamp=1.), None, (0.,0.)) is None
    assert p.selector.mode == "failed"


@pytest.mark.parametrize("loss_height", [.441,.145])
def test_offline_attitude_controller_uses_projected_ekf_feedback_until_ground(loss_height):
    from boom_birds_control.attitude_control import AttitudeController
    from boom_birds_control.px4_frames import RosLocalSetpoint
    p = projection()
    controller = AttitudeController()
    controller.offset = 0.
    x, y, vx, vy = 0., 0., 0., 0.
    height, now, dt = loss_height, 1., .02
    feature = FeatureWorldSample(now,now,(.04,-.03),(x,y),0.)
    feedback = p.step(now, feature, ekf(now,(float(10.+y),float(20.+x))), (vy,vx))
    assert feedback.source == "visual"
    while height > 0.:
        now += dt
        feedback = p.step(now, None, ekf(now,(float(10.+y),float(20.+x))), (vy,vx))
        assert feedback is not None and feedback.source == "ekf", (now, p.reason)
        command = RosLocalSetpoint((*feedback.target_world_xy,height), (0.,0.,0.),
                                   (0.,0.,0.), 0.)
        output = controller.calculate(command, (*feedback.position_world_xy,height),
                                      (*feedback.velocity_world_xy,0.), (0.,0.,0.,1.))
        ax, ay = output.acceleration_world[:2]
        vx += ax*dt
        vy += ay*dt
        x += vx*dt
        y += vy*dt
        height = max(0., height-.08*dt)
    assert math.dist((x,y),(.04,-.03)) < .03


def test_delayed_feature_pair_is_propagated_to_latest_ekf_time_without_switching():
    p = projection()
    feature = FeatureWorldSample(1.,1.,(3.2,-1.1),(3.,-1.),0.)
    result = p.step(1.1, feature, ekf(1.1,(10.02,20.03)), (.1,.2),
                    anchor_ekf=ekf(1.,(10.,20.)))
    assert result.source == "visual"
    assert result.position_world_xy == pytest.approx((3.03,-.98))
    assert result.target_world_xy == pytest.approx((3.2,-1.1))
    result = p.step(1.12, None, ekf(1.12,(10.02,20.03)), (.1,.2))
    assert result.source == "ekf" and result.target_world_xy == pytest.approx((3.2,-1.1))


def test_reset_between_feature_pair_and_latest_sample_is_rejected():
    p = projection()
    feature = FeatureWorldSample(1.,1.,(0.,0.),(0.,0.),0.)
    assert p.step(1.1, feature, ekf(1.1,reset=1), (0.,0.), anchor_ekf=ekf()) is None
    assert p.reason == "estimator_reset"
