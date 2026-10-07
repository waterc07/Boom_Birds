from dataclasses import replace, asdict
from pathlib import Path
import copy
import math
import cv2
import numpy as np
import pytest
import yaml
from boom_birds_sensing.platform_observation import BoardDetector, BoardObservation
from boom_birds_control.platform_landing import (
    PlatformLanding, RangeSample, FlightSample, HandoffFeedback, ned_from_flu,
    VELOCITY_YAW_RATE_MASK)

PROFILE = Path(__file__).parents[1] / "config/platform_landing_test.yaml"


def profile():
    return yaml.safe_load(PROFILE.read_text())


def inputs(t, height=1., xy=(0.,0.), yaw=0.):
    pose = np.eye(4)
    pose[:3,3] = [*xy, -height]
    o = BoardObservation(t, "origin_board", True, "ok", tuple(map(tuple, pose)), (7,), .1, 5., 100.)
    return o, RangeSample(t, height-.02), FlightSample(t, 0., 0., yaw, (0.,0.,0.), True, True, True)


def advance(core, t, height=1., xy=(0.,0.), fault=None, **kwargs):
    o, r, f = inputs(t, height, xy)
    if fault: o, r, f = fault(o, r, f)
    feedback = None
    if core.last_sent_sequence:
        seq = core.last_sent_sequence
        sent = core.sent_history[seq]
        feedback = HandoffFeedback(t, core.token, seq, f.epoch, True, False,
                                   sent[1], "TEST_ONLY_FAKE_PX4")
    out = core.step(t, o, r, f, feedback, navigation_revoked=True,navigation_ready=True, **kwargs)
    if out.velocity_ned is not None: core.note_sent(out, t, True)
    return out


def active(intent="land"):
    core = PlatformLanding(profile(), test_only=True)
    core.request(intent)
    for i in range(25): out = advance(core, i*.02)
    assert core.confirmed
    return core


def test_hardware_and_synthetic_gates():
    p = profile()
    with pytest.raises(ValueError, match="synthetic"): PlatformLanding(p)
    p["synthetic"] = False
    with pytest.raises(ValueError, match="evidence"): PlatformLanding(p)
    del p["camera"]["T_body_camera"]
    with pytest.raises(KeyError): PlatformLanding(p, test_only=True)


@pytest.mark.parametrize("mutation", [
    lambda p: p["guidance"].update(max_down_m_s=-1),
    lambda p: p["guidance"].update(acquire_samples=True),
    lambda p: p["guidance"].update(flare_height_m=.01),
    lambda p: p["camera"].update(T_body_camera=np.zeros((4,4)).tolist()),
    lambda p: p["board"]["tags"].append(copy.deepcopy(p["board"]["tags"][0])),
    lambda p: p["board"]["tags"][0].update(size_m=0),
    lambda p: p["board"].update(family="wrong"),
    lambda p: p["quality"].update(min_tags=2),
    lambda p: p["range"].update(T_body_sensor=np.eye(4).tolist()),
])
def test_config_rejects(mutation):
    p = profile(); mutation(p)
    with pytest.raises((ValueError, KeyError)): PlatformLanding(p, test_only=True)


def test_pnp_extrinsics_layout_landing_and_nonzero_yaw():
    p = profile()
    p["board"]["tags"][0]["T_platform_tag"][0][3] = .2
    detector = BoardDetector(p, test_only=True)
    rvec = np.array([math.pi-.12, .07, .02])
    tvec = np.array([.11, -.15, 1.6])
    points = detector.tag_corners(p["board"]["tags"][0])
    pixels = cv2.projectPoints(points, rvec, tvec, detector.K, detector.D)[0].reshape(4,2)
    o = detector.estimate([(7,pixels)], 12.)
    assert o.valid, o.reason
    expected = np.eye(4); expected[:3,:3] = cv2.Rodrigues(rvec)[0]; expected[:3,3]=tvec
    assert np.allclose(o.pose(), detector.T_B_C@expected, atol=1e-5)
    assert not detector.estimate([(8,pixels)], 12.).valid
    assert not detector.estimate([(7,pixels),(7,pixels)], 12.).valid
    assert np.allclose(ned_from_flu(0.,0.,math.pi/2)@[1,0,0], [0,1,0], atol=1e-8)


def test_actual_image_detection():
    p = profile(); detector = BoardDetector(p, test_only=True)
    image = np.full((480,640),255,np.uint8)
    marker = cv2.aruco.drawMarker(detector.dictionary,7,120)
    image[180:300,260:380] = marker
    o = detector.observe(image,10.)
    assert o.valid, o.reason
    assert o.tag_ids == (7,)
    assert o.pose()[2,3] < 0
    image[180:300,260:380] = cv2.aruco.drawMarker(detector.dictionary,8,120)
    assert not detector.observe(image,10.).valid


def test_no_mode_only_confirmation_no_release_and_timeout():
    core = PlatformLanding(profile(), test_only=True); core.request()
    for i in range(120):
        t=i*.02; o,r,f=inputs(t)
        out=core.step(t,o,r,f,navigation_revoked=True,navigation_ready=True)
        if out.velocity_ned is not None:
            assert out.velocity_ned[2] == 0 and out.vio_required and not out.release_compute
            core.note_sent(out,t,True)
    assert core.state == "NATIVE_LAND" and not core.confirmed


@pytest.mark.parametrize("change", [
    {"token":"wrong"}, {"attitude_active":True}, {"velocity_active":False},
    {"source":"local_send"}, {"epoch":1}, {"echoed_velocity_ned":(0.,0.,.2)},
    {"sequence":999}, {"stamp":-1.},
])
def test_confirmation_requires_correlated_px4_feedback(change):
    core=PlatformLanding(profile(),test_only=True); core.request()
    for i in range(15):
        t=i*.02; o,r,f=inputs(t)
        ack=None
        if core.last_sent_sequence:
            ack=HandoffFeedback(t,core.token,core.last_sent_sequence,0,True,False,
                                (0.,0.,0.),"TEST_ONLY_FAKE_PX4")
            ack=replace(ack,**change)
        out=core.step(t,o,r,f,ack,navigation_revoked=True,navigation_ready=True)
        if out.velocity_ned is not None: core.note_sent(out,t,True)
    assert not core.confirmed and not core.released


@pytest.mark.parametrize("fault,reason", [
    (lambda o,r,f:(replace(o,valid=False),r,f),"board_not_visible"),
    (lambda o,r,f:(replace(o,board="wrong"),r,f),"board_not_visible"),
    (lambda o,r,f:(replace(o,stamp=o.stamp-1),r,f),"observation_stale_or_reordered"),
    (lambda o,r,f:(replace(o,stamp=o.stamp+1),r,f),"observation_stale_or_reordered"),
    (lambda o,r,f:(replace(o,T_body_platform=((1,0,0,2),(0,1,0,0),(0,0,1,-1),(0,0,0,1))),r,f),"pose_jump"),
    (lambda o,r,f:(o,replace(r,distance_m=2.),f),"range_jump"),
    (lambda o,r,f:(o,replace(r,distance_m=.01),f),"range_blind_or_invalid"),
    (lambda o,r,f:(o,replace(r,distance_m=float("nan")),f),"range_blind_or_invalid"),
    (lambda o,r,f:(o,replace(r,stamp=r.stamp-1),f),"range_stale_or_reordered"),
    (lambda o,r,f:(o,r,replace(f,velocity_estimator_valid=False)),"px4_velocity_estimator_unavailable"),
    (lambda o,r,f:(o,r,replace(f,roll=.5)),"tilt_limit"),
    (lambda o,r,f:(o,r,replace(f,stamp=f.stamp-1)),"telemetry_stale"),
])
def test_fault_during_descent_immediately_revokes_downward_velocity(fault,reason):
    core=active()
    assert core.last_velocity[2] > 0
    out=advance(core,.5,fault=fault)
    assert out.reason==reason
    assert not out.descent_permitted
    assert out.velocity_ned is None or out.velocity_ned[2] <= 0
    assert not out.request_disarm


def test_acquisition_requires_distinct_samples_and_navigation_revocation():
    core=PlatformLanding(profile(),test_only=True);core.request()
    o,r,f=inputs(0.)
    for i in range(6):
        out=core.step(i*.01,o,r,replace(f,stamp=i*.01),navigation_revoked=True,navigation_ready=True)
    assert core.state=="ACQUIRE" and core.good==1
    for i in range(6,20):
        o,r,f=inputs(i*.01)
        out=core.step(i*.01,o,r,f,navigation_revoked=False,navigation_ready=True)
    assert core.state=="ACQUIRE" and out.navigation_allowed and out.vio_required


def test_descent_requires_alignment_and_rate_bounds():
    core=active()
    last=core.last_velocity.copy()
    for i in range(25,45):
        out=advance(core,i*.02,xy=(.2,.0))
        assert out.velocity_ned[2] <= 0 and not out.descent_permitted
        assert np.linalg.norm(out.velocity_ned[:2]) <= core.cfg.max_xy_m_s+1e-10
        # Vertical safety stops bypass downward slew intentionally.
        assert np.linalg.norm(np.array(out.velocity_ned[:2])-last[:2]) <= core.cfg.max_accel_m_s2*.02+1e-10
        last=np.array(out.velocity_ned)
    for i in range(45,75):
        out=advance(core,i*.02)
        assert out.velocity_ned[2] <= core.cfg.max_down_m_s
    assert out.descent_permitted


def test_release_after_confirmation_and_no_vio_reacquisition_on_loss():
    core=active();out=advance(core,.5,release_ack=True)
    assert core.released and not out.vio_required and not out.release_compute
    for i in range(26,60):
        out=advance(core,i*.02,fault=lambda o,r,f:(replace(o,valid=False),r,f))
    assert out.request_native_land and not out.navigation_allowed and not out.vio_required


def test_landed_confirmation_and_disarm_window():
    core=active()
    for i in range(25,125):
        height=max(.08,1.-(i-25)*.02)
        out=advance(core,i*.02,height=height,
                    fault=lambda o,r,f:(o,r,replace(f,landed=height<=.08)))
    assert out.request_disarm
    o,r,f=inputs(2.5,.08)
    ack=HandoffFeedback(2.5,core.token,core.last_sent_sequence,0,True,False,
                        core.sent_history[core.last_sent_sequence][1],"TEST_ONLY_FAKE_PX4")
    out=core.step(2.5,o,r,replace(f,landed=True,armed=False),ack,navigation_revoked=True,navigation_ready=True)
    assert out.state=="COMPLETE" and out.velocity_ned is None


def test_unknown_landed_never_disarms_and_blind_range_falls_back():
    core=active()
    for i in range(25,100):
        out=advance(core,i*.02,height=max(.08,1.-(i-25)*.02))
        assert not out.request_disarm
    for i in range(100,135):
        out=advance(core,i*.02,height=.03)
        assert not out.request_disarm
    assert out.request_native_land


def test_restart_clock_reset_mode_loss_and_send_failure():
    for fault in ("epoch","clock","mode","send"):
        core=active()
        if fault=="epoch": out=advance(core,.5,fault=lambda o,r,f:(o,r,replace(f,epoch=1)))
        elif fault=="clock": out=advance(core,.1)
        elif fault=="mode": out=advance(core,.5,fault=lambda o,r,f:(o,r,replace(f,offboard=False)))
        else:
            out=advance(core,.5);core.note_sent(out,.5,False);out=core.step(.52)
        assert out.request_native_land


def test_takeoff_retains_vio_and_configuration_changes_height():
    p=profile();p["guidance"]["takeoff_height_m"]=2.
    core=PlatformLanding(p,test_only=True);core.request("takeoff")
    for i in range(30): out=advance(core,i*.02)
    assert out.velocity_ned[2]<0 and out.vio_required and not out.release_compute
    for i in range(30,110):
        out=advance(core,i*.02,height=min(2.,1.+(i-30)*.02))
    assert out.state=="TAKEOFF_HOLD" and out.vio_required
    assert VELOCITY_YAW_RATE_MASK & (1|2|4|64|128|256|1024)==1479

def test_navigation_dependency_required_until_confirmation():
    core=PlatformLanding(profile(),test_only=True);core.request()
    o,r,flight=inputs(0.)
    out=core.step(0.,o,r,flight,navigation_revoked=True)
    assert out.reason=="navigation_dependency_unavailable" and out.velocity_ned is None
    for i in range(1,7):
        o,r,flight=inputs(i*.02)
        out=core.step(i*.02,o,r,flight,navigation_revoked=True,navigation_ready=True)
        if out.velocity_ned is not None:core.note_sent(out,i*.02,True)
    assert core.state=="PREPARE"
    o,r,flight=inputs(.14)
    out=core.step(.14,o,r,flight,navigation_revoked=True,navigation_ready=False)
    assert out.request_native_land and out.vio_required and not out.release_compute


def test_landed_confirmation_window_resets_on_observation_loss():
    core=active()
    for i in range(25,72):
        advance(core,i*.02,height=max(.08,1.-(i-25)*.02),
                fault=lambda o,r,f:(o,r,replace(f,landed=True)))
    advance(core,1.44,height=.08,fault=lambda o,r,f:(replace(o,valid=False),r,replace(f,landed=True)))
    out=advance(core,1.46,height=.08,fault=lambda o,r,f:(o,r,replace(f,landed=True)))
    assert not out.request_disarm


def test_repeated_valid_confirmation_does_not_reset_consecutive_ack_count():
    core=PlatformLanding(profile(),test_only=True);core.request()
    for i in range(30):
        t=i*.02;o,r,f=inputs(t)
        if i%4==0 and core.last_sent_sequence:
            ack=HandoffFeedback(t,core.token,core.last_sent_sequence,0,True,False,
                               (0.,0.,0.),"TEST_ONLY_FAKE_PX4")
        elif i<8:ack=None
        out=core.step(t,o,r,f,ack,navigation_revoked=True,navigation_ready=True)
        if out.velocity_ned is not None:core.note_sent(out,t,True)
    assert core.confirmed


def test_wrong_id_and_missing_quality_cannot_authorize_descent():
    for modification in ({"tag_ids":(8,)},{"min_edge_px":0.},{"reprojection_px":float("nan")},
                         {"ambiguity_ratio":1.},{"tag_ids":(7,7)}):
        core=active()
        out=advance(core,.5,fault=lambda o,r,f:(replace(o,**modification),r,f))
        assert not out.descent_permitted and not out.request_disarm
        assert out.velocity_ned is None or out.velocity_ned[2]<=0
