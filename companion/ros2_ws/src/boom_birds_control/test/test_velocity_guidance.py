"""速度导引的方向、限速、停发与 PX4 估计有效性；不连接设备。"""
from types import SimpleNamespace
import math
import numpy as np
import pytest
from boom_birds_control.velocity_guidance import VelocityGuidance, VelocityConfig, VelocitySetpoint, velocityToNed
from boom_birds_control.px4_frames import TypeMask, RosLocalSetpoint, rosLocalToNedSetpoint, LocalFrameAlignment
from boom_birds_control.mavros_backend import MavrosPx4Backend


def guidance(yaw=0.):
    g = VelocityGuidance()
    g.offset = yaw
    return g


def desired(p=(1.,0.,0.),v=(0.,0.,0.),a=(0.,0.,0.),yaw=0.):
    return SimpleNamespace(position_m=p,velocity_m_s=v,acceleration_m_s2=a,yaw_rad=yaw)


def test_velocity_converges_with_speed_and_acceleration_bounds():
    g = guidance()
    previous = np.zeros(3)
    for i in range(100):
        result = g.calculate(desired(),(0.,0.,0.),(0.,0.,0.),(0.,0.,0.,1.),now=1.+i*.02)
        current = np.array(result.velocity_m_s)
        assert np.linalg.norm(current) <= .3+1e-9
        assert np.linalg.norm(current-previous) <= .3*.02+1e-9
        previous=current
    assert result.velocity_m_s == pytest.approx((.3,0,0))
    assert result.acceleration_m_s2 == (0,0,0)
    assert not hasattr(result,'thrust')


@pytest.mark.parametrize('axis', range(3))
@pytest.mark.parametrize('sign', (-1,1))
def test_position_error_corrects_towards_target(axis,sign):
    p=[0.,0.,0.];p[axis]=sign*.2
    result=guidance().calculate(desired(p=p),(0,0,0),(0,0,0),(0,0,0,1),now=1.)
    assert result.velocity_m_s[axis]*sign>0


def test_clock_gap_clears_slew_state_instead_of_spending_elapsed_budget():
    g=guidance()
    g.calculate(desired(),(0,0,0),(0,0,0),(0,0,0,1),now=1.)
    with pytest.raises(ValueError,match='gap'):
        g.calculate(desired(),(0,0,0),(0,0,0),(0,0,0,1),now=2.)
    result=g.calculate(desired(),(0,0,0),(0,0,0),(0,0,0,1),now=2.02)
    assert result.velocity_m_s[0] == pytest.approx(.006)
    g.invalidate('reference_reset')
    with pytest.raises(ValueError,match='reference_reset'):
        g.calculate(desired(),(0,0,0),(0,0,0),(0,0,0,1),now=2.04)


def test_velocity_mask_never_enables_position_acceleration_or_force():
    mask=int(TypeMask.forMode('velocity','yaw'))
    assert mask==2503
    assert mask&7==7 and mask&448==448 and not mask&512
    wire=velocityToNed(VelocitySetpoint((.1,0,0),(0,0,0),0.), math.pi/2)
    assert wire.velocity_m_s == pytest.approx((.1,0,0),abs=1e-9)
    assert wire.yaw_rad == pytest.approx(0.)


@pytest.mark.parametrize('offset,expected,yaw', ((0.,(0.,.1,-.2),math.pi/2),
    (math.pi/2,(.1,0.,-.2),0.),(math.pi,(0.,-.1,-.2),-math.pi/2)))
def test_vio_to_px4_enu_reference_reaches_expected_ned_axes(offset,expected,yaw):
    wire=velocityToNed(VelocitySetpoint((.1,0,.2),(0,0,0),0.),offset)
    assert wire.velocity_m_s==pytest.approx(expected,abs=1e-9)
    assert wire.yaw_rad==pytest.approx(yaw,abs=1e-9)


def test_feedforward_disabled_unless_explicitly_enabled():
    d=desired(a=(.1,0,0))
    g=guidance();assert g.calculate(d,(0,0,0),(0,0,0),(0,0,0,1),now=1.).acceleration_m_s2==(0,0,0)
    g=VelocityGuidance(VelocityConfig(acceleration_feedforward=True));g.offset=0.
    assert g.calculate(d,(0,0,0),(0,0,0),(0,0,0,1),now=1.).acceleration_m_s2==(.1,0,0)


@pytest.mark.parametrize('failure', ('missing','stale','flags','ratio','epoch','position_age','nan_velocity'))
def test_px4_velocity_requires_fresh_valid_estimator_and_actual_velocity(failure):
    b=object.__new__(MavrosPx4Backend)
    state=SimpleNamespace(connected=True,restart_epoch=0,frame_reset_epoch=0,
        position_age_s=.05,velocity_ned_m_s=(0,0,0))
    b.readVehicleState=lambda:state;b._clock=lambda:10.
    b._velocityEstimate=(9.95,7,.1,(0,0))
    assert b.velocityControlReady()==(True,'ready')
    if failure=='missing':b._velocityEstimate=None
    if failure=='stale':b._velocityEstimate=(9.,7,.1,(0,0))
    if failure=='flags':b._velocityEstimate=(9.95,1,.1,(0,0))
    if failure=='ratio':b._velocityEstimate=(9.95,7,1.1,(0,0))
    if failure=='epoch':state.frame_reset_epoch=1
    if failure=='position_age':state.position_age_s=.3
    if failure=='nan_velocity':state.velocity_ned_m_s=(float('nan'),0,0)
    assert b.velocityControlReady()[0] is False


def test_velocity_backend_suppresses_attitude_and_rejects_position_mask():
    from boom_birds_control.px4_frames import Px4LocalSetpoint
    from boom_birds_control.px4_backend import ManualClock
    clock=ManualClock(10.)
    node=SimpleNamespace(count_publishers=lambda topic: int(topic.endswith('/local')))
    b=MavrosPx4Backend(node,control_mode='px4_velocity',clock=clock)
    b.velocityControlReady=lambda:(True,'ready')
    sp=Px4LocalSetpoint((0,0,0),(.1,.2,-.1),(0,0,0),0.)
    assert b.sendSetpoint(sp,2503)
    assert not b.sendSetpoint(sp,int(TypeMask.forMode('position_velocity')))
    assert not b.sendAttitudeSetpoint(SimpleNamespace(orientation_xyzw=(0,0,0,1),thrust=.5))
    node.count_publishers=lambda topic:1
    assert not b.sendSetpoint(sp,2503)


def test_velocity_node_output_uses_current_health_and_never_calls_attitude(monkeypatch):
    from boom_birds_control.px4_interface_node import Px4InterfaceNode
    import boom_birds_control.px4_interface_node as module
    monkeypatch.setattr(module.time,'monotonic',lambda:10.)
    wire=[]
    backend=SimpleNamespace(sendSetpoint=lambda sp, type_mask:(wire.append((sp,type_mask)) or True))
    node=SimpleNamespace(get_parameter=lambda name:SimpleNamespace(value='fake'),
        _vioFeedback=((0,0,0),(0,0,0),(0,0,0,1),9.95),
        attitude_controller=guidance(math.pi/2),backend=backend,
        _velocityReady=lambda:(True,'ready'))
    assert Px4InterfaceNode._sendVelocity(node,desired())
    assert len(wire)==1 and wire[0][1]==2503
    assert wire[0][0].velocity_m_s==pytest.approx((.006,0,0),abs=1e-9)
    node._velocityReady=lambda:(False,'px4_velocity_estimator_invalid')
    with pytest.raises(ValueError,match='estimator_invalid'):
        Px4InterfaceNode._sendVelocity(node,desired())
    assert len(wire)==1


def test_px4_hover_estimator_requires_readback_and_rechecks_after_restart():
    b=object.__new__(MavrosPx4Backend)
    state=SimpleNamespace(restart_epoch=1)
    b.readVehicleState=lambda:state
    b._hoverEstimatorEnabled=None;b._hoverParamEpoch=1
    b._hoverParamAt=9.95;b._clock=lambda:10.
    assert not b.hoverEstimatorReady()[0]
    b._hoverEstimatorEnabled=True
    assert b.hoverEstimatorReady()[0]
    state.restart_epoch=2
    assert not b.hoverEstimatorReady()[0]


def test_velocity_profile_contains_no_pi_hover_or_thrust_parameters():
    from pathlib import Path
    import yaml
    path=Path(__file__).resolve().parents[1]/'config/velocity_hardware.yaml'
    profile=yaml.safe_load(path.read_text())
    cfg=VelocityConfig(**profile['controller'])
    assert profile['control_mode']=='px4_velocity' and not cfg.acceleration_feedforward
    assert not any('thrust' in key or 'hover' in key for key in profile['controller'])


def test_sih_always_on_hover_evidence_cannot_authorize_real_endpoint():
    b=object.__new__(MavrosPx4Backend)
    b._sihHteAlwaysOn=True;b._sihReceiptTime=True;b._linkIsLoopback=False
    b._hoverEstimatorEnabled=None;b._hoverParamEpoch=0
    b.readVehicleState=lambda:SimpleNamespace(restart_epoch=0)
    assert not b.hoverEstimatorReady()[0]
    b._linkIsLoopback=True
    assert b.hoverEstimatorReady()==(True,'TEST_ONLY_SIH_HTE_ALWAYS_ON')


def test_velocity_ros_controller_constructs_and_ticks_without_attitude_or_thrust_output():
    from pathlib import Path
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    from boom_birds_control.px4_interface_node import Px4InterfaceNode
    context=Context();rclpy.init(context=context)
    config=Path(__file__).resolve().parents[1]/'config/velocity_hardware.yaml'
    params=dict(control_mode='px4_velocity',attitude_config_file=str(config),backend='fake',
                require_session=True,connect_on_start=False)
    node=Px4InterfaceNode(context=context,parameter_overrides=[Parameter(k,value=v) for k,v in params.items()])
    try:
        assert isinstance(node.attitude_controller,VelocityGuidance)
        assert node.touchdown is None
        node._tick()
        assert node.count_publishers('/mavros/setpoint_raw/attitude')==0
        assert not hasattr(node.attitude_controller.config,'hover_thrust')
    finally:
        node.destroy_node();context.shutdown()


def test_guidance_brakes_approaching_target_before_position_overshoot():
    g=VelocityGuidance(VelocityConfig(kp=(2.,)*3,kv=(1.,)*3));g.offset=0.
    result=g.calculate(desired(p=(.05,0,0)),(0,0,0),(.2,0,0),(0,0,0,1),now=1.)
    assert result.velocity_m_s[0] < 0
    assert abs(result.velocity_m_s[0]) <= .006+1e-9


def test_px4_local_requires_active_aiding_and_external_vision_disabled():
    from pymavlink.dialects.v20.common import ESTIMATOR_PRED_POS_HORIZ_REL

    backend = object.__new__(MavrosPx4Backend)
    state = SimpleNamespace(connected=True, restart_epoch=0, frame_reset_epoch=0,
                            position_age_s=.05, velocity_ned_m_s=(0., 0., 0.))
    backend.readVehicleState = lambda: state
    backend._clock = lambda: 10.
    backend._hoverParamEpoch = 0
    backend._externalVisionParamAt = 9.95
    backend._externalVisionControl = 4
    backend._velocityEstimate = (9.95, 15 | ESTIMATOR_PRED_POS_HORIZ_REL, .1, (0, 0))
    assert backend.localEstimatorReady() == (False, "EKF2_EV_CTRL_disabled_not_confirmed")
    backend._externalVisionControl = 0
    assert backend.localEstimatorReady() == (True, "ready")
    backend._velocityEstimate = (9.95, 15, .1, (0, 0))
    assert backend.localEstimatorReady() == (False, "px4_horizontal_aiding_unavailable")


def test_px4_local_velocity_output_does_not_require_vio(monkeypatch):
    from boom_birds_control.px4_interface_node import Px4InterfaceNode
    import boom_birds_control.px4_interface_node as module

    monkeypatch.setattr(module.time, "monotonic", lambda: 10.)
    wire = []
    backend = SimpleNamespace(sendSetpoint=lambda sp, type_mask: (wire.append((sp, type_mask)) or True))
    node = SimpleNamespace(
        px4LocalFeedback=True, get_parameter=lambda name: SimpleNamespace(value="fake"),
        _localFeedback=((0., 0., 0.), (0., 0., 0.), (0., 0., 0., 1.), 9.95),
        attitude_controller=VelocityGuidance(), backend=backend,
        _localFeedbackReady=lambda: (True, "ready"))
    assert Px4InterfaceNode._sendVelocity(node, desired((0., 0., .8)))
    assert wire[0][1] == 2503
    assert wire[0][0].velocity_m_s == pytest.approx((0., 0., -.006))
    assert wire[0][0].yaw_rad == pytest.approx(math.pi/2)


@pytest.mark.parametrize(("reason", "session", "lastArmed", "land", "mode", "epoch", "latched"), (
    ("ros_clock_changed", "", False, False, None, 1, ""),
    ("ros_clock_changed", "active", False, False, None, 1, "flight_controller_restart"),
    ("ros_clock_changed", "", True, False, None, 1, "flight_controller_restart"),
    ("px4_boot_time_regressed", "", False, False, None, 1, "flight_controller_restart"),
    ("ros_clock_changed", "", True, True, "altctl", 1, ""),
    ("ros_clock_changed", "", True, True, "offboard", 1, "flight_controller_restart"),
    ("ros_clock_changed", "active", True, True, "altctl", 1, "flight_controller_restart"),
    ("px4_boot_time_regressed", "", True, True, "altctl", 1, "flight_controller_restart"),
    ("ros_clock_changed", "", True, True, "altctl", 2, "flight_controller_restart"),
))
def test_px4_local_clock_step_recovers_only_before_session(reason, session, lastArmed, land, mode, epoch, latched):
    from boom_birds_control.control_protocol import ControlIngress
    from boom_birds_control.px4_interface_node import Px4InterfaceNode

    ingress = ControlIngress()
    ingress.session = session
    node = SimpleNamespace(px4LocalFeedback=True, airborneLanding=land, _controlEpoch=0,
        _controlLastArmed=lastArmed, _controlLastManual=mode == "altctl",
        _airborneManual=Px4InterfaceNode._airborneManual, ingress=ingress, _cmd=object(),
        _localFeedback=object(), _localPoseStamp=1, _localFrameEpoch=(0, 0),
        attitude_controller=VelocityGuidance(),
        backend=SimpleNamespace(restart_events=[{"epoch": epoch, "reason": reason}]))
    node._clearLocalFeedback = lambda: Px4InterfaceNode._clearLocalFeedback(node)
    Px4InterfaceNode._observeControlEpoch(node, SimpleNamespace(
        restart_epoch=epoch, connected=False, armed=False, mode_name=None))
    assert node._controlEpoch == epoch and node._cmd is None and ingress.session == ""
    assert node._localFeedback is node._localPoseStamp is node._localFrameEpoch is None
    assert node.attitude_controller.latched == latched


@pytest.mark.parametrize(("land", "session", "mode", "latched"), (
    (True, "", "altctl", ""),
    (True, "active", "altctl", "px4_local_frame_reset"),
    (True, "active", "offboard", "px4_local_frame_reset"),
    (True, "", "offboard", "px4_local_frame_reset"),
    (False, "", "altctl", "px4_local_frame_reset"),
))
def test_px4_local_frame_reset_waits_for_new_pose_only_in_manual_land(land, session, mode, latched, monkeypatch):
    from boom_birds_control.px4_interface_node import Px4InterfaceNode
    import boom_birds_control.px4_interface_node as module

    now = 10.
    monkeypatch.setattr(module.time, "monotonic", lambda: now)
    state = SimpleNamespace(restart_epoch=0, frame_reset_epoch=1, connected=True, armed=True,
        mode_name=mode, position_age_s=.02, position_ned_m=(1., 2., -.8), velocity_ned_m_s=(0.,)*3)
    poses = []
    node = SimpleNamespace(airborneLanding=land, ingress=SimpleNamespace(session=session),
        _airborneManual=Px4InterfaceNode._airborneManual, _localObservedEpoch=(0, 0),
        _localFrameEpoch=(0, 0), _localFeedback=object(), _localPoseStamp=1, _localFeedbackNotBefore=-math.inf,
        _liftStarted=None, _liftResetReference=None, attitude_controller=VelocityGuidance(),
        backend=SimpleNamespace(readVehicleState=lambda: state, attitudeAt=lambda *_: (0.,)*3),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(now*1e9))),
        frame_world="world", _localPosePub=SimpleNamespace(publish=poses.append))

    Px4InterfaceNode._updateLocalFeedback(node)
    assert node.attitude_controller.latched == latched
    assert node._localFeedback is None and poses == []
    if not latched:
        # A second reset before feedback has resumed must also move the boundary.
        now = 10.06
        state.frame_reset_epoch = 2
        Px4InterfaceNode._updateLocalFeedback(node)
        now = 10.09
        Px4InterfaceNode._updateLocalFeedback(node)
        assert node._localFeedback is None and poses == []
    now = 10.15
    Px4InterfaceNode._updateLocalFeedback(node)
    if latched:
        assert node._localFeedback is None and poses == []
    else:
        assert node._localFrameEpoch == (0, 2)
        assert len(poses) == 1 and poses[0].pose.pose.position.z == pytest.approx(.8)


def test_vertical_lift_accepts_valid_z_without_horizontal_aiding():
    backend = object.__new__(MavrosPx4Backend)
    backend.readVehicleState = lambda: SimpleNamespace(connected=True, restart_epoch=0, frame_reset_epoch=0,
        position_age_s=.05, velocity_ned_m_s=(0., 0., 0.))
    backend._clock = lambda: 10.
    backend._hoverParamEpoch = 0
    backend._externalVisionParamAt = backend._hoverParamAt = 9.95
    backend._externalVisionControl = 0
    backend._hoverEstimatorEnabled = True
    backend._sihReceiptTime = False
    backend._velocityEstimate = (9.95, 239, float("nan"), (0, 0))
    assert backend.localLiftReady() == (True, "ready")
    assert backend.localEstimatorReady() == (False, "px4_horizontal_aiding_unavailable")
    backend._velocityEstimate = (9.95, 239 & ~4, float("nan"), (0, 0))
    assert backend.localLiftReady() == (False, "px4_vertical_estimator_invalid")


def test_vertical_lift_uses_native_vertical_velocity_ramp_and_has_timeout():
    from boom_birds_control.px4_interface_node import Px4InterfaceNode

    wire = []
    node = SimpleNamespace(_liftGroundZ=0., _liftStarted=1., _liftOutputAt=None,
        _liftUpSpeed=0., _liftOverspeedSince=None, attitude_controller=VelocityGuidance(), backend=SimpleNamespace(
            sendSetpoint=lambda sp, **kwargs: wire.append((sp, kwargs)) or True))
    q = (0., 0., 0., 1.)
    assert Px4InterfaceNode._sendVerticalLift(node, desired((0.,0.,.8)), (0.,0.,0.), (0.,0.,0.), q, False, .5)
    assert wire[-1][0].acceleration_m_s2 == (0.,0.,0.)
    assert Px4InterfaceNode._sendVerticalLift(node, desired((0.,0.,.8)), (0.,0.,0.), (0.,0.,0.), q, True, 2.)
    assert wire[-1][0].velocity_m_s == pytest.approx((0.,0.,-.0125))
    assert wire[-1][0].acceleration_m_s2 == (0.,0.,0.)
    assert wire[-1][1] == {"type_mask": 2335, "verticalLift": True}
    for i in range(1, 40):
        assert Px4InterfaceNode._sendVerticalLift(node, desired((0.,0.,.8)),
            (0.,0.,0.), (0.,0.,0.), q, True, 2. + i*.02)
    assert wire[-1][0].velocity_m_s == pytest.approx((0.,0.,-.18))
    with pytest.raises(ValueError, match="px4_vertical_lift_limit"):
        Px4InterfaceNode._sendVerticalLift(node, desired((0.,0.,.8)), (0.,0.,.22), (0.,0.,0.), q, True, 7.)


def test_vertical_lift_switches_once_after_aiding_is_ready(monkeypatch):
    from boom_birds_control.px4_interface_node import Px4InterfaceNode
    import boom_birds_control.px4_interface_node as module

    monkeypatch.setattr(module.time, "monotonic", lambda: 10.)
    wire = []
    stage = []
    backend = SimpleNamespace(readVehicleState=lambda: SimpleNamespace(armed=True, landed_state=2),
        localEstimatorReady=lambda: (False, "px4_horizontal_aiding_unavailable"),
        sendSetpoint=lambda sp, **kwargs: wire.append(kwargs["type_mask"]) or True)
    node = SimpleNamespace(px4LocalFeedback=True, get_parameter=lambda name: SimpleNamespace(value="fake"),
        _localFeedback=((0.,0.,.12),(0.,0.,0.),(0.,0.,0.,1.),9.95),
        _localFeedbackReady=lambda: (True,"ready"), _liftComplete=False, _liftGroundZ=0., _liftStarted=8.,
        attitude_controller=VelocityGuidance(), backend=backend,
        _sendVerticalLift=lambda *args: stage.append("vertical_lift") or True)
    assert Px4InterfaceNode._sendVelocity(node, desired((0.,0.,.8)))
    assert stage == ["vertical_lift"] and not wire
    backend.localEstimatorReady = lambda: (True, "ready")
    assert Px4InterfaceNode._sendVelocity(node, desired((0.,0.,.8)))
    assert wire == [2503] and node._liftComplete


@pytest.mark.parametrize(("samples", "aborted"), (
    (((1.1,.472), (1.2,.60), (1.5,.49)), False),
    (((1.1,.59), (1.46,.60)), True),
    (((1.1,.81),), True),
))
def test_vertical_lift_brakes_before_aborting_on_overspeed(samples, aborted):
    from boom_birds_control.px4_interface_node import Px4InterfaceNode

    wire = []
    node = SimpleNamespace(_liftGroundZ=0., _liftStarted=1., _liftOutputAt=None,
        _liftUpSpeed=0., _liftOverspeedSince=None, attitude_controller=VelocityGuidance(),
        backend=SimpleNamespace(sendSetpoint=lambda sp, **kwargs: wire.append(sp) or True))
    for i, (now, speed) in enumerate(samples):
        args = (node, desired((0.,0.,.8)), (0.,0.,.06), (0.,0.,speed), (0.,0.,0.,1.), True, now)
        if aborted and i == len(samples)-1:
            with pytest.raises(ValueError, match="px4_vertical_lift_motion_limit"):
                Px4InterfaceNode._sendVerticalLift(*args)
        else:
            assert Px4InterfaceNode._sendVerticalLift(*args)
            assert wire[-1].acceleration_m_s2 == (0.,0.,0.)
            if speed > .55:
                assert wire[-1].velocity_m_s == (0.,0.,0.)


def test_vertical_lift_backend_ignores_xy_velocity_and_rejects_horizontal_acceleration():
    from boom_birds_control.px4_frames import Px4LocalSetpoint, VERTICAL_LIFT_MASK
    from boom_birds_control.px4_backend import ManualClock

    node = SimpleNamespace(count_publishers=lambda topic: int(topic.endswith("/local")))
    backend = MavrosPx4Backend(node, control_mode="px4_velocity", clock=ManualClock(10.))
    backend.localLiftReady = lambda: (True, "ready")
    sp = Px4LocalSetpoint((0.,)*3, (0.,0.,-.12), (0.,)*3, 0.)
    assert backend.sendSetpoint(sp, VERTICAL_LIFT_MASK, verticalLift=True)
    assert backend.last_setpoint["vz"] == -.12
    assert backend.last_setpoint["type_mask"] == 2335
    wrong = Px4LocalSetpoint((0.,)*3, (0.,0.,-.12), (.1,0.,0.), 0.)
    assert not backend.sendSetpoint(wrong, VERTICAL_LIFT_MASK, verticalLift=True)


@pytest.mark.parametrize(("mode", "session", "ready", "reason"), (
    ("altctl", "", False, "px4_local_pose_or_attitude_stale"),
    ("offboard", "", True, "airborne_manual_session_required"),
    ("altctl", "existing", True, "airborne_manual_session_required"),
    ("altctl", "", True, "session_opened"),
))
def test_land_open_session_checks_current_manual_mode_and_reference(mode, session, ready, reason):
    from boom_birds_control.control_protocol import ControlIngress
    from boom_birds_control.px4_interface_node import Px4InterfaceNode
    from boom_birds_interfaces.srv import VehicleAction

    state = SimpleNamespace(connected=True, mode_name=mode, armed=True, landed_state=2,
        landed_age_s=.05, restart_epoch=1)
    ingress = ControlIngress()
    ingress.session = session
    node = SimpleNamespace(airborneLanding=True, px4LocalFeedback=True, ingress=ingress,
        backend=SimpleNamespace(readVehicleState=lambda: state), _airborneManual=Px4InterfaceNode._airborneManual,
        _observeControlEpoch=lambda _: None, _updateLocalFeedback=lambda: None,
        _localFeedbackReady=lambda: (ready, "ready" if ready else "px4_local_pose_or_attitude_stale"),
        core=SimpleNamespace(monitor=SimpleNamespace(clearPlanningRejected=lambda: None)),
        attitude_controller=VelocityGuidance(), control_mode="px4_velocity", touchdown=None)
    result = Px4InterfaceNode._vehicleAction(node,
        VehicleAction.Request(action=VehicleAction.Request.OPEN_SESSION), VehicleAction.Response())
    assert result.reason == reason
    assert result.accepted == (reason == "session_opened")
    if result.accepted:
        assert result.session_id and result.session_id == ingress.session
    else:
        assert ingress.session == session
