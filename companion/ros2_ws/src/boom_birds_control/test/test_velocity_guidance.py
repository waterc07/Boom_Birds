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
