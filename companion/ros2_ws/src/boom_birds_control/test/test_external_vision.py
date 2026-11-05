"""外部视觉输入的机体系、原时间、协方差与实机 dry-run 边界。"""
from types import SimpleNamespace as NS
import math
import pytest
from nav_msgs.msg import Odometry
from boom_birds_control.external_vision import externalVisionOdometry
from boom_birds_control.mavros_backend import MavrosPx4Backend


def odom():
    m=Odometry();m.header.frame_id="global";m.child_frame_id="body"
    m.header.stamp.sec=100;m.pose.pose.orientation.w=1.
    m.pose.pose.position.x=2.;m.twist.twist.linear.y=.2
    return m


def test_external_vision_preserves_body_velocity_and_source_timestamp():
    m=odom();r=externalVisionOdometry(m,100.05)
    assert r.header.frame_id=="odom" and r.child_frame_id=="base_link"
    assert r.header.stamp==m.header.stamp
    assert r.twist.twist.linear.y==.2 and r.pose.pose.position.x==2.
    assert r.twist.covariance[7]==pytest.approx(.04)
    assert m.header.frame_id=="global" and m.twist.covariance[7]==0.


@pytest.mark.parametrize("fault",["stale","future","world","body","quaternion","velocity","angular"])
def test_external_vision_rejects_unusable_inputs(fault):
    m=odom();now=100.05
    if fault=="stale":now=100.21
    if fault=="future":now=99.99
    if fault=="world":m.header.frame_id="map"
    if fault=="body":m.child_frame_id="camera"
    if fault=="quaternion":m.pose.pose.orientation.w=2.
    if fault=="velocity":m.twist.twist.linear.y=math.nan
    if fault=="angular":m.twist.twist.angular.x=math.inf
    with pytest.raises(ValueError):externalVisionOdometry(m,now)


def test_external_vision_transport_requires_velocity_only_fusion_and_never_transmits_in_dry_run():
    sent=[];b=MavrosPx4Backend(NS(count_publishers=lambda _:1),control_mode="px4_velocity")
    b._visionPub=NS(publish=sent.append,get_subscription_count=lambda:1);b._endpointVerified=True
    b._hoverParamEpoch=0;b._externalVisionControl=4
    b._externalVisionParamAt=b._clock()
    b.readVehicleState=lambda:NS(restart_epoch=0)
    assert not b.sendExternalVision(odom()) and not sent
    b.dry_run=False
    b._externalVisionControl=15
    assert not b.sendExternalVision(odom()) and not sent
    b._externalVisionControl=4
    assert b.sendExternalVision(odom()) and len(sent)==1
    b.readVehicleState=lambda:NS(restart_epoch=1)
    assert not b.sendExternalVision(odom()) and len(sent)==1


def test_node_suppresses_external_vision_even_when_ready_in_dry_run():
    from boom_birds_control.px4_interface_node import Px4InterfaceNode
    sent=[]
    n=NS(attitude_controller=NS(ready=True,latched="",config=NS(max_velocity=2.,reset_position_margin=.2)),frame_world="global",
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_050_000_000)),
        get_parameter=lambda _:NS(value=True),_externalVisionPrepared=0,_externalVisionLastStamp=None,_externalVisionLastPosition=None,
        _hardwareActionsAllowed=lambda:True,backend=NS(sendExternalVision=sent.append))
    Px4InterfaceNode._publishExternalVision(n,odom())
    assert n._externalVisionReason=="dry_run_suppressed" and n._externalVisionPrepared==1 and not sent


def test_parameter_cache_refresh_retains_bounded_confirmation_but_explicit_disable_stops():
    from boom_birds_control.px4_backend import ManualClock
    clock=ManualClock(10.)
    b=MavrosPx4Backend(NS(),clock=clock,control_mode="px4_velocity")
    b.readVehicleState=lambda:NS(restart_epoch=0)
    b.isConnected=lambda:True
    reply=[NS(type=2,integer_value=1),NS(type=2,integer_value=4)]
    class Future:
        def result(self):return NS(values=reply)
        def add_done_callback(self,cb):cb(self)
    b._hoverPullClient=NS(service_is_ready=lambda:False)
    b._hoverParamClient=NS(service_is_ready=lambda:True,call_async=lambda _:Future())
    b._paramGetType=NS(Request=NS)
    b._checkHoverEstimator()
    assert b.hoverEstimatorReady()[0] and b.externalVisionReady()[0]
    clock.advance(2.1);reply[:]=[NS(type=0),NS(type=0)]
    b._checkHoverEstimator()
    assert b.hoverEstimatorReady()[0] and b.externalVisionReady()[0]
    clock.advance(8.1)
    assert not b.hoverEstimatorReady()[0] and not b.externalVisionReady()[0]
    reply[:]=[NS(type=2,integer_value=0),NS(type=2,integer_value=15)]
    b._checkHoverEstimator()
    assert not b.hoverEstimatorReady()[0] and not b.externalVisionReady()[0]


def test_velocity_origin_reset_keeps_yaw_but_restart_still_latches():
    from collections import deque
    from boom_birds_control.px4_interface_node import Px4InterfaceNode
    from boom_birds_control.velocity_guidance import VelocityGuidance
    g=VelocityGuidance();g.offset=0.
    state=NS(restart_epoch=0,frame_reset_epoch=1,armed=True,landed_state=2)
    n=NS(attitude_controller=g,backend=NS(readVehicleState=lambda:state),
        _attitudeFeedbackEpoch=(0,0),_vioFeedback=((0,0,0),)*4,_pendingAttitudeOdom=deque(),
        control_mode="px4_velocity",get_parameter=lambda _:NS(value=True))
    Px4InterfaceNode._drainAttitudeOdom(n)
    assert g.ready and g.offset==0. and n._vioFeedback is None
    state.restart_epoch=1
    Px4InterfaceNode._drainAttitudeOdom(n)
    assert g.latched=="flight_controller_reference_changed"


@pytest.mark.parametrize("actual,ready",[((0.,.1,0.),True),((.5,.1,0.),False)])
def test_velocity_evidence_compares_same_axes_vio_and_px4_feedback(actual,ready):
    from boom_birds_control.px4_interface_node import Px4InterfaceNode
    from boom_birds_control.velocity_guidance import VelocityGuidance
    g=VelocityGuidance();g.offset=0.
    b=NS(velocityControlReady=lambda:(True,"ready"),hoverEstimatorReady=lambda:(True,"ready"),
         readVehicleState=lambda:NS(velocity_ned_m_s=actual))
    n=NS(attitude_controller=g,backend=b,get_parameter=lambda _:NS(value=False),
        _vioFeedback=((0,0,0),(.1,0.,0.),(0,0,0,1),10.))
    assert Px4InterfaceNode._velocityReady(n)[0] is ready


def bootstrapNode():
    from collections import deque
    from boom_birds_control.velocity_guidance import VelocityGuidance
    state=NS(connected=True,armed=False,landed_state=1,heartbeat_age_s=.1,landed_age_s=.1)
    n=NS(control_mode="px4_velocity",attitude_controller=VelocityGuidance(),require_session=True,
         ingress=NS(session=""),backend=NS(readVehicleState=lambda:state),
         _pendingAttitudeOdom=deque([object()]),_externalVisionWarmUntil=123.)
    return n,state


@pytest.mark.parametrize("fault",["armed","reference_frozen","session_started","stale_heartbeat","not_landed","stale_landed","latched"])
def test_vision_bootstrap_may_not_reset_committed_or_uncertain_reference(fault):
    from boom_birds_control.px4_interface_node import Px4InterfaceNode
    n,state=bootstrapNode()
    if fault=="armed":state.armed=True
    elif fault=="reference_frozen":n.attitude_controller.offset=0.
    elif fault=="session_started":n.ingress.session="started"
    elif fault=="stale_heartbeat":state.heartbeat_age_s=2.
    elif fault=="not_landed":state.landed_state=2
    elif fault=="stale_landed":state.landed_age_s=None
    else:n.attitude_controller.latched="vio_position_reset"
    assert not Px4InterfaceNode._restartVisionBootstrap(n)
    assert n._externalVisionWarmUntil==123. and len(n._pendingAttitudeOdom)==1


def test_vision_bootstrap_retries_are_bounded_and_require_new_warmup():
    from boom_birds_control.px4_interface_node import Px4InterfaceNode
    n,_=bootstrapNode()
    assert Px4InterfaceNode._restartVisionBootstrap(n)
    assert n._externalVisionWarmUntil is None and not n._pendingAttitudeOdom
    assert Px4InterfaceNode._restartVisionBootstrap(n)
    assert not Px4InterfaceNode._restartVisionBootstrap(n)
    assert n._externalVisionBootstrapRestarts==2 and n.attitude_controller.offset is None


def test_startup_vision_gap_restarts_only_uncommitted_ground_warmup():
    from boom_birds_control.px4_interface_node import Px4InterfaceNode
    n,_=bootstrapNode()
    n.frame_world="global";n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_050_000_000))
    n.get_parameter=lambda _:NS(value=True)
    n._externalVisionPrepared=0;n._externalVisionLastStamp=99.5
    n._externalVisionLastPosition=__import__('numpy').array([2.,0.,0.])
    Px4InterfaceNode._publishExternalVision(n,odom())
    assert n._externalVisionLastStamp==100. and n._externalVisionBootstrapRestarts==1
    assert n._externalVisionReason=="dry_run_suppressed" and not n.attitude_controller.latched


def test_feedback_startup_requires_own_observations_and_current_valid_estimator():
    from boom_birds_control.external_vision import ExternalVisionStartup
    gate=ExternalVisionStartup()
    assert not gate.ready(5.,True)
    gate.markSent(5.)
    assert gate.ready(5.,True)
    assert not gate.ready(5.1,False)
    assert gate.ready(5.2,True)
    assert not gate.ready(5.1,True)
    assert not gate.ready(6.,True)
    gate.markSent(6.)
    assert gate.ready(6.,True)
    gate.reset()
    assert not gate.ready(8.,True)


def test_feedback_startup_clock_regression_cannot_preserve_readiness():
    from boom_birds_control.external_vision import ExternalVisionStartup
    gate=ExternalVisionStartup();gate.markSent(1.)
    for i in range(10):assert gate.ready(1.+i*.1,True)
    assert gate.ready(2.,True)
    assert not gate.ready(1.5,True)
    assert gate.firstSent is None


def test_external_vision_rate_limit_keeps_original_sample_times(monkeypatch):
    from boom_birds_control import px4_interface_node as module
    sent=[];clock=[1.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    node=NS(attitude_controller=NS(ready=True,latched='',config=NS(max_velocity=2.,reset_position_margin=.2)),
        frame_world='global',get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_100_000_000)),
        get_parameter=lambda key:NS(value=0 if key=='sih_pid' else False),
        _hardwareActionsAllowed=lambda:True,_externalVisionLastStamp=None,_externalVisionLastPosition=None,
        _externalVisionPrepared=0,_externalVisionWarmUntil=None,
        backend=NS(sendExternalVision=lambda m:sent.append(m) or True))
    for elapsed in (0.,.02,.06):
        clock[0]=1.+elapsed;m=odom();m.header.stamp.nanosec=round(elapsed*1e9)
        module.Px4InterfaceNode._publishExternalVision(node,m)
    assert len(sent)==2 and node._externalVisionPrepared==3
    assert [m.header.stamp.nanosec for m in sent]==[0,60_000_000]


@pytest.mark.parametrize('mid,age',[(30,.16),(32,.21),(230,.21)])
def test_expired_queued_control_observation_is_dropped_before_decoding(mid,age):
    from boom_birds_control.mavros_backend import MavrosPx4Backend
    from mavros_msgs.msg import Mavlink
    bumped=[]
    b=NS(_endpointVerified=True,_clock=lambda:10.,_rosOffset=None,_sourceAgeLimit=2.5,
        _node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_000_000_000))),_bump=bumped.append)
    m=Mavlink(msgid=mid);stamp=100.-age;m.header.stamp.sec=int(stamp);m.header.stamp.nanosec=round((stamp-int(stamp))*1e9)
    MavrosPx4Backend._onRaw(b,m)
    assert bumped==['stale_mavros_control_observations']


def test_fast_reference_uses_three_samples_and_wider_pairing_only_before_freeze():
    from boom_birds_control.px4_interface_node import Px4InterfaceNode
    from boom_birds_control.velocity_guidance import VelocityConfig,VelocityGuidance
    paired=[];state=NS(armed=False,attitude_age_s=.1)
    guide=VelocityGuidance(VelocityConfig(alignment_samples=3))
    n=NS(attitude_controller=guide,control_mode="px4_velocity",_fastVisionStartup=True,
        frame_world="global",get_parameter=lambda _:NS(value=False),
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_100_000_000)),
        backend=NS(readVehicleState=lambda:state,
                   attitudeAt=lambda stamp,tolerance:paired.append(tolerance) or (0.,0.,0.)),
        _onOdomImpl=lambda *args,**kwargs:None)
    for i in range(4):
        m=odom();m.header.stamp.nanosec=i*20_000_000
        m.twist.twist.linear.x=m.twist.twist.linear.y=m.twist.twist.linear.z=0.
        assert Px4InterfaceNode._observeAttitudeOdom(n,m,observed=10.+i*.02)==""
        assert guide.ready is (i>=2)
    assert paired==[.1,.1,.1,.03]
    state.attitude_age_s=.2;m=odom();m.header.stamp.nanosec=80_000_000
    assert Px4InterfaceNode._observeAttitudeOdom(n,m,observed=10.08)=="fcu_attitude_missing_or_stale"
    assert len(paired)==4
