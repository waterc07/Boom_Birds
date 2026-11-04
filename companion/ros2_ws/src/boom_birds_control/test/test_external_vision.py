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
