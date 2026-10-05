from dataclasses import replace
import math
import pytest
from boom_birds_bringup.attitude_lifecycle import AttitudeLifecycle
from boom_birds_bringup.lifecycle import Observation, State
from boom_birds_control.runtime_config import DEFAULTS


def ready(**kwargs):
    return replace(Observation(session='s',connected=True,landed=1,status_age=0,pose_age=0,
        position=(0,0,0),velocity=(0,0,0),alignment=True,sensors_ready=True,sending=True),**kwargs)


def launch():
    f=AttitudeLifecycle(DEFAULTS)
    assert f.start('s',(1,0,1.5),0)
    assert f.tick(.1,ready()) == ['hold_setpoint']
    assert f.state == State.HOLD_READY
    assert f.tick(1.4,ready()) == ['hold_setpoint','offboard']
    assert f.state == State.OFFBOARD_PENDING
    off=ready(mode='offboard',mode_detail='offboard',offboard=True)
    assert f.tick(1.5,off) == ['hold_setpoint','arm']
    return f,off


def test_ground_stream_before_arm_and_vio_takeoff():
    f,o=launch()
    assert f.tick(1.52,replace(o,pose_age=math.inf,sensors_ready=False,sending=False)) == ["hold_setpoint"]
    assert f.state == State.TAKEOFF
    p,v,a=f.hold_setpoint(2.)
    assert 0<p[2]<.1 and 0<v[2]<=.3
    assert f.takeoff_origin==(0,0,0)
    assert 'takeoff' not in f.tick(2.,replace(o,armed=True,landed=2))


def test_missing_pose_during_flight_cannot_keep_old_alignment():
    f,o=launch()
    f.tick(2.,replace(o,armed=True,landed=2,pose_age=math.inf))
    actions=f.tick(2.2,replace(o,armed=True,landed=2,pose_age=math.inf))
    assert f.state==State.FAULT_LATCHED and 'land' in actions
    assert not f.goal_reached


def test_time_budget_retires_planner_and_lands():
    f,o=launch(); f.state=State.EXECUTING
    actions=f.tick(23.5,replace(o,armed=True,landed=2,position=(.3,0,1.5)))
    assert f.state==State.LANDING and not f.goal_reached
    assert f.reason=='mission_time_budget'
    assert actions==['disable_planner','cancel','hold_setpoint']
    p,v,a=f.hold_setpoint(24.5)
    assert p[2]==pytest.approx(1.25) and v[2]==-.25


def test_disarm_only_after_continuous_landed_and_fault_closes():
    f,o=launch(); f.hold=(0,0,1.5); f.land(3.)
    o=replace(o,armed=True)
    assert 'disarm' not in f.tick(3.1,o)
    assert 'disarm' in f.tick(4.2,o)
    assert f.tick(4.3,replace(o,armed=False))==['cancel']
    assert f.state==State.COMPLETE
    f,o=launch(); assert 'land' in f.cancel(2.)
    assert f.state==State.FAULT_LATCHED


def test_operator_mode_change_never_requests_native_land():
    f,o=launch()
    actions=f.tick(2.,replace(o,armed=True,landed=2,mode='manual',mode_detail='manual',offboard=False))
    assert f.state==State.FAULT_LATCHED and 'land' not in actions
    assert f.latch(2.1,'land_request_rejected')==[]
