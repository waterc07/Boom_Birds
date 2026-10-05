import math
from types import SimpleNamespace as NS
import numpy as np
import pytest
from boom_birds_control.attitude_control import (AttitudeController, AttitudeConfig,
    rotation, yaw_rotation, px4_attitude_enu_flu, NED_TO_ENU, FRD_TO_FLU)
from boom_birds_control.mavros_backend import MavrosPx4Backend


def aligned(offset=.7):
    c = AttitudeController()
    for i in range(10):
        c.observe((0,0,0),(0,0,0),(0,0,0,1),yaw_rotation(offset),1+i*.02,False)
    assert c.ready
    return c


def target(p=(0,0,0),v=(0,0,0),a=(0,0,0),yaw=0.):
    return NS(position_m=p,velocity_m_s=v,acceleration_m_s2=a,yaw_rad=yaw)


def test_hover_and_heading_transform():
    c = aligned()
    s = c.calculate(target(),(0,0,0),(0,0,0),(0,0,0,1))
    assert s.thrust == pytest.approx(.5)
    assert np.allclose(rotation(s.orientation_xyzw),yaw_rotation(.7))
    assert np.allclose(px4_attitude_enu_flu(0,0,0),yaw_rotation(math.pi/2))


def test_acceleration_direction_and_limits():
    c = aligned(0)
    s = c.calculate(target((.5,0,0)),(0,0,0),(0,0,0),(0,0,0,1))
    z = rotation(s.orientation_xyzw)[:,2]
    assert z[0] > 0 and z[1] == pytest.approx(0)
    s = c.calculate(target(v=(1.4,1.4,0)),(0,0,0),(-1.4,-1.4,0),(0,0,0,1))
    assert s.saturated
    assert math.acos(rotation(s.orientation_xyzw)[2,2]) <= c.config.max_tilt_rad+1e-9
    assert c.config.min_thrust <= s.thrust <= c.config.max_thrust
    with pytest.raises(ValueError,match='position_error'): c.calculate(target((2,0,0)),(0,0,0),(0,0,0),(0,0,0,1))
    with pytest.raises(ValueError): c.calculate(target(a=(math.nan,0,0)),(0,0,0),(0,0,0),(0,0,0,1))


@pytest.mark.parametrize('kind', ['position','heading','clock','gravity'])
def test_estimator_reset_latches(kind):
    c = aligned()
    args = [(0,0,0),(0,0,0),(0,0,0,1),yaw_rotation(.7),1.2,True]
    if kind == 'position': args[0]=(1,0,0)
    elif kind == 'heading': args[3]=yaw_rotation(1.)
    elif kind == 'clock': args[4]=1.1
    else: args[3]=np.diag([1.,-1.,-1.])
    with pytest.raises(ValueError): c.observe(*args)
    assert c.latched and not c.ready
    with pytest.raises(ValueError): c.calculate(target(),(0,0,0),(0,0,0),(0,0,0,1))


def test_requires_disarmed_alignment_and_valid_profile():
    c=AttitudeController()
    with pytest.raises(ValueError,match='disarmed'): c.observe((0,0,0),(0,0,0),(0,0,0,1),np.eye(3),1,True)
    for kwargs in ({'hover_thrust':.8},{'max_acceleration':10.},{'alignment_samples':0},{'kp':[1,math.nan,1]}):
        with pytest.raises(ValueError): AttitudeConfig(**kwargs)


def test_attitude_backend_suppresses_dryrun_and_other_mode():
    s=aligned().calculate(target(),(0,0,0),(0,0,0),(0,0,0,1))
    assert not MavrosPx4Backend(NS()).send_attitude_setpoint(s)
    b=MavrosPx4Backend(NS(),control_mode='companion_attitude')
    assert b.send_attitude_setpoint(s)
    b.dry_run=False
    assert not b.send_attitude_setpoint(s)


def test_touchdown_ramp_requires_contact_and_resets_on_height_change():
    from boom_birds_control.attitude_control import TouchdownRamp
    ramp=TouchdownRamp(.08)
    s=aligned().calculate(target(),(0,0,0),(0,0,0),(0,0,0,1))
    def apply(t,z=0,v=(0,0,0),landing=True):
        return ramp.apply(s,position=(0,0,z),velocity=v,ground_z=0,now=t,landing=landing)
    assert apply(1,z=1).thrust==.5
    assert apply(2).thrust==.5
    for t in np.arange(2.1,3.51,.1): result=apply(float(t))
    assert result.thrust==pytest.approx(.29)
    for t in np.arange(3.6,4.01,.1): result=apply(float(t))
    assert result.thrust==pytest.approx(.08)
    assert apply(4.1,z=.3).thrust==.5
    assert apply(5).thrust==.5
    assert apply(6,landing=False).thrust==.5


def test_bad_sih_pid_cannot_enable_simulation_clock():
    b=MavrosPx4Backend(NS(),sih_pid=999999999,control_mode='companion_attitude')
    assert not b._sih_receipt_time
    b._attitude_history.append((1.,(0.,0.,0.)))
    assert b.attitude_at(1.,.03) is None


def test_landing_command_still_expires_and_cancel_retires_it():
    from dataclasses import replace
    from boom_birds_control.control_protocol import Command, ControlIngress, HOLD, EXECUTE, CANCEL
    g=ControlIngress(); session=g.open_session()
    c=Command(session,1,1,10.,.1,HOLD,landing=True)
    assert not g.receive(replace(c,kind=EXECUTE),10.,20.)[0]
    assert g.receive(c,10.,20.)[0]
    assert g.active(10.11,20.11) is None
    c=replace(c,trajectory_id=2,sequence=2,stamp=11.)
    assert g.receive(c,11.,21.)[0]
    assert g.receive(replace(c,kind=CANCEL,sequence=3,stamp=0.),11.,21.)[0]
    assert not g.receive(replace(c,sequence=4),11.,21.)[0]
    assert g.active(11.,21.) is None


def test_gap_revokes_frozen_attitude_reference():
    c=aligned()
    with pytest.raises(ValueError,match='vio_gap_reference_invalid'):
        c.observe((0,0,0),(0,0,0),(0,0,0,1),yaw_rotation(.7),1.6,True)
    assert c.latched and not c.ready


def test_vio_publisher_restart_clears_feedback_even_without_pose_jump():
    from boom_birds_control.px4_interface_node import Px4InterfaceNode
    node=NS(attitude_controller=aligned(),_vio_publisher_gid=b'old',_vio_feedback=object())
    Px4InterfaceNode._on_odom(node,NS(),NS(publisher_gid=b'new'))
    assert node.attitude_controller.latched=='vio_producer_changed'
    assert node._vio_feedback is None
