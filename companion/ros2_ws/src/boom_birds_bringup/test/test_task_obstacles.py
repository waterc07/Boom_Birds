from dataclasses import replace
import math
from pathlib import Path
import numpy as np
import pytest
from boom_birds_bringup.task_depth_guard import ObstacleConfig,ObstacleReport,DepthCorridor,loadObstacleConfig
from boom_birds_bringup.task_avoidance import TaskAvoidance
from boom_birds_bringup.tag_flight_task import TagFlightTask,LandingFix,loadTask
from boom_birds_bringup.attitude_lifecycle import AttitudeMissionConfig
from boom_birds_bringup.lifecycle import State,Observation
from boom_birds_control.runtime_config import DEFAULTS
from boom_birds_control.attitude_control import wrap

ROOT=Path(__file__).resolve().parents[1]
CAM=np.eye(4)
CAM[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
K=np.array([[100.,0,80.],[0,100.,60.],[0,0,1.]])


def check(depth,**kwargs):
    guard=DepthCorridor(ObstacleConfig())
    body=np.eye(4);body[2,3]=.6
    return guard,guard.observe(depth,10.,K,CAM,body,groundZ=0.,speed=.3,**kwargs)


def test_clear_finite_room():
    guard,report=check(np.full((120,160),4.,np.float32))
    assert report.state=="CLEAR"
    assert guard.fresh(10.29).state=="CLEAR"
    assert guard.fresh(10.31).state=="UNKNOWN"


@pytest.mark.parametrize("value",[float("nan"),float("inf"),0.,-1.,6.])
def test_unknown_pixels_do_not_authorize_motion(value):
    depth=np.full((120,160),4.,np.float32);depth[60,80]=value
    _,report=check(depth)
    assert report.state=="UNKNOWN"


def test_single_close_pixel_stops_and_emergency_is_distinct():
    depth=np.full((120,160),4.,np.float32);depth[60,80]=.6
    _,r=check(depth)
    assert r.state=="BLOCKED" and not r.emergency
    depth[60,80]=.1
    _,r=check(depth)
    assert r.state=="BLOCKED" and r.emergency


def test_outside_corridor_is_not_an_obstacle():
    depth=np.full((120,160),4.,np.float32);depth[0,0]=.6
    _,r=check(depth)
    assert r.state=="CLEAR"


def test_body_transform_and_camera_translation():
    guard=DepthCorridor(ObstacleConfig())
    camera=CAM.copy();camera[0,3]=.2
    depth=np.full((120,160),4.,np.float32);depth[60,80]=.6
    body=np.eye(4);body[2,3]=.6
    r=guard.observe(depth,10.,K,camera,body,groundZ=0.,speed=.3)
    assert r.state=="CLEAR"
    assert guard.observe(depth,9.,K,camera,body,groundZ=0.,speed=.3).state=="UNKNOWN"


def test_required_corridor_must_fit_camera_fov():
    guard=DepthCorridor(ObstacleConfig(half_width_m=2.))
    assert guard.observe(np.full((120,160),4.,np.float32),10.,K,CAM,np.eye(4),groundZ=0.,speed=.3).reason=="corridor_outside_fov"


def test_stopping_distance_includes_speed_and_latency():
    c=ObstacleConfig()
    assert c.distance(1.) > c.distance(.3)
    assert c.distance(1.) >= c.body_front_m+c.clearance_m+c.reaction_s+1/(2*c.braking_accel_m_s2)


def test_live_obstacle_profile_cannot_use_design_assumptions():
    path=ROOT/"config/tag_obstacle_guard.yaml"
    loadObstacleConfig(path)
    with pytest.raises(ValueError,match="hardware_evidence"):
        loadObstacleConfig(path,live=True)


def airborne(name="tag_forward_land",budget=90.):
    task=loadTask(ROOT/"config/flight_tasks"/(name+".yaml"))
    f=TagFlightTask(DEFAULTS,replace(AttitudeMissionConfig(),flight_budget_s=budget),task)
    f.start("s",(0.,0.,0.),0.)
    f.takeoffOrigin=(0.,0.,0.);f.groundZ=0.
    f.hold=(0.,0.,.6);f.currentYaw=f.initialYaw=f.yawSetpoint=0.
    f.flightStarted=0.;f.armRequested=f.armed_seen=True
    f.transition(State.EXECUTING,1.)
    f.beginStage(1.);f.beginStage(1.)
    o=Observation(session="s",connected=True,armed=True,landed=2,status_age=0.,pose_age=0.,
        position=(.2,0.,.6),velocity=(0.,)*3,mode_detail="offboard",alignment=True,sensors_ready=True,sending=True)
    f.obstacle=ObstacleReport(1.,"CLEAR","TEST_ONLY")
    f.tick(1.,o)
    f.obstacle=ObstacleReport(1.02,"BLOCKED","TEST_ONLY",.5)
    f.tick(1.02,o)
    return f,o


def drive(f,o,*,failDetour=False,blockReturn=False):
    trace=[];actions=[];nativeLand=False
    for i in range(1,4000):
        now=1.02+i*.02
        p,v,_=f.holdSetpoint(now)
        if nativeLand:
            p=(*p[:2],max(0.,o.position[2]-.2*.02))
            v=(0.,0.,-.2) if p[2]>0 else (0.,)*3
        f.currentYaw=wrap(f.yawSetpoint)
        o=replace(o,position=tuple(p),velocity=tuple(v))
        a=f.avoidance
        state="CLEAR"
        if a.state=="BRAKE" and a.intent=="DETOUR":state="BLOCKED"
        if failDetour and a.purpose=="DETOUR" and a.state=="OBSERVE":state="BLOCKED"
        if blockReturn and a.purpose=="RETURN" and a.state in ("OBSERVE","MOVE"):state="BLOCKED"
        f.obstacle=ObstacleReport(now,state,"TEST_ONLY",.5 if state=="BLOCKED" else None)
        f.fix=LandingFix(now,(1.,0.,0.),max(.01,p[2]))
        if f.state==State.LANDING and p[2]<.025:o=replace(o,landed=1,velocity=(0.,)*3)
        result=f.tick(now,o);actions+=result
        if "land" in result:
            nativeLand=True;o=replace(o,mode_detail="auto:land")
        if "disarm" in result:o=replace(o,armed=False)
        trace.append((now,a.state,a.purpose,tuple(p)))
        if f.state in (State.COMPLETE,State.FAULT_LATCHED):break
    return trace,actions


def test_detour_once_resumes_original_goal():
    f,o=airborne()
    trace,actions=drive(f,o)
    assert f.state==State.COMPLETE,(f.reason,trace[-1])
    assert f.avoidance.used and not f.obstacleMissionFailed
    assert max(row[3][1] for row in trace)>.3
    assert f.segmentGoal[:2]==(1.,0.)
    assert actions.count("land")==1


def test_detour_failure_retraces_then_requests_home_landing():
    f,o=airborne()
    trace,actions=drive(f,o,failDetour=True)
    assert f.reachedHome and f.obstacleMissionFailed,(f.reason,trace[-1])
    assert f.reason=="returned_home_native_land" and "land" in actions
    assert math.dist(trace[-1][3][:2],(0.,0.))<.08
    assert f.avoidance.used
    assert not f.goal_reached


def test_return_obstruction_does_not_keep_flying():
    f,o=airborne()
    trace,actions=drive(f,o,failDetour=True,blockReturn=True)
    assert f.state==State.FAULT_LATCHED and "land" in actions
    assert not f.reachedHome and not f.goal_reached


def test_budget_does_not_get_extended_for_detour():
    f,o=airborne(budget=45.)
    trace,actions=drive(f,o)
    assert f.mission.flight_budget_s==45.
    assert f.state in (State.COMPLETE,State.FAULT_LATCHED)
    assert trace[-1][0]<45.


def test_braking_reference_respects_acceleration_bound():
    f,o=airborne()
    a=f.avoidance
    o=replace(o,velocity=(.3,0.,0.))
    a.brake(2.,o,0.,"RETURN")
    for t in np.linspace(2.,3.2,30):
        p,v,acc,yaw=a.sample(t)
        assert np.linalg.norm(acc)<=a.config.braking_accel_m_s2+1e-9
        assert abs(v[0])<=.3
    assert a.sample(3.2)[0][0]==pytest.approx(.35)


def test_breadcrumbs_keep_corner_and_do_not_evict_home():
    task=loadTask(ROOT/"config/flight_tasks/tag_forward_land.yaml")
    a=TaskAvoidance(task,ObstacleConfig(breadcrumb_capacity=8))
    for x in np.arange(0.,1.01,.1):assert a.record((x,0.,.6))
    for y in np.arange(.1,1.01,.1):assert a.record((1.,y,.6))
    assert len(a.breadcrumbs)<=4
    assert any(np.linalg.norm(p-(1.,0.,.6))<.03 for p in a.breadcrumbs)
    assert tuple(a.breadcrumbs[0])==(0.,0.,.6)


def test_unknown_depth_prevents_precheck_and_arm():
    task=loadTask(ROOT/"config/flight_tasks/tag_forward_land.yaml")
    f=TagFlightTask(DEFAULTS,AttitudeMissionConfig(),task)
    f.currentYaw=0.;f.start("s",(0.,0.,0.),0.)
    o=Observation(session="s",connected=True,landed=1,status_age=0.,pose_age=0.,
                  position=(0.,0.,0.),velocity=(0.,)*3,alignment=True,sensors_ready=True,sending=True)
    assert f.tick(.1,o)==[]
    assert f.state==State.PRECHECK and not f.armRequested


def test_lost_tag_revokes_velocity_feedforward():
    from test_tag_flight_task import airborneTask,tickClear
    f,o=airborneTask()
    f.fix=LandingFix(1.1,(0.,0.,0.),.6)
    tickClear(f,1.1,o)
    f.fix=LandingFix(1.2,(0.,0.,0.),.6)
    tickClear(f,1.2,o)
    f.fix=LandingFix(1.3,(0.,0.,0.),.6)
    tickClear(f,1.3,o)
    f.fix=LandingFix(1.4,(0.,0.,0.),.6)
    tickClear(f,1.4,o)
    f.fix=LandingFix(1.5,(0.,0.,0.),.6)
    tickClear(f,1.5,o)
    f.fix=LandingFix(1.6,(0.,0.,0.),.6)
    tickClear(f,1.6,o)
    f.fix=LandingFix(1.7,(0.,0.,0.),.6)
    tickClear(f,1.7,o)
    f.fix=LandingFix(1.8,(0.,0.,0.),.6)
    tickClear(f,1.8,o)
    assert f.holdSetpoint(1.8)[1][2]<0
    f.fix=None
    tickClear(f,1.82,o)
    assert f.holdSetpoint(1.82)[1][2]==0.
    tickClear(f,2.01,o)
    assert f.alignSince is None


def test_missing_heading_during_landing_does_not_raise_or_descend():
    from test_tag_flight_task import airborneTask,tickClear
    f,o=airborneTask()
    f.state=State.LANDING
    f.landingSetpoint=[0.,0.,.4]
    f.landingVelocity=(0.,0.,-.15)
    f.currentYaw=None
    actions=tickClear(f,1.2,o)
    assert actions==["hold_setpoint"]
    assert f.landingVelocity==(0.,0.,0.)


def test_hover_confirmation_and_tag_samples_are_separate():
    task=loadTask(ROOT/"config/flight_tasks/tag_return_turn.yaml")
    f=TagFlightTask(DEFAULTS,AttitudeMissionConfig(),task)
    f.start("s",(0.,0.,0.),0.)
    f.takeoffOrigin=(0.,0.,0.);f.groundZ=0.
    f.hold=(0.,0.,.6);f.currentYaw=f.initialYaw=f.yawSetpoint=0.
    f.flightStarted=0.;f.armRequested=f.armed_seen=True
    f.transition(State.EXECUTING,1.);f.beginStage(1.)
    o=Observation(session="s",connected=True,armed=True,landed=2,status_age=0.,pose_age=0.,
        position=f.hold,velocity=(0.,)*3,mode_detail="offboard",alignment=True,sensors_ready=True,sending=True)
    for i in range(61):
        t=1.+i*.02
        f.fix=LandingFix(t,(0.,0.,0.),.6) if i%3 else None
        f.obstacle=ObstacleReport(t,"CLEAR","TEST_ONLY")
        f.tick(t,o)
    assert f.stageIndex==1 and f.phase=="MOVE"


def test_pending_offboard_cannot_arm_with_stale_pose():
    f,o=airborne()
    f.state=State.OFFBOARD_PENDING
    f.armRequested=False
    f.flightStarted=None
    f.entered=2.
    o=replace(o,armed=False,landed=1,pose_age=math.inf)
    f.obstacle=ObstacleReport(2.01,"CLEAR","TEST_ONLY")
    assert "arm" not in f.tick(2.01,o)


def test_floor_clipping_before_takeoff():
    from tools.run_tag_flight_sih import Room
    room=Room()
    camera=CAM.copy()
    body=np.eye(4)
    depth=room.render(camera,np.zeros(3))
    g=DepthCorridor(ObstacleConfig())
    assert g.observe(depth,1.,K,camera,body,groundZ=0.,speed=.3).state=="CLEAR"


def test_native_final_land_requires_mode_and_ground_readback():
    from test_tag_flight_task import airborneTask
    f,o=airborneTask()
    f.state=State.LANDING;f.nativeLandStarted=2.;f.flightStarted=1.
    assert f.tick(2.1,o)==[]
    assert not f.nativeLandConfirmed
    o=replace(o,mode_detail="auto:land")
    assert f.tick(2.2,o)==[] and f.nativeLandConfirmed
    o=replace(o,landed=1)
    assert f.tick(2.3,o)==[]
    assert f.tick(3.4,o)==["disarm"]
    assert f.tick(3.5,replace(o,armed=False))==["cancel"]
    assert f.state==State.COMPLETE


def test_native_final_land_does_not_fight_operator_mode():
    from test_tag_flight_task import airborneTask
    f,o=airborneTask()
    f.state=State.LANDING;f.nativeLandStarted=2.;f.flightStarted=1.
    actions=f.tick(2.1,replace(o,mode_detail="manual"))
    assert f.state==State.FAULT_LATCHED and "land" not in actions


def test_native_final_land_timeout_is_not_success():
    from test_tag_flight_task import airborneTask
    f,o=airborneTask()
    f.state=State.LANDING;f.nativeLandStarted=2.;f.flightStarted=1.
    actions=f.tick(2.+f.config.mode_timeout_s+.1,o)
    assert f.state==State.FAULT_LATCHED and "land" in actions



def test_final_approach_stops_descent_before_waiting_for_low_speed():
    from test_tag_flight_task import airborneTask
    f,o=airborneTask()
    f.state=State.LANDING;f.phase="TAG_DESCEND";f.segmentGoal=(0.,0.,.6)
    f.landingSetpoint=[0.,0.,.1];f.lastLandingTick=2.;f.alignSince=1.
    o=replace(o,position=(0.,0.,.12),velocity=(0.,0.,-.12))
    f.fix=LandingFix(2.02,(0.,0.,-.05),.17)
    f.obstacle=ObstacleReport(2.02,"CLEAR","TEST_ONLY")
    actions=f.tick(2.02,o)
    assert f.landingVelocity==(0.,0.,0.) and f.landingSetpoint[2]==.12
    assert "land" not in actions and f.nearGroundSince is None


def test_native_final_land_rejects_disconnected_status():
    from test_tag_flight_task import airborneTask
    f,o=airborneTask()
    f.state=State.LANDING;f.nativeLandStarted=2.;f.flightStarted=1.
    f.tick(2.1,replace(o,connected=False,mode_detail="auto:land",landed=1,armed=False))
    assert f.state==State.FAULT_LATCHED


def test_expired_flight_cannot_complete_even_with_disarmed_ground_status():
    from test_tag_flight_task import airborneTask
    f,o=airborneTask()
    f.state=State.LANDING;f.nativeLandStarted=44.;f.flightStarted=0.
    f.tick(45.1,replace(o,mode_detail="auto:land",landed=1,armed=False))
    assert f.state==State.FAULT_LATCHED



def test_px4_mode_reset_after_confirmed_landing_and_disarm_is_complete():
    from test_tag_flight_task import airborneTask
    f,o=airborneTask()
    f.state=State.LANDING;f.nativeLandStarted=2.;f.flightStarted=1.
    f.tick(2.1,replace(o,mode_detail="auto:land"))
    actions=f.tick(2.2,replace(o,mode_detail="offboard",landed=1,armed=False))
    assert f.state==State.COMPLETE and actions==["cancel"]


def test_px4_mode_reset_while_airborne_still_aborts():
    from test_tag_flight_task import airborneTask
    f,o=airborneTask()
    f.state=State.LANDING;f.nativeLandStarted=2.;f.flightStarted=1.
    f.tick(2.1,replace(o,mode_detail="auto:land"))
    f.tick(2.2,replace(o,mode_detail="offboard"))
    assert f.state==State.FAULT_LATCHED
