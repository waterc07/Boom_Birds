from copy import deepcopy
from dataclasses import replace
import math
from pathlib import Path
import numpy as np
import pytest
import yaml
from boom_birds_bringup.attitude_lifecycle import AttitudeMissionConfig
from boom_birds_bringup.lifecycle import Observation, State
from boom_birds_bringup.tag_flight_task import TagFlightTask, LandingFix, loadTask
from boom_birds_bringup.tag_landing_fix import TagLandingFix
from boom_birds_control.attitude_control import wrap
from boom_birds_control.platform_model import BoardObservation
from boom_birds_control.runtime_config import DEFAULTS

ROOT = Path(__file__).resolve().parents[1]
TASKS = ROOT/"config/flight_tasks"
TAG = ROOT.parent/"boom_birds_sim/config/platform_landing_test.yaml"


def tickClear(f, now, observation):
    from boom_birds_bringup.task_depth_guard import ObstacleReport
    f.obstacle = ObstacleReport(now,"CLEAR","TEST_ONLY")
    return f.tick(now,observation)


def makeTask(name="tag_takeoff_land"):
    return TagFlightTask(DEFAULTS, AttitudeMissionConfig(), loadTask(TASKS/(name+".yaml")))


def runTask(name, yaw=0.):
    f = makeTask(name)
    f.currentYaw = yaw
    assert f.start("test", (99.,99.,99.), 0.)
    origin = (2., -3., .4)
    tag = (origin if name != "tag_forward_land" else
           (origin[0]+math.cos(yaw), origin[1]+math.sin(yaw), origin[2]))
    offboard = armed = nativeLand = False
    landed = 1
    position, velocity = origin, (0.,)*3
    phases, points, actionsSeen = [], [], []
    for index in range(2500):
        now = index*.02
        if f.hold is not None and not nativeLand:
            position, velocity, _ = f.holdSetpoint(now)
            f.currentYaw = wrap(f.yawSetpoint)
        if nativeLand:
            position = (*position[:2], max(origin[2], position[2]-.2*.02))
            velocity = (0.,0.,-.2) if position[2] > origin[2] else (0.,)*3
        if f.state == State.LANDING and position[2] <= origin[2]+.025:
            landed, velocity = 1, (0.,)*3
        height = position[2]-origin[2]
        visible = math.dist(position[:2], tag[:2]) < .3 and height > 0.
        f.fix = LandingFix(now, tag, height) if visible else None
        o = Observation(session="test", connected=True, armed=armed, landed=landed,
            status_age=0., pose_age=0., position=tuple(position), velocity=tuple(velocity),
            mode_detail="auto:land" if nativeLand else ("offboard" if offboard else "manual"), alignment=True,
            sensors_ready=True, sending=True)
        actions = tickClear(f,now, o)
        assert "enable_planner" not in actions
        actionsSeen += actions
        if "offboard" in actions: offboard = True
        if "land" in actions: nativeLand = True
        if "arm" in actions: armed, landed = True, 2
        if "disarm" in actions: armed = False
        if not phases or phases[-1] != (f.phase, f.stageIndex):
            phases.append((f.phase, f.stageIndex))
        points.append((now, position, f.currentYaw, f.state))
        if f.state in (State.COMPLETE, State.FAULT_LATCHED):
            break
    assert f.state == State.COMPLETE, (f.state, f.reason, phases, now)
    assert f.goal_reached and actionsSeen.count("land") == 1
    assert now < f.mission.flight_budget_s
    assert math.dist(position[:2], tag[:2]) < .05
    return f, points, phases


@pytest.mark.parametrize("name", ["tag_takeoff_land", "tag_return_turn", "tag_forward_land"])
@pytest.mark.parametrize("yaw", [0., math.pi/2, 2.8])
def test_complete_routes_in_takeoff_frame(name, yaw):
    f, points, phases = runTask(name, yaw)
    if name == "tag_return_turn":
        assert [phase for phase,index in phases if phase in ("MOVE","TURN")] == ["MOVE","TURN","MOVE"]
        assert abs(wrap(f.yawSetpoint-yaw-math.pi)) < .001
        far = max(points, key=lambda sample: math.dist(sample[1][:2], (2.,-3.)))
        assert far[1][0] == pytest.approx(2+math.cos(yaw), abs=.01)
        assert far[1][1] == pytest.approx(-3+math.sin(yaw), abs=.01)
    elif name == "tag_forward_land":
        assert not any(phase == "TURN" for phase,index in phases)


def airborneTask():
    f = makeTask()
    f.currentYaw = .3
    f.start("test", (0.,0.,0.), 0.)
    o = Observation(session="test", connected=True, armed=True, landed=2, status_age=0.,
        pose_age=0., position=(0.,0.,.6), velocity=(0.,)*3, mode_detail="offboard",
        alignment=True, sensors_ready=True, sending=True)
    f.initialYaw = f.yawSetpoint = .3
    f.takeoffOrigin = (0.,0.,0.)
    f.groundZ = 0.
    f.hold = (0.,0.,.6)
    f.flightStarted = 0.
    f.armRequested = True
    f.armed_seen = True
    f.state = State.EXECUTING
    f.beginStage(1.)
    f.beginStage(1.1)
    return f, o


def test_tag_loss_stops_descent_and_falls_back_without_disarming():
    f,o = airborneTask()
    f.fix = LandingFix(1.1, (0.,0.,0.), .6)
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
    f.fix = LandingFix(1.7, (0.,0.,0.), .6)
    tickClear(f,1.7,o)
    assert f.state == State.LANDING
    f.fix = LandingFix(1.8, (0.,0.,0.), .6)
    tickClear(f,1.8,o)
    assert f.landingSetpoint[2] < .6
    f.fix = None
    tickClear(f,1.82,o)
    assert f.landingSetpoint[2] == .6
    actions = tickClear(f,2.4,o)
    assert f.state == State.FAULT_LATCHED and f.reason == "task_tag_lost"
    assert "land" in actions and "disarm" not in actions


@pytest.mark.parametrize("fault", ["manual", "pose", "boundary", "wrong_landing_area"])
def test_faults_do_not_continue_the_route(fault):
    f,o = airborneTask()
    if fault == "manual":
        o = replace(o, mode_detail="manual")
    elif fault == "pose":
        o = replace(o, alignment=False)
    elif fault == "boundary":
        o = replace(o, position=(2.,0.,.6))
    else:
        f.fix = LandingFix(1.2, (.8,0.,0.), .6)
    actions = tickClear(f,1.2,o)
    assert f.state == State.FAULT_LATCHED
    assert "disarm" not in actions
    assert ("land" in actions) == (fault != "manual")


def test_budget_uses_native_land_instead_of_untagged_descent():
    f,o = airborneTask()
    f.phase = "MOVE"
    actions = tickClear(f,31.,o)
    assert f.state == State.FAULT_LATCHED
    assert "land" in actions and "hold_setpoint" not in actions


def test_tag_required_before_departure_but_not_for_task_three():
    f = makeTask("tag_return_turn")
    f.currentYaw = f.initialYaw = f.yawSetpoint = 0.
    f.start("s", (0.,0.,0.), 0.)
    f.currentYaw = f.initialYaw = f.yawSetpoint = 0.
    f.hold = (0.,0.,.6)
    f.takeoffOrigin = (0.,0.,0.)
    f.groundZ = 0.
    f.state = State.EXECUTING
    f.beginStage(1.)
    o = Observation(position=f.hold, velocity=(0.,)*3)
    for t in [1.,2.,3.]:
        f._executingActions(t,o)
    assert f.stageIndex == 0


@pytest.mark.parametrize("bad", ["nan", "no_land", "unknown_step", "radius"])
def test_task_validation(tmp_path,bad):
    t = loadTask(TASKS/"tag_return_turn.yaml")
    if bad == "nan": t["max_speed_m_s"] = float("nan")
    if bad == "no_land": t["steps"].pop()
    if bad == "unknown_step": t["steps"][0]["action"] = "planner"
    if bad == "radius": t["steps"][1]["forward_m"] = 5.
    path = tmp_path/"task.yaml"
    path.write_text(yaml.safe_dump(t))
    with pytest.raises(ValueError):
        loadTask(path)


def trackerInput(stamp=10., yaw=math.pi/2):
    profile = yaml.safe_load(TAG.read_text())
    tracker = TagLandingFix(profile, testOnly=True)
    q = (0.,0.,math.sin(yaw/2),math.cos(yaw/2))
    tracker.addPose(stamp, (2.,3.,.6), q)
    tracker.addRange(stamp, .58, .04, 5., "range_sensor")
    pose = np.eye(4)
    # Board +z is up in body FLU; board point is forward .1 m and below .6 m.
    pose[:3,3] = [.1,0.,-.6]
    observation = BoardObservation(stamp, profile["board"]["name"], True, "ok",
        tuple(map(tuple,pose)), (7,), .1, 5., 100.)
    tracker.submit(observation)
    return tracker, observation


def test_tag_geometry_uses_sampled_body_rotation():
    tracker,o = trackerInput()
    fix = tracker.evaluate(10.01)
    assert fix.target == pytest.approx((2.,3.1,0.))
    assert fix.height == pytest.approx(.6)
    assert fix.stamp == 10.


@pytest.mark.parametrize("bad", ["stale","future","wrong_id","frame","ambiguous","reprojection",
                                  "no_range","range_jump","pair","tilt","reordered"])
def test_tag_bad_inputs_revoke_fix(bad):
    tracker,o = trackerInput()
    assert tracker.evaluate(10.01)
    if bad == "stale":
        assert tracker.evaluate(10.2) is None
        return
    if bad == "future": o = replace(o,stamp=11.)
    if bad == "wrong_id": o = replace(o,tag_ids=(8,))
    if bad == "frame": o = replace(o,frame="optical")
    if bad == "ambiguous": o = replace(o,ambiguity_ratio=1.)
    if bad == "reprojection": o = replace(o,reprojection_px=20.)
    if bad == "no_range": tracker.ranges.clear()
    if bad == "range_jump":
        tracker.ranges.clear()
        tracker.addRange(10.,1.,.04,5.,"range_sensor")
    if bad == "pair": o = replace(o,stamp=10.08)
    if bad == "tilt":
        pose = np.asarray(o.T_body_platform).copy()
        pose[:3,:3] = np.diag([1.,-1.,-1.])
        o = replace(o,T_body_platform=tuple(map(tuple,pose)))
    if bad == "reordered": o = replace(o,stamp=9.99)
    tracker.submit(o)
    assert tracker.evaluate(10.1) is None


def test_repeated_tag_does_not_refresh_sample_age():
    tracker,o = trackerInput()
    assert tracker.evaluate(10.01)
    tracker.submit(o)
    assert tracker.evaluate(10.14).stamp == 10.
    assert tracker.evaluate(10.16) is None


def test_range_invalid_clears_old_sample():
    tracker,o = trackerInput()
    assert tracker.evaluate(10.01)
    tracker.addRange(10.02, float("nan"), .04,5.,"range_sensor")
    assert tracker.evaluate(10.03) is None


def test_tag_target_jump_is_rejected():
    tracker,o = trackerInput()
    assert tracker.evaluate(10.01)
    pose = np.asarray(o.T_body_platform).copy()
    pose[0,3] = 1.
    tracker.addPose(10.02, (2.,3.,.6), (0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)))
    tracker.addRange(10.02,.58,.04,5.,"range_sensor")
    tracker.submit(replace(o,stamp=10.02,T_body_platform=tuple(map(tuple,pose))))
    assert tracker.evaluate(10.03) is None
    assert tracker.reason == "tag_target_jump"


def test_reacquire_after_180_degree_turn_uses_new_rotation_baseline():
    tracker,o = trackerInput()
    first = tracker.evaluate(10.01)
    tracker.addPose(11., (2.,3.,.6), (0.,0.,math.sin(3*math.pi/4),math.cos(3*math.pi/4)))
    tracker.addRange(11.,.58,.04,5.,"range_sensor")
    pose = np.eye(4)
    pose[:3,:3] = np.diag([-1.,-1.,1.])
    pose[:3,3] = [-.1,0.,-.6]
    tracker.submit(replace(o,stamp=11.,T_body_platform=tuple(map(tuple,pose))))
    fix = tracker.evaluate(11.01)
    assert fix is not None,tracker.reason
    assert fix.target == pytest.approx(first.target)


def test_impossible_route_budget_is_rejected_before_start():
    task = loadTask(TASKS/"tag_return_turn.yaml")
    task["max_speed_m_s"] = .01
    with pytest.raises(ValueError,match="duration_exceeds"):
        TagFlightTask(DEFAULTS, AttitudeMissionConfig(), task)


def test_offline_feature_loss_ekf_projection_drives_landing_state_machine():
    from boom_birds_control.landing_ekf_reference import EkfReferenceSample, LandingEkfReference
    from boom_birds_control.landing_ekf_projection import EkfLandingProjection, FeatureWorldSample
    f, obs = airborneTask()
    for i in range(7):
        t = 1.1+i*.1
        f.fix = LandingFix(t,(0.,0.,0.),.6,source="resolved_tag")
        tickClear(f,t,obs)
    assert f.state == State.LANDING
    ref = EkfLandingProjection(LandingEkfReference(max_age_s=.2,max_pair_skew_s=.05,
        max_accuracy_m=.2,max_duration_s=12.,max_displacement_m=.5,max_speed_m_s=1.))
    def sample(t):
        return EkfReferenceSample(t,(10.,20.),True,False,.05,0,0)
    start = 1.8
    feature = FeatureWorldSample(start,start,(0.,0.),(0.,0.),0.)
    assert ref.step(start,feature,sample(start),(0.,0.)).source == "visual"
    # Synthetic EKF and range remain valid after the feature stream disappears.
    height=.6
    native=False
    for i in range(250):
        t=start+i*.05
        xy=ref.step(t,None,sample(t),(0.,0.))
        assert xy is not None and xy.source=="ekf", (t,ref.reason)
        z=max(0.,height-f.obstacleConfig.ground_clearance_m)
        obs=replace(obs,position=(*xy.position_world_xy,z),velocity=(0.,0.,0.))
        f.fix=LandingFix(t,(*xy.target_world_xy,0.),height,source="ekf")
        actions=tickClear(f,t,obs)
        if "land" in actions:
            native=True
            break
        assert f.state==State.LANDING, (f.state,f.reason,t)
        height=max(.01,height-.08*.05)
    assert native and f.phase=="PX4_FINAL_LAND"
