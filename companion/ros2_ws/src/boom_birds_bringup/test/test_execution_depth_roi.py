from types import SimpleNamespace
import numpy as np
import pytest
from boom_birds_bringup.task_depth_guard import ObstacleConfig, executionDepthEnvelope
from boom_birds_bringup.task_depth_bridge import TaskDepthBridge
from boom_birds_bringup.lifecycle import State
from boom_birds_sensing.depth_roi import projectEnvelope

CAM = np.eye(4)
CAM[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]


def test_envelope_contains_near_and_braking_distance_at_one_m_s():
    c = ObstacleConfig()
    points = executionDepthEnvelope(c, CAM, 1.)
    assert points.shape == (8, 3)
    assert points[:, 2].min() == pytest.approx(.2)
    assert points[:, 2].max() == pytest.approx(2.3666666667)
    k = np.array([[181.7, 0, 161.18], [0, 181.7, 120.62], [0, 0, 1]])
    assert projectEnvelope(points, k, (320, 240), .5).bounds is None


def test_camera_offset_is_applied_and_invalid_direction_rejected():
    camera = CAM.copy(); camera[0, 3] = .1
    points = executionDepthEnvelope(ObstacleConfig(), camera)
    assert points[:, 2].min() == pytest.approx(.1)
    with pytest.raises(ValueError):
        executionDepthEnvelope(ObstacleConfig(), np.eye(4))


def bridge():
    b = TaskDepthBridge.__new__(TaskDepthBridge)
    b.roiSpeedLimit = 1.
    b.node = SimpleNamespace(fsm=SimpleNamespace(state=State.EXECUTING,
        avoidance=SimpleNamespace(active=False), stageIndex=0, phase="FORWARD",
        segmentYawGoal=0., segmentYawStart=0., segmentGoal=np.array([1., 0., 0.]),
        segmentStart=np.zeros(3), task={"max_speed_m_s": .3}),
        control=SimpleNamespace(control_velocity_world=SimpleNamespace(x=.3, y=0., z=0.)))
    return b


def test_turn_lateral_motion_overspeed_and_missing_feedback_force_full_frame():
    b = bridge()
    assert b._forwardDemand(np.eye(3))
    b.node.fsm.avoidance.active = True
    assert not b._forwardDemand(np.eye(3))
    b.node.fsm.avoidance.active = False
    b.node.fsm.segmentYawGoal = .1
    assert not b._forwardDemand(np.eye(3))
    b.node.fsm.segmentYawGoal = 0.
    b.node.fsm.segmentGoal[1] = .1
    assert not b._forwardDemand(np.eye(3))
    b.node.fsm.segmentGoal[1] = 0.
    b.node.control.control_velocity_world.x = 1.01
    assert not b._forwardDemand(np.eye(3))
    b.node.control = None
    assert not b._forwardDemand(np.eye(3))


def test_launch_wires_candidate_and_rejects_external_sensing(tmp_path):
    from pathlib import Path
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters
    from boom_birds_bringup.tag_task_launch import taskLaunch, taskNodes
    from test_hardware_profile import bundle
    root = Path(__file__).resolve().parents[1]
    ctx = LaunchContext()
    for action in taskLaunch("tag_forward_land").entities:
        if isinstance(action, DeclareLaunchArgument):
            ctx.launch_configurations[action.name] = perform_substitutions(ctx, action.default_value)
    ctx.launch_configurations.update(bundle(tmp_path))
    ctx.launch_configurations.update(depth_roi="true", tag_test_only="true",
        tag_config_file=str(root.parent/"boom_birds_sim/config/platform_landing_test.yaml"),
        flight_task_file=str(root/"config/flight_tasks/tag_forward_land.yaml"),
        obstacle_config_file=str(root/"config/tag_obstacle_guard.yaml"))
    params = [evaluate_parameters(ctx, a._Node__parameters)[-1]
              for a in taskNodes(ctx) if isinstance(a, Node)]
    mission = next(p for p in params if isinstance(p, dict) and "depth_roi" in p)
    depth = next(p for p in params if isinstance(p, dict) and "roi_demand_topic" in p)
    assert mission["depth_roi"] and mission["depth_roi_speed_limit_m_s"] == 1.
    assert depth["roi_margin_fraction"] == .5
    assert depth["roi_demand_topic"].endswith("/execution_envelope")
    ctx.launch_configurations["start_sensing"] = "false"
    with pytest.raises(ValueError, match="depth_roi_candidate"):
        taskNodes(ctx)
    ctx.launch_configurations["start_sensing"] = "true"
    ctx.launch_configurations["dry_run"] = "false"
    with pytest.raises(ValueError, match="depth_roi_candidate"):
        taskNodes(ctx)



@pytest.mark.parametrize("previousState", ["CLEAR", "BLOCKED"])
def test_waiting_for_depth_pair_keeps_report_until_its_original_expiry(previousState):
    from boom_birds_bringup.task_depth_guard import DepthCorridor, ObstacleReport

    b = bridge()
    b.config = ObstacleConfig()
    b.guard = DepthCorridor(b.config)
    b.guard.report = ObstacleReport(1., previousState, "prior")
    b.pending = SimpleNamespace(stamp=1.2)
    b.node.stamp = lambda msg: msg.stamp
    b.node.rosNow = lambda: 1.25
    b.node.tracker = SimpleNamespace(poses=[(1.,)])
    b.infos = [(1.2,)]
    b.cameras = [(1.,)]
    assert b.evaluate().state == previousState
    assert b.pending is not None
    b.node.rosNow = lambda: 1.31
    assert b.evaluate().reason == "depth_stale"
    assert b.pending is not None



def test_trial_corridor_speed_requires_fresh_feedback_and_reserves_landing_speed(monkeypatch):
    from boom_birds_bringup.task_depth_guard import DepthCorridor

    b = bridge()
    b.guard = DepthCorridor(ObstacleConfig(), fixedFloor=False)
    b.node.config = SimpleNamespace(pose_timeout_s=.15)
    b.node.fsm.task["max_landing_xy_m_s"] = .1
    b.node.controlReceived = 10.
    b.node.stamp = lambda msg: 100.
    b.node.control.control_pose_known = True
    b.node.control.control_pose_age_s = .01
    b.node.control.control_velocity_world.x = .02
    monkeypatch.setattr("boom_birds_bringup.task_depth_bridge.time.monotonic", lambda: 10.01)
    assert b.corridorSpeed(100.01) == .1
    b.node.control.control_velocity_world.x = .3
    assert b.corridorSpeed(100.01) == .3
    assert b.corridorSpeed(100.2) is None
