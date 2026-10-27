from dataclasses import asdict
import json
import math
from pathlib import Path
import time
from unittest.mock import patch
import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from launch import LaunchContext
from launch_ros.actions import Node
from launch.utilities import perform_substitutions
from boom_birds_bringup.lifecycle import State
from boom_birds_bringup.tag_flight_node import TagFlightNode
from boom_birds_bringup import tag_task_launch
from boom_birds_interfaces.msg import ExecutionStatus, ControlCommand
from boom_birds_interfaces.srv import Mission

ROOT = Path(__file__).resolve().parents[1]
TAG = ROOT.parent/"boom_birds_sim/config/platform_landing_test.yaml"
ATTITUDE = ROOT.parent/"boom_birds_control/config/attitude_hardware.yaml"


@pytest.fixture
def taskNode():
    context = Context()
    rclpy.init(context=context)
    node = TagFlightNode(context=context, parameter_overrides=[
        Parameter("control_mode", value="companion_attitude"),
        Parameter("attitude_config_file", value=str(ATTITUDE)),
        Parameter("flight_task_file", value=str(ROOT/"config/flight_tasks/tag_return_turn.yaml")),
        Parameter("tag_config_file", value=str(TAG)),
        Parameter("tag_test_only", value=True),
        Parameter("feature_tracking_enabled",value=False),
        Parameter("require_landing_inputs",value=False),
        Parameter("obstacle_config_file",value=str(ROOT/"config/tag_obstacle_guard.yaml"))])
    from boom_birds_bringup.task_depth_guard import ObstacleReport
    node.depthBridge.evaluate = lambda: ObstacleReport(time.monotonic(),"CLEAR","TEST_ONLY")
    try:
        yield node
    finally:
        node.destroy_node()
        context.shutdown()


def setAirborne(node):
    now = time.monotonic()
    f = node.fsm
    f.currentYaw = 0.
    f.start("test",(0.,0.,0.),now-2.)
    f.initialYaw = f.yawSetpoint = 0.
    f.takeoffOrigin = (0.,0.,0.)
    f.groundZ = 0.
    f.hold = (0.,0.,.6)
    f.flightStarted = now-2.
    f.armRequested = f.armed_seen = True
    f.transition(State.EXECUTING,now)
    f.beginStage(now-1.)
    node.ingress.session = f.session
    msg = ExecutionStatus(session_id=f.session, connected=True, armed=True,
        landed_known=True, landed_state=2, sending=True, sensors_ready=True,
        alignment_valid=True, control_mode="companion_attitude", control_pose_known=True,
        control_pose_age_s=0., control_yaw_world_rad=0., mode_detail="offboard")
    msg.control_position_world.z = .6
    msg.header.stamp = node.get_clock().now().to_msg()
    node._controlStatus(msg)
    return now


def test_tick_streams_hold_without_planner_or_map(taskNode):
    node = taskNode
    setAirborne(node)
    with patch.object(node, "_request") as planner, patch.object(node, "_publishCommand") as publish:
        node._tick()
    assert not planner.called
    assert publish.called
    command = publish.call_args[0][0]
    assert command.command_type == ControlCommand.HOLD
    assert command.position.z == .6
    assert node.fsm.state == State.EXECUTING


def test_hold_uses_interpolated_task_yaw(taskNode):
    node = taskNode
    now = setAirborne(node)
    node.fsm.beginStage(now-10.)
    node.fsm.beginStage(now-5.)
    with patch.object(node, "_publishCommand") as publish:
        node._hold()
    command = publish.call_args[0][0]
    assert 0. < command.yaw < math.pi
    assert command.position.x == pytest.approx(1.)


def test_landing_flag_does_not_leak_into_cruise(taskNode):
    node = taskNode
    now = setAirborne(node)
    with patch.object(node, "_publishCommand") as publish:
        node._hold()
    assert not publish.call_args[0][0].landing
    node.fsm.state = State.LANDING
    node.fsm.landingSetpoint = [0.,0.,.4]
    with patch.object(node, "_publishCommand") as publish:
        node._hold()
    assert publish.call_args[0][0].landing


def test_duplicate_start_does_not_erase_flying_observations(taskNode):
    node = taskNode
    setAirborne(node)
    sentinel = object()
    node.tracker.pending = sentinel
    response = node._mission(Mission.Request(action=Mission.Request.START), Mission.Response())
    assert not response.accepted
    assert node.tracker.pending is sentinel


def test_invalid_tag_json_replaces_previous_sample(taskNode):
    taskNode.tracker.pending = object()
    from std_msgs.msg import String
    taskNode.onTag(String(data='{"valid":"true","stamp":"nan"}'))
    assert taskNode.tracker.pending is None


def test_task_three_launch_has_depth_without_ego_or_platform_velocity(tmp_path,monkeypatch):
    from test_hardware_profile import bundle
    args = bundle(tmp_path)
    context = LaunchContext()
    context.launch_configurations.update(args)
    context.launch_configurations.update(flight_task_file=str(ROOT/"config/flight_tasks/tag_forward_land.yaml"),
        tag_config_file=str(TAG),obstacle_config_file=str(ROOT/"config/tag_obstacle_guard.yaml"), tag_test_only="true", dry_run="true", allow_arming="false",
        allow_non_loopback="true", allow_hardware_actions="false", hardware_validation_note="",
        downward_image_topic="/test/downward", downward_input="external",range_input="mavros",
        downward_camera_config_file="",downward_native_config_file="",
        downward_media_device="",downward_sensor_entity="",downward_raw10_native_library="",range_sensor_id="-1",range_target_system="1",range_target_component="1",downward_frame_id="downward_optical",camera_device="/dev/test", capture_width="1280",
        capture_height="480", capture_fps="60", start="false", start_sensing="true",
        handover="true", tag_compute_profile="configured", feature_tracking_diagnostics="false",
        platform_status_timeout_s="0.5", compute_log_directory="",
        platform_trace_directory="")
    executable = lambda node: (node.node_executable if isinstance(node.node_executable, str)
                               else perform_substitutions(context,node.node_executable))
    nodes = [action for action in tag_task_launch.taskNodes(context) if isinstance(action,Node)]
    # 交接入口由任务自有的 compute_supervisor 持有 VIO/适配器子进程，不再直接
    # 作为 launch Node 启动 ov_msckf/pose_adapter。
    assert {executable(node) for node in nodes} == {
        "px4_interface_node","tag_flight_task","platform_observation_node",
        "mavros_range_node","mavros_imu_node","stereo_source","depth_node","compute_supervisor"}
    context.launch_configurations["handover"] = "false"
    nodes = [action for action in tag_task_launch.taskNodes(context) if isinstance(action,Node)]
    assert {executable(node) for node in nodes} == {
        "px4_interface_node","tag_flight_task","platform_observation_node",
        "mavros_range_node","mavros_imu_node","stereo_source","depth_node",
        "run_subscribe_msckf","pose_adapter"}
    context.launch_configurations["start_sensing"] = "false"
    assert len([a for a in tag_task_launch.taskNodes(context) if isinstance(a,Node)]) == 4
    # 交接必须持有自有 VIO 进程，且真实控制入口在速度确认适配器核验前拒绝。
    context.launch_configurations["handover"] = "true"
    with pytest.raises(ValueError, match="tag_handover_requires_owned_vio_processes"):
        tag_task_launch.taskNodes(context)
    context.launch_configurations["start_sensing"] = "true"
    context.launch_configurations["dry_run"] = "false"
    with pytest.raises(ValueError, match="tag_test_only_requires_dry_run"):
        tag_task_launch.taskNodes(context)
    context.launch_configurations["tag_test_only"] = "false"
    with pytest.raises(ValueError, match="tag_handover_requires_dry_run_until_hardware_verified"):
        tag_task_launch.taskNodes(context)



def test_reprocessing_same_tag_keeps_original_monotonic_sample_time():
    from boom_birds_bringup.tag_flight_task import LandingFix
    from types import SimpleNamespace
    clock=SimpleNamespace(tagClockStamp=None,tagMonoStamp=None,rosNow=lambda:100.)
    fix=LandingFix(99.95,(0.,0.,0.),.5)
    first=TagFlightNode.monotonicFix(clock,fix,10.)
    clock.rosNow=lambda:100.019
    second=TagFlightNode.monotonicFix(clock,fix,10.02)
    assert first.stamp==second.stamp
    assert second.stamp==pytest.approx(9.95)


@pytest.mark.parametrize("failure",["publishers","old_image","old_range","old_feature"])
def test_live_ingress_gate_rejects_missing_or_stale_sources(taskNode,failure):
    from types import SimpleNamespace as NS
    node=taskNode;node.requireLandingInputs=True;node.useFeatureTracking=True
    now=time.monotonic();ros=node.rosNow()
    node.inputGraphAt=now;node.inputPublishersReady=failure!="publishers"
    node.tracker.pending=NS(stamp=ros-(.3 if failure=="old_image" else .01))
    node.featureFix.pending=NS(stamp=ros-(.3 if failure=="old_feature" else .01))
    node.tracker.ranges.append((ros-(.3 if failure=="old_range" else .01),.6,.04,5.,"range_sensor"))
    assert not node.landingInputs(now)
    response=node._mission(Mission.Request(action=Mission.Request.START),Mission.Response())
    assert not response.accepted and not node.fsm.armRequested


def test_input_heartbeat_allows_no_tag_on_ground_but_requires_fresh_measurements(taskNode):
    from types import SimpleNamespace as NS
    node=taskNode;node.requireLandingInputs=True;node.useFeatureTracking=True
    now=time.monotonic();ros=node.rosNow()
    node.inputGraphAt=now;node.inputPublishersReady=True
    node.tracker.pending=NS(stamp=ros-.01,valid=False)
    node.featureFix.pending=NS(stamp=ros-.01,valid=False)
    node.tracker.ranges.append((ros-.01,.05,.04,5.,"range_sensor"))
    assert node.landingInputs(now)


@pytest.mark.parametrize("handover",[False,True])
def test_feature_diagnostics_do_not_gate_full_tag_handover(taskNode,monkeypatch,handover):
    from types import SimpleNamespace as NS
    node=taskNode;node.requireLandingInputs=True;node.useFeatureTracking=True;node.handover=handover
    now=time.monotonic();ros=node.rosNow()
    node.inputGraphAt=-math.inf
    monkeypatch.setattr(node,"count_publishers",lambda t:0 if t.endswith("feature_observation") else 1)
    node.tracker.pending=NS(stamp=ros-.01)
    node.featureFix.pending=None
    node.tracker.ranges.append((ros-.01,.6,.04,5.,"range_sensor"))
    assert node.landingInputs(now) is handover
    # 完整解码和测距的时效仍是交接必需条件。
    node.tracker.pending=NS(stamp=ros-1.)
    assert not node.landingInputs(now)


def test_balanced_profile_keeps_geometry_and_configured_profile_unmodified():
    chosen=tag_task_launch.tagComputeProfile("balanced")
    assert "max_cameras" not in chosen["vio"] and "use_stereo" not in chosen["vio"]
    assert chosen["vio"]["use_multi_threading_subs"] is False
    assert max(chosen["stereo"]["rate_hz"],chosen["downward"]["publish_rate_hz"])==10.
    assert chosen["depth"]["max_processing_rate_hz"]==8.
    assert all(not x for x in tag_task_launch.tagComputeProfile("configured").values())
    with pytest.raises(ValueError,match="tag_compute_profile"):
        tag_task_launch.tagComputeProfile("unknown")
