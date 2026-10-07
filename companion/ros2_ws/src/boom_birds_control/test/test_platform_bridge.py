from types import SimpleNamespace as NS
from pathlib import Path
import time
import pytest
import rclpy
from rclpy.parameter import Parameter
from boom_birds_control.mavros_backend import MavrosPx4Backend
from boom_birds_control.px4_frames import Px4LocalSetpoint
from boom_birds_control.platform_landing import VELOCITY_YAW_RATE_MASK

PROFILE=Path(__file__).parents[1]/"config/platform_landing_test.yaml"


def test_mavros_handoff_retires_attitude_before_velocity_and_latches_token():
    context=rclpy.context.Context();rclpy.init(context=context)
    node=rclpy.create_node("platform_backend_probe",context=context)
    backend=MavrosPx4Backend(node,control_mode="companion_attitude",dry_run=True)
    try:
        assert backend.connect()
        assert backend._attitude_pub is not None and backend._pub is None
        attitude=NS(orientation_xyzw=(0.,0.,0.,1.),thrust=.5)
        assert backend.send_attitude_setpoint(attitude)
        assert not backend.activate_platform_velocity("lease")
        backend.allow_platform_handoff=True
        assert backend.activate_platform_velocity("lease")
        assert backend._attitude_pub is None and backend._position_watch is None
        assert backend._pub is not None
        assert not backend.send_attitude_setpoint(attitude)
        p=Px4LocalSetpoint((0,0,0),(.1,.2,.0),(0,0,0),0,.1)
        assert backend.send_setpoint(p,VELOCITY_YAW_RATE_MASK)
        assert not backend.send_setpoint(p,448)
        assert backend.activate_platform_velocity("lease")
        assert not backend.activate_platform_velocity("old_session")
    finally:
        backend.close();node.destroy_node();rclpy.shutdown(context=context)


def test_actual_control_node_opt_in_is_closed_without_navigation_dependency():
    from boom_birds_control.px4_interface_node import Px4InterfaceNode
    context=rclpy.context.Context();rclpy.init(context=context)
    node=Px4InterfaceNode(context=context,parameter_overrides=[
        Parameter("backend",value="fake"),Parameter("platform_config_file",value=str(PROFILE)),
        Parameter("platform_test_only",value=True)])
    try:
        assert node.platform is not None
        node.platform.executor.request()
        node.platform.tick(time.monotonic())
        assert node.platform.output.reason=="navigation_dependency_unavailable"
        assert node.platform.output.velocity_ned is None
        assert node.platform.output.vio_required
    finally:
        node.shutdown();node.destroy_node();rclpy.shutdown(context=context)


def test_actual_control_node_real_entry_rejects_synthetic_config():
    from boom_birds_control.px4_interface_node import Px4InterfaceNode
    context=rclpy.context.Context();rclpy.init(context=context)
    try:
        with pytest.raises(ValueError,match="synthetic"):
            Px4InterfaceNode(context=context,parameter_overrides=[
                Parameter("backend",value="fake"),Parameter("platform_config_file",value=str(PROFILE))])
    finally:rclpy.shutdown(context=context)


@pytest.mark.parametrize("bad", [
    {"valid": "true"}, {"stamp": None}, {"quality": []},
    {"tag_ids": [True]}, {"min_edge_px": "NaN"}, {"T_body_platform": [[1]]}])
def test_malformed_wire_observation_is_rejected_before_control_tick(bad):
    import json
    from dataclasses import asdict
    from std_msgs.msg import String
    from boom_birds_control.platform_bridge import PlatformBridge
    from boom_birds_control.platform_model import BoardObservation
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node("platform_wire_schema", context=context)
    bridge = PlatformBridge.__new__(PlatformBridge)
    bridge.node = node
    value = asdict(BoardObservation(node.get_clock().now().nanoseconds*1e-9, "origin_board", True, "test",
                   ((1,0,0,0),(0,1,0,0),(0,0,1,-1),(0,0,0,1)), (7,), .1, 5., 100.))
    value.update(bad)
    try:
        bridge.on_observation(String(data=json.dumps(value)))
        assert bridge.observation is None
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


def test_completed_release_future_cannot_bypass_same_tick_timeout():
    from concurrent.futures import Future
    from boom_birds_control.px4_interface_node import Px4InterfaceNode
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = Px4InterfaceNode(context=context, parameter_overrides=[
        Parameter("backend", value="fake"),
        Parameter("platform_config_file", value=str(PROFILE)),
        Parameter("platform_test_only", value=True)])
    try:
        bridge = node.platform
        ex = bridge.executor
        ex.core.state, ex.core.intent, ex.core.token = "ALIGN", "land", "late-token"
        ex.core.confirmed = True
        ex.owner = "platform"
        ex.release_state, ex.release_token = "WAIT_RESULT", ex.core.token
        ex.release_since = time.monotonic()-3.
        bridge.release_future_token = ex.core.token
        bridge.release_future = Future()
        bridge.release_future.set_result(NS(success=True))
        bridge.tick(time.monotonic())
        assert not ex.release_verified and not ex.core.released
        assert ex.release_state in ("FAILED", "CANCELLED")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
