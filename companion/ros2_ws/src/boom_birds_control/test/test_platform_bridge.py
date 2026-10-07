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
