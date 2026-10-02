import math
import struct
from types import SimpleNamespace as NS
import pytest
from boom_birds_control.mavros_backend import MavrosPx4Backend, ned_to_enu
from boom_birds_control.mavros_observation import decode_observation
from boom_birds_control.px4_backend import ManualClock, assert_no_actuator_surface
from boom_birds_control.px4_frames import Px4LocalSetpoint, TypeMask


def raw(mid, payload, *, sysid=1, compid=1, framing=1):
    padded = payload + bytes((-len(payload)) % 8)
    return NS(msgid=mid, framing_status=framing, len=len(payload),
        payload64=list(struct.unpack("<" + "Q"*(len(padded)//8), padded)), sysid=sysid, compid=compid, seq=3)


def test_coordinate_and_partial_axis_masks():
    assert ned_to_enu((1., 2., -3.)) == (2., 1., 3.)


def test_raw_mode_boot_reset_and_framing():
    b = MavrosPx4Backend(NS(), clock=ManualClock())
    b._conn = NS()
    for mid, payload in [(0, struct.pack("<IBBBBB", 6<<16, 2, 12, 128, 4, 3)),
        (30, struct.pack("<Iffffff", 10000, .1, .2, .3, 0., 0., .4)),
        (32, struct.pack("<Iffffff", 10000, 1., 2., -3., 4., 5., -6.)),
        (436, struct.pack("<IIB", (6<<24)|(4<<16), (5<<24)|(4<<16), 0))]:
        b._handle_message(decode_observation(raw(mid, payload)))
    s = b.read_vehicle_state()
    assert s.position_ned_m == (1., 2., -3.)
    assert s.current_mode_detail == "auto:land" and s.intended_mode_detail == "auto:rtl"
    payload = bytearray(233); payload[230] = 2
    b._handle_message(decode_observation(raw(331, bytes(payload))))
    payload[230] = 3
    b._handle_message(decode_observation(raw(331, bytes(payload))))
    assert b.read_vehicle_state().frame_reset_epoch == 1
    b._handle_message(decode_observation(raw(32, struct.pack("<Iffffff", 100, 0., 0., 0., 0., 0., 0.))))
    assert b.read_vehicle_state().restart_epoch == 1
    assert not b.is_connected()
    assert decode_observation(raw(0, b"x", framing=2)) is None
    assert_no_actuator_surface(MavrosPx4Backend)


def test_dryrun_and_endpoint_guards():
    b = MavrosPx4Backend(NS())
    p = Px4LocalSetpoint((1., 2., -3.), (.1, .2, .3), (.4, .5, .6), .7)
    assert b.send_setpoint(p, int(TypeMask.for_mode("position_velocity")))
    assert not b.arm() and not b.set_mode("offboard")
    b.dry_run = False
    assert not b.send_setpoint(p, int(TypeMask.for_mode("position_velocity")))
    assert not MavrosPx4Backend(NS(), fcu_url="udp://0.0.0.0:14540@192.168.1.2:14580").connect()
    assert not b.send_setpoint(p, None)


def test_runtime_nodes_start_without_pymavlink():
    import subprocess
    import sys
    script = """
import builtins
original = builtins.__import__
def guarded(name, *args, **kwargs):
    if name == 'pymavlink' or name.startswith('pymavlink.'):
        raise ImportError('production must use MAVROS')
    return original(name, *args, **kwargs)
builtins.__import__ = guarded
import rclpy
from rclpy.node import Node
from boom_birds_control.mavros_backend import MavrosPx4Backend
from boom_birds_sensing.mavros_imu_node import MavrosImuNode
rclpy.init()
node = Node('mavros_dependency_probe')
backend = MavrosPx4Backend(node)
assert backend.connect()
imu = MavrosImuNode()
backend.close()
imu.destroy_node()
node.destroy_node()
rclpy.shutdown()
"""
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stderr
