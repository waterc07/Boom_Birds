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



@pytest.mark.parametrize("mode,expected", [
    ("companion_attitude", {30:50000.,32:500000.,245:1000000.,436:1000000.,331:1000000.}),
    ("px4_position", {30:20000.,32:20000.,245:1000000.,436:100000.,331:100000.}),
])
def test_observation_rates_preserve_mode_and_reset_observations(mode, expected):
    backend=MavrosPx4Backend(NS(),clock=ManualClock(),control_mode=mode)
    backend.is_connected=lambda:True
    requests=[]
    class Client:
        def service_is_ready(self):return True
        def call_async(self,request):
            requests.append(request)
            return NS(add_done_callback=lambda cb:cb(NS(result=lambda:NS(success=True))))
    backend._command_client=Client();backend._command_type=NS(Request=NS)
    backend.request_observation_streams()
    assert {int(r.param1):r.param2 for r in requests} == expected
    assert all(r.command==511 for r in requests)
    assert backend._observation_streams_requested
    backend.request_observation_streams()
    assert len(requests)==5
    assert backend._time_gate.hold_last_sync_on_rtt == (mode=="companion_attitude")
    assert backend._time_gate.max_rtt_s==.02
    assert backend._time_gate.sync_timeout_s==1.
    assert backend._time_gate.converge_samples==5


class PendingTimer:
    def __init__(self): self.canceled=True
    def is_canceled(self): return self.canceled
    def reset(self): self.canceled=False
    def cancel(self): self.canceled=True


def observation_probe(boot_ms=100005):
    clock=ManualClock(100.)
    node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=round((clock()+900.)*1e9))))
    backend=MavrosPx4Backend(node,clock=clock,control_mode="companion_attitude")
    backend._endpoint_verified=True
    backend._observation_timer=PendingTimer()
    gate=backend._time_gate;gate.count=5;gate.last_sync=100.;gate.offset_ns=900_000_000_000
    data=struct.pack("<Iffffff",boot_ms,.1,.2,.3,0.,0.,0.)
    padded=data+bytes((-len(data))%8)
    message=NS(msgid=30,framing_status=1,len=len(data),payload64=list(struct.unpack("<"+"Q"*(len(padded)//8),padded)),sysid=1,compid=1,seq=1,header=NS(stamp=NS(sec=1000,nanosec=0)))
    return backend,clock,message

def test_future_attitude_waits_without_changing_timestamp():
    b,clock,m=observation_probe();b._on_raw(m)
    assert not b._attitude_history and len(b._pending_observations)==1
    assert not b._observation_timer.is_canceled()
    b._drain_pending_observations();assert not b._attitude_history
    clock.advance(.006);b._drain_pending_observations()
    assert len(b._attitude_history)==1 and not b._pending_observations
    assert b._observation_timer.is_canceled()
    assert b._attitude_history[0][0]==pytest.approx(100.005)
    assert m.header.stamp.sec==1000 and m.header.stamp.nanosec==0
    assert b.attitude_at(100.005,.001) is not None

@pytest.mark.parametrize("kind", ["too_far_future","expired_mapping","mapping_reset","future_after_wait"])
def test_future_buffer_preserves_rejections(kind):
    b,clock,m=observation_probe(100025 if kind=="too_far_future" else 100005)
    b._on_raw(m)
    if kind=="too_far_future":assert not b._pending_observations
    elif kind=="expired_mapping":clock.advance(1.01)
    elif kind=="mapping_reset":b._time_gate.reset("timesync_stale")
    else:
        b._time_gate.offset_ns+=100_000_000
        clock.advance(.021)
    b._drain_pending_observations()
    assert not b._attitude_history and not b._pending_observations
    assert b._observation_timer.is_canceled()
