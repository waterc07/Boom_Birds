from dataclasses import replace
import subprocess
import sys
from test_platform_landing import profile, inputs
from boom_birds_control.platform_executor import PlatformExecutor
from boom_birds_control.platform_landing import HandoffFeedback, VELOCITY_YAW_RATE_MASK
from boom_birds_control.px4_backend import FakePx4Backend, ManualClock
from boom_birds_control.px4_frames import Px4LocalSetpoint
from boom_birds_bringup.compute_resources import ComputeResources
import pytest


def test_single_owner_actual_fake_backend_calls_and_owned_process_release():
    clock=ManualClock(0.)
    b=FakePx4Backend(clock=clock);b.connect()
    processes=[subprocess.Popen([sys.executable,"-c","import time; time.sleep(60)"]) for _ in range(2)]
    resources=ComputeResources(processes[:1],processes[1:])
    calls=[]
    def release(token):
        resources.confirm(ex.core._output(None))
        calls.append(("release",ex.core.confirmed,token))
        return resources.release(token)
    ex=PlatformExecutor(b,profile(),test_only=True,
        revoke_navigation=lambda:calls.append(("revoke",)) or True,
        release_compute=release,native_land=lambda reason:True,disarm=lambda:True)
    p=Px4LocalSetpoint((0,0,-1),(0,0,0),(0,0,0),0)
    assert ex.navigation_position(p,448)
    ex.request()
    try:
        for i in range(35):
            t=i*.02;clock.set(t);b.feed_heartbeat()
            o,r,f=inputs(t)
            ack=None
            if ex.core.last_sent_sequence:
                seq=ex.core.last_sent_sequence
                ack=HandoffFeedback(t,ex.core.token,seq,0,True,False,
                    ex.core.sent_history[seq][1],"TEST_ONLY_FAKE_PX4")
            out=ex.tick(t,o,r,f,ack,navigation_ready=True)
            if ex.owner=="platform":
                assert not ex.navigation_position(p,448)
            if not ex.core.confirmed:
                assert all(process.poll() is None for process in processes)
        assert ex.core.confirmed and ex.core.released and resources.released
        assert calls[0]==("revoke",) and calls[1][0:2]==("release",True)
        masks=[c.args[1] if len(c.args)>1 else None for c in b.calls_named("send_setpoint")]
        assert b.counters["setpoints_sent"]>0
        assert out.velocity_ned[2]>0
    finally:
        for process in processes:
            if process.poll() is None: process.terminate()
            process.wait(timeout=3)


def test_resource_release_without_handoff_rejected():
    resources=ComputeResources([],[])
    with pytest.raises(ValueError):resources.release("wrong")


@pytest.mark.parametrize("revoke_ok,send_ok",[(False,True),(True,False)])
def test_revoke_and_send_failure_preserve_dependencies(revoke_ok,send_ok):
    b=FakePx4Backend();b.connect();b.set_setpoint_acceptance(send_ok)
    releases=[]
    ex=PlatformExecutor(b,profile(),test_only=True,revoke_navigation=lambda:revoke_ok,
        release_compute=lambda token:releases.append(token) or True,
        native_land=lambda reason:True,disarm=lambda:True)
    ex.request()
    for i in range(12):
        o,r,f=inputs(i*.02);out=ex.tick(i*.02,o,r,f,navigation_ready=True)
    assert out.request_native_land and out.vio_required and not releases
