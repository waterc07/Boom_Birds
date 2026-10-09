#!/usr/bin/env python3
"""Deterministic software plant and fake FCU. Synthetic parameters only."""
import argparse
from platform_evidence import load_profile
from dataclasses import asdict,replace
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import numpy as np
from boom_birds_control.platform_model import BoardObservation
from boom_birds_control.platform_landing import FlightSample,RangeSample,HandoffFeedback
from boom_birds_control.platform_executor import PlatformExecutor
from boom_birds_control.px4_backend import FakePx4Backend,ManualClock
from boom_birds_bringup.compute_resources import ComputeResources


def run(profile,scenario,out):
    clock=ManualClock(0.)
    backend=FakePx4Backend(clock=clock);backend.connect()
    children=[subprocess.Popen([sys.executable,"-c","import time; time.sleep(60)"]) for _ in range(2)]
    resources=ComputeResources(children[:1],children[1:])
    fallback=[];disarms=[]
    def release(token):
        resources.confirm(executor.core._output(None))
        return resources.release(token)
    executor=PlatformExecutor(backend,profile,test_only=True,
        revoke_navigation=lambda:True,release_compute=release,
        native_land=lambda reason:fallback.append(reason) or True,
        disarm=lambda:disarms.append(True) or True)
    # The same arbiter initially routes a navigation attitude target.
    assert executor.navigation_attitude(SimpleNamespace(orientation_xyzw=(0.,0.,0.,1.),thrust=.5))
    executor.request()
    height=profile["guidance"]["takeoff_height_m"]
    xy=np.array([.15,-.10]) if scenario=="return" else np.zeros(2)
    velocity=np.zeros(3);armed=True;trace=[]
    result="FAIL"
    try:
        for i in range(1200):
            t=i*.02;clock.set(t);backend.feed_heartbeat()
            pose=np.eye(4);pose[:3,3]=[*xy,-height]
            observation=BoardObservation(t,profile["board"]["name"],True,"synthetic",
                tuple(map(tuple,pose)),tuple(tag["id"] for tag in profile["board"]["tags"]),
                .1,5.,100.,source="TEST_ONLY_fake")
            range_sample=RangeSample(t,height-.02)
            if executor.core.confirmed and t>1.:
                if scenario=="tag_loss":observation=replace(observation,valid=False)
                elif scenario=="range_jump":range_sample=replace(range_sample,distance_m=height+.98)
                elif scenario=="wrong_id":observation=replace(observation,tag_ids=(999,))
                elif scenario=="stale":observation=replace(observation,stamp=t-1.)
                elif scenario=="pose_jump":
                    p=pose.copy();p[0,3]+=2;observation=replace(observation,T_body_platform=tuple(map(tuple,p)))
            flight=FlightSample(t,0.,0.,0.,tuple(velocity),True,True,True,
                                bool(height<=.08),armed,0)
            ack=None
            if executor.core.last_sent_sequence and scenario!="handoff_failure":
                seq=executor.core.last_sent_sequence
                ack=HandoffFeedback(t,executor.core.token,seq,0,True,False,
                    executor.core.sent_history[seq][1],"TEST_ONLY_FAKE_PX4")
            output=executor.tick(t,observation,range_sample,flight,ack,navigation_ready=True)
            trace.append(dict(now=t,input=asdict(observation),range=asdict(range_sample),
                flight=asdict(flight),feedback=asdict(ack) if ack else None,output=asdict(output),
                owner=executor.owner,resources_alive=[p.poll() is None for p in children]))
            if not executor.core.confirmed:assert all(p.poll() is None for p in children)
            if executor.owner!="navigation":
                assert not executor.navigation_attitude(SimpleNamespace(orientation_xyzw=(0.,0.,0.,1.),thrust=.5))
            if output.velocity_ned is not None:
                velocity=np.array(output.velocity_ned)
                assert velocity[2]<=0 or output.descent_permitted
            elif fallback:velocity=np.array([0.,0.,.25])
            else:velocity=np.zeros(3)
            xy-=np.array([velocity[0],-velocity[1]])*.02
            height=max(.08,height-velocity[2]*.02)
            if disarms or fallback and height<=.08:armed=False
            if output.state=="COMPLETE" or fallback and not armed:
                result="PASS";break
        if scenario=="handoff_failure":
            assert not executor.core.confirmed and not resources.released
        else:assert executor.core.confirmed
        report=dict(kind="OFFLINE_FAKE_FCU",scenario=scenario,result=result,
            config=profile,confirmed=executor.core.confirmed,compute_release=resources.released,
            landed_disarmed=bool(not armed and height<=.08),fallback=fallback,disarm_requests=len(disarms),
            samples=len(trace),plant="synthetic kinematic integration; not PX4 dynamics")
        out.mkdir(parents=True,exist_ok=False)
        (out/"trace.jsonl").write_text("".join(json.dumps(s,allow_nan=False)+"\n" for s in trace))
        (out/"report.json").write_text(json.dumps(report,indent=2))
        return report
    finally:
        for child in children:
            if child.poll() is None:child.terminate()
            child.wait(timeout=3.)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--config",required=True);ap.add_argument("--out",required=True)
    args=ap.parse_args()
    profile=load_profile(args.config)
    out=Path(args.out);out.mkdir(parents=True,exist_ok=False)
    reports=[]
    for scenario in ("origin","return","wrong_id","stale","tag_loss","pose_jump","range_jump","handoff_failure"):
        reports.append(run(profile,scenario,out/scenario))
    summary=dict(kind="OFFLINE_FAKE_FCU",reports=reports,
                 config_sha256=hashlib.sha256(Path(args.config).read_bytes()).hexdigest())
    (out/"report.json").write_text(json.dumps(summary,indent=2))
    print(json.dumps({r["scenario"]:r["result"] for r in reports}))
    return 0 if all(r["result"]=="PASS" for r in reports) else 1

if __name__=="__main__":raise SystemExit(main())
