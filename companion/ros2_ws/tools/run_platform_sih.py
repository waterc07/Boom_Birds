#!/usr/bin/env python3
"""TEST-ONLY local SIH truth -> platform guidance; never connects hardware."""
import argparse
from platform_evidence import loadProfile
from dataclasses import asdict
import json
import hashlib
import os
from pathlib import Path
import subprocess
import time
import numpy as np
import cv2
from boom_birds_control.platform_executor import PlatformExecutor
from boom_birds_control.platform_landing import FlightSample,RangeSample
from boom_birds_control.platform_sih_feedback import SihVelocityFeedback
from boom_birds_control.px4_backend import MavlinkPx4Backend
from boom_birds_control.px4_frames import Px4LocalSetpoint,TypeMask
from boom_birds_control.sih_guard import verifySihProcess
from boom_birds_sensing.platform_observation import BoardObservation
from boom_birds_control.platform_landing import nedFromFlu

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--config",required=True)
    ap.add_argument("--out",required=True)
    ap.add_argument("--px4-source",default="/home/waterc/PX4-Autopilot")
    ap.add_argument("--scenario",choices=("origin","return","tag_loss","range_jump","handoff_failure"),default="origin")
    ap.add_argument("--observation-input",choices=("image","pose"),default="image")
    ap.add_argument("--allow-simulated-arming",action="store_true")
    args=ap.parse_args()
    if not args.allow_simulated_arming:raise ValueError("explicit simulated arming required")
    out=Path(args.out);out.mkdir(parents=True,exist_ok=False)
    source=Path(args.px4_source)
    binary=source/"build/px4_sitl_default/bin/px4"
    # Refuse a second instance; do not terminate somebody else's simulator.
    for candidate in Path("/proc").iterdir():
        if candidate.name.isdigit():
            try:
                if candidate.joinpath("exe").resolve()==binary.resolve():
                    raise ValueError("existing_px4_instance")
            except (FileNotFoundError,PermissionError):pass
    profile=loadProfile(args.config)
    from boom_birds_control.platform_landing import PlatformLanding
    PlatformLanding(profile,test_only=True)
    from boom_birds_sensing.platform_observation import BoardDetector
    detector=BoardDetector(profile,test_only=True)
    def renderBoard(pose):
        cameraPose=np.linalg.inv(detector.T_B_C)@pose
        rvec=cv2.Rodrigues(cameraPose[:3,:3])[0]
        image=np.full(tuple(reversed(profile["camera"]["image_size"])),255,np.uint8)
        for tag in profile["board"]["tags"]:
            points=detector.tagCorners(tag)
            if np.min((cameraPose[:3,:3]@points.T+cameraPose[:3,3,None]).T[:,2])<=0:continue
            pixels=cv2.projectPoints(points,rvec,cameraPose[:3,3],detector.K,detector.D)[0].reshape(4,2)
            marker=cv2.aruco.drawMarker(detector.dictionary,tag["id"],160)
            homography=cv2.getPerspectiveTransform(np.float32([[0,0],[159,0],[159,159],[0,159]]),np.float32(pixels))
            rendered=cv2.warpPerspective(marker,homography,profile["camera"]["image_size"],borderValue=255)
            image=np.minimum(image,rendered)
        return image
    env=dict(os.environ,PX4_SIM_MODEL="sihsim_quadx",PX4_SIMULATOR="sihsim",PX4_SYS_AUTOSTART="10040")
    log=(out/"px4-console.log").open("w")
    process=subprocess.Popen([str(binary),"-d","-i","0"],cwd=source,env=env,stdout=log,stderr=subprocess.STDOUT)
    backend=None;reader=None
    report=dict(kind="SIH",scenario=args.scenario,result="FAIL",test_only=True,
                synthetic_input="PX4 truth -> synthetic tag image/PnP and synthetic range; no OpenVINS or camera",
                observation_input=args.observation_input,
                config=profile,px4_pid=process.pid,parameter_writes=False,
                source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in [Path(__file__),Path(__file__).parents[1]/"src/boom_birds_control/boom_birds_control/platform_landing.py",
                              Path(__file__).parents[1]/"src/boom_birds_control/boom_birds_control/platform_sih_feedback.py",
                              Path(args.config)]})
    samples=[]
    started=time.monotonic()
    try:
        for _ in range(100):
            if verifySihProcess(process.pid):break
            time.sleep(.05)
        if not verifySihProcess(process.pid):raise ValueError("SIH process verification")
        backend=MavlinkPx4Backend(connection="udpin:127.0.0.1:14540",dry_run=False,
            allow_arming=True,allow_non_loopback=False)
        if not backend.connect():raise ValueError("loopback_connect")
        deadline=time.monotonic()+12.
        while time.monotonic()<deadline:
            if backend.isConnected():
                backend.requestObservationStreams()
                state=backend.readVehicleState()
                if state.position_ned_m is not None and state.yaw_rad is not None:break
            time.sleep(.05)
        else:raise ValueError("telemetry_timeout")
        home=np.array(state.position_ned_m)
        height=profile["guidance"]["takeoff_height_m"]
        goal=home+np.array([0.,0.,-height])
        positionMask=int(TypeMask.forMode("position_velocity"))
        def nav(target):
            sp=Px4LocalSetpoint(tuple(target),(0.,0.,0.),(0.,0.,0.),0.)
            if not backend.sendSetpoint(sp,positionMask):raise ValueError("nav_send_failed")
        for _ in range(75):nav(goal);time.sleep(.02)
        if not backend.setOffboardMode():raise ValueError("offboard_request_rejected")
        for _ in range(30):nav(goal);time.sleep(.02)
        # Wait for PX4's own health check; local command acceptance is not arming.
        from boom_birds_control.platform_sih_feedback import fields
        preflightDeadline=time.monotonic()+12.
        while time.monotonic()<preflightDeadline:
            nav(goal)
            if not verifySihProcess(process.pid):raise ValueError("sih_process_changed")
            health=subprocess.run([str(binary.with_name("px4-listener")),"vehicle_status","-n","1"],
                cwd=binary.parent.parent/"rootfs/0",capture_output=True,text=True,timeout=.4)
            checked,_=fields(health.stdout)
            if health.returncode==0 and checked.get("pre_flight_checks_pass")=="True":
                (out/"preflight.txt").write_text(health.stdout)
                break
            time.sleep(.04)
        else:raise ValueError("preflight_checks_not_ready")
        state=backend.readVehicleState()
        home=np.array(state.position_ned_m)
        goal=home+np.array([0.,0.,-height])
        if not backend.arm(True):raise ValueError("sim_arm_rejected")
        deadline=time.monotonic()+15.
        while time.monotonic()<deadline:
            nav(goal);state=backend.readVehicleState()
            if state.armed and state.is_offboard and np.linalg.norm(np.array(state.position_ned_m)-goal)<.12:break
            time.sleep(.02)
        else:raise ValueError("takeoff_not_reached")
        phases=[("takeoff",tuple(state.position_ned_m))]
        if args.scenario=="return":
            for name,offset in (("outbound",1.),("return",0.)):
                target=goal+np.array([offset,0.,0.])
                deadline=time.monotonic()+15.
                while time.monotonic()<deadline:
                    nav(target);state=backend.readVehicleState()
                    if np.linalg.norm(np.array(state.position_ned_m)-target)<.08:break
                    time.sleep(.02)
                else:raise ValueError(name+"_not_reached")
                for _ in range(30):nav(target);time.sleep(.02)
                phases.append((name,tuple(state.position_ned_m)))
        executor=PlatformExecutor(backend,profile,test_only=True,
            revokeNavigation=lambda:True,release_compute=lambda token:True,
            nativeLand=lambda reason:backend.set_mode("auto:land"),disarm=backend.disarm)
        # No VIO/camera processes run here; compute release is a recorded logical test receipt.
        executor.request()
        reader=SihVelocityFeedback(process.pid,executor.core,out/"feedback")
        landingStarted=time.monotonic()
        deadline=landingStarted+35.
        faultInjected=False
        while time.monotonic()<deadline:
            now=time.monotonic();state=backend.readVehicleState()
            ack,valid=reader.snapshot()
            if state.position_ned_m is None or state.velocity_ned_m_s is None:
                time.sleep(.02);continue
            r=nedFromFlu(state.roll_rad,state.pitch_rad,state.yaw_rad)
            pose=np.eye(4);pose[:3,:3]=r.T@np.diag([1.,-1.,-1.])
            # Platform +x is NED north, +y west, +z up.
            pose[:3,3]=r.T@(home-np.array(state.position_ned_m))
            h=float(home[2]-state.position_ned_m[2])
            observation=BoardObservation(now,"origin_board",True,"synthetic_truth",
                tuple(map(tuple,pose)),tuple(tag["id"] for tag in profile["board"]["tags"]),.1,5.,100.,source="TEST_ONLY_SIH_truth")
            if args.observation_input=="image":
                image=renderBoard(pose)
                observation=detector.observe(image,now,"TEST_ONLY_SIH_rendered_image")
                if len(samples)<3 or not observation.valid and not (out/"first-invalid.png").exists():
                    cv2.imwrite(str(out/(f"frame-{len(samples)}.png" if observation.valid else "first-invalid.png")),image)
            distance=(h-float((r@np.array(profile["range"]["T_body_sensor"])[:3,3])[2]))/float(
                (r@np.array(profile["range"]["T_body_sensor"])[:3,2])[2])
            rs=RangeSample(now,distance)
            if executor.core.confirmed and now-landingStarted>1.:
                if args.scenario=="tag_loss":
                    from dataclasses import replace
                    observation=replace(observation,valid=False,reason="injected_tag_loss");faultInjected=True
                if args.scenario=="range_jump":
                    rs=RangeSample(now,distance+1.);faultInjected=True
            if args.scenario=="handoff_failure":ack=None;faultInjected=True
            fa=max(state.attitude_age_s or 0.,state.position_age_s or 0.)
            flight=FlightSample(now-fa,state.roll_rad,state.pitch_rad,state.yaw_rad,
                state.velocity_ned_m_s,state.connected,state.is_offboard,valid,
                state.landed_state==1 if state.landed_age_s is not None and state.landed_age_s<1.5 else None,
                state.armed,state.restart_epoch+state.frame_reset_epoch)
            output=executor.tick(now,observation,rs,flight,ack,navigationReady=True)
            if executor.owner=="navigation":nav(goal)
            samples.append(dict(now=now,input=asdict(observation),range=asdict(rs),flight=asdict(flight),
                feedback=asdict(ack) if ack else None,output=asdict(output),position_ned=state.position_ned_m))
            if output.request_native_land:
                report["fallback_reason"]=output.reason
            if not state.armed and state.landed_state==1:
                report["landed_disarmed"]=True;break
            time.sleep(.02)
        else:raise ValueError("landing_timeout")
        report.update(confirmed=executor.core.confirmed,compute_release=executor.core.released,
            compute_release_kind="logical_TEST_ONLY_no_processes",phases=phases,
            faultInjected=faultInjected,final_state=asdict(backend.readVehicleState()))
        downward=[s for s in samples if s["output"]["velocity_ned"] and s["output"]["velocity_ned"][2]>0]
        if any(not s["output"]["descent_permitted"] for s in downward):raise ValueError("unauthorized_descent")
        if args.scenario in ("origin","return") and not executor.core.confirmed:raise ValueError("handoff_not_confirmed")
        if args.scenario=="handoff_failure" and executor.core.confirmed:raise ValueError("unexpected_confirmation")
        if args.scenario in ("tag_loss","range_jump") and not faultInjected:raise ValueError("fault_not_injected")
        report["result"]="PASS"
    except Exception as exc:
        report["error"]=str(exc)
    finally:
        if backend is not None:
            state=backend.readVehicleState()
            if state.armed:
                backend.set_mode("auto:land")
                deadline=time.monotonic()+18.
                while time.monotonic()<deadline and backend.readVehicleState().armed:time.sleep(.1)
            report["cleanup_landed_disarmed"]=(not backend.readVehicleState().armed
                and backend.readVehicleState().landed_state==1)
            backend.close()
        if reader:reader.close()
        process.terminate()
        try:process.wait(timeout=4.)
        except subprocess.TimeoutExpired:process.kill();process.wait(timeout=2.)
        log.close()
        report["elapsed_s"]=time.monotonic()-started
        (out/"trace.jsonl").write_text("".join(json.dumps(s,allow_nan=False)+"\n" for s in samples))
        (out/"report.json").write_text(json.dumps(report,indent=2,default=str))
    print(json.dumps({k:v for k,v in report.items() if k not in ("config","final_state","phases")},indent=2))
    return 0 if report["result"]=="PASS" else 1

if __name__=="__main__":raise SystemExit(main())
