#!/usr/bin/env python3
"""TEST-ONLY: file tasks -> ROS controller -> MAVROS -> local PX4 SIH."""
import argparse
from copy import deepcopy
from dataclasses import asdict, replace
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu,Image,Range,CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from boom_birds_control.runtime_config import DEFAULTS
from boom_birds_control.frames import rotToQuat
from boom_birds_control.platform_landing import nedFromFlu
from boom_birds_control.px4_interface_node import Px4InterfaceNode
from boom_birds_control.sih_guard import verifySihProcess
from boom_birds_interfaces.srv import Mission
from boom_birds_bringup.tag_flight_node import TagFlightNode
from boom_birds_bringup.compute_release_node import ComputeReleaseNode
from boom_birds_bringup.compute_supervisor import OwnedProcesses
from boom_birds_control.platform_landing import VELOCITY_YAW_RATE_MASK
from boom_birds_sensing.platform_observation import BoardDetector
from boom_birds_sensing.platform_observation_node import PlatformObservationNode
from tools.sensor_replay.platform_replay import renderBoard

TASKS=("tag_takeoff_land","tag_return_turn","tag_forward_land")
SCENARIOS=("nominal","detour","detour_failure","return_blocked","depth_loss","vio_loss",
           "tag_border_loss","handover_timeout","handover_ack_mismatch","feedback_interruption",
           "tag_temporary_loss","tag_persistent_loss","stale_observation","wrong_tag_id","cancel")
SUCCESS_SCENARIOS=("nominal","detour","tag_temporary_loss")
PRELAND_SCENARIOS=("tag_temporary_loss","tag_persistent_loss","stale_observation")
ROOT=Path(__file__).resolve().parents[1]


def stamp(seconds):
    ns=round(seconds*1e9)
    return Time(sec=ns//1000000000,nanosec=ns%1000000000)


class Room:
    def __init__(self):
        rows,cols=np.indices((120,160))
        self.rays=np.stack(((cols-80.)/100.,(rows-60.)/100.,np.ones_like(cols)),axis=-1)
        self.obstacles=[]

    def render(self, worldCamera, home):
        origin=worldCamera[:3,3]
        rays=self.rays @ worldCamera[:3,:3].T
        depth=np.full(rays.shape[:2],np.inf)
        # Finite room surfaces, including a floor 50 mm below the initial body center.
        for axis,plane in ((0,home[0]-3),(0,home[0]+3),(1,home[1]-3),(1,home[1]+3),
                           (2,home[2]-.05),(2,home[2]+3)):
            den=rays[...,axis]
            t=np.divide(plane-origin[axis],den,out=np.full_like(den,np.inf),where=np.abs(den)>1e-9)
            depth=np.minimum(depth,np.where(t>0,t,np.inf))
        for low,high in self.obstacles:
            with np.errstate(divide="ignore",invalid="ignore"):
                a=(low-origin)/rays;b=(high-origin)/rays
                entry=np.max(np.minimum(a,b),axis=-1)
                leave=np.min(np.maximum(a,b),axis=-1)
            depth=np.minimum(depth,np.where((leave>=entry)&(entry>0),entry,np.inf))
        return depth.astype(np.float32)



class PhysicsTruth:
    def __init__(self,node):
        import struct
        from mavros_msgs.msg import Mavlink
        from rclpy.qos import qos_profile_sensor_data
        self.node=node
        self.latest=None
        self.origin=None
        self.lastUsec=None
        self.count=0
        self.fault=None
        node.create_subscription(Mavlink,"/uas1/mavlink_source",self.onRaw,qos_profile_sensor_data)

    def onRaw(self,msg):
        import struct
        from boom_birds_control.frames import quatToRot
        if msg.msgid!=115 or msg.sysid!=1 or msg.compid!=1 or msg.framing_status!=1:
            return
        data=b"".join(struct.pack("<Q",int(v)) for v in msg.payload64)[:int(msg.len)]
        if not 1<=len(data)<=64:return
        values=struct.unpack("<Q4f3f3i3h2H3h",data.ljust(64,bytes(1)))
        usec=values[0]
        if self.lastUsec is not None and usec<=self.lastUsec:
            if usec<self.lastUsec:self.fault="truth_clock_reset"
            return
        self.lastUsec=usec
        qw,qx,qy,qz=values[1:5]
        if abs(np.linalg.norm([qw,qx,qy,qz])-1.)>.01:return
        lat,lon,alt=values[8:11]
        if self.origin is None:self.origin=(lat,lon,alt)
        lat0,lon0,alt0=self.origin
        north=math.radians((lat-lat0)*1e-7)*6371000.
        east=math.radians((lon-lon0)*1e-7)*6371000.*math.cos(math.radians(lat0*1e-7))
        p=np.array([north,-east,(alt-alt0)*.001])
        v=np.array(values[11:14],float)*[.01,-.01,-.01]
        axis=np.diag([1.,-1.,-1.])
        r=axis @ quatToRot((qx,qy,qz,qw)) @ axis
        rosStamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        age=self.node.get_clock().now().nanoseconds*1e-9-rosStamp
        if not 0<=age<=.1:return
        self.latest=dict(p=p,v=v,r=r,ros=rosStamp,mono=time.monotonic()-age,usec=usec)
        self.count+=1


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--task",choices=TASKS,required=True)
    ap.add_argument("--scenario",choices=SCENARIOS,default="nominal")
    ap.add_argument("--feedback-source",choices=("uorb","mavlink"),default="uorb")
    ap.add_argument("--no-handover",action="store_true",help="legacy VIO landing path (no platform handoff)")
    ap.add_argument("--no-feedback",action="store_true",help="DIAGNOSTIC: disable SIH uORB velocity feedback reader")
    ap.add_argument("--observe-ekf-reference",action="store_true",
        help="TEST-ONLY: observe MAVLink EKF XY and reset status; does not change control")
    ap.add_argument("--sih-ekf-fallback",action="store_true",
        help="TEST-ONLY: connect EKF feedback after feature loss on local SIH")
    ap.add_argument("--sih-range-loss",action="store_true",
        help="TEST-ONLY: suppress range after EKF takeover")
    ap.add_argument("--compute-load",choices=("spin","light","idle"),default="light",
        help="synthetic VIO worker CPU load; light=~30%% of one core, idle=measurement only")
    ap.add_argument("--tag-config",type=Path,default=ROOT/"src/boom_birds_sim/config/platform_landing_70mm_test.yaml")
    ap.add_argument("--out",required=True)
    ap.add_argument("--allow-simulated-arming",action="store_true")
    ap.add_argument("--flight-budget",type=float)
    ap.add_argument("--px4-source",default="/home/waterc/PX4-Autopilot")
    args=ap.parse_args()
    if not args.allow_simulated_arming:ap.error("--allow-simulated-arming required")
    if args.sih_ekf_fallback and (not args.no_handover or not args.observe_ekf_reference
                                  or args.scenario != "tag_border_loss"):
        ap.error("--sih-ekf-fallback requires --no-handover --observe-ekf-reference and tag_border_loss")
    if args.sih_range_loss and not args.sih_ekf_fallback:
        ap.error("--sih-range-loss requires --sih-ekf-fallback")
    out=Path(args.out).resolve();out.mkdir(parents=True,exist_ok=False)
    source=Path(args.px4_source).resolve()
    binary=source/"build/px4_sitl_default/bin/px4"
    # 上一例异常退出可能留下占用 SIH UDP 端口的进程；先显式拒绝，避免静默跑成另一实例。
    def portsInUse():
        inUse={}
        for path in (Path("/proc/net/udp"),Path("/proc/net/udp6")):
            try: lines=path.read_text().splitlines()[1:]
            except OSError: continue
            for line in lines:
                fields=line.split()
                if len(fields)<2: continue
                try: port=int(fields[1].split(":")[1],16)
                except (ValueError,IndexError): continue
                if port in (14540,14580): inUse[port]=fields[1]
        return inUse
    for _ in range(60):
        inUse=portsInUse()
        if not inUse: break
        time.sleep(.1)
    else:
        raise RuntimeError("stale_sih_udp_port:"+json.dumps(inUse))
    for candidate in Path("/proc").iterdir():
        if candidate.name.isdigit():
            try:
                if candidate.joinpath("exe").resolve()==binary:
                    raise RuntimeError("existing_px4_instance")
            except (FileNotFoundError,PermissionError):pass
    handover = not args.no_handover
    report=dict(kind="TAG_TASK_ROS_MAVROS_PX4_SIH",task=args.task,scenario=args.scenario,
        result="FAIL",test_only=True,hardware_connected=False,handover=handover,
        inputs="PX4 SIH HIL_STATE_QUATERNION physical truth -> synthetic body VIO, IMU, raycast depth, rendered AprilTag images and range",
        compute_release="owned synthetic VIO/adapter CPU workers; no OpenVINS or camera",
        openvins_run=False,stereo_matching_run=False,ego_run=False,parameter_writes=False)
    taskFile=ROOT/"src/boom_birds_bringup/config/flight_tasks"/(args.task+".yaml")
    attitudeFile=ROOT/"src/boom_birds_sim/config/tag_attitude_sih.yaml"
    if args.flight_budget is not None:
        profile=yaml.safe_load(attitudeFile.read_text())
        profile["mission"]["flight_budget_s"]=args.flight_budget
        attitudeFile=out/"attitude_sih.yaml"
        attitudeFile.write_text(yaml.safe_dump(profile))
    tagFile=args.tag_config.resolve()
    obstacleFile=ROOT/"src/boom_birds_bringup/config/tag_obstacle_guard.yaml"
    tagProfile=yaml.safe_load(tagFile.read_text())
    detector=BoardDetector(tagProfile,test_only=True)
    report["feedback_source"]=args.feedback_source
    report["configuration"]={k:str(v) for k,v in dict(task=taskFile,attitude=attitudeFile,tag=tagFile,obstacle=obstacleFile).items()}
    watched=[Path(__file__),taskFile,attitudeFile,tagFile,obstacleFile,
        *list((ROOT/"src/boom_birds_bringup/boom_birds_bringup").glob("tag_*.py")),
        *list((ROOT/"src/boom_birds_bringup/boom_birds_bringup").glob("task_*.py")),
        ROOT/"src/boom_birds_control/boom_birds_control/attitude_control.py",
        ROOT/"src/boom_birds_control/boom_birds_control/px4_interface_node.py"]
    report["source_sha256"]={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in watched}
    for package in ("boom_birds_bringup","boom_birds_control","boom_birds_sensing"):
        watched.extend((ROOT/"src"/package/package).glob("*.py"))
    watched.extend([ROOT/"src/boom_birds_bringup/config/mavros.yaml",
                    ROOT/"tools/sensor_replay/platform_replay.py"])
    watched.append(Path(__file__).with_name("sih_feedback_probe.py"))
    if args.sih_ekf_fallback:
        watched.append(ROOT/"src/boom_birds_sim/boom_birds_sim/sih_feature_ekf.py")
    report["environment"] = {key:os.environ.get(key) for key in (
        "RMW_IMPLEMENTATION", "CYCLONEDDS_URI", "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY",
        "FASTDDS_BUILTIN_TRANSPORTS")}
    report["source_sha256"]={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in set(watched)}
    snapshots=out/"inputs";snapshots.mkdir()
    for name,path in report["configuration"].items():
        shutil.copy2(path,snapshots/(name+".yaml"))
    report["px4_binary_sha256"]=hashlib.sha256(binary.read_bytes()).hexdigest()
    processes=[];logs=[];nodes=[];executor=None;control=None;mission=None
    owned=releaseNode=releaseExecutor=releaseThread=decoy=None
    trace=(out/"trace.jsonl").open("w")
    started=time.monotonic()
    ulogs=set(source.rglob("*.ulg"))
    def spawn(command,name,**kwargs):
        log=(out/(name+".log")).open("w");logs.append(log)
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,**kwargs)
        processes.append(process);return process
    env=dict(os.environ,PX4_SIM_MODEL="sihsim_quadx",PX4_SIMULATOR="sihsim",PX4_SYS_AUTOSTART="10040")
    px4=spawn([str(binary),"-d","-i","0"],"px4",cwd=source,env=env)
    try:
        until=time.monotonic()+6
        while not verifySihProcess(px4.pid) and time.monotonic()<until:time.sleep(.05)
        if not verifySihProcess(px4.pid):raise RuntimeError("local_sih_guard")
        spawn(["ros2","run","mavros","mavros_node","--ros-args","--params-file",
               str(ROOT/"src/boom_birds_bringup/config/mavros.yaml"),
               "-p","fcu_url:=udp://127.0.0.1:14540@127.0.0.1:14580",
               "-p",'gcs_url:=""',"-p","tgt_system:=1","-p","tgt_component:=1"],"mavros")
        spawn(["ros2","run","boom_birds_sensing","mavros_config_node"],"mavros-config")
        time.sleep(6.)  # Let the SIH estimator finish its initial resets before constructing control.
        paramsRead=subprocess.run([str(binary.parent/"px4-param"),"show","SIH_*"],
            cwd=source,env=env,capture_output=True,text=True,timeout=3)
        (out/"sih-parameters.txt").write_text(paramsRead.stdout+paramsRead.stderr)
        if paramsRead.returncode:
            raise RuntimeError("sih_parameter_readback_failed")
        # uORB 速度回执由独立探针进程产生；控制进程只读文件，避免 SIH 内轮询子进程
        # 拖慢控制/任务周期（这也是平台 SIH 与 Tag 任务共载时的实测瓶颈）。
        feedbackPath=out/"sih-feedback.json"
        if handover and args.feedback_source == "uorb":
            os.environ["BB_SIH_FEEDBACK_FILE"]=str(feedbackPath)
        rclpy.init()
        # 等 MAVROS 与 PX4 握手完成后再构造控制/任务节点：SIH 连续运行时偶发
        # mavros 启动竞争，过早构造会让后端一直 connected=false 且 HIL 真值不启动。
        from mavros_msgs.msg import State as MavrosState
        bootNode=Node("tag_task_boot_wait")
        bootState={"connected":False}
        bootNode.create_subscription(MavrosState,"/mavros/state",
            lambda msg:bootState.update(connected=bool(msg.connected)),10)
        bootExecutor=SingleThreadedExecutor();bootExecutor.add_node(bootNode)
        bootUntil=time.monotonic()+25.
        while time.monotonic()<bootUntil and not bootState["connected"]:
            bootExecutor.spin_once(timeout_sec=.05)
        bootExecutor.remove_node(bootNode);bootNode.destroy_node()
        if not bootState["connected"]:
            raise RuntimeError("mavros_not_connected_before_node_construction")
        params=dict(control_mode="companion_attitude",attitude_config_file=str(attitudeFile),
            backend="mavros",dry_run=False,allow_arming=True,sih_pid=px4.pid,require_session=True,
            odom_topic=DEFAULTS.odom_body_topic,camera_health_topic=DEFAULTS.stereo_left_topic)
        if handover:
            params.update(platform_config_file=str(tagFile),platform_test_only=True,
                          platform_feedback_source="mavlink_candidate" if args.feedback_source == "mavlink" else "sih_uorb",
                          platform_trace_directory=str(out/"control-trace"))
        control=Px4InterfaceNode(parameter_overrides=[Parameter(k,value=v) for k,v in params.items()])
        mission=TagFlightNode(parameter_overrides=[Parameter(k,value=v) for k,v in dict(
            control_mode="companion_attitude",attitude_config_file=str(attitudeFile),
            flight_task_file=str(taskFile),tag_config_file=str(tagFile),obstacle_config_file=str(obstacleFile),
            dry_run=False,tag_test_only=True,sih_pid=px4.pid,
            platform_landing_enabled=handover,handover_enabled=handover,
            feature_tracking_enabled=not handover,
            platform_status_timeout_s=.5).items()])
        observer=PlatformObservationNode(parameter_overrides=[Parameter("config_file",value=str(tagFile)),
                                                            Parameter("test_only",value=True),Parameter("feature_tracking_enabled",value=not handover)])
        owned=releaseNode=releaseExecutor=releaseThread=None
        decoy=None
        if handover:
            # 任务自有的"VIO/适配器"计算进程：一个忙等占满单核，一个空闲；释放后必须退出。
            loads=dict(spin="while True: pass",
                       light="import time\nwhile True:\n    time.sleep(.14)\n    t=time.perf_counter()\n    while time.perf_counter()-t<.06:\n        pass",
                       idle="import time;time.sleep(600)")
            owned=OwnedProcesses([dict(name="vio-synthetic",command=[sys.executable,"-c",loads[args.compute_load]]),
                                  dict(name="adapter-synthetic",command=[sys.executable,"-c","import time;time.sleep(600)"])],
                                 logDirectory=out/"compute-logs")
            releaseNode=ComputeReleaseNode(owned.children(),[],lambda: mission.fsm.session or "",
                                           stopOwned=owned.stopAll)
            releaseExecutor=SingleThreadedExecutor();releaseExecutor.add_node(releaseNode)
            releaseThread=threading.Thread(target=releaseExecutor.spin,daemon=True);releaseThread.start()
            # 非本任务进程：清理必须按自有句柄执行，不能误停无关计算。
            decoy=subprocess.Popen([sys.executable,"-c","import time;time.sleep(600)"],start_new_session=True)
        if args.no_feedback and control.platform is not None and control.platform.feedbackReader is not None:
            control.platform.feedbackReader.close()
            control.platform.feedbackReader=None
        harness=Node("tag_task_sih_harness")
        ekfReader = None
        ekfStats = dict(polls=0, usable=0, airborne_polls=0, airborne_usable=0, reasons={})
        if args.observe_ekf_reference:
            from boom_birds_control.mavros_ekf_reference import MavrosEkfReference
            control.backend.ekf_reference_required = True
            # Reapply intervals if the backend finished its initial request before
            # this explicitly enabled observer was created. Local SIH only.
            control.backend._observationStreamsRequested = False
            ekfReader = MavrosEkfReference(harness,control.backend)
        ekfFallback = None
        if args.sih_ekf_fallback:
            from boom_birds_sim.sih_feature_ekf import SihFeatureEkf
            ekfFallback = SihFeatureEkf(mission,control,ekfReader,px4.pid,
                                           injectRangeLoss=args.sih_range_loss)
        nodes=[control,mission,observer,harness]
        executor=SingleThreadedExecutor()
        for node in nodes:executor.add_node(node)
        bodyPub=harness.create_publisher(Odometry,DEFAULTS.odom_body_topic,10)
        egoPub=harness.create_publisher(Odometry,DEFAULTS.odom_topic,10)
        imuPub=harness.create_publisher(Imu,DEFAULTS.imu_topic,10)
        cameraPub=harness.create_publisher(PoseStamped,DEFAULTS.camera_pose_topic,10)
        depthPub=harness.create_publisher(Image,DEFAULTS.depth_topic,10)
        infoPub=harness.create_publisher(CameraInfo,DEFAULTS.camera_info_topic,10)
        rawPub=harness.create_publisher(Image,DEFAULTS.stereo_left_topic,10)
        downPub=harness.create_publisher(Image,"/boom_birds/downward/image",10)
        rangePub=harness.create_publisher(Range,"/boom_birds/platform/range",10)
        from mavros_msgs.msg import AttitudeTarget, PositionTarget
        from rclpy.qos import qos_profile_sensor_data
        wire=dict(attitude=0,position=0,platform_velocity=0,violations=0)
        platformRows=[]
        def wireCapture(msg,kind):
            wire[kind]+=1
            age=harness.get_clock().now().nanoseconds*1e-9-(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
            produced=time.monotonic()-max(0.,age)
            core=control.platform.executor.core if control.platform else None
            if core is not None and core.pendingSince is not None and produced>=core.pendingSince:
                if kind=="attitude" or getattr(msg,"type_mask",None)!=VELOCITY_YAW_RATE_MASK:
                    wire["violations"]+=1
                else:
                    wire["platform_velocity"]+=1
            trace.write(json.dumps(dict(kind="mavros_"+kind,now=time.monotonic(),
                mask=getattr(msg,"type_mask",None),
                owner=None if control.platform is None else control.platform.executor.owner))+"\n")
        harness.create_subscription(AttitudeTarget,"/mavros/setpoint_raw/attitude",
                                    lambda m:wireCapture(m,"attitude"),qos_profile_sensor_data)
        harness.create_subscription(PositionTarget,"/mavros/setpoint_raw/local",
                                    lambda m:wireCapture(m,"position"),qos_profile_sensor_data)
        def onPlatformStatus(msg):
            try:
                data=json.loads(msg.data)
                platformRows.append(data)
                trace.write(json.dumps(dict(kind="platform",now=time.monotonic(),data=data))+"\n")
                if data.get("velocity_ned") and data["velocity_ned"][2]>0 and not data.get("descent_permitted"):
                    raise RuntimeError("unauthorized_descent")
                if (not data.get("confirmed") and owned is not None
                        and not all(process.alive() for process in owned.processes)):
                    raise RuntimeError("premature_compute_release")
            except (ValueError,KeyError):
                pass
        harness.create_subscription(String,"/boom_birds/platform/status",onPlatformStatus,10)
        def capture(msg,kind):
            data=json.loads(msg.data)
            trace.write(json.dumps(dict(kind=kind,now=time.monotonic(),data=data))+"\n")
        harness.create_subscription(String,DEFAULTS.control_status_topic,lambda m:capture(m,"control"),10)
        harness.create_subscription(String,"/boom_birds/tag_task/status",lambda m:capture(m,"task"),10)
        harness.create_subscription(String,"/boom_birds/platform/observation",lambda m:capture(m,"tag"),10)
        harness.create_subscription(String,"/boom_birds/platform/feature_observation",lambda m:capture(m,"feature"),10)
        wrongProfile=deepcopy(tagProfile);wrongProfile["board"]["tags"][0]["id"]=5
        wrongDetector=BoardDetector(wrongProfile,test_only=True)
        bridge=CvBridge();room=Room();physics=PhysicsTruth(harness)
        physicsRequested=False;physicsFuture=None;physicsAttempt=-math.inf
        home=None;board=None;lastSource=-math.inf;lastImage=-math.inf;startAt=None
        rows=[];phases=[]
        # 错误 ID 场景从第一帧就是注入条件（渲染 ID5，解码器拒绝为身份不符）。
        injected=args.scenario=="wrong_tag_id"
        detourBlock=False;returnBlock=False;originLatched=False
        collision=False;minimumClearance=math.inf
        vioReleased=False;handoverRequested=False;altitudeAtRequest=None;blankUntil=None
        cancelSent=False;feedbackPatched=False;downSuppressed=0
        optical=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
        def pump():
            nonlocal home,board,lastSource,lastImage,injected,detourBlock,returnBlock,originLatched,physicsRequested,physicsFuture,physicsAttempt,collision,minimumClearance
            nonlocal vioReleased,handoverRequested,altitudeAtRequest,blankUntil,cancelSent,feedbackPatched,downSuppressed
            if not verifySihProcess(px4.pid):raise RuntimeError("sih_process_changed")
            state=control.backend.readVehicleState()
            now=time.monotonic();rosNow=harness.get_clock().now().nanoseconds*1e-9
            if ekfReader is not None and now-getattr(pump,"ekfAt",0.)>=.05:
                pump.ekfAt=now
                ekf=ekfReader.snapshot()
                ekfStats["polls"]+=1
                ekfStats["usable"]+=int(ekf is not None)
                ekfStats["airborne_polls"]+=int(state.armed)
                ekfStats["airborne_usable"]+=int(state.armed and ekf is not None)
                ekfStats["reasons"][ekfReader.reason]=ekfStats["reasons"].get(ekfReader.reason,0)+1
                trace.write(json.dumps(dict(kind="ekf_reference_observation",now=now,armed=state.armed,
                    reason=ekfReader.reason,
                    message_ages_s={str(mid): now-item[0] for mid,item in ekfReader.samples.items()},
                    status_flags=None if 230 not in ekfReader.samples else int(ekfReader.samples[230][1].flags),
                    sample=None if ekf is None else asdict(ekf[0]),
                    velocity=None if ekf is None else ekf[1]))+chr(10))
            if physics.fault:raise RuntimeError(physics.fault)
            if owned is not None:
                owned.sample(now)
                if not vioReleased and control.platform.executor.release_verified:
                    vioReleased=True
                    trace.write(json.dumps(dict(kind="vio_release",now=now,
                        cpu_before_percent=owned.cpuBefore,
                        snapshot=owned.snapshot(now)))+"\n")
            if control.platform is not None:
                core=control.platform.executor.core
                reader=control.platform.feedbackReader
                if reader is not None and now-getattr(pump,"feedbackAt",0.)>=.05:
                    pump.feedbackAt=now
                    feedback, estimatorValid=reader.snapshot()
                    trace.write(json.dumps(dict(kind="feedback",now=now,
                        ack=None if feedback is None else asdict(feedback),
                        estimator_valid=estimatorValid,
                        diagnostic=getattr(reader,"diagnostic",None)))+chr(10))
                acquireKey=(core.state,core.good,control.platform.executor.revoked)
                if acquireKey!=getattr(pump,"acquireKey",None):
                    pump.acquireKey=acquireKey
                    trace.write(json.dumps(dict(kind="acquire",now=now,state=core.state,good=core.good,
                        revoked=control.platform.executor.revoked,reason=core.reason,
                        lastObservationStamp=core.lastObservationStamp,
                        lastRangeStamp=core.lastRangeStamp))+chr(10))
                if control.platform.executor.release_verified:
                    owned.finalizeRelease()
                if now-getattr(pump,"releaseProbeAt",0.)>=1.:
                    pump.releaseProbeAt=now
                    trace.write(json.dumps(dict(kind="release_probe",now=now,
                        confirmed=core.confirmed,releaseState=control.platform.executor.release_state,
                        attempts=control.platform.executor.release_attempts,
                        serviceReady=control.platform.releaseService.service_is_ready(),
                        services=[name for name,_ in harness.get_service_names_and_types()
                                  if "compute" in name]))+chr(10))
                if core.pendingSince is not None and not handoverRequested:
                    handoverRequested=True
                    altitudeAtRequest=physics.latest["p"][2] if physics.latest else None
                if not feedbackPatched and args.scenario in ("handover_timeout","handover_ack_mismatch","feedback_interruption"):
                    reader=control.platform.feedbackReader
                    if reader is not None:
                        original=reader.snapshot
                        def patched(original=original,scenario=args.scenario,core=core):
                            ack,valid=original()
                            if scenario=="handover_timeout":
                                return None,valid
                            if scenario=="feedback_interruption":
                                return (None,valid) if core.confirmed else (ack,valid)
                            if ack is not None:
                                return replace(ack,echoed_velocity_ned=tuple(v+.5 for v in ack.echoed_velocity_ned)),valid
                            return ack,valid
                        reader.snapshot=patched
                        feedbackPatched=True
                        injected=True
            if physicsFuture is not None and physicsFuture.done():
                physicsRequested=physicsFuture.result().success
                physicsFuture=None
            # HIL 真值流请求只需要已连接的 MAVROS 服务；不再等 LOCAL_POSITION_NED，
            # 因为 PX4 SIH 在未收到 HIL 时可能没有位置输出，会与请求形成死锁。
            if (not physicsRequested and physicsFuture is None and now-physicsAttempt>1.
                    and state.connected
                    and control.backend._commandClient.service_is_ready()):
                from mavros_msgs.srv import CommandLong
                physicsFuture=control.backend._commandClient.call_async(CommandLong.Request(command=511,param1=115.,param2=20000.))
                physicsAttempt=now
            sample=physics.latest
            valid=(sample is not None and 0<=now-sample["mono"]<.1
                   and state.connected and state.attitude_age_s is not None and state.attitude_age_s<.1)
            if valid:
                observed=sample["mono"]
                if observed>lastSource+.005:
                    lastSource=observed
                    sampleRos=sample["ros"]
                    p,v,r=sample["p"],sample["v"],sample["r"]
                    q=rotToQuat(r)
                    if home is None and not state.armed and control.attitude_controller.ready:
                        home=p.copy()
                        yaw=math.atan2(r[1,0],r[0,0])
                        board=home+np.array([math.cos(yaw),math.sin(yaw),0.])*(args.task=="tag_forward_land")
                        board[2]-=.05
                    if not originLatched and mission.fsm.takeoffOrigin is not None and not state.armed:
                        home=np.asarray(mission.fsm.takeoffOrigin)
                        yaw=mission.fsm.initialYaw
                        board=home+np.array([math.cos(yaw),math.sin(yaw),0.])*(args.task=="tag_forward_land")
                        board[2]-=.05
                        originLatched=True
                    odom=Odometry();odom.header.stamp=stamp(sampleRos);odom.header.frame_id=DEFAULTS.world_frame
                    odom.child_frame_id=DEFAULTS.body_frame
                    odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=map(float,p)
                    o=odom.pose.pose.orientation
                    o.x,o.y,o.z,o.w=map(float,q)
                    bv=r.T @ v
                    odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z=map(float,bv)
                    # VIO 释放后停止一切 VIO 派生输入：odom、世界系速度、camera_pose。
                    lost=(args.scenario=="vio_loss" and injected) or vioReleased
                    if not lost:
                        bodyPub.publish(odom)
                        ego=deepcopy(odom)
                        ego.twist.twist.linear.x,ego.twist.twist.linear.y,ego.twist.twist.linear.z=map(float,v)
                        egoPub.publish(ego)
                    imu=Imu();imu.header=odom.header;imu.orientation=o;imu.linear_acceleration_covariance[0]=-1.
                    imuPub.publish(imu)
                    raw=bridge.cv2_to_imgmsg(np.zeros((8,8),np.uint8),encoding="mono8")
                    raw.header=odom.header;rawPub.publish(raw)
                    worldCamera=np.eye(4);worldCamera[:3,:3]=r@optical;worldCamera[:3,3]=p
                    cam=PoseStamped();cam.header=odom.header;cam.pose.position=odom.pose.pose.position
                    cq=rotToQuat(worldCamera[:3,:3])
                    cam.pose.orientation.x,cam.pose.orientation.y,cam.pose.orientation.z,cam.pose.orientation.w=map(float,cq)
                    if not lost:cameraPub.publish(cam)
                    a=mission.fsm.avoidance
                    if home is not None and startAt is not None:
                        if not injected and args.scenario not in ("nominal","tag_border_loss") and mission.fsm.phase=="MOVE" and math.dist(p[:2],home[:2])>.05:
                            injected=True
                            if args.scenario in ("detour","detour_failure","return_blocked"):
                                d=np.array([math.cos(mission.fsm.initialYaw),math.sin(mission.fsm.initialYaw),0.])
                                center=home+d*.55+np.array([0.,0.,.6])
                                room.obstacles.append((center-[.06,.07,.25],center+[.06,.07,.25]))
                        if args.scenario in ("detour_failure","return_blocked") and a.purpose=="DETOUR" and a.state=="OBSERVE" and not detourBlock:
                            center=p+r[:,0]*.5
                            room.obstacles.append((center-[.06,.06,.2],center+[.06,.06,.2]));detourBlock=True
                        if args.scenario=="return_blocked" and a.purpose=="RETURN" and a.state=="OBSERVE" and not returnBlock:
                            center=p+r[:,0]*.5
                            room.obstacles.append((center-[.06,.06,.2],center+[.06,.06,.2]));returnBlock=True
                    if home is not None and now-lastImage>=.048:
                        lastImage=now
                        if not(args.scenario=="depth_loss" and injected):
                            depth=room.render(worldCamera,home)
                            dm=bridge.cv2_to_imgmsg(depth,encoding="32FC1");dm.header=odom.header;dm.header.frame_id=DEFAULTS.camera_frame
                            info=CameraInfo();info.header=dm.header;info.width=160;info.height=120
                            info.k=[100.,0.,80.,0.,100.,60.,0.,0.,1.]
                            info.p=[100.,0.,80.,0.,0.,100.,60.,0.,0.,0.,1.,0.]
                            infoPub.publish(info);depthPub.publish(dm)
                        pose=np.eye(4);pose[:3,:3]=r.T;pose[:3,3]=r.T@(board-p)
                        image=renderBoard(wrongDetector if args.scenario=="wrong_tag_id" else detector,pose)
                        handoffConfirmed=(control.platform is not None
                                          and control.platform.executor.core.confirmed)
                        if handoffConfirmed and args.scenario in ("tag_temporary_loss","tag_persistent_loss"):
                            injected=True
                            if blankUntil is None and args.scenario=="tag_temporary_loss":
                                blankUntil=now+.35
                            if args.scenario=="tag_persistent_loss" or now<blankUntil:
                                image=np.full_like(image,255)
                        if args.scenario=="tag_border_loss" and mission.fsm.phase=="TAG_DESCEND" and p[2]-board[2]<.45:
                            import cv2
                            cameraPose=np.linalg.inv(detector.T_B_C)@pose
                            tag=detector.board["tags"][0]
                            pixels=cv2.projectPoints(detector.tagCorners(tag),
                                cv2.Rodrigues(cameraPose[:3,:3])[0],cameraPose[:3,3],
                                detector.K,detector.D)[0].reshape(-1,2)
                            center=pixels.mean(axis=0)
                            inner=center+(pixels-center)*.8
                            mask=np.zeros_like(image)
                            cv2.fillConvexPoly(mask,inner.astype(np.int32),255)
                            image=np.where(mask,image,255).astype(np.uint8)
                            injected=True
                        staleImage=(handoffConfirmed and args.scenario=="stale_observation")
                        if staleImage:
                            downSuppressed+=1
                            injected=True
                        else:
                            down=bridge.cv2_to_imgmsg(image,encoding="mono8");down.header=odom.header;down.header.frame_id="downward_optical"
                            downPub.publish(down)
                        rangeMount=np.array(tagProfile["range"]["T_body_sensor"],float)
                        ray=r@rangeMount[:3,2]
                        rangeOrigin=p+r@rangeMount[:3,3]
                        distance=(home[2]-.05-rangeOrigin[2])/ray[2]
                        ranging=Range();ranging.header=odom.header;ranging.header.frame_id="range_sensor"
                        ranging.min_range,ranging.max_range=.02,5.;ranging.range=float(distance)
                        rangePub.publish(ranging)
                    if startAt is not None:
                        # Separating-axis test: rotated body envelope against each static box.
                        bodyHalf=np.array([.15,.2,.18])
                        for low,high in room.obstacles:
                            center=(low+high)/2;half=(high-low)/2
                            axes=[*np.eye(3),*r.T]
                            axes.extend(np.cross(a,b) for a in np.eye(3) for b in r.T)
                            gaps=[]
                            for axis in axes:
                                norm=np.linalg.norm(axis)
                                if norm < 1e-8:continue
                                axis=axis/norm
                                gaps.append(abs(float((center-p)@axis))
                                    -float(half@np.abs(axis))-float(bodyHalf@np.abs(r.T@axis)))
                            clearance=max(0.,max(gaps))
                            minimumClearance=min(minimumClearance,clearance)
                            collision=collision or max(gaps)<=0
                        row=dict(t=now-startAt,position=p.tolist(),velocity=v.tolist(),yaw=math.atan2(r[1,0],r[0,0]),
                                 state=mission.fsm.state.value,phase=mission.fsm.phase,
                                 landing_source=None if mission.fsm.fix is None else mission.fsm.fix.source,
                                 feature_reason=mission.featureFix.reason,
                                 avoidance=a.state,purpose=a.purpose,armed=state.armed,landed=state.landed_state,
                                 obstacle=mission.fsm.obstacle.state,obstacle_reason=mission.fsm.obstacle.reason,
                                 confirmed=bool(control.platform and control.platform.executor.core.confirmed),
                                 platform_state=None if control.platform is None else control.platform.executor.core.state,
                                 owner=None if control.platform is None else control.platform.executor.owner,
                                 release_verified=bool(control.platform and control.platform.executor.release_verified),
                                 handover_requested=handoverRequested,
                                 acquire_good=None if control.platform is None else control.platform.executor.core.good,
                                 navigation_revoked=None if control.platform is None else control.platform.executor.revoked)
                        rows.append(row);trace.write(json.dumps(dict(kind="truth",**row))+"\n")
                        if not phases or phases[-1]["phase"]!=row["phase"]:phases.append(row)
            until=time.monotonic()+.006
            while time.monotonic()<until:executor.spin_once(timeout_sec=.001)
            return control.backend.readVehicleState()
        deadline=time.monotonic()+35.
        while time.monotonic()<deadline:
            state=pump()
            if (home is not None and control.attitude_controller.ready and mission.fsm.obstacleClear(time.monotonic())
                    and state.landed_state==1 and state.connected and mission.fsm.landingInputsReady):break
        else:
            state=control.backend.readVehicleState()
            raise RuntimeError("sensors_not_ready:"+json.dumps(dict(
                connected=state.connected,landed=state.landed_state,armed=state.armed,
                attitudeReady=bool(control.attitude_controller.ready),
                landingInputs=bool(mission.fsm.landingInputsReady),
                obstacle=mission.fsm.obstacle.state,obstacleReason=mission.fsm.obstacle.reason,
                home=home is not None,landingInputReason=mission.inputReason,
                physics=physicsRequested,commandService=control.backend._commandClient.service_is_ready())))
        # 解锁前等待 PX4 预检通过：SIH 复位后 EKF/罗盘就绪需要时间，过早请求解锁会被拒绝。
        preflightUntil=time.monotonic()+20.
        while time.monotonic()<preflightUntil:
            pump()
            try:
                statusRun=subprocess.run([str(binary.with_name("px4-listener")),"vehicle_status","-n","1"],
                    cwd=binary.parent.parent/"rootfs/0",capture_output=True,text=True,timeout=.4)
                if statusRun.returncode==0 and "pre_flight_checks_pass: True" in statusRun.stdout:
                    break
            except (OSError,subprocess.TimeoutExpired):
                pass
        else:
            trace.write(json.dumps(dict(kind="preflight_timeout",now=time.monotonic()))+chr(10))
        # 探针只在飞控/反馈链路已就绪后启动，避免其 uORB 轮询影响 SIH 启动与 MAVROS 握手。
        if handover and args.feedback_source == "uorb":
            spawn([sys.executable,str(Path(__file__).with_name("sih_feedback_probe.py")),
                   "--pid",str(px4.pid),"--out",str(feedbackPath)],"sih-feedback-probe")
        client=harness.create_client(Mission,DEFAULTS.mission_service)
        future=client.call_async(Mission.Request(action=Mission.Request.START))
        until=time.monotonic()+5
        while not future.done() and time.monotonic()<until:pump()
        if not future.done() or not future.result().accepted:
            reason="" if not future.done() else str(future.result().reason)
            # 输入链/服务发现的瞬时抖动：短间隔重试，仍拒绝才失败。
            retried=False
            for _ in range(3):
                for _ in range(50):pump()
                future=client.call_async(Mission.Request(action=Mission.Request.START))
                until2=time.monotonic()+3
                while not future.done() and time.monotonic()<until2:pump()
                if future.done() and future.result().accepted:
                    retried=True;break
                reason="" if not future.done() else str(future.result().reason)
            if not retried:
                raise RuntimeError("start_rejected:"+reason)
            trace.write(json.dumps(dict(kind="start_retried",now=time.monotonic()))+chr(10))
        startAt=time.monotonic()
        trace.write(json.dumps(dict(kind="start",now=startAt))+"\n")
        until=startAt+mission.fsm.mission.flight_budget_s+20.
        armedSeen=False
        while time.monotonic()<until:
            state=pump();armedSeen=armedSeen or state.armed
            if (args.scenario=="cancel" and not cancelSent and armedSeen
                    and time.monotonic()-startAt>2.5):
                cancelRequest=Mission.Request();cancelRequest.action=Mission.Request.CANCEL
                cancelFuture=client.call_async(cancelRequest);cancelSent=True
                trace.write(json.dumps(dict(kind="cancel_request",now=time.monotonic()))+"\n")
            if armedSeen and not state.armed and state.landed_state==1:
                for _ in range(15):pump()
                break
            if mission.fsm.state.value=="FAULT_LATCHED" and not armedSeen:break
        else:raise RuntimeError("flight_timeout")
        final=control.backend.readVehicleState()
        report.update(final_state=asdict(final),phases=phases,armed_seen=armedSeen,
                      mission_state=mission.fsm.state.value,mission_reason=mission.fsm.reason,
                      detour_used=mission.fsm.avoidance.used,reached_home=mission.fsm.reachedHome,
                      mission_aborted=mission.fsm.obstacleMissionFailed or mission.fsm.avoidance.aborted,
                      fault_injected=injected,home_world=home.tolist(),board_world=board.tolist(),
                      physics_truth_samples=physics.count,physics_stream_accepted=physicsRequested)
        core=control.platform.executor.core if control.platform else None
        gate=mission.platform_gate
        report["handover_report"]=dict(enabled=handover,requested=handoverRequested,
            state=None if core is None else core.state,reason=None if core is None else core.reason,
            confirmed=bool(core is not None and core.confirmed),
            released=bool(core is not None and core.released),
            yaw_offset_rad=None if core is None else core.yawOffset,
            release_state=None if control.platform is None else control.platform.executor.release_state,
            release_verified=bool(control.platform is not None and control.platform.executor.release_verified),
            wire=wire,platform_rows=len(platformRows),
            compute=None if owned is None else owned.snapshot(),
            decoy_alive=None if decoy is None else decoy.poll() is None,
            vio_inputs_stopped=vioReleased,image_suppressed=downSuppressed,
            handover_altitude_m=altitudeAtRequest,
            gate_failed=bool(gate.failed),gate_failure=gate.failure,
            gate_evidence=gate.evidence)
        if altitudeAtRequest is not None:
            # 只统计交接阶段（ACQUIRE/PREPARE）且尚未确认的高度；失败后
            # 原生 Land 的下降不能算作"确认前下降"。
            beforeConfirm=[row["position"][2] for row in rows
                           if row["handover_requested"] and not row["confirmed"]
                           and row.get("platform_state") in ("ACQUIRE", "PREPARE")]
            # 只统计向下的高度损失；确认前的轻微爬升/保持漂移不算"提前下降"。
            report["handover_report"]["descent_before_confirm_m"]=(
                0. if not beforeConfirm else max(0., max(altitudeAtRequest-z for z in beforeConfirm)))
        report["collision_detected"]=collision
        report["minimum_obstacle_clearance_m"]=minimumClearance if math.isfinite(minimumClearance) else None
        flightStart=mission.fsm.flightStarted if mission.fsm.flightStarted is not None else startAt
        report["mission_elapsed_s"]=(mission.fsm.entered if mission.fsm.state.value=="COMPLETE"
            else time.monotonic())-flightStart
        report["detour_block_injected"]=detourBlock
        report["return_block_injected"]=returnBlock
        report["obstacles"]=[dict(low=low.tolist(),high=high.tolist()) for low,high in room.obstacles]
        report["command"]=[sys.executable,str(Path(__file__).resolve()),*sys.argv[1:]]
        report["compute_load"]=args.compute_load
        report["thresholds"]=dict(landing_xy_error_m=.12,descent_before_confirm_m=.06,
            handover_confirm_s=f"<{mission.fsm.task.get('tag_acquire_timeout_s','')}",
            release_requires="release_verified and all owned children exited",
            final="PX4 landed and disarmed",wire_violations=0,
            task2_outbound_m=[.9,1.15],task2_final_yaw_error_rad=.15)
        returning=[i for i,row in enumerate(rows) if row["purpose"]=="RETURN" and row["state"]=="EXECUTING"]
        if returning:
            flown=np.asarray([row["position"] for row in rows[:returning[0]+1]])
            a,b=flown[:-1],flown[1:]
            d=b-a;norm=np.einsum("ij,ij->i",d,d)
            deviation=0.
            for index in returning:
                point=np.asarray(rows[index]["position"])
                ratio=np.divide(np.einsum("ij,ij->i",point-a,d),norm,
                    out=np.zeros_like(norm),where=norm>1e-12).clip(0.,1.)
                deviation=max(deviation,float(np.linalg.norm(point-a-ratio[:,None]*d,axis=1).min()))
            report["return_trace_deviation_m"]=deviation
            if deviation>.12:raise RuntimeError("return_departed_flown_trace")

        if collision:raise RuntimeError("inflated_body_collision")
        if not armedSeen:raise RuntimeError("never_armed:"+mission.fsm.reason)
        if final.armed or final.landed_state!=1:raise RuntimeError("not_landed_disarmed")
        end=np.asarray(rows[-1]["position"])
        target=home if args.task!="tag_forward_land" or report["mission_aborted"] else board
        error=float(np.linalg.norm(end[:2]-target[:2]))
        report["landing_xy_error_m"]=error
        report["feature_target_frames"]=sum(row["landing_source"]=="features" for row in rows)
        report["resolved_tag_frames"]=sum(row["landing_source"]=="resolved_tag" for row in rows)
        hr=report["handover_report"]
        success=args.scenario in SUCCESS_SCENARIOS
        if args.scenario=="tag_border_loss":
            # 该场景只覆盖交接关闭时的 LK/PnP 末段续跟，不用于本轮交接验收。
            if handover:raise RuntimeError("tag_border_loss_requires_legacy_path")
            if mission.fsm.state.value!="COMPLETE" or report["mission_aborted"]:
                raise RuntimeError("task_not_complete:"+mission.fsm.reason)
            if error>.12:raise RuntimeError("landing_xy_error")
            if not injected or report["feature_target_frames"]<10:
                raise RuntimeError("cropped_tag_continuation_not_exercised")
            if args.sih_ekf_fallback and (ekfFallback.switches != 1
                    or ekfFallback.ekfTicks < 2 or ekfFallback.outputs < 2):
                raise RuntimeError("ekf_fallback_not_exercised")
            if args.sih_range_loss and (not ekfFallback.rangeLossInjected
                    or ekfFallback.rangeLossOutputs < 2):
                raise RuntimeError("ekf_height_fallback_not_exercised")
        elif success:
            if not handover:raise RuntimeError("handover_required_for_success_scenario")
            if not handoverRequested:raise RuntimeError("handover_not_requested")
            if not hr["confirmed"]:raise RuntimeError("handoff_not_confirmed:"+str(hr["reason"]))
            if not hr["release_verified"] or not hr["released"]:
                raise RuntimeError("compute_release_not_verified:"+str(hr["release_state"]))
            if not vioReleased:raise RuntimeError("vio_inputs_not_stopped")
            if owned is not None and not owned.snapshot()["all_stopped"]:
                raise RuntimeError("owned_compute_still_running")
            if wire["violations"]:raise RuntimeError("wire_output_conflict")
            if not wire["platform_velocity"]:raise RuntimeError("platform_velocity_missing")
            if decoy is not None and decoy.poll() is not None:
                raise RuntimeError("unrelated_process_stopped")
            if mission.fsm.state.value!="COMPLETE" or gate.failed:
                raise RuntimeError("task_not_complete:"+str(gate.failure))
            if report["mission_aborted"]:raise RuntimeError("task_aborted")
            if error>.12:raise RuntimeError("landing_xy_error")
            if hr.get("descent_before_confirm_m",0.)>.06:
                raise RuntimeError("descent_before_handoff_confirmed")
            if args.scenario=="detour" and not report["detour_used"]:
                raise RuntimeError("detour_not_exercised")
        else:
            if handover and args.scenario in ("handover_timeout","handover_ack_mismatch"):
                if not handoverRequested:raise RuntimeError("handover_not_requested")
                if hr["confirmed"] or hr["released"] or hr["release_verified"]:
                    raise RuntimeError("failed_handoff_released_compute_or_descent")
                if hr.get("descent_before_confirm_m",0.)>.06:
                    raise RuntimeError("descent_before_handoff_confirmed")
                if owned is not None and not all(p.alive() for p in owned.processes):
                    raise RuntimeError("compute_stopped_without_confirmed_handoff")
            if handover and args.scenario in ("feedback_interruption","tag_persistent_loss","stale_observation"):
                if not hr["confirmed"]:raise RuntimeError("handoff_not_confirmed")
                if hr["released"] and not hr["release_verified"]:
                    raise RuntimeError("released_without_exit_readback")
                if hr["released"] and not vioReleased:raise RuntimeError("vio_inputs_not_stopped")
            if handover and args.scenario=="wrong_tag_id":
                if handoverRequested:raise RuntimeError("wrong_id_requested_handover")
                if owned is not None and owned.releaseVerified:
                    raise RuntimeError("wrong_id_released_compute")
            if args.scenario=="cancel" and not cancelSent:raise RuntimeError("cancel_not_injected")
            if not injected and args.scenario!="cancel":raise RuntimeError("fault_not_injected")
            if (not report["mission_aborted"] and mission.fsm.state.value!="FAULT_LATCHED"
                    and not gate.failed):
                raise RuntimeError("fault_did_not_abort")
            if args.scenario=="detour_failure":
                if not mission.fsm.reachedHome or error>.12:
                    raise RuntimeError("failed_detour_did_not_return_home")
        if decoy is not None and decoy.poll() is not None:
            raise RuntimeError("unrelated_process_stopped_at_end")
        if args.task=="tag_return_turn" and args.scenario=="nominal":
            yaw0=mission.fsm.initialYaw
            delta=np.array([math.cos(yaw0),math.sin(yaw0)])
            far=max(float((np.array(row["position"][:2])-home[:2])@delta) for row in rows)
            report["outbound_distance_m"]=far
            report["final_yaw_error_rad"]=abs(math.atan2(math.sin(rows[-1]["yaw"]-yaw0-math.pi),math.cos(rows[-1]["yaw"]-yaw0-math.pi)))
            if not .9<=far<=1.15 or report["final_yaw_error_rad"]>.15:raise RuntimeError("outbound_or_180_turn_failed")
        report["result"]="PASS"
    except Exception as exc:
        report["error"]=str(exc)
        if mission:
            report["mission_diagnostic"]=dict(state=mission.fsm.state.value,phase=mission.fsm.phase,
                reason=mission.fsm.reason,depth_reason=mission.fsm.obstacle.reason,tag_reason=mission.tracker.reason,
                last_depth_rejection=mission.depthBridge.guard.lastRejected)
        if "physics" in locals():
            report["physics_diagnostic"]=dict(samples=physics.count,fault=physics.fault,stream_accepted=physicsRequested)
        if control:
            report["controller_diagnostic"]=dict(error=control._attitudeFeedbackError,
                latched=control.attitude_controller.latched,counters=dict(control._attitudeFeedbackCounters),
                backend=control.backend.streamDiagnostics(),outcome=vars(control._lastOutcome) if control._lastOutcome else None)
    finally:
        if args.sih_ekf_fallback and "ekfFallback" in locals() and ekfFallback is not None:
            report["sih_ekf_fallback"] = ekfFallback.report()
        if args.observe_ekf_reference and "ekfStats" in locals():
            report["ekf_reference_observation"]=ekfStats
            report["ekf_reference_observation"]["scope"]=(
                "local SIH control experiment" if args.sih_ekf_fallback else
                "telemetry only, not connected to control")
            if ekfReader is not None:
                ekfReader.close()
        if control:
            if control.backend.readVehicleState().armed:
                control.backend.set_mode("auto:land")
                until=time.monotonic()+18
                while time.monotonic()<until and control.backend.readVehicleState().armed:
                    executor.spin_once(timeout_sec=.01)
            report["cleanup_landed_disarmed"]=not control.backend.readVehicleState().armed and control.backend.readVehicleState().landed_state==1
            control.shutdown()
        # 释放服务与自有计算进程在销毁 ROS 节点前收口；只操作本脚本持有的句柄。
        if releaseExecutor is not None:
            releaseExecutor.shutdown(timeout_sec=3.)
            releaseThread.join(timeout=3.)
            releaseNode.destroy_node()
        computeVerified=None
        if owned is not None:
            computeVerified=owned.stopAll()
            report["compute_cleanup"]=dict(verified=computeVerified,snapshot=owned.snapshot(),
                decoy_alive=None if decoy is None else decoy.poll() is None)
        if decoy is not None and decoy.poll() is None:
            decoy.terminate()
            try:decoy.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                decoy.kill();decoy.wait(timeout=3.)
        if executor:executor.shutdown(timeout_sec=2.)
        for node in reversed(nodes):node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGTERM)
                try:process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=2)
        trace.close()
        for log in logs:log.close()
        created=sorted(set(source.rglob("*.ulg"))-ulogs,key=lambda p:p.stat().st_mtime)
        if created:
            shutil.copy2(created[-1],out/"px4.ulg")
            report["ulog_sha256"]=hashlib.sha256((out/"px4.ulg").read_bytes()).hexdigest()
        report["elapsed_s"]=time.monotonic()-started
        (out/"report.json").write_text(json.dumps(report,indent=2,default=str))
    print(json.dumps({k:v for k,v in report.items() if k not in ("source_sha256","final_state","phases","configuration","controller_diagnostic")},indent=2))
    return 0 if report["result"]=="PASS" else 1


if __name__=="__main__":raise SystemExit(main())
