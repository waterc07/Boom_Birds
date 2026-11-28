"""生成 27UC 场景并启动任务 1～3 的本机 Gazebo、PX4 SITL 和 ROS 输入。"""

import argparse
from contextlib import ExitStack
import json
import math
import os
from pathlib import Path
import re
import sys
import subprocess
import time
import xml.etree.ElementTree as ET

import yaml

import run_aircraft_motor_sitl as sim
from rmuc2027_visuals import compactTerrainVisuals

LOOPBACK_DDS = """<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>task1_loopback</transport_id>
      <type>UDPv4</type>
      <maxInitialPeersRange>32</maxInitialPeersRange>
      <interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
    </transport_descriptor>
    <transport_descriptor>
      <transport_id>task2_camera_shm</transport_id>
      <type>SHM</type>
      <segment_size>16777216</segment_size>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="task1_local" is_default_profile="true">
    <rtps>
      <useBuiltinTransports>false</useBuiltinTransports>
      <userTransports><transport_id>task1_loopback</transport_id></userTransports>
      <builtin><initialPeersList><locator><udpv4><address>127.0.0.1</address></udpv4></locator></initialPeersList>
        <avoid_builtin_multicast>true</avoid_builtin_multicast></builtin>
    </rtps>
  </participant>
</profiles>
"""

def enableAircraftFollow():
    topic = ["gz", "topic", "-t", "/gui/track", "-m", "gz.msgs.CameraTrack", "-p"]
    command = (f'track_mode: FOLLOW follow_target {{name: "{sim.AIRCRAFT}"}} '
               'follow_offset {x: -2.6 y: -0.9 z: 1.9} follow_pgain: 0.16 track_pgain: 0.16')
    for _ in range(3):
        try:
            subprocess.run([*topic, command], check=True, timeout=3)
            status = subprocess.check_output(
                ["gz", "topic", "-e", "-n", "1", "-t", "/gui/currently_tracked"],
                text=True, timeout=3)
            if f'name: "{sim.AIRCRAFT}"' in status and "track_mode: FOLLOW" in status:
                return True
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            continue
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assets", required=True, type=Path)
    parser.add_argument("--terrain", type=Path, help="use a pad collision crop STL for comparison")
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--px4", type=Path, default=Path.home() / "PX4-Autopilot")
    parser.add_argument("--ros-domain", type=int, default=58)
    parser.add_argument("--hide-camera", action="store_true")
    parser.add_argument("--task", choices=("1", "2", "3"), default="1")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    px = args.px4.resolve(strict=True)
    build = px / "build/px4_sitl_default"
    assets = args.assets.resolve(strict=True)
    dds = out / "local-dds.xml"
    ddsXml = LOOPBACK_DDS
    if args.task != "1":
        ddsXml = ddsXml.replace("</userTransports>",
                                "<transport_id>task2_camera_shm</transport_id></userTransports>")
    dds.write_text(ddsXml)
    os.environ.update(FASTDDS_DEFAULT_PROFILES_FILE=str(dds), FASTRTPS_DEFAULT_PROFILES_FILE=str(dds))
    config=yaml.safe_load((sim.SIM/'config/aircraft_340g.yaml').read_text())
    config['stereo']['rate_hz'] = 15 if args.task != '1' else 2
    curve=sim.ThrustCurve(assets/'models/aircraft_340g_collision/motor_thrust.yaml')
    model,params=sim.buildModel(px,assets,curve,config)
    for resource in ('magnetometer_sensor','air_pressure_sensor','navsat_sensor'):
        body=model.find('link')
        body.remove(body.find(f"sensor[@name='{resource}']"))
    params.update(SYS_HAS_MAG=0,SYS_HAS_BARO=0,SYS_HAS_GPS=0,SIM_GZ_EN_MAG=0,SIM_GZ_EN_BARO=0,SIM_GZ_EN_GPS=0,SIM_GZ_EN_LIDAR=0,EKF2_MAG_TYPE=5,EKF2_BARO_CTRL=0,EKF2_GPS_CTRL=0,EKF2_EV_CTRL=0,EKF2_OF_CTRL=1,EKF2_RNG_CTRL=2,EKF2_HGT_REF=2,COM_ARM_WO_GPS=1,COM_RC_IN_MODE=4)
    if args.task == "3":
        from arena_lap_gazebo import ARENA_YAW_RATE_RAD_S
        params.update(MC_YAWRATE_MAX=math.ceil(math.degrees(ARENA_YAW_RATE_RAD_S)), MPC_TILTMAX_AIR=18.)
    worldFile=ET.parse(sim.SIM/'worlds/rmuc2027.sdf')
    world=worldFile.getroot().find('world')
    step = float(world.findtext("physics/max_step_size"))
    # Sample the landing impulse on each physics step.
    imuSensor = model.find("./link/sensor[@name='imu_sensor']")
    imuSensor.find("update_rate").text = str(round(1.0 / step))
    if args.task != "1":
        from task2_gazebo_vio import writeVioConfig
        from ament_index_python.packages import get_package_prefix
        gravityElement = world.find("gravity")
        if gravityElement is None:
            gravityElement = sim.put(world, "gravity", "0 0 -9.8")
        gravity = math.hypot(*(float(value) for value in gravityElement.text.split()))
        vioConfig = writeVioConfig(out, config["stereo"], imuSensor, gravity)
        vioBinary = Path(get_package_prefix("ov_msckf")) / "lib/ov_msckf/run_subscribe_msckf"
        if not vioBinary.is_file():
            raise RuntimeError("OpenVINS executable not found in the sourced ROS install")
    for item in world.findall('plugin'):
        if item.get('filename') in ('gz-sim-air-pressure-system','gz-sim-navsat-system','gz-sim-magnetometer-system'):
            world.remove(item)
    if args.terrain is None:
        terrainDir = sim.SIM / "assets/rmuc2027/models/rmuc2027_terrain"
        padXY = config["scenes"]["rmuc2027"]["landing_deck"]["pose"][:2]
        compactTerrainVisuals(terrainDir, world, out, padXY)
    else:
        world.remove(world.find('include'))
        terrain=sim.put(world,'model',name='terrain_crop')
        sim.put(terrain,'static','true')
        ground=sim.put(terrain,'link',name='ground')
        collision=sim.put(ground,'collision',name='terrain_collision')
        sim.put(sim.put(sim.put(collision,'geometry'),'mesh'),'uri',args.terrain.resolve(strict=True))
        def marker(name, xyz, size, color):
            visual=sim.put(ground,'visual',name=name)
            sim.put(visual,'pose',sim.vector((*xyz,0,0,0)))
            sim.put(sim.put(sim.put(visual,'geometry'),'box'),'size',sim.vector(size))
            material=sim.put(visual,'material')
            for channel in ('ambient','diffuse'):
                sim.put(material,channel,color)
        marker('surface', (1.03237595,1.03237595,1.752),(3.8,3.8,.02),'0.26 0.30 0.33 1')
        for index, side in enumerate((-1,1)):
            marker('pad_edge_x'+str(index),(1.03237595+side*.23,1.03237595,1.763),(.013,.47,.001),'0.90 0.66 0.21 1')
            marker('pad_edge_y'+str(index),(1.03237595,1.03237595+side*.23,1.763),(.47,.013,.001),'0.90 0.66 0.21 1')
    world.find('scene/shadows').text='false'
    world.find('light/cast_shadows').text='false'
    scene = config["scenes"]["rmuc2027"]
    (out / "task-number").write_text(args.task + "\n")
    worldFile.write(out/'base.sdf',encoding='utf-8')
    if args.task == "3" and not args.hide_camera:
        truth = sim.put(model, "plugin", filename="gz-sim-pose-publisher-system",
                        name="gz::sim::systems::PosePublisher")
        for key, value in (("publish_model_pose", "true"), ("publish_link_pose", "false"),
                           ("update_frequency", "20")):
            sim.put(truth, key, value)
    worldName=sim.writeWorld(out/'base.sdf',out/'world.sdf',model,scene)
    (out/'bridge.yaml').write_text(yaml.safe_dump([row for row in sim.bridgeConfig(worldName) if row["ros_topic_name"] != "/boom_birds/downward/image"],sort_keys=False))
    mavrosConfig=yaml.safe_load((sim.WS/'src/boom_birds_bringup/config/mavros.yaml').read_text())
    mavrosConfig['/**']['ros__parameters']['plugin_allowlist'].append('param')
    (out/'mavros.yaml').write_text(yaml.safe_dump(mavrosConfig))
    (out/'params.json').write_text(json.dumps(params,indent=2))
    gui=(sim.SIM/'config/gazebo/foreground_gui.config').read_text()
    gui=gui.replace('<engine>ogre2</engine>','<engine>ogre</engine>')
    cameraPose = "2.1 2.2 3.2 0 0.71 -2.318" if args.terrain is None else "1.73 -0.16 2.49 0 0.43 2.10"
    gui=re.sub(r"<camera_pose>[^<]*</camera_pose>", f"<camera_pose>{cameraPose}</camera_pose>", gui)
    for panel in ('ComponentInspector','EntityTree'):
        gui=re.sub(r'<plugin filename="'+panel+r'".*?</plugin>','',gui,flags=re.S)
    if not args.hide_camera:
        gui = gui.replace("<width>1000</width>", "<width>1490</width>", 1)
        gui = gui.replace("<height>700</height>", "<height>960</height>", 1)
        gui += """
<plugin filename="ImageDisplay" name="Flight instruments">
  <topic>/boom_birds/gui/flight_instruments</topic>
  <topic_picker>false</topic_picker>
  <gz-gui>
    <title>Flight instruments</title>
    <property type="bool" key="showTitleBar">false</property>
    <property type="string" key="state">docked</property>
    <property type="double" key="width">510</property>
    <property type="double" key="height">890</property>
  </gz-gui>
</plugin>
"""
    (out/'gui.config').write_text(gui)
    for name in tuple(os.environ):
        if name.startswith('PX4_PARAM_') or name=='ROS_LOCALHOST_ONLY':
            del os.environ[name]
    os.environ.update(GZ_PARTITION='bb_task1_'+str(os.getpid()), GZ_IP='127.0.0.1',
        GZ_SIM_RESOURCE_PATH=':'.join(map(str,[assets/'models',px/'Tools/simulation/gz/models',sim.SIM/'assets/rmuc2027/models'])),
        ROS_DOMAIN_ID=str(args.ros_domain),ROS_AUTOMATIC_DISCOVERY_RANGE='SYSTEM_DEFAULT',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
        GZ_SIM_SERVER_CONFIG_PATH=str(px/'src/modules/simulation/gz_bridge/server.config'),
        GZ_SIM_SYSTEM_PLUGIN_PATH=str(build/'src/modules/simulation/gz_plugins'),
        PX4_GZ_STANDALONE='1',PX4_GZ_WORLD=worldName,PX4_GZ_MODEL_NAME=sim.AIRCRAFT,PX4_SYS_AUTOSTART='4001',
        PX4_SIM_MODEL='gz_x500',HEADLESS='1',GALLIUM_DRIVER='llvmpipe',LIBGL_ALWAYS_SOFTWARE='1',LP_NUM_THREADS='5',QT_QPA_PLATFORM='xcb',
        **{f'PX4_PARAM_{key}':str(val) for key,val in params.items()})
    (out/'partition').write_text(os.environ['GZ_PARTITION'])
    from gz.transport13 import Node
    from gz.msgs10.laserscan_pb2 import LaserScan
    from pymavlink import mavutil
    node=Node()
    relay=sim.ThrustRelay(node,curve)
    rangeRelay=sim.DownRangeRelay(node,config['flow_ranging_module']['rangefinder']['limits_m'])
    link=mavutil.mavlink_connection('udpout:127.0.0.1:18571',source_system=255,source_component=190)
    scan = [math.nan, 0., 0., 0., 0]
    def onRange(msg):
        if msg.ranges:
            scan[:]=[msg.ranges[0],msg.range_min,msg.range_max,time.monotonic(),scan[4]+1]
    node.subscribe(LaserScan,'/aircraft_340g/down_range',onRange)
    sent=0
    with ExitStack() as stack:
        processes=[]
        def start(args,name):
            if name in ('bridge','mavros'):
                args=['bash','-c','source /opt/ros/jazzy/setup.bash && exec "$@"','ros',*args]
            proc=sim.start(stack,args,out/f'{name}.log',out)
            (out/f'{name}.pid').write_text(str(proc.pid))
            return proc
        processes.append(start(['gz','sim','-s','-r','--headless-rendering',str(out/'world.sdf')],'gazebo'))
        frontend=start(['gz','sim','-g','--gui-config',str(out/'gui.config')],'gui')
        if not args.hide_camera:
            viewer = [sys.executable, str(Path(__file__).with_name("show_gazebo_downward.py")), str(out)]
            if args.task != "1":
                viewer.append("--stereo")
            if args.task == "3":
                viewer.append("--trajectory")
            start(viewer, "downward")
        deadline = time.monotonic() + (90. if args.terrain is None else 20.)
        while scan[4] < 12 and processes[0].poll() is None and time.monotonic() < deadline:
            time.sleep(.025)
        if scan[4] < 12:
            raise RuntimeError("Gazebo sensors did not start")
        if args.task == "3":
            following = enableAircraftFollow()
            print(f"Gazebo camera follow {sim.AIRCRAFT}: {following}", flush=True)
        autopilot=start([str(build/'bin/px4'),str(build/'etc'),'-d','-i','1'],'px4')
        processes.append(autopilot)
        bridgeProcess = start(['/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge','--ros-args','-p',f'config_file:={out}/bridge.yaml'],'bridge')
        processes.append(bridgeProcess)
        processes.append(start(['/opt/ros/jazzy/lib/mavros/mavros_node','--ros-args','--params-file',str(out/'mavros.yaml'),'-p','fcu_url:=udp://127.0.0.1:14541@127.0.0.1:14581','-p','use_sim_time:=true','-p','tgt_system:=2','-p','tgt_component:=1','-p','gcs_url:=""'],'mavros'))
        if args.task != "1":
            bridgeLog = out / "bridge.log"
            bridgeDeadline = time.monotonic() + 8.
            while "-> /boom_birds/sim/imu_flu" not in bridgeLog.read_text():
                if bridgeProcess.poll() is not None or time.monotonic() >= bridgeDeadline:
                    raise RuntimeError("ROS bridge did not create the simulation IMU publisher")
                time.sleep(.05)
            processes.append(start([str(vioBinary), str(vioConfig), "--ros-args", "-r", "__ns:=/ov_msckf",
                                    "-p", "use_sim_time:=true", "-p", "camera_diagnostics:=true",
                                    "-p", "verbosity:=INFO"], "openvins"))
        lastHeartbeat=lastLog=0
        with (out/'range.log').open('w',buffering=1) as evidence:
            while all(p.poll() is None for p in processes):
                relay.tick()
                now=time.monotonic()
                distance,low,high,stamp,seq=scan[:]
                if now-lastHeartbeat>=1:
                    link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
                    lastHeartbeat=now
                if stamp>sent and now-stamp<0.3 and math.isfinite(distance) and low <= distance <= high:
                    link.mav.distance_sensor_send(0,round(low*100),round(high*100),round(distance*100),0,12,25,0)
                    sent=stamp
                if now-lastLog>=1:
                    print(f'{seq} {distance:.6f} {low:.3f} {high:.3f} {now-stamp:.3f} {sent>0} gui={frontend.poll()}',file=evidence)
                    lastLog=now
                time.sleep(.01)
        exited = [(p.pid, p.poll()) for p in processes]
        raise RuntimeError(f"Task {args.task} Gazebo session stopped: {exited}")



if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
