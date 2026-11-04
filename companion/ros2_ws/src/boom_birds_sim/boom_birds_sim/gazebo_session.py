"""受控 Gazebo 感知采集：实际输入就绪后才开始静止／运动计时。"""

import argparse
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from .gazebo_arena import loadArena
from .gazebo_fixture import prepare
from .gazebo_run import Processes, claimRosDomain


def writeJson(path, value):
    path = Path(path)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2))
    temporary.replace(path)


def gzRequest(service, requestType, request):
    reply = subprocess.run(["gz", "service", "-s", service,
                            "--reqtype", requestType, "--reptype", "gz.msgs.Boolean",
                            "--timeout", "5000", "--req", request],
                           text=True, capture_output=True, timeout=8)
    if reply.returncode or "data: true" not in reply.stdout:
        raise RuntimeError(f"Gazebo service rejected {service}: {reply.stdout} {reply.stderr}")
    return reply.stdout


def runtimeManifest(requiredRoot=None):
    modules = {}
    for name in ("boom_birds_sim.gazebo_session", "boom_birds_sim.gazebo_texture", "boom_birds_control.frames",
                 "boom_birds_sensing.depth_core", "stereo_depth"):
        module = importlib.import_module(name)
        path = Path(module.__file__).resolve()
        if requiredRoot and not path.is_relative_to(Path(requiredRoot).resolve()):
            raise ValueError(f"module outside runtime root: {name} = {path}")
        modules[name] = dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    binary = Path(get_package_prefix("ov_msckf")) / "lib/ov_msckf/run_subscribe_msckf"
    if requiredRoot and not binary.resolve().is_relative_to(Path(requiredRoot).resolve()):
        raise ValueError(f"OpenVINS outside runtime root: {binary}")
    return dict(python=sys.executable, modules=modules,
                openvins_binary=str(binary), openvins_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                openvins_ldd=subprocess.check_output(["ldd", str(binary)], text=True),
                hardware_connected=False, control_started=False)


class GzClock:
    def __init__(self):
        from gz.transport13 import Node
        from gz.msgs10.clock_pb2 import Clock
        self.nanoseconds = 0
        self.node = Node()
        if not self.node.subscribe(Clock, "/clock", self.receive):
            raise RuntimeError("cannot subscribe to native Gazebo clock")

    def receive(self, message):
        self.nanoseconds = message.sim.sec * 1000000000 + message.sim.nsec


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--package-root", default=get_package_share_directory("boom_birds_sim"))
    parser.add_argument("--root", default="/home/waterc/workspace/Boom_Birds")
    parser.add_argument("--runtime-root")
    parser.add_argument("--texture-model", help="explicit generated texture diagnostic model; CAD remains default")
    parser.add_argument("--case", choices=("S00_static", "perception_motion"), default="perception_motion")
    parser.add_argument("--duration", type=float, default=45)
    parser.add_argument("--wall-timeout", type=float, default=1200)
    parser.add_argument("--startup-timeout", type=float, default=300)
    parser.add_argument("--headless-rendering", action="store_true")
    parser.add_argument("--gui", action="store_true", help="open the Gazebo GUI in the foreground desktop")
    parser.add_argument("--gui-config", help="Gazebo GUI configuration; requires --gui")
    parser.add_argument("--pause-test", action="store_true")
    parser.add_argument("--reset-test", action="store_true")
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.duration, args.wall_timeout, args.startup_timeout)):
        parser.error("duration and timeouts must be finite and positive")
    if args.gui_config and (not args.gui or not Path(args.gui_config).is_file()):
        parser.error("gui-config requires --gui and an existing file")
    if args.gui and args.headless_rendering:
        parser.error("GUI and headless rendering are mutually exclusive")
    if args.pause_test and args.reset_test:
        parser.error("select at most one clock fault")
    out = Path(args.out).resolve()
    if out.exists():
        parser.error("out must not exist")
    arena = loadArena(args.package_root)
    textureManifest = None
    if args.texture_model:
        from .gazebo_texture import validateTextureModel
        textureModel = Path(args.texture_model).resolve()
        textureManifest = validateTextureModel(arena["model"], textureModel)
        arena["model"] = textureModel
        arena["resource_path"] = textureModel.parent
    runtime = runtimeManifest(args.runtime_root)
    out.mkdir(parents=True)
    writeJson(out / "runtime.json", runtime)
    if textureManifest:
        writeJson(out / "texture-manifest.json", textureManifest)
    domain, domainLock = claimRosDomain()
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1",
                      GZ_PARTITION=f"bb_session_{os.getpid()}", GZ_IP="127.0.0.1",
                      GZ_CONFIG_PATH="/usr/share/gz", OPENBLAS_NUM_THREADS="1")
    os.environ["GZ_SIM_RESOURCE_PATH"] = str(arena["resource_path"])
    plugin = Path(get_package_prefix("boom_birds_sim")) / "lib/boom_birds_sim/libbb_sensor_motion.so"
    template = Path(get_package_share_directory("ov_msckf")) / "config/euroc_mav"
    if not template.is_dir():
        template = Path(args.root) / "companion/ros2_ws/src/open_vins/config/euroc_mav"
    manifest = prepare(out, template / "estimator_config.yaml", template / "kalibr_imu_chain.yaml",
                       plugin, motion_profile="static" if args.case == "S00_static" else "nominal",
                       duration_sim_s=args.duration, arena_model=arena["model"],
                       rig_pose=arena["rig_pose"], controlled_motion=True, vio_profile="stereo")
    manifest.update(case=args.case, arena_config={k:str(v) if isinstance(v, Path) else v for k,v in arena.items()},
                    truth_bag="truth", input_bag="inputs", bag_format_version=2,
                    appearance_profile="procedural_texture_diagnostic" if textureManifest else "original_cad",
                    gui=bool(args.gui),
                    gui_config_sha256=None if not args.gui_config else hashlib.sha256(Path(args.gui_config).read_bytes()).hexdigest())
    writeJson(out / "manifest.json", manifest)
    writeJson(out / "environment.json", dict(ros_domain_id=domain,gz_partition=os.environ["GZ_PARTITION"],
                                              resource_path=os.environ["GZ_SIM_RESOURCE_PATH"]))
    clock = GzClock()
    procs = Processes(out)
    events = []
    result = dict(pass_session=False, events=events, errors=[])
    def event(state, **fields):
        events.append(dict(state=state, wall_monotonic_s=time.monotonic(), **fields))
        writeJson(out / "session.json", result)
    def status():
        path = out / "capture-status.json"
        return json.loads(path.read_text()) if path.is_file() else {}
    try:
        event("PREPARE")
        bridge = procs.start("bridge", ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args", "-p", f"config_file:={out/'bridge.yaml'}"])
        capture = procs.start("capture", [sys.executable, "-m", "boom_birds_sim.gazebo_capture", "--out", str(out),
                                          "--duration", str(args.duration), "--wall-timeout", str(args.wall_timeout+args.startup_timeout), "--controlled"])
        command = ["gz", "sim"]
        if not args.gui: command.append("-s")
        if args.headless_rendering: command.append("--headless-rendering")
        if args.gui_config: command.extend(["--gui-config", str(Path(args.gui_config).resolve())])
        gazebo = procs.start("gazebo", [*command,str(out/"sensor_world.sdf")])
        event("WORLD_PAUSED")
        startupDeadline = time.monotonic() + args.startup_timeout
        stepTarget = None
        while not status().get("ready"):
            if any(p.poll() is not None for p in (bridge,capture,gazebo)):
                raise RuntimeError("process exited during input readiness")
            if time.monotonic() > startupDeadline:
                raise TimeoutError("actual stereo/CameraInfo/IMU/depth/truth/clock readiness timed out")
            if stepTarget is None or clock.nanoseconds >= stepTarget:
                # 服务响应只表示入队；上一批实际走完前不能再排入 pause/step 请求。
                stepTarget = clock.nanoseconds + 100000000
                try:
                    gzRequest("/world/bb_sensor/control", "gz.msgs.WorldControl", "pause: true, multi_step: 100")
                except RuntimeError:
                    stepTarget = None
            time.sleep(.1)
        while stepTarget is not None and clock.nanoseconds < stepTarget:
            if time.monotonic() > startupDeadline:
                raise TimeoutError("warm-up steps did not finish")
            time.sleep(.05)
        state = status()
        if state.get("errors"): raise RuntimeError(str(state["errors"]))
        event("INPUT_READY", sim_ns=state["sim_ns"], counts=state["counts"], input_publishers=state["input_publishers"])
        # 暂停状态下置位插件，再用相同阶段原点启动录制完成计时。
        gzRequest("/bb/rig/start_motion", "gz.msgs.Boolean", "data: true")
        origin = clock.nanoseconds
        writeJson(out / "recording-start.json", dict(sim_ns=origin, static_duration_s=5.0, motion_stop_offset_s=40.0))
        event("STATIC_INIT", sim_ns=origin)
        gzRequest("/world/bb_sensor/control", "gz.msgs.WorldControl", "pause: false")
        deadline = time.monotonic() + args.wall_timeout
        faultDone = False
        motionNoted = False
        settlingNoted = False
        while capture.poll() is None:
            if bridge.poll() is not None or gazebo.poll() is not None:
                raise RuntimeError("simulation process exited before capture completed")
            if time.monotonic() > deadline:
                raise TimeoutError("session wall timeout")
            state = status()
            elapsed = (state.get("sim_ns", 0) - origin) * 1e-9
            if args.case == "perception_motion" and elapsed >= 5 and not motionNoted:
                event("MOTION_WINDOW", sim_ns=state["sim_ns"])
                motionNoted = True
            if args.case == "perception_motion" and elapsed >= 40 and not settlingNoted:
                event("SETTLING", sim_ns=state["sim_ns"])
                settlingNoted = True
            if not faultDone and (args.pause_test or args.reset_test) and state.get("sim_ns",0) >= origin+2000000000:
                if args.reset_test:
                    gzRequest("/world/bb_sensor/control", "gz.msgs.WorldControl", "reset: {all: true}")
                    event("CLOCK_RESET_INJECTED")
                else:
                    gzRequest("/world/bb_sensor/control", "gz.msgs.WorldControl", "pause: true")
                    time.sleep(.8)
                    before=status().get("sim_ns",0)
                    time.sleep(1.)
                    after=status().get("sim_ns",0)
                    result["pause_clock_advance_ns"] = after-before
                    gzRequest("/world/bb_sensor/control", "gz.msgs.WorldControl", "pause: false")
                    event("CLOCK_RESUMED", pause_clock_advance_ns=after-before)
                faultDone=True
            time.sleep(.1)
        if not (out/"capture.json").is_file():
            raise RuntimeError(f"capture exited {capture.returncode} without report; see capture.log")
        report=json.loads((out/"capture.json").read_text())
        result["capture_pass"]=report["pass"]
        if args.reset_test:
            result["pass_session"]=faultDone and capture.returncode==1 and "simulation clock moved backwards" in report["errors"]
        else:
            result["pass_session"]=capture.returncode==0 and report["pass"] and (not args.pause_test or (faultDone and result.get("pause_clock_advance_ns")==0))
        event("STOPPING")
    except (Exception, KeyboardInterrupt) as error:
        result["errors"].append(f"{type(error).__name__}: {error}")
        event("FAILED")
    finally:
        procs.close()
        exits = json.loads((out / "processes.json").read_text())
        result["process_exits"] = exits
        result["pass_session"] = result["pass_session"] and all(
            code == (1 if args.reset_test and name == "capture" else 0)
            for name, code in exits.items())
        event("CLOSED")
    print(json.dumps(result,indent=2))
    return 0 if result["pass_session"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
