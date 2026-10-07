#!/usr/bin/env python3
"""TEST-ONLY：受控导航输入→真实 ROS 控制/任务节点→MAVROS→本机 SIH。"""
import argparse
from dataclasses import asdict
import json
import hashlib
import shutil
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import cv2
import numpy as np
import yaml
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Range
from std_msgs.msg import String
from cv_bridge import CvBridge
from boom_birds_control.px4_interface_node import Px4InterfaceNode
from boom_birds_control.platform_landing import ned_from_flu
from boom_birds_control.platform_sih_feedback import fields
from boom_birds_control.sih_guard import verify_sih_process
from boom_birds_control.runtime_config import DEFAULTS
from boom_birds_interfaces.msg import ControlCommand
from boom_birds_interfaces.srv import VehicleAction, Mission
from boom_birds_sensing.platform_observation import BoardDetector
from boom_birds_sensing.platform_synthetic import render_board
from boom_birds_sensing.platform_observation_node import PlatformObservationNode
from boom_birds_bringup.lifecycle_node import LifecycleNode
from boom_birds_bringup.compute_release_node import ComputeReleaseNode


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--scenario", choices=("origin", "return", "service_late", "tag_loss", "range_jump", "old_session"), default="origin")
    ap.add_argument("--control-mode", choices=("px4_position", "companion_attitude"), default="px4_position")
    ap.add_argument("--allow-simulated-arming", action="store_true")
    args = ap.parse_args()
    if not args.allow_simulated_arming:
        raise ValueError("explicit_simulated_arming_required")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    source = Path("/home/waterc/PX4-Autopilot")
    binary = source/"build/px4_sitl_default/bin/px4"
    for path in Path("/proc").iterdir():
        if path.name.isdigit():
            try:
                if path.joinpath("exe").resolve() == binary.resolve():
                    raise ValueError("existing_px4_instance")
            except (FileNotFoundError, PermissionError):
                pass
    profile = yaml.safe_load(Path(args.config).read_text())
    detector = BoardDetector(profile, test_only=True)
    env = dict(os.environ, PX4_SIM_MODEL="sihsim_quadx", PX4_SIMULATOR="sihsim", PX4_SYS_AUTOSTART="10040")
    processes, logs, nodes = [], [], []
    report = dict(kind="ROS_MAVROS_SIH", scenario=args.scenario, control_mode=args.control_mode,
                  result="FAIL", test_only=True, parameter_writes=False, config=profile,
                  navigation_input="controlled ControlCommand + synthetic VIO/IMU/depth",
                  compute_processes="actual owned synthetic input processes; no OpenVINS or camera",
                  navigation_arrival_tolerance=dict(xy_m=.08, z_m=.12))
    watched = [Path(__file__), Path(args.config), *list((Path(__file__).parents[1]/"src/boom_birds_control/boom_birds_control").glob("platform_*.py")),
               Path(__file__).parents[1]/"src/boom_birds_sensing/boom_birds_sensing/platform_observation.py",
               Path(__file__).parents[1]/"src/boom_birds_bringup/boom_birds_bringup/platform_mission.py",
               Path(__file__).parents[1]/"src/boom_birds_bringup/boom_birds_bringup/compute_release_node.py"]
    report["source_sha256"] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in watched}
    existing_ulog = set(source.rglob("*.ulg"))
    trace = (out/"trace.jsonl").open("w")
    control = service = service_executor = service_thread = executor = None
    started = time.monotonic()
    def spawn(command, name, **kwargs):
        log = (out/(name+".log")).open("w")
        logs.append(log)
        p = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, **kwargs)
        processes.append(p)
        return p
    px4 = spawn([str(binary), "-d", "-i", "0"], "px4", cwd=source, env=env)
    try:
        deadline = time.monotonic()+5.
        while time.monotonic() < deadline and not verify_sih_process(px4.pid):
            time.sleep(.05)
        if not verify_sih_process(px4.pid):
            raise ValueError("local_sih_guard")
        config = Path(__file__).parents[1]/"src/boom_birds_bringup/config/mavros.yaml"
        spawn(["ros2", "run", "mavros", "mavros_node", "--ros-args", "--params-file", str(config),
               "-p", "fcu_url:=udp://127.0.0.1:14540@127.0.0.1:14580", "-p", 'gcs_url:=""',
               "-p", "tgt_system:=1", "-p", "tgt_component:=1", "-p", "fcu_protocol:=v2.0"], "mavros")
        spawn(["ros2", "run", "boom_birds_sensing", "mavros_config_node"], "mavros-config")
        # 先等待本机估计器初始化；启动期间的 reset 不得带入导航会话。
        time.sleep(5.)
        rclpy.init()
        overrides = {
            "backend": "mavros", "dry_run": False, "allow_arming": True,
            "sih_pid": px4.pid, "require_session": True, "frame_alignment": "identity",
            "frame_alignment_observed": True, "frame_alignment_origin_evidence": True,
            "frame_alignment_origin_note": "TEST-ONLY: ROS input derives from this SIH EKF",
            "control_mode": args.control_mode,
            "attitude_config_file": str(Path(__file__).parents[1]/"src/boom_birds_control/config/attitude_sih.yaml"),
            "platform_config_file": str(Path(args.config).resolve()), "platform_test_only": True,
            "platform_trace_directory": str(out/"control-trace")}
        control = Px4InterfaceNode(parameter_overrides=[Parameter(k, value=v) for k, v in overrides.items()])
        mission = LifecycleNode(parameter_overrides=[
            Parameter("platform_landing_enabled", value=True),
            Parameter("control_mode", value=args.control_mode),
            Parameter("attitude_config_file", value=overrides["attitude_config_file"])])
        observer = PlatformObservationNode(parameter_overrides=[
            Parameter("config_file", value=str(Path(args.config).resolve())), Parameter("test_only", value=True)])
        harness = Node("platform_ros_sih_harness")
        nodes = [control, mission, observer, harness]
        executor = SingleThreadedExecutor()
        for n in nodes:
            executor.add_node(n)
        truth_pub = harness.create_publisher(String, "/boom_birds/platform/test_truth", 10)
        image_pub = harness.create_publisher(__import__("sensor_msgs.msg", fromlist=["Image"]).Image, "/boom_birds/downward/image", 10)
        range_pub = harness.create_publisher(Range, "/boom_birds/platform/range", 10)
        cmd_pub = harness.create_publisher(ControlCommand, DEFAULTS.command_topic, 10)
        request_pub = harness.create_publisher(String, "/boom_birds/platform/request", 10)
        navigation = spawn([sys.executable, str(Path(__file__).with_name("platform_test_inputs.py")), "--kind", "navigation"], "synthetic-navigation")
        camera = spawn([sys.executable, str(Path(__file__).with_name("platform_test_inputs.py")), "--kind", "camera"], "synthetic-camera")
        bridge = CvBridge()
        seq = 0
        home = None
        goal = None
        status_rows = 0
        wire = dict(attitude=0, position=0, platform_velocity=0, violations=0)
        from mavros_msgs.msg import AttitudeTarget, PositionTarget
        from rclpy.qos import qos_profile_sensor_data
        def wire_capture(msg, kind):
            wire[kind] += 1
            age = harness.get_clock().now().nanoseconds*1e-9 - (msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
            produced = time.monotonic()-age
            core = control.platform.executor.core
            if core.pending_since is not None and produced >= core.pending_since:
                if kind == "attitude" or msg.type_mask != 1479:
                    wire["violations"] += 1
                else:
                    wire["platform_velocity"] += 1
            trace.write(json.dumps(dict(kind="mavros_"+kind, now=time.monotonic(),
                sample_ros=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                mask=getattr(msg,"type_mask",None), owner=control.platform.executor.owner))+"\n")
        harness.create_subscription(AttitudeTarget, "/mavros/setpoint_raw/attitude", lambda m: wire_capture(m, "attitude"), qos_profile_sensor_data)
        harness.create_subscription(PositionTarget, "/mavros/setpoint_raw/local", lambda m: wire_capture(m, "position"), qos_profile_sensor_data)
        def capture(msg):
            nonlocal status_rows
            data = json.loads(msg.data)
            status_rows += 1
            trace.write(json.dumps(dict(kind="status", now=time.monotonic(), data=data,
                                       processes_alive=[navigation.poll() is None, camera.poll() is None]))+"\n")
            if data["velocity_ned"] and data["velocity_ned"][2] > 0 and not data["descent_permitted"]:
                raise ValueError("unauthorized_descent")
            if not control.platform.executor.core.confirmed and (navigation.poll() is not None or camera.poll() is not None):
                raise ValueError("premature_compute_release")
        harness.create_subscription(String, "/boom_birds/platform/status", capture, 10)
        def pump(target=None, landing=False):
            nonlocal seq
            if not verify_sih_process(px4.pid):
                raise ValueError("sih_process_changed")
            state = control.backend.read_vehicle_state()
            if state.position_ned_m is not None and state.velocity_ned_m_s is not None and all(x is not None for x in (state.roll_rad, state.pitch_rad, state.yaw_rad)):
                truth_pub.publish(String(data=json.dumps(dict(position=state.position_ned_m, velocity=state.velocity_ned_m_s,
                                    attitude=(state.roll_rad, state.pitch_rad, state.yaw_rad)))))
                if home is not None:
                    r = ned_from_flu(state.roll_rad, state.pitch_rad, state.yaw_rad)
                    pose = np.eye(4)
                    pose[:3, :3] = r.T @ np.diag([1., -1., -1.])
                    pose[:3, 3] = r.T @ (home-np.array(state.position_ned_m))
                    image = render_board(detector, pose)
                    if landing and args.scenario == "tag_loss" and control.platform.executor.core.confirmed:
                        image[:] = 255
                    m = bridge.cv2_to_imgmsg(image, encoding="mono8")
                    m.header.stamp = harness.get_clock().now().to_msg()
                    m.header.frame_id = "downward_optical"
                    image_pub.publish(m)
                    distance = (home[2]-state.position_ned_m[2]-float((r@np.asarray(profile["range"]["T_body_sensor"])[:3, 3])[2]))/float((r@np.asarray(profile["range"]["T_body_sensor"])[:3, 2])[2])
                    m = Range()
                    m.header.stamp = harness.get_clock().now().to_msg()
                    m.header.frame_id = "range_sensor"
                    m.min_range, m.max_range = .04, 5.
                    m.range = float(distance+1. if landing and args.scenario == "range_jump" and control.platform.executor.core.confirmed else distance)
                    range_pub.publish(m)
                    trace.write(json.dumps(dict(kind="input", now=time.monotonic(), position=state.position_ned_m,
                                                range=m.range, image_sha256=__import__("hashlib").sha256(image.tobytes()).hexdigest()))+"\n")
            if target is not None and control.platform.executor.owner == "navigation":
                seq += 1
                command = ControlCommand()
                command.header.stamp = harness.get_clock().now().to_msg()
                command.header.frame_id = DEFAULTS.world_frame
                command.session_id = control.ingress.session
                command.command_type = command.HOLD
                command.trajectory_id = max(1, control.ingress.retired+1)
                command.sequence = seq
                command.valid_for.nanosec = 100000000
                reference = np.asarray(target)
                if args.control_mode == "companion_attitude" and state.position_ned_m is not None:
                    current = np.asarray(state.position_ned_m)
                    delta = reference-current
                    norm = np.linalg.norm(delta)
                    if norm > .5:
                        reference = current+delta*.5/norm
                command.position.x, command.position.y, command.position.z = float(reference[0]), -float(reference[1]), -float(reference[2])
                command.yaw = .1
                cmd_pub.publish(command)
            until = time.monotonic()+.02
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=.002)
            return control.backend.read_vehicle_state()
        def wait_for(predicate, timeout, target=None):
            end = time.monotonic()+timeout
            while time.monotonic() < end:
                state = pump(target)
                if predicate(state):
                    return state
            diagnostic = dict(vehicle=asdict(control.backend.read_vehicle_state()),
                              protocol=control._protocol_reason, alignment=control.alignment_report(),
                              monitor=control.core.monitor.report(),
                              outcome=vars(control._last_outcome) if control._last_outcome else None,
                              backend=control.backend.stream_diagnostics())
            (out/"wait-timeout.json").write_text(json.dumps(diagnostic, default=str, indent=2))
            raise ValueError("wait_timeout")
        wait_for(lambda s: s.connected and s.position_ned_m is not None and s.yaw_rad is not None, 18.)
        def sensors_ready(state):
            last = control.core.monitor.report()["last_seen"]
            now = time.monotonic()
            return all(last.get(key) is not None and 0 <= now-last[key] < .1
                       for key in ("vio_pose", "imu", "camera", "odom_ego"))
        wait_for(sensors_ready, 8.)
        action_client = harness.create_client(VehicleAction, DEFAULTS.action_service)
        request = VehicleAction.Request()
        request.action = request.OPEN_SESSION
        future = action_client.call_async(request)
        wait_for(lambda s: future.done(), 5.)
        if not future.result().accepted:
            raise ValueError("session_open_failed")
        mission.fsm.session = future.result().session_id
        mission.ingress.session = mission.fsm.session
        home = np.array(control.backend.read_vehicle_state().position_ned_m)
        goal = home+np.array([0., 0., -profile["guidance"]["takeoff_height_m"]])
        wait_for(lambda s: control._last_outcome is not None and control._last_outcome.sent, 8., goal)
        prestream_until = time.monotonic()+1.4
        while time.monotonic() < prestream_until:
            pump(goal)
        request = VehicleAction.Request()
        request.action, request.session_id = request.OFFBOARD, mission.fsm.session
        future = action_client.call_async(request)
        wait_for(lambda s: future.done(), 4., goal)
        if not future.result().accepted:
            raise ValueError("offboard_service_rejected:"+future.result().reason)
        wait_for(lambda s: s.is_offboard, 5., goal)
        end = time.monotonic()+12.
        while time.monotonic() < end:
            pump(goal)
            health = subprocess.run([str(binary.with_name("px4-listener")), "vehicle_status", "-n", "1"],
                                    cwd=binary.parent.parent/"rootfs/0", capture_output=True, text=True, timeout=.4)
            checked, _ = fields(health.stdout)
            if checked.get("pre_flight_checks_pass") == "True":
                break
        else:
            raise ValueError("preflight_checks_not_ready")
        request = VehicleAction.Request()
        request.action, request.session_id = request.ARM, mission.fsm.session
        future = action_client.call_async(request)
        wait_for(lambda s: future.done(), 5., goal)
        if not future.result().accepted:
            raise ValueError("arm_service_rejected:"+future.result().reason)
        wait_for(lambda s: s.armed and np.linalg.norm(np.asarray(s.position_ned_m)-goal) < .12, 20., goal)
        if args.scenario == "return":
            for offset in (1., 0.):
                target = goal+np.array([offset, 0., 0.])
                wait_for(lambda s: np.linalg.norm((np.asarray(s.position_ned_m)-target)[:2]) < .08
                         and abs(s.position_ned_m[2]-target[2]) < .12, 15., target)
        wait_for(lambda s: np.linalg.norm(np.asarray(s.velocity_ned_m_s)) < .05
                 and np.linalg.norm((np.asarray(s.position_ned_m)-goal)[:2]) < .05
                 and abs(s.position_ned_m[2]-goal[2]) < .12, 10., goal)
        settle_end = time.monotonic()+.5
        while time.monotonic() < settle_end:
            pump(goal)
        # 服务故意晚于交接确认上线，验证有界重试。
        def start_service():
            nonlocal service, service_executor, service_thread
            service = ComputeReleaseNode([navigation], [camera], lambda: control.ingress.session)
            service_executor = SingleThreadedExecutor()
            service_executor.add_node(service)
            service_thread = threading.Thread(target=service_executor.spin, daemon=True)
            service_thread.start()
        if args.scenario != "service_late":
            start_service()
        mission.command_seq = seq+10
        land_client = harness.create_client(Mission, DEFAULTS.mission_service)
        request = Mission.Request()
        request.action = request.LAND
        future = land_client.call_async(request)
        wait_for(lambda s: future.done(), 4., goal)
        if not future.result().accepted:
            raise ValueError("mission_land_rejected:"+future.result().reason)
        landing_start = time.monotonic()
        if args.scenario == "old_session":
            request_pub.publish(String(data=json.dumps(dict(action="cancel", session="old"))))
        deadline = landing_start+35.
        while time.monotonic() < deadline:
            state = pump(landing=True)
            if args.scenario == "service_late" and service is None and control.platform.executor.core.confirmed and time.monotonic()-landing_start > .8:
                start_service()
            if not state.armed and state.landed_state == 1 and control.platform.executor.core.state == "COMPLETE":
                break
        else:
            raise ValueError("landing_timeout")
        ex = control.platform.executor
        report.update(confirmed=ex.core.confirmed, released=ex.core.released,
                      release_state=ex.release_state, release_attempts=ex.release_attempts,
                      children_stopped=navigation.poll() is not None and camera.poll() is not None,
                      mission_complete=mission.fsm.state.value == "COMPLETE", status_rows=status_rows,
                      wire=wire, final_state=asdict(state))
        if not ex.core.confirmed:
            raise ValueError("handoff_not_confirmed")
        if args.scenario not in ("tag_loss", "range_jump") and not report["children_stopped"]:
            raise ValueError("compute_not_stopped")
        if not report["mission_complete"]:
            # COMPLETE 发布需要一个 ROS 调度周期。
            for _ in range(10):
                pump()
            report["mission_complete"] = mission.fsm.state.value == "COMPLETE"
        if not report["mission_complete"]:
            raise ValueError("mission_not_complete")
        if wire["violations"] or not wire["platform_velocity"]:
            raise ValueError("wire_output_conflict_or_missing_velocity")
        if args.control_mode == "companion_attitude" and not wire["attitude"]:
            raise ValueError("attitude_navigation_not_observed")
        report["result"] = "PASS"
    except Exception as exc:
        report["error"] = str(exc)
    finally:
        if control:
            if control.backend.read_vehicle_state().armed:
                control.backend.set_mode("auto:land")
                until = time.monotonic()+18.
                while time.monotonic() < until and control.backend.read_vehicle_state().armed:
                    if executor:
                        executor.spin_once(timeout_sec=.05)
            report["cleanup_landed_disarmed"] = not control.backend.read_vehicle_state().armed and control.backend.read_vehicle_state().landed_state == 1
            if control.platform.feedback_reader:
                control.platform.feedback_reader.outdir = out/"feedback"
                control.platform.feedback_reader.outdir.mkdir(exist_ok=True)
            control.shutdown()
        if service_executor:
            service_executor.shutdown(timeout_sec=3.)
            service_thread.join(timeout=3.)
            service.destroy_node()
        if executor:
            executor.shutdown(timeout_sec=2.)
        for n in reversed(nodes):
            n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGTERM)
                try:
                    p.wait(timeout=4.)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL)
                    p.wait(timeout=2.)
        trace.close()
        for log in logs:
            log.close()
        created_ulog = sorted(set(source.rglob("*.ulg"))-existing_ulog, key=lambda p:p.stat().st_mtime)
        if created_ulog:
            shutil.copy2(created_ulog[-1], out/"px4.ulg")
            report["px4_ulog_sha256"] = hashlib.sha256((out/"px4.ulg").read_bytes()).hexdigest()
        report["elapsed_s"] = time.monotonic()-started
        (out/"report.json").write_text(json.dumps(report, indent=2, default=str))
    print(json.dumps({k: v for k, v in report.items() if k not in ("config", "final_state")}, indent=2))
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
