"""在已启动的独立 Gazebo/PX4 SITL 会话执行任务 1～3。"""

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
import re
import time
from unittest.mock import patch

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from rclpy.time import Time
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Range
import yaml

from boom_birds_bringup.lifecycle_node import DEFAULTS
from boom_birds_bringup.tag_flight_node import TagFlightNode
from boom_birds_control import mavros_backend, sih_guard
from boom_birds_control.px4_interface_node import Px4InterfaceNode
from boom_birds_control.px4_backend import MavlinkPx4Backend
from boom_birds_interfaces.srv import Mission, VehicleAction
from boom_birds_sensing.platform_observation_node import PlatformObservationNode
from task1_gazebo_bootstrap import GazeboTask1Px4, ImuTakeoff, field

SRC = Path(__file__).resolve().parents[1] / "src"


def parameters(**values):
    return {"parameter_overrides": [Parameter(name, value=value) for name, value in values.items()]}


def profiles(px4, continuousDescent, taskNumber, *, baseTurnCheck=False):
    scope = px4.runDirectory / "task1-sim-scope.txt"
    scope.write_text(f"Gazebo PX4 SITL instance 1, local Task {taskNumber} only.\n")
    velocity = yaml.safe_load((SRC / "boom_birds_control/config/velocity_hardware.yaml").read_text())
    velocity["flight_trial"] = dict(authorized=True, task="tag_takeoff_land",
                                    control_mode="px4_velocity", record=str(scope))
    if taskNumber == "3":
        velocity["controller"].update(max_velocity=7., max_command_velocity=5.9, max_acceleration=2.4)
        velocity["mission"]["flight_budget_s"] = 1200.
    velocityFile = px4.runDirectory / "task1-velocity.yaml"
    velocityFile.write_text(yaml.safe_dump(velocity))
    camera = yaml.safe_load((SRC / "boom_birds_sim/config/platform_landing_70mm_test.yaml").read_text())
    geometry = yaml.safe_load((SRC / "boom_birds_sim/config/aircraft_340g.yaml").read_text())
    fx, fy, cx, cy = geometry["downward"]["intrinsics"]
    camera["camera"]["K"] = [[fx, 0., cx], [0., fy, cy], [0., 0., 1.]]
    for index in range(3):
        camera["camera"]["T_body_camera"][index][3] = geometry["downward"]["pose"][index]
        camera["range"]["T_body_sensor"][index][3] = geometry["flow_ranging_module"]["rangefinder"]["pose"][index]
    taskName = "tag_return_turn" if taskNumber != "1" else "tag_takeoff_land"
    taskFile = SRC / "boom_birds_bringup/config/flight_tasks" / f"{taskName}.yaml"
    if taskNumber != "1" or continuousDescent:
        task = yaml.safe_load(taskFile.read_text())
        if taskNumber == "2":
            task.update(height_m=.40, flare_height_m=.30, final_land_height_m=.20,
                        max_speed_m_s=.24, max_yaw_rate_rad_s=.55)
        if taskNumber == "3":
            from arena_lap_gazebo import ARENA_CRUISE_SPEED_M_S, ARENA_YAW_RATE_RAD_S, arenaRoute
            steps, points = arenaRoute(geometry["scenes"]["rmuc2027"]["spawn_pose"], baseTurnCheck=baseTurnCheck)
            task.update(name="tag_arena_laps", steps=steps, max_radius_m=15.,
                        height_m=.30, flare_height_m=.245, final_land_height_m=.20,
                        max_speed_m_s=ARENA_CRUISE_SPEED_M_S, max_accel_m_s2=2.,
                        max_yaw_rate_rad_s=ARENA_YAW_RATE_RAD_S, max_yaw_accel_rad_s2=10.,
                        max_tag_correction_m=.75)
            (px4.runDirectory / "arena-route.json").write_text(json.dumps(points))
        if continuousDescent:
            task.update(continuous_final_descent=True, final_land_height_m=.16,
                        tag_align_m=.035, stable_speed_m_s=.055)
            camera.setdefault("feature_tracking", {})["capture_min_m"] = .12
        taskFile = px4.runDirectory / f"task{taskNumber}-gazebo.yaml"
        taskFile.write_text(yaml.safe_dump(task))
    cameraFile = px4.runDirectory / "task1-tag.yaml"
    cameraFile.write_text(yaml.safe_dump(camera))
    return str(velocityFile), str(cameraFile), str(taskFile)


def gazeboScope(px4, pid):
    if pid != px4.pid:
        return False
    try:
        px4.verify()
    except (RuntimeError, OSError):
        return False
    return True


def gazeboBackend(px4, simTime):
    source = px4.binary.parents[3]
    revision = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    controller = source / "src/modules/mc_pos_control/MulticopterPositionControl.cpp"
    subprocess.run(["git", "-C", str(source), "diff", "--quiet", "HEAD", "--", str(controller)], check=True)
    code = controller.read_text()
    alwaysOn = (revision in px4.command("ver", "git")
                and "updateHoverThrust(hte.hover_thrust)" in code and "_param_mpc_use_hte" not in code)
    parent = mavros_backend.MavrosPx4Backend

    class LocalGazeboBackend(parent):
        def __init__(self, *args, **kwargs):
            kwargs["target_system"] = px4.system
            super().__init__(*args, **kwargs)
            self.samplePool = ThreadPoolExecutor(max_workers=5)

        _direct = None
        _counters = None
        _directEpoch = 0
        _directAt = -math.inf

        def connect(self):
            self._endpointVerified = gazeboScope(px4, px4.pid)
            self._conn = px4.wire if self._endpointVerified else None
            return self._endpointVerified

        def requestObservationStreams(self):
            # Gazebo uses the isolated PX4 uORB client for state readback.
            return

        def _sendCommandLong(self, name, command, **kwargs):
            if not gazeboScope(px4, px4.pid):
                return False
            if name == "disarm" and kwargs.get("commanded_armed") is False:
                state = self.readVehicleState()
                if not (state.connected and state.landed_state == 1
                        and state.landed_age_s is not None and 0. <= state.landed_age_s <= .2):
                    return False
                kwargs["isArming"] = False
            return MavlinkPx4Backend._sendCommandLong(self, name, command, **kwargs)

        def sendSetpoint(self, setpoint, type_mask=None, *, verticalLift=False):
            ready, _ = self.localEstimatorReady()
            if not ready or verticalLift or type_mask not in (2503, 2055) or not gazeboScope(px4, px4.pid):
                return False
            return MavlinkPx4Backend.sendSetpoint(self, setpoint, type_mask)

        def refresh(self):
            from boom_birds_control.px4_backend import VehicleState, px4CustomMode, px4CustomMainModeName, px4CustomSubModeName
            topics = ("vehicle_status", "vehicle_attitude", "vehicle_local_position",
                      "vehicle_land_detected", "estimator_status_flags")
            status, attitude, pos, landed, flags = self.samplePool.map(px4.sample, topics)
            nav = int(field(status, "nav_state"))
            main, sub = {0:(1,0), 1:(2,0), 2:(3,0), 4:(4,3), 12:(4,6), 14:(6,0),
                         15:(7,0), 18:(4,6)}.get(nav,(0,0))
            custom = px4CustomMode(main, sub)
            q = [float(x) for x in re.search(r"q: \[([^]]+)\]", attitude).group(1).split(",")]
            w,x,y,z = q
            roll = math.atan2(2.*(w*x+y*z),1.-2.*(x*x+y*y))
            pitch = math.asin(max(-1.,min(1.,2.*(w*y-z*x))))
            yaw = math.atan2(2.*(w*z+x*y),1.-2.*(y*y+z*z))
            position = tuple(float(field(pos,a)) for a in ("x","y","z"))
            velocity = tuple(float(field(pos,a)) for a in ("vx","vy","vz"))
            counters = tuple(int(field(pos,a)) for a in ("xy_reset_counter","z_reset_counter",
                                                          "vxy_reset_counter","vz_reset_counter","heading_reset_counter"))
            boot = int(field(pos,"timestamp"))//1000
            if self._direct is not None and boot <= self._direct.boot_time_ms:
                return
            if self._counters is not None and self._counters != counters:
                acceptedRefusion = False
                if (getattr(self, "inertialTurn", False)
                        and getattr(self, "inertialRebase", None) is None
                        and self._direct is not None):
                    from arena_lap_gazebo import visualRefusionReset
                    positionShift = tuple(new-old for old, new in zip(self._direct.position_ned_m, position))
                    velocityShift = tuple(new-old for old, new in zip(self._direct.velocity_ned_m_s, velocity))
                    if visualRefusionReset(self._counters, counters, positionShift, velocityShift):
                        # The parallel flag listener can sample just before the position reset.
                        flags = px4.sample("estimator_status_flags")
                        acceptedRefusion = all(field(flags, name) == "True"
                                               for name in ("cs_ev_pos", "cs_ev_vel", "cs_ev_hgt"))
                        if acceptedRefusion:
                            self.inertialRebase = (positionShift, velocityShift)
                if not acceptedRefusion:
                    self._directEpoch += 1
                    self._resetAt = self._clock()
            self._counters = counters
            now = self._clock()
            self._directAt = now
            self._resetAt = getattr(self, "_resetAt", now)
            self._direct = VehicleState(
                connected=True, armed=field(status,"arming_state")=="2", mode_name=px4CustomMainModeName(custom),
                custom_mode=custom, custom_main_mode=main, custom_sub_mode=sub,
                current_mode_detail=px4CustomSubModeName(custom), intended_mode_detail=px4CustomSubModeName(custom),
                current_mode_age_s=0., landed_state=1 if field(landed,"landed")=="True" else 2,
                position_ned_m=position,velocity_ned_m_s=velocity, roll_rad=roll,pitch_rad=pitch,yaw_rad=yaw,
                attitude_received_mono_s=now,position_received_mono_s=now,
                heartbeat_age_s=0.,attitude_age_s=0.,position_age_s=0.,landed_age_s=0.,
                frame_reset_epoch=self._directEpoch,frame_reset_age_s=0.,boot_time_ms=boot,
                source="gazebo_px4_uorb")
            aiding = 1
            if field(pos,"v_xy_valid")=="True": aiding |= 2
            if field(pos,"xy_valid")=="True": aiding |= 8
            if field(pos,"v_z_valid")=="True": aiding |= 4
            self.zValid = field(pos,"z_valid") == "True"
            if field(flags,"cs_rng_hgt")=="True": aiding |= 32
            if field(pos,"dist_bottom_valid")=="True": aiding |= 64
            self.rangeHeight = field(flags,"cs_rng_hgt") == "True"
            self.vioHeight = field(flags,"cs_ev_hgt") == "True"
            self.vioPosition = field(flags,"cs_ev_pos") == "True"
            self.vioVelocity = field(flags,"cs_ev_vel") == "True"
            self.opticalFlow = field(flags,"cs_opt_flow") == "True"
            if self.opticalFlow or self.vioVelocity: aiding |= 256
            self._velocityEstimate = (now,aiding,0.,(0,self._directEpoch))

        def localEstimatorReady(self):
            if getattr(self, "inertialTurn", False):
                if not self._localVisionDisabled():
                    return False, "gazebo_scope_lost"
                return self.velocityControlReady()
            return super().localEstimatorReady()

        def _localVisionDisabled(self):
            return gazeboScope(px4, px4.pid)

        def isConnected(self):
            if self._direct is None or not self._endpointVerified or not gazeboScope(px4, px4.pid):
                return False
            simAge = simTime() - self._direct.boot_time_ms / 1000.
            wallAge = self._clock() - self._directAt
            return -.02 <= simAge <= .20 and 0. <= wallAge <= 1.2

        def velocityControlReady(self):
            state = self.readVehicleState()
            sample = self._velocityEstimate
            if (not state.connected or sample is None
                    or sample[3] != (state.restart_epoch, state.frame_reset_epoch)
                    or state.velocity_ned_m_s is None
                    or not all(math.isfinite(v) for v in state.velocity_ned_m_s)):
                return False, "px4_velocity_evidence_missing_or_stale"
            if sample[1] & 7 != 7:
                return False, "px4_velocity_estimator_invalid"
            return True, "ready"

        def readVehicleState(self):
            if self._direct is None:
                return super().readVehicleState()
            age = max(0., simTime()-self._direct.boot_time_ms / 1000.)
            return replace(self._direct,connected=self.isConnected(),
                           heartbeat_age_s=age,attitude_age_s=age,position_age_s=age,
                           current_mode_age_s=age,landed_age_s=age,
                           frame_reset_age_s=self._clock()-self._resetAt)

        def localFeedbackStamp(self, state):
            return state.boot_time_ms * 1_000_000

        def linkState(self):
            state = self.readVehicleState()
            return {"connected": state.connected, "heartbeat_age_s": state.heartbeat_age_s,
                    "restart_epoch": state.restart_epoch}

        def attitudeAt(self, observedMono, toleranceS):
            if self._direct is None or abs(self._directAt-observedMono)>toleranceS:
                return None
            return self._direct.roll_rad, self._direct.pitch_rad, self._direct.yaw_rad

        def hoverEstimatorReady(self):
            if alwaysOn and self._linkIsLoopback and self._endpointVerified and gazeboScope(px4, px4.pid):
                return True, "gazebo_px4_hte_always_on"
            return super().hoverEstimatorReady()

    return LocalGazeboBackend


class LocalPublisher:
    def __init__(self, publisher, receive):
        self.publisher = publisher
        self.receive = receive

    def publish(self, message):
        # Keep the public ROS diagnostics and deliver the sample within this SITL process.
        self.receive(message)
        self.publisher.publish(message)


class LocalVehicleAction:
    def __init__(self, control, record):
        self.control = control
        self.record = record

    def service_is_ready(self):
        return True

    def call_async(self, request):
        future = Future()
        response = self.control._vehicleAction(request, VehicleAction.Response())
        self.record("vehicle_action", action=request.action, accepted=response.accepted, reason=response.reason)
        future.set_result(response)
        return future


def removeSubscription(node, topic):
    for subscription in tuple(node.subscriptions):
        if subscription.topic_name == topic:
            node.destroy_subscription(subscription)


def runLanding(px4, bootstrap, continuousDescent, decodeLossHeight, taskNumber, *, baseTurnCheck=False):
    roundTrip = taskNumber != "1"
    velocity, camera, taskFile = profiles(px4, continuousDescent, taskNumber, baseTurnCheck=baseTurnCheck)
    with patch.object(mavros_backend, "MavrosPx4Backend", gazeboBackend(px4, lambda: bootstrap.simTime)):
        control = Px4InterfaceNode(**parameters(
            control_mode="px4_velocity", attitude_config_file=velocity, backend="mavros",
            use_sim_time=True, dry_run=False, require_session=True, allow_arming=False,
            px4_local_feedback=True, airborne_landing=True, external_vision=False,
            fcu_url="udp://127.0.0.1:14541@127.0.0.1:14581"))
    if not control.set_parameters([Parameter("sih_pid", value=px4.pid)])[0].successful:
        raise RuntimeError("failed to bind the isolated simulator PID")
    mission = TagFlightNode(**parameters(
        control_mode="px4_velocity", attitude_config_file=velocity, tag_config_file=camera,
        flight_task_file=taskFile,
        obstacle_config_file=str(SRC / "boom_birds_bringup/config/tag_obstacle_guard.yaml"),
        use_sim_time=True, dry_run=False, tag_test_only=True, sih_pid=px4.pid,
        px4_local_feedback=True, airborne_landing=True, handover_enabled=False,
        feature_tracking_enabled=True))
    if roundTrip:
        steadyClock = Clock(clock_type=ClockType.STEADY_TIME)
        # Arrival watchdogs use wall time while targets and sensor samples use Gazebo time.
        for node in (control, mission):
            for timer in tuple(node.timers):
                if timer.callback == node._tick:
                    node.destroy_timer(timer)
                    node.create_timer(.04, node._tick, clock=steadyClock)
                    break
            else:
                raise RuntimeError("Task 2 control timer could not be rebound")
    mission.rangeOptionalInTransit = roundTrip
    mission.fsm.enforceTaskDeadlines = False
    mission.fsm.motionClock = lambda: bootstrap.simTime
    hoverHeight = mission.fsm.task["height_m"]
    imuTurn = None
    if taskNumber == "3":
        from arena_lap_gazebo import ImuTurnRecovery
        imuTurn = ImuTurnRecovery()
        mission.fsm.airborneTagSearchHeight = .52
    observer = PlatformObservationNode(**parameters(
        config_file=camera, test_only=True, feature_tracking_enabled=True, use_sim_time=True,
        process_latest_only=False, opencv_threads=1))
    if decodeLossHeight is not None:
        detect = observer.detector.findCorners
        dropped = False

        def findCorners(image):
            nonlocal dropped
            height, _ = bootstrap.rangeEstimate()
            if (not dropped and mission.fsm.phase == "TAG_DESCEND"
                    and height is not None and height <= decodeLossHeight):
                bootstrap.record("tag_decoder_disabled", tof_m=height)
                dropped = True
            if dropped:
                return [], 0
            return detect(image)

        observer.detector.findCorners = findCorners
    probe = Node("task1_gazebo")
    if roundTrip:
        from task2_gazebo_vio import VioHeightAid
        vio = VioHeightAid(px4, probe)
        if taskNumber == "3":
            vio.recordTrajectory(probe)
    else:
        vio = None
    executor = SingleThreadedExecutor()
    nodes = (control, mission, observer, probe)
    for node in nodes:
        executor.add_node(node)
    for node in (control, mission, observer):
        node._time_source.detach_node()

    for topic in (DEFAULTS.status_topic, "/boom_birds/px4/local_pose",
                  "/boom_birds/downward/image", "/boom_birds/platform/observation",
                  "/boom_birds/platform/feature_observation"):
        removeSubscription(mission, topic)
    removeSubscription(control, DEFAULTS.command_topic)
    observer.destroy_subscription(observer.sub)
    control.pubExecution = LocalPublisher(control.pubExecution, mission._controlStatus)
    control._localPosePub = LocalPublisher(control._localPosePub, mission.onPose)
    mission.pubCommand = LocalPublisher(mission.pubCommand, control._onControlCommand)
    observer.pub = LocalPublisher(observer.pub, mission.onTag)
    observer.featurePub = LocalPublisher(observer.featurePub, mission.onFeature)
    mission.client = LocalVehicleAction(control, bootstrap.record)

    def controlDiagnostics(message):
        data = json.loads(message.data)
        if control.ingress.session:
            outcome = control._lastOutcome
            bootstrap.record("control", reasons=outcome.reasons,
                             sent=outcome.sent, protocol=control._protocolReason,
                             last_output=data.get("last_control_output_error"),
                             feedback=data.get("velocity_controller"))

    control.pubStatus = LocalPublisher(control.pubStatus, controlDiagnostics)

    from gz.msgs10.image_pb2 import Image as GazeboImage
    if not re.search(r": 0\s*\n", px4.command("param", "show", "EKF2_EV_CTRL")):
        raise RuntimeError("unexpected PX4 external vision aiding")

    # The camera relay shares the task process, avoiding large UDP image fragments across DDS.
    frames = deque(maxlen=12)
    if not bootstrap.node.subscribe(GazeboImage, "/aircraft_340g/downward/image", frames.append):
        raise RuntimeError("could not subscribe to the native downward camera")
    cameraPublisher = probe.create_publisher(Image, "/boom_birds/downward/image", qos_profile_sensor_data)
    lastFrame = None
    lastHeartbeatFrame = None
    lastRange = None
    from gz.msgs10.laserscan_pb2 import LaserScan
    ranges = deque(maxlen=1)
    if not bootstrap.node.subscribe(LaserScan, "/aircraft_340g/down_range", ranges.append):
        raise RuntimeError("could not subscribe to the native ToF")
    removeSubscription(mission, "/boom_birds/platform/range")

    inertialStartPose = None
    inertialStartYaw = None
    inertialScanSince = None
    inertialVerticalExcessSince = None
    inertialScanAngle = 0.
    inertialDirection = 1.
    inertialDrift = .20

    def nativeInputs():
        nonlocal lastFrame, lastHeartbeatFrame, lastRange, inertialStartPose
        if time.monotonic()-control.backend._directAt >= .025:
            control.backend.refresh()
            backend = control.backend
            rebase = getattr(backend, "inertialRebase", None)
            if rebase is not None and not backend.inertialRebaseRecorded:
                positionShift, velocityShift = rebase
                east, north, up = mission.fsm.inertialTurnHold
                dn, de, _ = positionShift
                mission.fsm.inertialTurnHold = (east + de, north + dn, up)
                if mission.fsm.phase in ("HOLD", "TURN"):
                    goalX, goalY, goalZ = mission.fsm.segmentGoal
                    mission.fsm.segmentGoal = mission.fsm.hold = (goalX+de, goalY+dn, goalZ)
                startNorth, startEast, startDown = inertialStartPose
                inertialStartPose = (startNorth + dn, startEast + de, startDown)
                mission.tracker.clear()
                mission.fsm.lastTagSearch = None
                # The scoped backend just verified a newer pose in the same control epoch.
                control._updateLocalFeedback()
                backend.inertialRebaseRecorded = True
                bootstrap.record("arena_imu_turn_xy_rebase", shift_m=math.hypot(dn, de),
                                 velocity_shift_m_s=math.hypot(*velocityShift))
        instant = Time(nanoseconds=round(bootstrap.simTime * 1e9))
        for node in (control, mission, observer):
            node.get_clock().set_ros_time_override(instant)
        if ranges:
            scan = ranges[-1]
            stamp = (scan.header.stamp.sec, scan.header.stamp.nsec)
            if scan.ranges and stamp != lastRange and (stamp[0]+stamp[1]*1e-9) <= bootstrap.simTime:
                lastRange = stamp
                message = Range()
                message.header.stamp.sec, message.header.stamp.nanosec = stamp
                message.header.frame_id = "range_sensor"
                message.range, message.min_range, message.max_range = scan.ranges[0], scan.range_min, scan.range_max
                mission.onRange(message)
        if frames:
            tolerance = mission.tracker.pairTolerance
            poses = mission.tracker.poses
            measurements = mission.tracker.ranges
            for frame in reversed(tuple(frames)):
                stamp = (frame.header.stamp.sec, frame.header.stamp.nsec)
                stampS = stamp[0] + stamp[1]*1e-9
                if lastFrame is not None and stamp <= lastFrame:
                    break
                if not (frame.step == frame.width and .025 <= bootstrap.simTime-stampS <= .11):
                    continue
                heartbeat = lastHeartbeatFrame is None or stamp > lastHeartbeatFrame
                paired = (any(abs(item[0]-stampS) <= tolerance for item in poses)
                          and ((roundTrip and mission.fsm.phase in ("RETURN_LANDING", "TAG_ACQUIRE"))
                               or any(abs(item[0]-stampS) <= tolerance for item in measurements)))
                if not (heartbeat or paired):
                    continue
                message = Image()
                message.header.stamp.sec, message.header.stamp.nanosec = stamp
                message.header.frame_id = "downward_optical"
                message.height, message.width, message.step = frame.height, frame.width, frame.step
                message.encoding = "mono8"
                message.data = frame.data
                if heartbeat:
                    lastHeartbeatFrame = stamp
                    mission.onDownwardImage(message)
                    cameraPublisher.publish(message)
                if paired:
                    lastFrame = stamp
                    observer.onImage(message)
                    break

    taskStatus = {}
    def taskUpdate(msg):
        nonlocal taskStatus
        taskStatus = json.loads(msg.data)
        bootstrap.record("task", **{key: value for key, value in taskStatus.items() if key != "stage"})

    mission.pubTask = LocalPublisher(mission.pubTask, taskUpdate)
    hoverImage = px4.runDirectory / "downward-hover.png"
    def saveDownward(msg):
        if hoverImage.exists() or msg.encoding != "mono8":
            return
        height, _ = bootstrap.rangeEstimate()
        if height is None or abs(height - hoverHeight) > .08:
            return
        import cv2
        import numpy as np
        image = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height,msg.step)[:, :msg.width]
        cv2.imwrite(str(hoverImage), image)

    probe.create_subscription(Image, "/boom_birds/downward/image", saveDownward, qos_profile_sensor_data)

    startRequest = None
    offboardRequested = False
    posctlRequestedAt = None
    posctlConfirmed = False
    centeredSince = None
    centeredReported = False
    lastReport = -math.inf
    hoverTagReported = False

    def hoverTagTarget():
        nonlocal hoverTagReported
        nativeInputs()
        executor.spin_once(timeout_sec=0.)
        fix = mission.fsm.fix
        if not (fix is not None and fix.source in ("tag", "resolved_tag") and fix.yaw_valid
                and 0. <= time.monotonic()-fix.stamp <= .25 and .15 <= fix.height <= .95):
            return None
        state = control.backend.readVehicleState()
        if state.position_ned_m is None:
            return None
        north, east, _ = state.position_ned_m
        if math.hypot(east-fix.target[0], north-fix.target[1]) > .30:
            return None
        if not hoverTagReported:
            bootstrap.record("hover_tag_reference", target=fix.target, height_m=fix.height)
            hoverTagReported = True
        return fix.target

    try:
        if vio is not None:
            bootstrap.pumpUntil(.10, lambda: executor.spin_once(timeout_sec=0.))
            initDeadline = bootstrap.simTime + 12.
            while not vio.initialized(bootstrap.simTime):
                if bootstrap.simTime >= initDeadline:
                    bootstrap.record("visual_initialization_failed", **vio.diagnostics(bootstrap.simTime))
                    raise RuntimeError("OpenVINS did not initialize while the aircraft was stationary")
                bootstrap.pumpUntil(.10, lambda: executor.spin_once(timeout_sec=0.))
            bootstrap.record("visual_initialized_on_pad")
        bootstrap.hover(*bootstrap.start(), visualTarget=hoverTagTarget, targetHeight=hoverHeight)
        cruiseHeightReady = vio is None
        rangeRestored = False
        fusedSince = None
        terrainRequested = False
        visionLostSince = None
        horizontalVioReady = taskNumber != "3"
        lastSimProgress = bootstrap.simTime
        lastClockProgress = time.monotonic()
        flowEnabled = True
        reportedVisionWait = False
        recordedTagHint = None
        while rclpy.ok():
            px4.verify()
            px4.neutralPilot()
            nativeInputs()
            px4.neutralPilot()
            executor.spin_once(timeout_sec=.005)
            state = control.backend.readVehicleState()
            if vio is not None:
                now = bootstrap.simTime
                simStep = max(0., now - lastSimProgress)
                if now > lastSimProgress:
                    lastSimProgress = now
                    lastClockProgress = time.monotonic()
                elif time.monotonic() - lastClockProgress > 3.:
                    raise RuntimeError("Gazebo clock stalled during Task 2")
                if not cruiseHeightReady:
                    if vio.offsetDown is None and state.connected and state.position_ned_m is not None:
                        if vio.anchor(now, state.position_ned_m[2]):
                            px4.parameter("EKF2_EVV_NOISE", .05)
                            px4.parameter("EKF2_EV_CTRL", 6)
                            bootstrap.record("vio_height_input_enabled")
                        elif not reportedVisionWait:
                            bootstrap.record("waiting_for_visual_height")
                            reportedVisionWait = True
                    fused = vio.fresh(now) and control.backend.vioHeight and control.backend.vioVelocity
                    fusedSince = (now if fusedSince is None else fusedSince) if fused else None
                    if fusedSince is not None and now - fusedSince >= .35:
                        if not terrainRequested:
                            px4.parameter("EKF2_RNG_CTRL", 3)
                            terrainRequested = True
                        elif not control.backend.rangeHeight:
                            cruiseHeightReady = True
                            bootstrap.record("cruise_height_vision_only")
                elif not rangeRestored and startRequest is not None and mission.fsm.phase in ("TAG_ACQUIRE", "TAG_DESCEND"):
                    fix = mission.fsm.fix
                    height, _ = bootstrap.rangeEstimate()
                    if (fix is not None and fix.source in ("tag", "resolved_tag")
                            and 0. <= time.monotonic()-fix.stamp <= .25
                            and height is not None and abs(fix.height-height) <= .12
                            and state.position_ned_m is not None
                            and math.hypot(state.position_ned_m[1]-fix.target[0],
                                           state.position_ned_m[0]-fix.target[1]) <= .075):
                        px4.parameter("EKF2_RNG_CTRL", 1)
                        rangeRestored = True
                        bootstrap.record("pad_range_aiding_enabled", tof_m=height)
                flowWanted = (mission.fsm.phase != "TURN"
                              and not (imuTurn is not None and imuTurn.start is not None
                                       and inertialScanSince is not None))
                if cruiseHeightReady and flowWanted != flowEnabled:
                    px4.parameter("EKF2_OF_CTRL", int(flowWanted))
                    flowEnabled = flowWanted
                    bootstrap.record("transit_flow_aiding", enabled=flowEnabled)
                vio.tick(now)
                invalidHeight = (cruiseHeightReady and not rangeRestored
                                 and (not vio.fresh(now) or not control.backend.vioHeight
                                      or not control.backend.vioVelocity
                                      or (taskNumber == "3" and horizontalVioReady
                                          and not control.backend.vioPosition)))
                if imuTurn is not None:
                    backend = control.backend
                    lastVisualStamp = vio.status.get("last_visual_update_stamp_s")
                    recentCamera = (lastVisualStamp is not None
                                    and -.004 <= now-lastVisualStamp <= .50)
                    fsm = mission.fsm
                    if (fsm.phase in ("MOVE", "RETURN_LANDING") and imuTurn.start is None
                            and state.position_ned_m is not None and backend.zValid):
                        north, east, down = state.position_ned_m
                        reference, _, _ = fsm.holdSetpoint(time.monotonic())
                        dx = fsm.segmentGoal[0]-fsm.segmentStart[0]
                        dy = fsm.segmentGoal[1]-fsm.segmentStart[1]
                        length = math.hypot(dx, dy)
                        ahead = ((reference[0]-east)*dx + (reference[1]-north)*dy)/max(.001, length)
                        # Pause when the reference leads the vehicle, giving it time to catch up.
                        delayFraction = min(1., max(0., (ahead - .40)/.25))
                        fsm.stageMotionSince += simStep*delayFraction
                    navigating = (startRequest is not None and not rangeRestored
                                  and fsm.phase in ("HOLD", "MOVE", "TURN", "RETURN_LANDING")
                                  and state.mode_name == "offboard" and state.landed_state == 2)
                    speed = math.hypot(*state.velocity_ned_m_s[:2]) if state.velocity_ned_m_s else math.inf
                    slow = speed <= .18
                    verticalSpeed = abs(state.velocity_ned_m_s[2]) if state.velocity_ned_m_s else math.inf
                    if imuTurn.start is None or verticalSpeed <= .12:
                        inertialVerticalExcessSince = None
                    elif inertialVerticalExcessSince is None:
                        inertialVerticalExcessSince = now
                    verticalSafe = (verticalSpeed <= .12 or (
                        inertialVerticalExcessSince is not None and verticalSpeed <= .25
                        and now-inertialVerticalExcessSince < .15))
                    tiltSafe = (state.roll_rad is not None and state.pitch_rad is not None
                                and math.cos(state.roll_rad)*math.cos(state.pitch_rad) >= math.cos(math.radians(20.)))
                    scanReady = (slow and verticalSpeed <= .12 and tiltSafe
                                 and math.cos(state.roll_rad)*math.cos(state.pitch_rad) >= math.cos(math.radians(12.)))
                    yaw = math.pi/2 - state.yaw_rad if state.yaw_rad is not None else None
                    if yaw is not None and math.isfinite(yaw) and fsm.yawSetpoint is not None:
                        yaw = fsm.yawSetpoint + math.remainder(yaw-fsm.yawSetpoint, 2*math.pi)
                    safe = (navigating and state.connected and state.armed and yaw is not None
                            and math.isfinite(yaw) and backend.velocityControlReady()[0]
                            and bool(backend._velocityEstimate[1] & 8)
                            and backend.zValid and state.position_ned_m is not None
                            and all(math.isfinite(value) for value in state.position_ned_m)
                            and speed <= .85 and verticalSafe
                            and tiltSafe and not control.attitude_controller.latched)
                    if imuTurn.start is not None and safe:
                        offset = math.dist(inertialStartPose[:2], state.position_ned_m[:2])
                        yawTravel = abs(math.remainder(yaw-inertialStartYaw, 2*math.pi))
                        safe = (offset <= inertialDrift and yawTravel <= math.radians(85.)
                                and abs(inertialStartPose[2]-state.position_ned_m[2]) <= .12
                                and (inertialScanSince is not None
                                     or now-imuTurn.start <= .90 or scanReady))
                    recovery = imuTurn.update(now, navigating=navigating, moving=not slow,
                                              visualReady=not invalidHeight and recentCamera,
                                              visualUpdates=vio.status.get("visual_update_count", 0),
                                              safe=safe)
                    if recovery == "started":
                        north, east, down = state.position_ned_m
                        fsm.inertialTurnHold = (east, north, -down)
                        fsm.inertialTurnYaw = yaw
                        inertialStartPose, inertialStartYaw = state.position_ned_m, yaw
                        inertialScanSince = None
                        inertialScanAngle = 0.
                        inertialDrift = .20 + (.65 * speed if not slow else 0.)
                        headingChange = fsm.segmentYawGoal - fsm.yawSetpoint
                        if fsm.phase != "TURN":
                            nextStep = fsm.task["steps"][fsm.stageIndex+1]
                            if nextStep["action"] == "turn":
                                nextHeading = fsm.initialYaw + math.radians(nextStep["yaw_deg"])
                            else:
                                targetX, targetY = fsm.takeoffOrigin[:2]
                                nextHeading = math.atan2(targetY-north, targetX-east)
                            headingChange = math.remainder(nextHeading-yaw, 2*math.pi)
                        inertialDirection = math.copysign(1., headingChange)
                        backend.inertialRebase = None
                        backend.inertialRebaseRecorded = False
                        backend.inertialTurn = True
                        bootstrap.record("arena_imu_turn_started", phase=fsm.phase, speed_m_s=speed,
                                         last_visual_s=lastVisualStamp)
                    elif recovery == "recovered":
                        north, east, down = state.position_ned_m
                        fsm.inertialTurnHold = (east, north, -down)
                        fsm.resumeInertialHold(time.monotonic(), yaw)
                        backend.inertialTurn = False
                        bootstrap.record("arena_imu_turn_recovered", phase=fsm.phase,
                                         last_visual_s=lastVisualStamp)
                    elif recovery == "failed":
                        bootstrap.record("arena_imu_turn_failed", phase=fsm.phase, local_state_safe=safe,
                                         speed_m_s=speed, vertical_speed_m_s=verticalSpeed, last_visual_s=lastVisualStamp,
                                         connected=state.connected, armed=state.armed, mode=state.mode_name,
                                         estimator=backend.velocityControlReady(), z_valid=backend.zValid,
                                         control_fault=control.attitude_controller.latched,
                                         pose_ned=state.position_ned_m, start_ned=inertialStartPose,
                                         attitude_rad=(state.roll_rad, state.pitch_rad, yaw),
                                         search_start_yaw_rad=inertialStartYaw)
                        raise RuntimeError("inertial navigation recovery ended without safe visual feedback")
                    if recovery in ("started", "scanning") and scanReady:
                        if inertialScanSince is None:
                            inertialScanSince = now
                            bootstrap.record("arena_imu_turn_scan", phase=fsm.phase)
                        if not (not invalidHeight and recentCamera):
                            inertialScanAngle = min(math.radians(80.), inertialScanAngle+simStep*.55)
                            fsm.inertialTurnYaw = inertialStartYaw + inertialDirection*inertialScanAngle
                    if recovery in ("started", "scanning"):
                        invalidHeight = False
                if invalidHeight:
                    if visionLostSince is None:
                        visionLostSince = now
                        bootstrap.record("visual_height_degraded", **vio.diagnostics(now))
                    elif now - visionLostSince >= .15:
                        bootstrap.record("visual_height_lost", **vio.diagnostics(now))
                        raise RuntimeError("VIO height or velocity is no longer valid during transit")
                else:
                    visionLostSince = None
            hint = mission.fsm.lastTagSearch
            if hint is not None and (recordedTagHint is None or hint[0]-recordedTagHint[0] >= .5
                                     or hint[4] != recordedTagHint[4]):
                bootstrap.record("arena_tag_search_hint", observed_s=hint[0], tag_world_m=hint[1],
                                 aircraft_world_m=hint[2], source=hint[4])
                recordedTagHint = hint
            if bootstrap.simTime-lastReport >= .4:
                lastReport = bootstrap.simTime
                bootstrap.record("landing_status", mode=state.mode_name, armed=state.armed,
                                 landed=state.landed_state, range_m=bootstrap.rangeEstimate()[0],
                                 ready=control._localFeedbackReady(), endpoint=control.backend._endpointVerified,
                                 tag_ready=taskStatus.get("airborne_tag_ready"),
                                 control_fault=control.attitude_controller.latched,
                                 pose_stamp=control._localPoseStamp, clock=control.get_clock().now().nanoseconds)
            if startRequest is None:
                if not cruiseHeightReady:
                    bootstrap.holdVelocity(hoverTagTarget(), rangeHeight=hoverHeight)
                    continue
                if state.mode_name == "posctl":
                    if not posctlConfirmed:
                        bootstrap.record("posctl_handoff_confirmed")
                    posctlConfirmed = True
                if posctlConfirmed and not horizontalVioReady:
                    if vio.positionAnchor is None and state.position_ned_m is not None:
                        if vio.anchorHorizontal(bootstrap.simTime, state.position_ned_m, state.yaw_rad):
                            px4.parameter("EKF2_EVP_NOISE", .09)
                            px4.parameter("EKF2_EV_CTRL", 7)
                            bootstrap.record("arena_horizontal_vision_enabled")
                    elif control.backend.vioPosition:
                        horizontalVioReady = True
                        centeredSince = None
                        bootstrap.record("arena_horizontal_vision_fused")
                    if not horizontalVioReady:
                        continue
                fix = mission.fsm.fix
                tagReady = bool(taskStatus.get("landing_inputs_ready") and taskStatus.get("airborne_tag_ready")
                                and fix is not None and fix.source in ("tag", "resolved_tag"))
                target = fix.target if tagReady else None
                centered = False
                if state.position_ned_m is not None and state.velocity_ned_m_s is not None and target is not None:
                    north, east, _ = state.position_ned_m
                    centered = (math.hypot(east-target[0], north-target[1]) <= .035
                                and math.hypot(*state.velocity_ned_m_s[:2]) <= .05)
                    if roundTrip:
                        mission.fsm.airborneRouteGroundZ = target[2]
                if centered:
                    centeredSince = time.monotonic() if centeredSince is None else centeredSince
                else:
                    centeredSince = None
                stableCentered = centeredSince is not None and time.monotonic()-centeredSince >= .35
                if not posctlConfirmed:
                    bootstrap.holdVelocity(target, rangeHeight=hoverHeight if vio is not None else None)
                if vio is not None and control.attitude_controller.latched:
                    raise RuntimeError(f"Task 2 position feedback: {control.attitude_controller.latched}")
                if stableCentered and not centeredReported:
                    bootstrap.record("tag_centered", target=target)
                    centeredReported = True
                if tagReady and stableCentered:
                    if posctlConfirmed and state.connected and state.mode_name == "posctl":
                        startRequest = mission._mission(Mission.Request(action=Mission.Request.START), Mission.Response())
                        bootstrap.record("task_start", accepted=startRequest.accepted, reason=startRequest.reason)
                    elif posctlRequestedAt is None or time.monotonic()-posctlRequestedAt >= 2.:
                        px4.command("commander", "mode", "posctl")
                        posctlRequestedAt = time.monotonic()
                        bootstrap.record("posctl_handoff_requested")
                continue
            if not startRequest.accepted:
                raise RuntimeError(f"Task {taskNumber} START rejected: {startRequest.reason}")
            if taskStatus.get("phase") == "WAIT_OFFBOARD" and not offboardRequested:
                px4.command("commander", "mode", "offboard")
                offboardRequested = True
                bootstrap.record("landing_offboard_requested")
            if taskStatus.get("state") == "FAULT_LATCHED":
                raise RuntimeError(f"Task {taskNumber} landing: {taskStatus.get('reason')}")
            if taskStatus.get("complete") and taskStatus.get("landed_disarmed"):
                bootstrap.record("complete", phase=taskStatus.get("phase"), landed=True, armed=False)
                return
        raise RuntimeError(f"Task {taskNumber} ROS context stopped")
    finally:
        executor.shutdown(timeout_sec=3)
        control.backend.samplePool.shutdown()
        for node in reversed(nodes):
            node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--px4", type=Path, default=Path.home()/"PX4-Autopilot/build/px4_sitl_default/bin/px4")
    parser.add_argument("--task", choices=("1", "2", "3"), default="1")
    parser.add_argument("--trace", type=lambda value: Path(value).name)
    parser.add_argument("--continuous-descent", action="store_true", help="test the 0.16 m moving handoff")
    parser.add_argument("--test-tag-decode-loss-below", type=float, metavar="METRES",
                        help="disable Tag decoding at this ToF distance, leaving camera frames intact")
    parser.add_argument("--test-base-turn", action="store_true",
                        help="Task 3: fly directly to the red base to exercise the outward head turn")
    args = parser.parse_args()
    if args.test_base_turn and args.task != "3":
        parser.error("--test-base-turn requires Task 3")
    if args.test_tag_decode_loss_below is not None and (
            not args.continuous_descent or not .16 < args.test_tag_decode_loss_below <= .35):
        parser.error("decoder-loss injection requires continuous descent and a distance in (0.16, 0.35] m")

    os.environ["MAVLINK20"] = "1"
    px4 = GazeboTask1Px4(args.run_directory, args.px4)
    taskMarker = px4.runDirectory / "task-number"
    preparedTask = taskMarker.read_text().strip() if taskMarker.exists() else "1"
    if args.task != preparedTask:
        parser.error(f"Gazebo scene was prepared for Task {preparedTask}")
    for stream, rate in (("HEARTBEAT",12), ("EXTENDED_SYS_STATE",20), ("ESTIMATOR_STATUS",25),
                         ("LOCAL_POSITION_NED",30), ("ATTITUDE",30)):
        px4.command("mavlink", "stream", "-u", "14581", "-s", stream, "-r", rate)
    portFile = px4.runDirectory / "mavros.pid"
    pid = int(portFile.read_text())
    inherited = dict(item.split(b"=",1) for item in (Path("/proc") / str(pid) / "environ").read_bytes().split(b"\0") if b"=" in item)
    if inherited.get(b"GZ_PARTITION") != px4.partition.encode():
        raise RuntimeError("MAVROS is outside the Gazebo session")
    for key in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_AUTOMATIC_DISCOVERY_RANGE", "FASTDDS_BUILTIN_TRANSPORTS",
                "FASTDDS_DEFAULT_PROFILES_FILE", "FASTRTPS_DEFAULT_PROFILES_FILE"):
        if key.encode() in inherited:
            os.environ[key] = inherited[key.encode()].decode()
        else:
            os.environ.pop(key, None)
    traceName = args.trace or f"task{args.task}-integrated.jsonl"
    with (px4.runDirectory/traceName).open("x") as trace:
        bootstrap = ImuTakeoff(px4, trace)
        rclpy.init()
        try:
            with patch.object(sih_guard, "verifySihProcess", lambda pid: gazeboScope(px4, pid)):
                if args.continuous_descent:
                    px4.parameter("MPC_LAND_CRWL", 0.20)
                runLanding(px4, bootstrap, args.continuous_descent, args.test_tag_decode_loss_below, args.task,
                           baseTurnCheck=args.test_base_turn)
        except (Exception, KeyboardInterrupt) as exc:
            bootstrap.record("failed", reason=str(exc) or "interrupted")
            bootstrap.abort()
            raise
        finally:
            for topic in bootstrap.node.subscribed_topics():
                bootstrap.node.unsubscribe(topic)
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
