"""核验本机 PX4 SITL 会话，通过模拟 IMU、光流与 ToF 完成起步。"""

from collections import deque
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time


class GazeboTask1Px4:
    def __init__(self, runDirectory, binary):
        from pymavlink import mavutil

        self.runDirectory = Path(runDirectory).resolve(strict=True)
        self.binary = Path(binary).resolve(strict=True)
        self.pid = int((self.runDirectory / "px4.pid").read_text())
        self.partition = (self.runDirectory / "partition").read_text().strip()
        self.verify()
        self.wire = mavutil.mavlink_connection(
            "udpout:127.0.0.1:18571", source_system=246, source_component=190)
        self.mav = mavutil.mavlink
        self.system = 2
        self.component = 1

    def verify(self):
        proc = Path("/proc") / str(self.pid)
        env = (proc / "environ").read_bytes().split(b"\0")
        argv = (proc / "cmdline").read_bytes().split(b"\0")
        if not ((proc / "exe").resolve(strict=True) == self.binary
                and self.binary.name == "px4" and "px4_sitl_default" in self.binary.parts
                and (proc / "cwd").resolve(strict=True) == self.runDirectory
                and argv[-3:] == [b"-i", b"1", b""]
                and re.fullmatch(r"bb_task1_[0-9]+", self.partition)
                and ("GZ_PARTITION=" + self.partition).encode() in env
                and b"GZ_IP=127.0.0.1" in env
                and b"PX4_GZ_STANDALONE=1" in env
                and b"PX4_GZ_MODEL_NAME=aircraft_340g" in env):
            raise RuntimeError("Task 1 requires the isolated Gazebo PX4 SITL instance")

    def command(self, module, *args):
        self.verify()
        command = self.binary.with_name("px4-" + module)
        return subprocess.check_output(
            [str(command), "--instance", "1", *map(str, args)], text=True, timeout=2)

    def sample(self, topic):
        return self.command("listener", topic, "-n", "1")

    def parameter(self, name, value):
        self.command("param", "set", name, value)
        text = self.command("param", "show", name)
        found = re.search(r":\s*(-?\d+(?:\.\d+)?)\s*$", text, re.MULTILINE)
        if found is None or float(found.group(1)) != value:
            raise RuntimeError(f"PX4 parameter readback failed: {name}")

    def attitude(self, quaternion, thrust):
        self.wire.mav.set_attitude_target_send(
            0, self.system, self.component, 7, quaternion, 0., 0., 0., thrust)

    def velocity(self, vx, vy, vz, yaw):
        self.wire.mav.set_position_target_local_ned_send(
            0, self.system, self.component, self.mav.MAV_FRAME_LOCAL_NED,
            2503, 0., 0., 0., vx, vy, vz, 0., 0., 0., yaw, 0.)

    def neutralPilot(self):
        self.wire.mav.manual_control_send(self.system, 0, 0, 500, 0, 0)


def bounded(value, lower, upper):
    return min(max(value, lower), upper)


def field(report, label):
    match = re.search(rf"^    {label}: ([^\n]+)", report, re.MULTILINE)
    if match is None:
        raise ValueError(f"PX4 field {label} is unavailable")
    return match.group(1).split()[0]


class ImuTakeoff:
    """Take off in attitude mode, then hand control to PX4 velocity using estimator evidence."""

    def __init__(self, px4, trace):
        from gz.msgs10.clock_pb2 import Clock
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.transport13 import Node

        self.px4 = px4
        self.trace = trace
        self.simTime = 0.
        self.ranges = deque(maxlen=8)
        os.environ.update(GZ_PARTITION=px4.partition, GZ_IP="127.0.0.1")
        self.node = Node()
        if not self.node.subscribe(Clock, "/world/rmuc2027_arena/clock", self.onClock):
            raise RuntimeError("Gazebo clock unavailable")
        if not self.node.subscribe(LaserScan, "/aircraft_340g/down_range", self.onRange):
            raise RuntimeError("Gazebo ToF unavailable")

    def onClock(self, msg):
        self.simTime = msg.sim.sec + msg.sim.nsec / 1e9

    def onRange(self, msg):
        if msg.ranges and math.isfinite(msg.ranges[0]):
            stamp = msg.header.stamp.sec + msg.header.stamp.nsec / 1e9
            self.ranges.append((stamp, msg.ranges[0]))

    def rangeEstimate(self):
        samples = tuple(self.ranges)
        if not samples or not 0 <= self.simTime - samples[-1][0] <= .25:
            return None, None
        duration = samples[-1][0] - samples[0][0]
        speed = (samples[-1][1] - samples[0][1]) / duration if duration > .05 else 0.
        return samples[-1][1], speed

    def record(self, stage, **fields):
        print(json.dumps(dict(stage=stage, sim_s=self.simTime, **fields)), file=self.trace, flush=True)

    def preflight(self):
        status = self.px4.sample("vehicle_status")
        land = self.px4.sample("vehicle_land_detected")
        attitude = self.px4.sample("vehicle_attitude")
        flags = self.px4.sample("failsafe_flags")
        if not (field(status, "arming_state") == "1" and field(land, "landed") == "True"
                and field(flags, "attitude_invalid") == "False"
                and field(flags, "local_altitude_invalid") == "True"
                and self.rangeEstimate()[0] is None):
            raise RuntimeError("expected landed, disarmed PX4 with IMU attitude and ToF below range")
        for name in ("SYS_HAS_MAG", "SYS_HAS_BARO", "SIM_GZ_EN_MAG", "SIM_GZ_EN_BARO"):
            if not re.search(r": 0\s*\n", self.px4.command("param", "show", name)):
                raise RuntimeError(f"{name} must be zero")
        q = [float(x) for x in re.search(r"q: \[([^]]+)\]", attitude).group(1).split(",")]
        self.yaw = math.atan2(2. * (q[0]*q[3] + q[1]*q[2]), 1. - 2.*(q[2]**2 + q[3]**2))
        self.level = [math.cos(self.yaw/2), 0., 0., math.sin(self.yaw/2)]
        self.record("preflight", range_valid=False, altitude_valid=False, attitude_valid=True)

    def pumpUntil(self, duration, action):
        limit = self.simTime + duration
        progress = self.simTime
        lastAdvance = time.monotonic()
        while self.simTime < limit:
            if self.simTime > progress:
                progress = self.simTime
                lastAdvance = time.monotonic()
            elif time.monotonic() - lastAdvance > 6.:
                raise RuntimeError("Gazebo clock stalled")
            action()
            time.sleep(.02)

    def start(self):
        deadline = time.monotonic() + 10.
        while not self.simTime and time.monotonic() < deadline:
            time.sleep(.04)
        self.px4.parameter("COM_RC_IN_MODE", 1)
        limit = time.monotonic() + 12.
        while True:
            try:
                self.preflight()
                break
            except (RuntimeError, ValueError):
                if time.monotonic() >= limit:
                    raise
                time.sleep(.1)

        def prestream():
            self.px4.neutralPilot()
            self.px4.attitude(self.level, 0.)

        self.pumpUntil(1.5, prestream)
        self.px4.command("commander", "mode", "offboard")
        self.pumpUntil(.25, prestream)
        if field(self.px4.sample("vehicle_status"), "nav_state") != "14":
            raise RuntimeError("attitude Offboard was not confirmed")
        self.px4.command("commander", "arm")
        self.pumpUntil(.25, prestream)
        if field(self.px4.sample("vehicle_status"), "arming_state") != "2":
            raise RuntimeError("attitude-mode arming was not confirmed")
        self.record("armed_attitude")
        groundPhaseUntil = self.simTime + 1.1
        verticalReady = False
        aidedSince = None
        nextStatus = 0.
        deadline = self.simTime + 18.
        progress = self.simTime
        lastAdvance = time.monotonic()
        firstRange = None
        rangeControlStarted = False
        while self.simTime < deadline:
            self.px4.verify()
            if self.simTime > progress:
                progress = self.simTime
                lastAdvance = time.monotonic()
            elif time.monotonic() - lastAdvance > 6.:
                raise RuntimeError("Gazebo clock stalled")
            self.px4.neutralPilot()
            height, speed = self.rangeEstimate()
            if height is not None and firstRange is None:
                firstRange = self.simTime
            if height is not None and height >= .08:
                rangeControlStarted = True
            if rangeControlStarted:
                if height is None:
                    raise RuntimeError("ToF lost after liftoff")
                if height > .38:
                    raise RuntimeError("attitude bootstrap exceeded height bound")
                climb = bounded(1.3*(.26-height), -.13, .16)
                thrust = bounded(.55 + .14*(climb - speed), .48, .60)
            else:
                thrust = .40 if self.simTime < groundPhaseUntil else .585
            self.px4.attitude(self.level, thrust)
            if self.simTime >= nextStatus:
                nextStatus = self.simTime + .10
                flags = self.px4.sample("estimator_status_flags")
                pos = self.px4.sample("vehicle_local_position")
                vertical = (field(flags, "cs_rng_hgt") == "True"
                            and field(pos, "z_valid") == "True"
                            and field(pos, "v_z_valid") == "True")
                if vertical and not verticalReady:
                    self.record("range_fusion", range_m=height, local_z=float(field(pos,"z")))
                    verticalReady = True
                aided = (vertical and field(flags, "cs_opt_flow") == "True"
                         and field(pos, "v_xy_valid") == "True"
                         and height is not None and .10 <= height <= .34)
                aidedSince = (self.simTime if aidedSince is None else aidedSince) if aided else None
                self.record("imu_climb", range_m=height, range_rate_m_s=speed,
                            thrust=thrust, range_fusion=vertical, flow=field(flags,"cs_opt_flow"))
                if aidedSince is not None and self.simTime-aidedSince >= .12:
                    self.record("flow_fusion", range_m=height)
                    return pos, height
                if firstRange is None and self.simTime-groundPhaseUntil > 1.4:
                    raise RuntimeError("ToF did not become available during bounded lift")
                if firstRange is not None and not rangeControlStarted and self.simTime-firstRange > 1.4:
                    raise RuntimeError("bounded lift did not clear the ToF startup band")
            time.sleep(.02)
        raise RuntimeError("range and flow did not stabilize during attitude startup")

    def hover(self, pos, height, visualTarget=None, *, targetHeight=.8):
        north, east, down = (float(field(pos, axis)) for axis in ("x", "y", "z"))
        downGoal = down + height - targetHeight
        self.record("velocity_handoff", ground_down=down + height, target_down=downGoal)
        self.hoverTarget = north, east, downGoal
        stableSince = None
        while True:
            self.px4.verify()
            self.px4.neutralPilot()
            pos = self.px4.sample("vehicle_local_position")
            if any(field(pos, name) != "True" for name in ("v_xy_valid", "v_z_valid", "z_valid")):
                raise RuntimeError("PX4 velocity estimate lost after switching to velocity")
            x, y, z, vz = (float(field(pos, name)) for name in ("x", "y", "z", "vz"))
            if math.hypot(x-north,y-east) > .25 or z < downGoal-.20:
                raise RuntimeError("velocity climb outside the Task 1 envelope")
            target = visualTarget() if visualTarget is not None else None
            if target is not None:
                self.hoverTarget = target[1], target[0], downGoal
            goalNorth, goalEast, _ = self.hoverTarget
            vn, ve = (float(field(pos, axis)) for axis in ("vx", "vy"))
            height, rangeRate = self.rangeEstimate()
            downSpeed = 1.2*(downGoal-z) if height is None else 1.2*(height-targetHeight)+.25*rangeRate
            self.px4.velocity(bounded(1.7*(goalNorth-x)-.35*vn,-.24,.24),
                              bounded(1.7*(goalEast-y)-.35*ve,-.24,.24),
                              bounded(downSpeed,-.25,.20), self.yaw)
            stable = (height is not None and abs(height-targetHeight) <= .08 and abs(vz) <= .08
                      and math.hypot(vn,ve) <= .06)
            stableSince = (self.simTime if stableSince is None else stableSince) if stable else None
            if stableSince is not None and self.simTime - stableSince >= 1.:
                self.hoverTarget = goalNorth, goalEast, z + height - targetHeight
                self.record("hover", range_m=height, down=z, speed_down=vz)
                return
            time.sleep(.025)

    def holdVelocity(self, tagTarget=None, *, rangeHeight=None):
        north, east, goal = self.hoverTarget
        if tagTarget is not None:
            north, east = tagTarget[1], tagTarget[0]
        pos = self.px4.sample("vehicle_local_position")
        if any(field(pos, name) != "True" for name in ("v_xy_valid", "v_z_valid", "z_valid")):
            raise RuntimeError("PX4 feedback lost while waiting for the landing task")
        x, y, z = (float(field(pos, name)) for name in ("x", "y", "z"))
        downSpeed = 1.2*(goal-z)
        if rangeHeight is not None:
            height, rate = self.rangeEstimate()
            if height is not None and abs(height-rangeHeight) <= .15:
                downSpeed = 1.2*(height-rangeHeight) + .25*rate
        self.px4.velocity(bounded(1.1*(north-x),-.2,.2), bounded(1.1*(east-y),-.2,.2),
                          bounded(downSpeed,-.20,.20), self.yaw)

    def abort(self):
        state = self.px4.sample("vehicle_status")
        if field(state, "arming_state") == "2":
            self.px4.command("commander", "land")
            self.record("abort_land_requested")
