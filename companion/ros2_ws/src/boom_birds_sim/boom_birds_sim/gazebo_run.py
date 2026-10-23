"""本机 Gazebo 基准入口；只管理本轮进程组，不连接硬件。"""

import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from .gazebo_fixture import prepare


def claimRosDomain():
    directory = Path.home() / ".cache/boom_birds_gazebo"
    directory.mkdir(parents=True, exist_ok=True)
    for domain in range(100, 200):
        lock = (directory / f"domain-{domain}.lock").open("a")
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return domain, lock
        except BlockingIOError:
            lock.close()
    raise RuntimeError("no free local simulation ROS domain")


class Processes:
    def __init__(self, out):
        self.out = Path(out)
        self.children = []
        self.logs = []

    def start(self, name, command, **kwargs):
        log = (self.out / f"{name}.log").open("w")
        self.logs.append(log)
        p = subprocess.Popen(
            command,
            stdout=log,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
            **kwargs,
        )
        self.children.append((name, p))
        return p

    def close(self):
        result = {}
        for name, p in reversed(self.children):
            # 子进程可能在父进程先退出后仍留在本轮进程组。
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                p.wait(timeout=5)
            result[name] = p.returncode
        for log in self.logs:
            log.close()
        (self.out / "processes.json").write_text(json.dumps(result, indent=2))


def baseline(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from mavros_msgs.msg import State
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import PoseStamped

    out = Path(args.out)
    procs = Processes(out)
    rclpy.init()
    node = Node("gazebo_px4_baseline")
    result = {"connected": False, "armed_seen": False, "imu_messages": 0, "local_pose_messages": 0}

    def state(m):
        result["connected"] |= m.connected
        result["armed_seen"] |= m.armed
        result["mode"] = m.mode

    def imu(m):
        result["imu_messages"] += 1

    def pose(m):
        result["local_pose_messages"] += 1

    subs = [
        node.create_subscription(State, "/mavros/state", state, 10),
        node.create_subscription(Imu, "/mavros/imu/data", imu, qos_profile_sensor_data),
        node.create_subscription(
            PoseStamped, "/mavros/local_position/pose", pose, qos_profile_sensor_data
        ),
    ]
    try:
        px4 = Path(args.px4).resolve()
        binary = px4 / "build/px4_sitl_default/bin/px4"
        if not binary.is_file():
            raise FileNotFoundError(binary)
        for proc in Path("/proc").iterdir():
            if not proc.name.isdigit():
                continue
            try:
                existing = (proc / "exe").resolve(strict=True)
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
            if existing == binary:
                raise RuntimeError(
                    f"PX4 is already running: PID {proc.name}; baseline requires a separate run"
                )
        env = dict(
            os.environ,
            HEADLESS="1",
            PX4_SIM_MODEL="gz_x500",
            PX4_SYS_AUTOSTART="4001",
            PX4_SIM_SPEED_FACTOR="1",
            GZ_IP="127.0.0.1",
        )
        procs.start(
            "px4",
            [str(px4 / "build/px4_sitl_default/bin/px4"), "-d"],
            cwd=px4 / "build/px4_sitl_default/rootfs",
            env=env,
        )
        procs.start(
            "mavros",
            [
                "ros2",
                "run",
                "mavros",
                "mavros_node",
                "--ros-args",
                "-p",
                "fcu_url:=udp://127.0.0.1:14540@127.0.0.1:14557",
                "-p",
                "plugin_allowlist:=['sys_status','imu','local_position','command','time']",
            ],
        )
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        result["pass"] = bool(
            result["connected"]
            and not result["armed_seen"]
            and result["imu_messages"] > 100
            and result["local_pose_messages"] > 10
        )
        (out / "baseline.json").write_text(json.dumps(result, indent=2))
    finally:
        procs.close()
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, indent=2))
    return 0 if result["pass"] else 1


def capture(args):
    out = Path(args.out)
    root = Path(args.root)
    config = root / "companion/ros2_ws/src/open_vins/config/euroc_mav"
    if args.motion_plugin:
        plugin = Path(args.motion_plugin)
    else:
        from ament_index_python.packages import get_package_prefix

        plugin = (
            Path(get_package_prefix("boom_birds_sim")) / "lib/boom_birds_sim/libbb_sensor_motion.so"
        )
    prepare(
        out,
        config / "estimator_config.yaml",
        config / "kalibr_imu_chain.yaml",
        plugin,
        args.scene,
        args.motion_profile,
        duration_sim_s=args.duration,
    )
    procs = Processes(out)
    try:
        procs.start(
            "bridge",
            [
                "ros2",
                "run",
                "ros_gz_bridge",
                "parameter_bridge",
                "--ros-args",
                "-p",
                f'config_file:={out/"bridge.yaml"}',
            ],
        )
        child = procs.start(
            "capture",
            [
                sys.executable,
                "-m",
                "boom_birds_sim.gazebo_capture",
                "--out",
                str(out),
                "--duration",
                str(args.duration),
            ],
        )
        time.sleep(2)
        procs.start("gazebo", ["gz", "sim", "-s", "-r", str(out / "sensor_world.sdf")])
        if args.pause_test or args.reset_test:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from rosgraph_msgs.msg import Clock

            rclpy.init()
            watch = Node("gazebo_pause_probe")
            state = {"time_s": 0.0}

            def clock(msg):
                state["time_s"] = msg.clock.sec + msg.clock.nanosec * 1e-9

            subscription = watch.create_subscription(
                Clock, "/clock", clock, qos_profile_sensor_data
            )
            pausedAt = None
            settledClock = None
            pausedOnce = False
            result = {}
            deadline = time.monotonic() + 330
            while child.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(watch, timeout_sec=0.05)
                if not pausedOnce and state["time_s"] >= 2.0:
                    reply = subprocess.run(
                        [
                            "gz",
                            "service",
                            "-s",
                            "/world/bb_sensor/control",
                            "--reqtype",
                            "gz.msgs.WorldControl",
                            "--reptype",
                            "gz.msgs.Boolean",
                            "--timeout",
                            "3000",
                            "--req",
                            "reset: {all: true}" if args.reset_test else "pause: true",
                        ],
                        text=True,
                        capture_output=True,
                        timeout=5,
                    )
                    result["reset_reply" if args.reset_test else "pause_reply"] = reply.stdout
                    pausedAt = None if args.reset_test else time.monotonic()
                    pausedOnce = True
                if (
                    pausedAt is not None
                    and time.monotonic() - pausedAt >= 0.5
                    and settledClock is None
                ):
                    settledClock = state["time_s"]
                if pausedAt is not None and time.monotonic() - pausedAt >= 2.0:
                    result["paused_clock_advance_s"] = state["time_s"] - settledClock
                    reply = subprocess.run(
                        [
                            "gz",
                            "service",
                            "-s",
                            "/world/bb_sensor/control",
                            "--reqtype",
                            "gz.msgs.WorldControl",
                            "--reptype",
                            "gz.msgs.Boolean",
                            "--timeout",
                            "3000",
                            "--req",
                            "pause: false",
                        ],
                        text=True,
                        capture_output=True,
                        timeout=5,
                    )
                    result["resume_reply"] = reply.stdout
                    result["resume_clock_s"] = state["time_s"]
                    pausedAt = None
            result["final_clock_s"] = state["time_s"]
            result["pass"] = bool(
                pausedOnce
                and result.get("paused_clock_advance_s", 1.0) <= 0.001
                and "true" in result.get("pause_reply", "")
                and "true" in result.get("resume_reply", "")
                and result["final_clock_s"] > result.get("resume_clock_s", 999.0)
            )
            if args.reset_test:
                data = json.loads((out / "capture.json").read_text())
                result["capture_errors"] = data["errors"]
                result["pass"] = bool(
                    "true" in result.get("reset_reply", "")
                    and child.poll() == 1
                    and "simulation clock moved backwards" in data["errors"]
                )
            (out / ("reset.json" if args.reset_test else "pause.json")).write_text(
                json.dumps(result, indent=2)
            )
            watch.destroy_node()
            rclpy.shutdown()
            code = child.poll()
            if args.reset_test:
                code = 0 if result["pass"] else 1
            elif not result["pass"]:
                code = 1
        else:
            code = child.wait(timeout=330)
        topics = subprocess.run(["gz", "topic", "-l"], text=True, capture_output=True, timeout=10)
        (out / "gz-topics.txt").write_text(topics.stdout + topics.stderr)
    finally:
        procs.close()
    return code


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("stage", choices=["baseline", "capture"])
    parser.add_argument("--out", required=True)
    parser.add_argument("--root", default="/home/waterc/workspace/Boom_Birds")
    parser.add_argument("--px4", default="/home/waterc/PX4-Autopilot")
    parser.add_argument("--duration", type=float, default=45)
    parser.add_argument("--motion-plugin", default="")
    parser.add_argument("--motion-profile", choices=["nominal", "gentle"], default="nominal")
    probes = parser.add_mutually_exclusive_group()
    probes.add_argument("--pause-test", action="store_true")
    probes.add_argument("--reset-test", action="store_true")
    parser.add_argument("--scene", choices=["range_3m", "range_5m"], default="range_3m")
    args = parser.parse_args()
    out = Path(args.out).resolve()
    if out.exists() and any(out.iterdir()):
        parser.error("out must be a new or empty directory")
    out.mkdir(parents=True, exist_ok=True)
    args.out = str(out)
    domain, domainLock = claimRosDomain()
    os.environ.update(
        PYTHONNOUSERSITE="1",
        ROS_LOCALHOST_ONLY="1",
        ROS_DOMAIN_ID=str(domain),
        GZ_PARTITION=f"bb_gazebo_{os.getpid()}",
        GZ_IP="127.0.0.1",
        GZ_CONFIG_PATH="/usr/share/gz",
    )
    (out / "environment.json").write_text(
        json.dumps(
            {
                "ros_domain_id": domain,
                "gz_partition": os.environ["GZ_PARTITION"],
                "gz_config_path": os.environ["GZ_CONFIG_PATH"],
                "python": sys.executable,
                "root": args.root,
                "px4": args.px4,
            },
            indent=2,
        )
    )
    if args.stage == "baseline":
        return baseline(args)
    return capture(args)


if __name__ == "__main__":
    raise SystemExit(main())
