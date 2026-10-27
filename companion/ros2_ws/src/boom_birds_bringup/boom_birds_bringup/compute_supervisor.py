"""Own VIO-exclusive child processes and host the compute-release service.

任务启动时由本节点（且只由本节点）持有 VIO/姿态适配器子进程：
交接确认后才允许停止，停止结果必须回读退出，退出后 CPU/内存如实写入状态。
不使用 pkill/进程名匹配；只操作本节点自己 Popen 的进程组。
"""
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import String as StringMsg
from .compute_release_node import ComputeReleaseNode

SYS_CLK_TCK = os.sysconf("SC_CLK_TCK")


def readStat(pid):
    """读取 /proc/<pid>/stat 的 CPU 时间与常驻内存；进程已退出返回 None。"""
    try:
        fields = Path(f"/proc/{int(pid)}/stat").read_text().split()
        cpu = (int(fields[13]) + int(fields[14])) / SYS_CLK_TCK
        rssKb = int(fields[23]) * os.sysconf("SC_PAGE_SIZE") / 1024
        return dict(cpu_s=cpu, rss_kb=rssKb)
    except (OSError, IndexError, ValueError):
        return None


def descendants(pid):
    """本进程组内的子进程 PID；仅用于确认退出，不用于终止其它进程。"""
    children = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / "stat").read_text().rsplit(")", 1)[1].split()
            if int(stat[2]) == int(pid) and stat[0] not in ("Z", "X"):
                children.append(int(entry.name))
        except (OSError, IndexError, ValueError):
            continue
    return children


class OwnedProcess:
    """调用者持有的单个子进程；终止只针对自己的进程组。"""

    def __init__(self, name, command, logPath=None, env=None):
        self.name = str(name)
        self.command = [str(item) for item in command]
        self.logPath = Path(logPath) if logPath else None
        self.log = None
        if self.logPath is not None:
            self.logPath.parent.mkdir(parents=True, exist_ok=True)
            self.log = self.logPath.open("w", encoding="utf-8")
        self.process = subprocess.Popen(
            self.command, stdin=subprocess.DEVNULL,
            stdout=self.log if self.log is not None else None,
            stderr=subprocess.STDOUT if self.log is not None else None,
            start_new_session=True, env=env)
        self.spawnedAt = time.monotonic()
        stat = readStat(self.process.pid) or {}
        self.spawnCpu = stat.get("cpu_s", 0.)
        self.lastCpu = self.spawnCpu
        self.lastSampleAt = self.spawnedAt
        self.cpuPercent = 0.
        self.stopped = False

    @property
    def pid(self):
        return self.process.pid

    def alive(self):
        return self.process.poll() is None

    def sample(self, now=None):
        now = time.monotonic() if now is None else now
        stat = readStat(self.pid)
        if stat is None:
            self.cpuPercent = 0.
            return None
        delta = max(0., now - self.lastSampleAt)
        # 过短间隔的重复回读只会把瞬时占用率打成 0；不足 20 ms 不更新统计。
        if delta >= .02:
            self.cpuPercent = max(0., (stat["cpu_s"] - self.lastCpu) / delta * 100.)
            self.lastCpu = stat["cpu_s"]
            self.lastSampleAt = now
        return stat

    def snapshot(self, now=None):
        stat = self.sample(now)
        return dict(name=self.name, pid=self.pid, alive=self.alive(),
                    cpu_s=None if stat is None else round(stat["cpu_s"], 3),
                    rss_kb=None if stat is None else int(stat["rss_kb"]),
                    cpu_percent=round(self.cpuPercent, 1), stopped=self.stopped,
                    command=self.command)

    def stop(self, timeout=2.):
        """先 SIGTERM 整个自有进程组，超时再 SIGKILL，并回读退出。"""
        self.stopped = True
        # ros2 run 的父进程退出后，其工作进程仍可能留在同一进程组。
        for sig in (signal.SIGTERM, signal.SIGKILL):
            if not self.alive() and not descendants(self.pid):
                break
            try:
                os.killpg(self.pid, sig)
            except ProcessLookupError:
                pass
            deadline = time.monotonic() + timeout
            try:
                self.process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                pass
            while descendants(self.pid) and time.monotonic() < deadline:
                time.sleep(.01)
        leftovers = descendants(self.pid)
        verified = (not self.alive()) and not leftovers
        if self.log is not None:
            self.log.flush()
        return verified

    def close(self):
        verified = self.stop()
        if self.log is not None:
            self.log.close()
            self.log = None
        return verified


class OwnedProcesses:
    """一组自有子进程 + 交接前后 CPU/内存快照。"""

    def __init__(self, specs=None, *, logDirectory=None, env=None):
        self.processes = []
        self.logDirectory = Path(logDirectory) if logDirectory else None
        self.baseline = None
        self.released = False
        self.releaseVerified = False
        self.cpuBefore = self.cpuAfter = None
        self.stoppedAt = None
        for spec in specs or []:
            if isinstance(spec, dict):
                name, command = spec.get("name"), spec.get("command")
            else:
                name, command = spec[0], spec[1]
            if not isinstance(name, str) or not name or not isinstance(command, (list, tuple)) or not command:
                raise ValueError("owned_process_spec")
            logPath = None if self.logDirectory is None else self.logDirectory / (name + ".log")
            self.processes.append(OwnedProcess(name, command, logPath=logPath, env=env))
        self.sample()

    @property
    def pids(self):
        return [process.pid for process in self.processes]

    def children(self):
        return [process.process for process in self.processes]

    def sample(self, now=None):
        now = time.monotonic() if now is None else now
        total = 0.
        for process in self.processes:
            process.sample(now)
            total += process.cpuPercent
        if not self.released and total > 0:
            # 交接前的峰值占用率就是释放收益的基准；子进程退出后无法再测。
            self.cpuBefore = max(self.cpuBefore or 0., round(total, 1))
        return total

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        total = self.sample(now)
        return dict(processes=[process.snapshot(now) for process in self.processes],
                    total_cpu_percent=round(total, 1),
                    cpu_before_percent=self.cpuBefore, cpu_after_percent=self.cpuAfter,
                    released=self.released, release_verified=self.releaseVerified,
                    all_alive=all(process.alive() for process in self.processes),
                    all_stopped=all(not process.alive() and not descendants(process.pid)
                                    for process in self.processes),
                    stopped_at=self.stoppedAt)

    def measureCpu(self):
        """按进程启动以来的 CPU 时间取平均占用率（% 单核），用于交接前后对比。"""
        now = time.monotonic()
        total = 0.
        for process in self.processes:
            stat = readStat(process.pid)
            if stat is not None:
                window = max(1e-6, now - process.spawnedAt)
                total += max(0., (stat["cpu_s"] - process.spawnCpu) / window) * 100.
        return round(total, 1)

    def finalizeRelease(self):
        """子进程已退出后记录释放回读与 CPU 归零；只读观测，不再次终止。"""
        if self.released:
            return self.releaseVerified
        if any(process.alive() or descendants(process.pid) for process in self.processes):
            return False
        if self.cpuBefore is None:
            self.cpuBefore = self.measureCpu()
        self.released = True
        self.releaseVerified = all(not process.alive() for process in self.processes)
        self.cpuAfter = round(self.sample(), 1)
        self.stoppedAt = time.monotonic()
        return self.releaseVerified

    def stopAll(self, timeout=2.):
        self.cpuBefore = self.measureCpu() if self.cpuBefore is None else self.cpuBefore
        self.released = True
        verified = True
        for process in self.processes:
            verified = process.close() and verified
        self.cpuAfter = round(self.sample(), 1)
        self.stoppedAt = time.monotonic()
        self.releaseVerified = verified and all(not process.alive() for process in self.processes)
        return self.releaseVerified


class ComputeSupervisorNode(Node):
    """Task-owned compute supervisor: children + compute-release service."""

    def __init__(self, **kwargs):
        super().__init__("boom_birds_compute_supervisor", **kwargs)
        self.declare_parameter("commands", [""])
        self.declare_parameter("log_directory", "")
        self.declare_parameter("test_only", False)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("status_rate_hz", 2.)
        self.declare_parameter("expected_session", "")
        commands = [item for item in list(self.get_parameter("commands").value) if item]
        parsed = [json.loads(item) for item in commands]
        specs = [dict(name="compute-%d" % index, command=command)
                 for index, command in enumerate(parsed)]
        if not self.get_parameter("test_only").value and not self.get_parameter("dry_run").value:
            raise ValueError("compute_supervisor_requires_test_only_or_dry_run")
        logDirectory = str(self.get_parameter("log_directory").value) or None
        self.session = str(self.get_parameter("expected_session").value)
        if not self.session:
            self.session = ""
        self.owned = OwnedProcesses(specs, logDirectory=logDirectory)
        self.releaseNode = ComputeReleaseNode(self.owned.children(), [], lambda: self.session,
                                              stopOwned=self.owned.stopAll, context=self.context)
        self.releaseExecutor = SingleThreadedExecutor(context=self.context)
        self.releaseExecutor.add_node(self.releaseNode)
        self.releaseThread = threading.Thread(target=self.releaseExecutor.spin, daemon=True)
        self.releaseThread.start()
        self.create_subscription(String, "/boom_birds/platform/status", self.onPlatform, 10)
        self.pubStatus = self.create_publisher(String, "/boom_birds/compute/status", 10)
        self.closed = False
        rate = max(.5, float(self.get_parameter("status_rate_hz").value))
        self.create_timer(1. / rate, self.publish)
        self.get_logger().info("compute supervisor: " + json.dumps(self.owned.snapshot()))

    def onPlatform(self, msg):
        try:
            data = json.loads(msg.data)
            session = data.get("session")
            if isinstance(session, str) and session:
                self.session = session
        except (ValueError, TypeError):
            pass

    def publish(self):
        if self.closed:
            return
        self.owned.sample()
        self.owned.finalizeRelease()
        snapshot = self.owned.snapshot()
        snapshot["session"] = self.session
        snapshot["kind"] = "TAG_TASK_COMPUTE_SUPERVISOR"
        self.pubStatus.publish(String(data=json.dumps(snapshot)))

    def stop(self):
        if self.closed:
            return self.owned.releaseVerified
        self.closed = True
        verified = self.owned.stopAll()
        self.releaseExecutor.shutdown(timeout_sec=2.)
        self.releaseThread.join(timeout=2.)
        try:
            self.releaseNode.destroy_node()
        except Exception:
            pass
        return verified

    def destroy_node(self):
        self.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ComputeSupervisorNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            verified = node.stop()
            node.get_logger().info("compute supervisor stop verified=" + str(verified))
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
