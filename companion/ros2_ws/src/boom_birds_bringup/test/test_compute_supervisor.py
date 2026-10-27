"""计算资源监管：只停止自有子进程，并把退出与 CPU 释放写进状态。"""
import json
import subprocess
import sys
import threading
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger
from boom_birds_bringup.compute_supervisor import OwnedProcesses, ComputeSupervisorNode


SPIN = [sys.executable, "-c", "while True: pass"]
SLEEP = [sys.executable, "-c", "import time;time.sleep(60)"]


def spinUntil(executor, predicate, timeout=10.):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        executor.spin_once(timeout_sec=.02)
    return predicate()


def test_owned_processes_stop_only_owned_and_report_cpu_release():
    decoy = subprocess.Popen(SLEEP)
    owned = OwnedProcesses([dict(name="vio", command=SPIN), dict(name="adapter", command=SLEEP)])
    try:
        time.sleep(.8)
        snapshot = owned.snapshot()
        assert snapshot["all_alive"] and len(snapshot["processes"]) == 2
        assert all(item["pid"] > 0 for item in snapshot["processes"])
        assert owned.stopAll()
        after = owned.snapshot()
        assert after["all_stopped"] and after["release_verified"]
        assert after["cpu_before_percent"] > 20
        assert after["cpu_after_percent"] == 0
        # 无关进程不属于本任务，必须仍然存活。
        assert decoy.poll() is None
    finally:
        if decoy.poll() is None:
            decoy.terminate()
            decoy.wait(timeout=5.)
        owned.stopAll()


def test_supervisor_release_needs_fresh_platform_lease_and_stops_children():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    supervisor = harness = None
    serverThread = None
    try:
        supervisor = ComputeSupervisorNode(context=context, parameter_overrides=[
            Parameter("commands", value=[json.dumps(SPIN), json.dumps(SLEEP)]),
            Parameter("test_only", value=True), Parameter("dry_run", value=True),
            Parameter("status_rate_hz", value=5.)])
        harness = Node("compute_supervisor_test_harness", context=context)
        releaseClient = harness.create_client(Trigger, "/boom_birds/compute/release")
        statusPublisher = harness.create_publisher(String, "/boom_birds/platform/status", 10)
        compute = []
        harness.create_subscription(String, "/boom_birds/compute/status",
                                    lambda msg: compute.append(json.loads(msg.data)), 10)
        executor = MultiThreadedExecutor(context=context)
        executor.add_node(supervisor)
        executor.add_node(harness)
        deadline = time.monotonic() + 10.
        while time.monotonic() < deadline and not releaseClient.service_is_ready():
            time.sleep(.02)
        assert releaseClient.service_is_ready()

        def publish(sequence=1, state="ALIGN", **overrides):
            payload = dict(session="s1", state=state, owner="platform", release_compute=True,
                           vio_required=True, token="tok", stamp_monotonic=time.monotonic(),
                           sequence=sequence, fcu_epoch=0)
            payload.update(overrides)
            statusPublisher.publish(String(data=json.dumps(payload)))

        # 未确认的交接（PREPARE）不能释放计算。
        publish(1, "PREPARE")
        assert spinUntil(executor, lambda: supervisor.session == "s1", 5.)
        spinUntil(executor, lambda: False, .2)
        future = releaseClient.call_async(Trigger.Request())
        assert spinUntil(executor, lambda: future.done(), 3.)
        assert future.result().success is False
        assert all(process.alive() for process in supervisor.owned.processes)
        # 新鲜且确认的租赁可以释放，并且必须回读子进程退出。
        for index in range(2, 6):
            publish(index, "ALIGN")
            spinUntil(executor, lambda: False, .05)
        future = releaseClient.call_async(Trigger.Request())
        assert spinUntil(executor, lambda: future.done(), 6.)
        assert future.result().success is True
        assert all(not process.alive() for process in supervisor.owned.processes)
        assert spinUntil(executor, lambda: compute and compute[-1]["all_stopped"]
                         and compute[-1]["released"], 6.)
        snapshot = supervisor.owned.snapshot()
        assert snapshot["released"] and snapshot["all_stopped"]
        assert snapshot["cpu_before_percent"] > 20
        assert compute[-1]["kind"] == "TAG_TASK_COMPUTE_SUPERVISOR"
        assert compute[-1]["session"] == "s1"
    finally:
        if supervisor is not None:
            supervisor.stop()
        if harness is not None:
            harness.destroy_node()
        if supervisor is not None:
            supervisor.destroy_node()
        if rclpy.ok(context=context):
            rclpy.shutdown(context=context)


def test_supervisor_refuses_live_non_test_compute_ownership():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        import pytest
        with pytest.raises(ValueError, match="compute_supervisor_requires_test_only_or_dry_run"):
            ComputeSupervisorNode(context=context, parameter_overrides=[
                Parameter("commands", value=[""]), Parameter("test_only", value=False),
                Parameter("dry_run", value=False)])
    finally:
        if rclpy.ok(context=context):
            rclpy.shutdown(context=context)



def test_release_service_stops_worker_after_launcher_exit(tmp_path):
    from pathlib import Path
    from boom_birds_bringup.compute_supervisor import descendants
    childPath = tmp_path / "worker.pid"
    worker = ("import os,signal,time,pathlib; "
              "signal.signal(signal.SIGTERM,signal.SIG_IGN); "
              "pathlib.Path(" + repr(str(childPath)) + ").write_text(str(os.getpid())); "
              "time.sleep(60)")
    launcher = "import subprocess,sys; subprocess.Popen([sys.executable,'-c'," + repr(worker) + "])"
    context = rclpy.context.Context()
    rclpy.init(context=context)
    supervisor = None
    decoy = subprocess.Popen(SLEEP)
    try:
        supervisor = ComputeSupervisorNode(context=context, parameter_overrides=[
            Parameter("commands", value=[json.dumps([sys.executable, "-c", launcher])]),
            Parameter("expected_session", value="s1"), Parameter("test_only", value=True)])
        parent = supervisor.owned.processes[0]
        deadline = time.monotonic() + 5.
        while (not childPath.exists() or parent.alive()) and time.monotonic() < deadline:
            time.sleep(.01)
        assert childPath.exists() and not parent.alive()
        workerPid = int(childPath.read_text())
        assert workerPid in descendants(parent.pid)
        assert supervisor.owned.finalizeRelease() is False
        lease = dict(session="s1", state="ALIGN", owner="platform", release_compute=True,
                     vio_required=True, token="tok", stamp_monotonic=time.monotonic(),
                     sequence=1, fcu_epoch=0)
        supervisor.releaseNode.onStatus(String(data=json.dumps(lease)))
        response = supervisor.releaseNode.release(Trigger.Request(), Trigger.Response())
        assert response.success
        assert not descendants(parent.pid)
        assert supervisor.owned.snapshot()["release_verified"]
        assert decoy.poll() is None
    finally:
        if supervisor is not None:
            supervisor.destroy_node()
        decoy.terminate()
        decoy.wait(timeout=5.)
        rclpy.shutdown(context=context)
