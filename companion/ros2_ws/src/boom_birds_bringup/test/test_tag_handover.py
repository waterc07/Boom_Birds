"""Tag 任务交接闸门：证据、VIO 保留、失败分支和完成口径。"""
import json
import math
import time
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
from boom_birds_bringup.platform_mission import PlatformMissionGate
from boom_birds_bringup.lifecycle import State
from boom_birds_bringup.tag_flight_task import (TagFlightTask, handoverSourceAllowed,
                                                loadTask)
from boom_birds_bringup.attitude_lifecycle import AttitudeMissionConfig
from boom_birds_control.runtime_config import SCENES

ROOT = Path(__file__).parents[2]
TASK = ROOT / "boom_birds_bringup/config/flight_tasks/tag_return_turn.yaml"


class Publisher:
    def __init__(self): self.sent = []
    def publish(self, msg): self.sent.append(json.loads(msg.data))


class Node:
    """最小任务节点替身：只实现闸门使用的接口。"""
    def __init__(self, evidence=None, yawOffset=0.):
        self.pubState = Publisher(); self.actions = []; self.holds = 0; self.retired = []
        self.capture = dict(ready=False, reason="not_captured") if evidence is None else evidence
        fsm = NS(session="s1", state=State.EXECUTING, goal_reached=True,
                 holdHere=lambda observation: True,
                 handoverEvidence=lambda: dict(self.capture),
                 handoverYawOffset=lambda: yawOffset)
        self.fsm = fsm
    def declare_parameter(self, *args): pass
    def get_parameter(self, name): return NS(value=True if name.endswith("enabled") else .5)
    def create_publisher(self, *args): return Publisher()
    def create_subscription(self, *args): pass
    def observation(self): return NS(position=(0,0,1.5))
    def _actions(self, actions): self.actions.extend(actions)
    def _retireTrajectory(self, reason): self.retired.append(reason)
    def _hold(self): self.holds += 1


def status(gate, **fields):
    gate.onStatus(NS(data=json.dumps(dict(dict(session="s1", stamp_monotonic=time.monotonic(),
        producer="p"), **fields))))


def handoverGate(evidence=None, yawOffset=0.):
    node = Node(evidence, yawOffset)
    gate = PlatformMissionGate(node)
    gate.handover = True
    gate.releaseRequired = True
    return node, gate


READY = dict(ready=True, ids=[7], source="tag", samples=4, yaw_valid=True)


def test_handover_requires_capture_evidence_before_request():
    node, gate = handoverGate()
    assert not gate.request()
    assert gate.pub.sent == []
    node.capture = dict(READY)
    assert gate.request()
    assert gate.active
    # 任务 2 的末段航向偏置随请求下发，不依赖 VIO 世界系航向。
    assert gate.pub.sent == [{"action": "land", "session": "s1", "yaw_offset_rad": 0.}]
    assert node.actions == ["cancel", "disable_planner"]
    assert node.retired == ["platform_landing"]


def test_handover_holds_altitude_until_platform_owns_output():
    node, gate = handoverGate(READY)
    gate.request()
    status(gate, state="ACQUIRE", owner="navigation", status_sequence=1)
    assert gate.tick(gate.received) and node.holds == 1
    status(gate, state="PREPARE", owner="platform", status_sequence=2)
    gate.tick(gate.received)
    assert node.holds == 1  # 平台已持有输出，任务不再发导航 setpoint
    status(gate, state="DESCEND", owner="platform", status_sequence=3,
           release_verified=True, release_state="RELEASED", vio_required=False,
           confirmed=True, token="t")
    gate.tick(gate.received)
    assert node.holds == 1


def test_success_requires_verified_compute_release():
    node, gate = handoverGate(READY)
    gate.request()
    status(gate, state="COMPLETE", owner="platform", status_sequence=1,
           release_verified=False, release_state="FAILED", vio_required=True,
           confirmed=True, token="t")
    gate.tick(gate.received)
    assert node.fsm.state == State.COMPLETE
    assert gate.failed and gate.failure == "compute_release_not_verified"


def test_verified_release_reports_success():
    node, gate = handoverGate(READY)
    gate.request()
    status(gate, state="COMPLETE", owner="platform", status_sequence=1,
           release_verified=True, release_state="RELEASED", vio_required=False,
           confirmed=True, token="t")
    gate.tick(gate.received)
    assert node.fsm.state == State.COMPLETE and not gate.failed


def test_native_land_fallback_is_failure_not_success():
    node, gate = handoverGate(READY)
    gate.request()
    status(gate, state="NATIVE_LAND", owner="fallback", status_sequence=1,
           reason="handoff_timeout", release_verified=False, release_state="CANCELLED",
           vio_required=True, confirmed=False, token="")
    gate.tick(gate.received)
    assert gate.failed and gate.failure == "handoff_timeout"


def test_status_loss_requests_one_native_land():
    node, gate = handoverGate(READY)
    gate.request()
    gate.tick(gate.requested + .6)
    assert gate.failed and gate.failure == "platform_status_lost"
    assert gate.pub.sent[-1] == {"action": "cancel", "session": "s1"}
    assert node.actions[-3:] == ["cancel", "disable_planner", "land"]
    gate.tick(gate.requested + 1.)
    assert node.actions.count("land") == 1


def test_takeover_yaw_offset_is_task_relative_and_validated():
    node, gate = handoverGate(READY, yawOffset=math.pi)
    assert gate.request()
    assert gate.pub.sent[0]["yaw_offset_rad"] == pytest.approx(math.pi)
    node2, gate2 = handoverGate(READY, yawOffset=math.pi + .5)
    assert not gate2.request()


def test_handover_source_gate():
    assert handoverSourceAllowed("tag", True)
    assert not handoverSourceAllowed("resolved_tag", True)
    assert not handoverSourceAllowed("features", True)
    assert handoverSourceAllowed("tag", False)
    assert handoverSourceAllowed("resolved_tag", False)
    assert not handoverSourceAllowed("features", False)


def makeTask(handover):
    return TagFlightTask(SCENES["local"], AttitudeMissionConfig(), loadTask(str(TASK)),
                         handover=handover)


def test_task_exposes_capture_evidence_and_relative_yaw():
    fsm = makeTask(True)
    assert fsm.handoverEvidence()["ready"] is False
    fsm.initialYaw, fsm.segmentYawGoal = 0., math.pi
    assert fsm.handoverYawOffset() == pytest.approx(math.pi)
    fsm.initialYaw, fsm.segmentYawGoal = -.5, -.5-math.pi
    assert fsm.handoverYawOffset() == pytest.approx(-math.pi)
    fsm.capture = dict(READY)
    assert fsm.handoverEvidence()["samples"] == 4
    legacy = makeTask(False)
    assert legacy.handover is False and legacy.capture["ready"] is False


def test_complete_preserves_failure_when_native_land_status_was_missed():
    node, gate = handoverGate(READY)
    assert gate.request()
    status(gate, state="COMPLETE", owner="none", status_sequence=1,
           reason="native_land_landed_disarmed", failure_reason="velocity_confirmation_lost",
           release_verified=True, confirmed=True)
    gate.tick(gate.received)
    assert gate.failed and gate.failure == "velocity_confirmation_lost"
    assert node.fsm.reason == "velocity_confirmation_lost"


def test_platform_completion_records_actual_completion_time_once():
    node, gate = handoverGate(READY)
    assert gate.request()
    status(gate,state="COMPLETE",owner="none",status_sequence=1,release_verified=True)
    now=gate.received
    gate.tick(now)
    assert node.fsm.entered==now
    gate.tick(now+.1)
    assert node.fsm.entered==now
    node.fsm.flightStarted=now-1.
    node.fsm.mission=NS(flight_budget_s=45.)
    gate.tick(now+46.)
    assert not gate.failed and node.actions.count("land")==0


def test_total_flight_budget_still_applies_after_platform_takeover():
    node, gate = handoverGate(READY)
    assert gate.request()
    node.fsm.flightStarted=gate.requested-46.
    node.fsm.mission=NS(flight_budget_s=45.)
    status(gate,state="DESCEND",owner="platform",status_sequence=1,release_verified=True)
    gate.tick(gate.received)
    assert gate.failed and gate.failure=="mission_time_budget"
    assert node.actions.count("land")==1
    gate.tick(gate.received+.1)
    assert node.actions.count("land")==1
    status(gate,state="COMPLETE",owner="none",status_sequence=2,release_verified=True)
    gate.tick(gate.received)
    assert gate.failed and node.fsm.reason=="mission_time_budget"
