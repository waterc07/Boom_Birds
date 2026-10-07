"""Suspend the navigation mission while the single PX4 interface owns platform landing."""
import json
import time
import math
from std_msgs.msg import String
from .lifecycle import State


class PlatformMissionGate:
    def __init__(self,node):
        self.node=node
        node.declare_parameter("platform_landing_enabled",False)
        node.declare_parameter("platform_status_timeout_s",.5)
        self.enabled=node.get_parameter("platform_landing_enabled").value
        self.timeout=float(node.get_parameter("platform_status_timeout_s").value)
        if self.timeout<=0:raise ValueError("platform_status_timeout_s")
        self.active=False
        self.received=None
        self.status=None
        self.requested=None
        self.failed=False
        self.producer=None
        self.last_status_sequence=-1
        self.status_rejected=0
        self.pub=node.create_publisher(String,"/boom_birds/platform/request",10)
        node.create_subscription(String,"/boom_birds/platform/status",self.on_status,10)

    def on_status(self,msg):
        if not self.enabled:return
        try:
            data=json.loads(msg.data)
            if data["session"]!=self.node.fsm.session:return
            if data["state"] not in ("NAVIGATION","ACQUIRE","PREPARE","ALIGN","DESCEND","FLARE",
                                    "TOUCHDOWN","NATIVE_LAND","COMPLETE"):return
            stamp, seq, producer = data["stamp_monotonic"], data["status_sequence"], data["producer"]
            now = time.monotonic()
            if (type(stamp) not in (int, float) or not math.isfinite(stamp)
                    or not 0 <= now-stamp <= self.timeout
                    or type(seq) is not int or seq <= self.last_status_sequence
                    or not isinstance(producer, str) or not producer):
                self.status_rejected += 1
                return
            if self.producer is not None and producer != self.producer:
                self.received = float("-inf")
                self.status_rejected += 1
                return
            self.producer, self.last_status_sequence = producer, seq
            self.status=data
            self.received=stamp
        except (ValueError,KeyError,TypeError):pass

    def request(self):
        if not self.enabled or self.active:return False
        if not self.node.fsm.hold_here(self.node.observation()):return False
        self.active=True;self.requested=time.monotonic();self.received=None;self.status=None
        self.node._actions(["cancel","disable_planner"])
        self.node._retire_trajectory("platform_landing")
        self.pub.publish(String(data=json.dumps({"action":"land","session":self.node.fsm.session})))
        return True

    def cancel(self):
        if self.active:
            self.pub.publish(String(data=json.dumps({"action":"cancel","session":self.node.fsm.session})))

    def tick(self,now):
        if not self.active:return False
        n=self.node
        fresh=self.received is not None and now-self.received<=self.timeout
        if not fresh:
            if now-self.requested>=self.timeout and not self.failed:
                self.failed=True
                self.cancel()
                n._actions(["cancel","disable_planner","land"])
            elif not self.failed:n._hold()
        elif self.status["state"] in ("NAVIGATION","ACQUIRE"):
            n._hold()  # VIO still owns output, hold altitude while acquiring.
        elif self.status["state"]=="COMPLETE":
            n.fsm.state=State.COMPLETE
            n.fsm.reason="platform_landed_disarmed"
        # Navigation/trajectory forwarding remains suspended even on fallback.
        n.pub_state.publish(String(data=json.dumps(dict(state="PLATFORM_LANDING",
            session=n.fsm.session,platform=self.status,status_fresh=fresh,
            failed=self.failed,goal_reached=bool(n.fsm.goal_reached)))))
        return True
