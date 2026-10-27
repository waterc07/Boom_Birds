"""Suspend the navigation mission while the single PX4 interface owns platform landing."""
import json
import time
import math
from std_msgs.msg import String
from .lifecycle import State

#: 平台状态里允许出现的引导状态；未知状态不刷新交接有效性。
PLATFORM_STATES = ("NAVIGATION", "ACQUIRE", "PREPARE", "ALIGN", "DESCEND", "FLARE",
                   "TOUCHDOWN", "NATIVE_LAND", "COMPLETE")


class PlatformMissionGate:
    """任务侧交接闸门：请求平台接管、停导航输出，并如实报告失败分支。

    VIO 是否已释放只来自平台状态里的 release_verified；任务节点不得自行推断。
    交接确认前不释放 VIO，任务保持平台 ACQUIRE 阶段的高度。
    """

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
        self.failure=""
        self.producer=None
        self.lastStatusSequence=-1
        self.statusRejected=0
        self.requestSent=False
        self.cancelSent=False
        # 由任务节点在启用 Tag 交接时显式打开；通用导航平台交接保持原行为。
        self.handover=False
        self.releaseRequired=False
        self.evidence={}
        self.yawOffset=0.
        self.pub=node.create_publisher(String,"/boom_birds/platform/request",10)
        node.create_subscription(String,"/boom_birds/platform/status",self.onStatus,10)

    def captureEvidence(self):
        """交接前由任务 FSM 提供的身份/质量/新鲜度/连续观测证据。"""
        read=getattr(self.node.fsm,"handoverEvidence",None)
        try:
            return dict(read()) if callable(read) else {}
        except (TypeError,ValueError):
            return {}

    def onStatus(self,msg):
        if not self.enabled:return
        try:
            data=json.loads(msg.data)
            if data["session"]!=self.node.fsm.session:return
            if data["state"] not in PLATFORM_STATES:return
            stamp, seq, producer = data["stamp_monotonic"], data["status_sequence"], data["producer"]
            # stamp_monotonic 只适用于同主机进程；重复序号和生产者重启不能刷新交接状态。
            now = time.monotonic()
            if (type(stamp) not in (int, float) or not math.isfinite(stamp)
                    or not 0 <= now-stamp <= self.timeout
                    or type(seq) is not int or seq <= self.lastStatusSequence
                    or not isinstance(producer, str) or not producer):
                self.statusRejected += 1
                return
            if self.producer is not None and producer != self.producer:
                self.received = float("-inf")
                self.statusRejected += 1
                return
            self.producer, self.lastStatusSequence = producer, seq
            self.status=data
            self.received=stamp
        except (ValueError,KeyError,TypeError):pass

    def request(self,yawOffset=None):
        if not self.enabled or self.active:return False
        if self.handover:
            self.evidence=self.captureEvidence()
            if self.evidence.get("ready") is not True:return False
            read=getattr(self.node.fsm,"handoverYawOffset",None)
            try:
                yawOffset=float(read()) if callable(read) else 0.
            except (TypeError,ValueError):
                return False
        elif yawOffset is None:
            yawOffset=0.
        if (type(yawOffset) not in (int, float) or not math.isfinite(yawOffset)
                or abs(yawOffset)>math.pi):
            return False
        if not self.node.fsm.holdHere(self.node.observation()):return False
        self.active=True;self.requested=time.monotonic();self.received=None;self.status=None
        self.yawOffset=float(yawOffset)
        self.node._actions(["cancel","disable_planner"])
        self.node._retireTrajectory("platform_landing")
        message=dict(action="land",session=self.node.fsm.session)
        if self.handover:message["yaw_offset_rad"]=self.yawOffset
        self.pub.publish(String(data=json.dumps(message)))
        self.requestSent=True
        return True

    def cancel(self):
        if self.active and not self.cancelSent:
            self.pub.publish(String(data=json.dumps({"action":"cancel","session":self.node.fsm.session})))
            self.cancelSent=True

    def releaseVerified(self):
        return self.status is not None and self.status.get("release_verified") is True

    def fail(self,reason):
        if reason and not self.failed:
            self.failed=True;self.failure=reason

    def report(self):
        status=self.status or {}
        return dict(state=status.get("state"),owner=status.get("owner"),
            reason=status.get("reason"),failure_reason=status.get("failure_reason"),
            vio_required=status.get("vio_required"),
            release_state=status.get("release_state"),release_verified=status.get("release_verified"),
            confirmed=status.get("confirmed"),yaw_offset_rad=self.yawOffset,
            evidence=self.evidence,failed=self.failed,failure=self.failure,
            status_rejected=self.statusRejected)

    def tick(self,now):
        if not self.active:return False
        n=self.node
        started=getattr(n.fsm,"flightStarted",None)
        mission=getattr(n.fsm,"mission",None)
        budget=getattr(mission,"flight_budget_s",None)
        if (not self.failed and started is not None and budget is not None
                and now-started>=budget):
            self.fail("mission_time_budget")
            self.cancel()
            n._actions(["cancel","disable_planner","land"])
        fresh=self.received is not None and now-self.received<=self.timeout
        if not fresh:
            if now-self.requested>=self.timeout and not self.failed:
                self.fail("platform_status_lost")
                self.cancel()
                n._actions(["cancel","disable_planner","land"])
            elif not self.failed:
                n._hold()
        else:
            state=self.status.get("state")
            owner=self.status.get("owner")
            if state=="COMPLETE":
                if self.status.get("failure_reason"):
                    self.fail(self.status["failure_reason"])
                if self.handover and self.releaseRequired and not self.releaseVerified():
                    # 已落地但 VIO 未确认退出：不把本条任务报告为成功。
                    self.fail("compute_release_not_verified")
                if n.fsm.state!=State.COMPLETE:
                    n.fsm.entered=now
                n.fsm.state=State.COMPLETE
                self.active=False
                n.fsm.reason=("platform_landed_disarmed" if not self.failed
                              else self.failure)
            elif state in ("NAVIGATION","ACQUIRE"):
                n._hold()  # VIO 仍持有输出；采集阶段维持当前高度。
            elif state=="NATIVE_LAND":
                # 近地按设计交给 PX4 Land 不是故障；其它原因进入 NATIVE_LAND 仍记失败。
                reason=self.status.get("reason") or "native_land_fallback"
                if reason!="near_ground_native_land":
                    self.fail(reason)
            else:
                # PREPARE 之后平台持有速度输出；任务节点不再发布任何导航 setpoint。
                pass
        n.pubState.publish(String(data=json.dumps(dict(state="PLATFORM_LANDING",
            session=n.fsm.session,platform=self.status,platform_report=self.report(),
            status_fresh=fresh,failed=self.failed,goal_reached=bool(n.fsm.goal_reached)))))
        return True
