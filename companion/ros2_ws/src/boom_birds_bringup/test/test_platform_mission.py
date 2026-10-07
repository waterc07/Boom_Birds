import json
from types import SimpleNamespace as NS
from boom_birds_bringup.platform_mission import PlatformMissionGate
from boom_birds_bringup.lifecycle import State


class Publisher:
    def __init__(self):self.sent=[]
    def publish(self,msg):self.sent.append(json.loads(msg.data))


class Node:
    def __init__(self):
        self.pub_state=Publisher();self.actions=[];self.holds=0;self.retired=[]
        self.fsm=NS(session="s1",hold_here=lambda observation:True,state=State.EXECUTING,
                    goal_reached=True)
    def declare_parameter(self,*args):pass
    def get_parameter(self,name):return NS(value=True if name.endswith("enabled") else .5)
    def create_publisher(self,*args):return Publisher()
    def create_subscription(self,*args):pass
    def observation(self):return NS(position=(0,0,1.5))
    def _actions(self,actions):self.actions.extend(actions)
    def _retire_trajectory(self,reason):self.retired.append(reason)
    def _hold(self):self.holds+=1


def test_platform_gate_holds_vio_during_acquisition_then_suspends_navigation():
    node=Node();gate=PlatformMissionGate(node)
    assert gate.request() and gate.active
    assert gate.pub.sent==[{"action":"land","session":"s1"}]
    assert node.actions==["cancel","disable_planner"]
    gate.on_status(NS(data=json.dumps(dict(session="s1",state="ACQUIRE"))))
    assert gate.tick(gate.received) and node.holds==1
    gate.on_status(NS(data=json.dumps(dict(session="s1",state="PREPARE"))))
    gate.tick(gate.received)
    assert node.holds==1
    gate.on_status(NS(data=json.dumps(dict(session="old",state="COMPLETE"))))
    assert gate.status["state"]=="PREPARE"
    gate.on_status(NS(data=json.dumps(dict(session="s1",state="COMPLETE"))))
    gate.tick(gate.received)
    assert node.fsm.state==State.COMPLETE


def test_platform_status_loss_cancels_once_and_requests_native_land():
    node=Node();gate=PlatformMissionGate(node);gate.request()
    gate.tick(gate.requested+.6)
    assert gate.failed and gate.pub.sent[-1]=={"action":"cancel"}
    assert node.actions[-3:]==["cancel","disable_planner","land"]
    gate.tick(gate.requested+1.)
    assert node.actions.count("land")==1
