from collections import deque
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
from boom_birds_sensing.mavros_range_node import MavrosRangeNode
from boom_birds_sensing import mavros_range_node


class PendingTimer:
    def __init__(self):self.canceled=True
    def is_canceled(self):return self.canceled
    def reset(self):self.canceled=False
    def cancel(self):self.canceled=True


def nodeWithClock(monkeypatch):
    node=object.__new__(MavrosRangeNode)
    clocks=[100.,1000.]
    monkeypatch.setattr(mavros_range_node.time,"monotonic",lambda:clocks[0])
    node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=round(clocks[1]*1e9)))
    node.connected=True;node.stateAt=100.;node.requested=True
    node.pending=deque(maxlen=32);node._pendingTimer=PendingTimer()
    node.published=0;node.lastPublished=None;node.pub=Mock()
    reading=NS(stamp=1000.01,field_of_view=.2,minimum_m=.02,maximum_m=6.,distance_m=.5)
    gate=NS(lastSync=100.,sync_timeout_s=1.,restarts=0)
    node.decoder=NS(map=Mock(return_value=reading),gate=gate,maxAge=.15,reject=Mock(),lastStamp=None,onSync=Mock())
    return node,clocks,reading


def test_future_range_waits_and_publishes_original_stamp_once(monkeypatch):
    node,clocks,reading=nodeWithClock(monkeypatch)
    node.onRaw(NS(msgid=132))
    assert node.pending and not node._pendingTimer.is_canceled()
    assert node.published==0
    clocks[:]=[100.009,1000.009];node.drain()
    assert node.published==0
    clocks[:]=[100.011,1000.011];node.drain();node.drain()
    assert node.published==1 and not node.pending and node._pendingTimer.is_canceled()
    message=node.pub.publish.call_args.args[0]
    assert message.header.stamp.sec==1000 and message.header.stamp.nanosec==10000000
    assert message.range==.5


@pytest.mark.parametrize("fault",["heartbeat","no_sync","expired_sync","future_sync"])
def test_unhealthy_clock_or_link_drops_waiting_range(monkeypatch,fault):
    node,clocks,_=nodeWithClock(monkeypatch);node.onRaw(NS(msgid=132))
    if fault=="heartbeat":node.connected=False
    elif fault=="no_sync":node.decoder.gate.lastSync=None
    elif fault=="expired_sync":node.decoder.gate.lastSync=98.
    else:node.decoder.gate.lastSync=101.
    node.drain()
    assert not node.pending and node._pendingTimer.is_canceled() and node.published==0


def test_overdue_waiting_range_is_not_published(monkeypatch):
    node,clocks,_=nodeWithClock(monkeypatch);node.onRaw(NS(msgid=132))
    clocks[:]=[100.3,1000.3];node.drain()
    assert node.published==0 and not node.pending and node._pendingTimer.is_canceled()
    node.decoder.reject.assert_called_once_with("range_publish_stale")


def test_sync_reset_clears_future_range(monkeypatch):
    node,_,_=nodeWithClock(monkeypatch);node.onRaw(NS(msgid=132))
    def reset(*args):node.decoder.gate.lastSync=None;node.decoder.gate.restarts+=1
    node.decoder.onSync.side_effect=reset;node.onSync(NS())
    assert not node.pending and node._pendingTimer.is_canceled() and not node.requested
    node.pub.publish.assert_not_called()


def test_irrelevant_raw_packet_does_not_enter_range_mapping(monkeypatch):
    node,_,_=nodeWithClock(monkeypatch)
    node.get_clock=Mock(side_effect=AssertionError("irrelevant packet read clock"))
    node.onRaw(NS(msgid=105))
    node.decoder.map.assert_not_called()
    assert not node.pending and node.published==0
