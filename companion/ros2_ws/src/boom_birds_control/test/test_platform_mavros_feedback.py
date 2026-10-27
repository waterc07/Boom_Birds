import struct
from types import SimpleNamespace as NS
import pytest
from boom_birds_control.platform_mavros_feedback import MavrosVelocityFeedback
from boom_birds_control.platform_landing import VELOCITY_YAW_RATE_MASK


def wire(mid, payload):
    data=payload+bytes((-len(payload))%8)
    return NS(msgid=mid,sysid=1,compid=1,framing_status=1,len=len(payload),
        header=NS(stamp=NS(sec=1000,nanosec=0)),
        payload64=struct.unpack("<"+"Q"*(len(data)//8),data))


def reader():
    r=MavrosVelocityFeedback.__new__(MavrosVelocityFeedback)
    r.samples={};r.lastBoot={};r.epoch=None;r.diagnostic={}
    r.node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=1000_000_000_000)))
    state=NS(restart_epoch=0,frame_reset_epoch=0,connected=True,is_offboard=True)
    b=NS(_endpointVerified=True,target_system=1,target_component=1,_sihReceiptTime=False,
         _clock=lambda:100.,readVehicleState=lambda:state,control_mode="px4_position",
         _attitudePub=None,_platformToken="t",_timeGate=NS(lastSync=99.9,count=5,
             converge_samples=5,sync_timeout_s=1.,offsetNs=900_000_000_000))
    r.backend=b;r.core=NS(token="t",sent_history={5:(99.95,(.1,0.,0.))})
    target=wire(85,struct.pack("<I11fHB",99980,0.,0.,0.,.1,0.,0.,0.,0.,0.,0.,0.,VELOCITY_YAW_RATE_MASK,1))
    estimate=wire(230,struct.pack("<Q8fH",99990000,*([0.]*8),7))
    r.onRaw(target);r.onRaw(estimate)
    return r,state,target,estimate


def test_raw_fcu_clock_estimator_flags_and_correlated_target():
    r,_,_,_=reader()
    ack,valid=r.snapshot()
    assert valid and ack.sequence==5 and ack.source==r.source
    assert ack.stamp==pytest.approx(99.98)
    assert ack.echoed_velocity_ned==pytest.approx((.1,0.,0.))
    # 接收时钟不刷新 PX4 采样时钟；文件/缓存重复回读会自然过期。
    r.backend._clock=lambda:100.3
    assert r.snapshot()==(None,False)


@pytest.mark.parametrize("fault",["mode","publisher","token","epoch","flags","mask","frame","target","sync","source"])
def test_candidate_cannot_confirm_missing_or_conflicting_evidence(fault):
    r,state,target,estimate=reader()
    if fault=="mode":state.is_offboard=False
    elif fault=="publisher":r.backend._attitudePub=object()
    elif fault=="token":r.backend._platformToken="old"
    elif fault=="epoch":state.frame_reset_epoch=1
    elif fault=="flags":r.samples[230][1].flags=1
    elif fault=="mask":r.samples[85][1].type_mask=0
    elif fault=="frame":r.samples[85][1].coordinate_frame=8
    elif fault=="target":r.samples[85][1].vx=.5
    elif fault=="sync":
        r.samples.clear();r.lastBoot.clear();r.backend._timeGate.count=0
        r.onRaw(target);r.onRaw(estimate)
    elif fault=="source":
        r.samples.clear();r.lastBoot.clear();target.sysid=2;estimate.compid=2
        r.onRaw(target);r.onRaw(estimate)
    assert r.snapshot()[0] is None


def test_reordered_and_duplicate_raw_target_cannot_refresh_evidence():
    r,_,target,_=reader()
    stamp=r.samples[85][0]
    r.node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=1000_100_000_000)))
    r.onRaw(target)
    assert r.samples[85][0]==stamp
