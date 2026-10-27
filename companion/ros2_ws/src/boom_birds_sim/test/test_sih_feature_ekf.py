from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np
import pytest
from boom_birds_bringup.lifecycle import State
from boom_birds_bringup.tag_flight_task import LandingFix
from boom_birds_control.landing_ekf_reference import EkfReferenceSample
from boom_birds_sim.sih_feature_ekf import SihFeatureEkf


def makeAdapter(monkeypatch):
    clock = NS(now=10.)
    monkeypatch.setattr("boom_birds_sim.sih_feature_ekf.verifySihProcess", lambda pid: pid == 123)
    monkeypatch.setattr("boom_birds_sim.sih_feature_ekf.time.monotonic", lambda: clock.now)
    visual = NS(fix=LandingFix(1000.,(.1,-.1,0.),.52,source="features"))
    tracker = NS(ranges=[(1000.,.5)],poses=[(1000.,np.zeros(3),np.eye(3))],
        rangeMount=np.array([[1.,0,0,0],[0,1.,0,0],[0,0,1.,-.02],[0,0,0,1.]]),
        profile={"guidance":{"range_timeout_s":.2,"telemetry_timeout_s":.2,
                             "range_min_m":.02,"range_max_m":5.,"max_tilt_rad":.35}})
    tracker.rangeMount[:3,2] = (0,0,-1)
    fsm = NS(state=State.LANDING,phase="TAG_DESCEND",entered=9.5)
    mission = NS(handover=False,fsm=fsm,tracker=tracker,
        featureFix=NS(evaluate=lambda now:visual.fix),landingInputs=lambda now:True,
        rosNow=lambda:1000.+clock.now-10.)
    control = NS(_vioFeedback=((0.,0.,.5),(0.,0.,0.),(0.,0.,0.,1.),10.),
        attitude_controller=NS(offset=0.),core=NS(),_publishExecution=lambda out:None)
    captured=[]
    def output(desired):
        captured.append(control._vioFeedback)
        return True
    control.core.outputHandler=output
    current=NS(sample=EkfReferenceSample(10.,(10.,20.),True,False,.3,0,0,-.5,0.,True,.1),
               velocity=(.1,.2))
    reader=NS(snapshot=lambda:(current.sample,current.velocity) if current.sample else None)
    adapter=SihFeatureEkf(mission,control,reader,123)
    return clock,visual,mission,control,current,adapter,captured


def test_local_sih_switch_uses_ekf_xy_and_latches_after_visual_return(monkeypatch):
    clock,visual,mission,control,current,a,captured=makeAdapter(monkeypatch)
    assert mission.featureFix.evaluate(1000.).source=="features"
    clock.now=10.1
    control._vioFeedback=((7.,8.,.5),(0.,0.,0.),(0.,0.,0.,1.),10.1)
    current.sample=EkfReferenceSample(10.1,(10.01,20.02),True,False,.3,0,0,-.49,.1,True,.1)
    visual.fix=None
    fix=mission.featureFix.evaluate(1000.1)
    assert fix.source=="ekf" and a.mode=="ekf"
    assert fix.target[:2] == pytest.approx((.1,-.1))
    assert control.core.outputHandler(None)
    assert captured[-1][0][:2] == pytest.approx((.02,.01))
    assert captured[-1][1][:2] == pytest.approx((.2,.1))
    assert control._vioFeedback[0][:2] == (7.,8.)
    assert mission.landingInputs(clock.now)
    visual.fix=LandingFix(1000.1,(8.,8.,0.),.52,source="features")
    assert mission.featureFix.evaluate(1000.1).source=="ekf"
    assert a.switches==1 and a.outputs==1


def test_invalid_ekf_stops_feedback_and_does_not_revert_to_visual(monkeypatch):
    clock,visual,mission,control,current,a,captured=makeAdapter(monkeypatch)
    mission.featureFix.evaluate(1000.)
    clock.now=10.1
    control._vioFeedback=((0.,0.,.5),(0.,0.,0.),(0.,0.,0.,1.),10.1)
    current.sample=EkfReferenceSample(10.1,(10.,20.),True,False,.3,0,1,-.5,0.,True,.1)
    visual.fix=None
    assert mission.featureFix.evaluate(1000.1) is None
    assert a.mode=="failed" and a.reason=="estimator_reset"
    visual.fix=LandingFix(1000.1,(0.,0.,0.),.52,source="features")
    assert mission.featureFix.evaluate(1000.1) is None
    with pytest.raises(ValueError,match="ekf_fallback_unavailable"):
        control.core.outputHandler(None)
    assert not captured


def test_cannot_attach_to_non_sih_or_handover(monkeypatch):
    monkeypatch.setattr("boom_birds_sim.sih_feature_ekf.verifySihProcess", lambda pid:False)
    with pytest.raises(ValueError,match="requires_local_sih"):
        SihFeatureEkf(NS(handover=False),NS(),NS(),0)


def test_sih_range_loss_uses_ekf_height_and_vertical_velocity(monkeypatch):
    clock,visual,mission,control,current,a,captured=makeAdapter(monkeypatch)
    assert mission.featureFix.evaluate(1000.).source=="features"
    a.injectRangeLoss=True
    a.ekfTicks=20
    clock.now=10.1
    control._vioFeedback=((7.,8.,7.),(0.,0.,4.),(0.,0.,0.,1.),10.1)
    current.sample=EkfReferenceSample(10.1,(10.01,20.02),True,False,.3,0,0,-.49,.1,True,.1)
    visual.fix=None
    fix=mission.featureFix.evaluate(1000.1)
    assert fix.source=="ekf" and fix.height==pytest.approx(.51)
    assert a.rangeLossInjected and a.heightReference.mode=="ekf"
    assert control.core.outputHandler(None)
    assert captured[-1][0][2]==pytest.approx(.49)
    assert captured[-1][1][2]==pytest.approx(-.1)
    assert a.rangeLossOutputs==1


def test_invalid_vertical_ekf_after_range_loss_stops_output(monkeypatch):
    clock,visual,mission,control,current,a,captured=makeAdapter(monkeypatch)
    mission.featureFix.evaluate(1000.)
    a.injectRangeLoss=True
    a.ekfTicks=20
    clock.now=10.1
    control._vioFeedback=((0.,0.,.5),(0.,0.,0.),(0.,0.,0.,1.),10.1)
    current.sample=EkfReferenceSample(10.1,(10.,20.),True,False,.3,0,0,-.49,.1,True,.1)
    visual.fix=None
    assert mission.featureFix.evaluate(1000.1).source=="ekf"
    clock.now=10.2
    control._vioFeedback=((0.,0.,.5),(0.,0.,0.),(0.,0.,0.,1.),10.2)
    current.sample=EkfReferenceSample(10.2,(10.,20.),True,False,.3,0,0,None,None,False,None)
    assert mission.featureFix.evaluate(1000.2) is None
    assert a.mode=="failed"
    with pytest.raises(ValueError,match="ekf_fallback_unavailable"):
        control.core.outputHandler(None)
    assert not captured
