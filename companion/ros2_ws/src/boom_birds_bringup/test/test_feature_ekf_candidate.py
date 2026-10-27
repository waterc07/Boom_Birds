from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
import yaml
from boom_birds_bringup.feature_ekf_candidate_node import loadCandidate
from boom_birds_bringup.feature_ekf_bridge import DryRunFeatureEkfBridge
from boom_birds_bringup.lifecycle import State
from boom_birds_bringup.tag_flight_task import LandingFix
from boom_birds_control.landing_ekf_reference import EkfReferenceSample


def config(tmp_path):
    paths={}
    for name in ("attitude_config_file","flight_task_file","tag_config_file","obstacle_config_file"):
        p=tmp_path/(name+".yaml");p.write_text("{}")
        paths[name]=str(p)
    return dict(schema=1,max_accuracy_m=.2,control=dict(dry_run=True,allow_arming=False,
        allow_hardware_actions=False,require_session=True,control_mode="companion_attitude",
        backend="mavros",attitude_config_file=paths["attitude_config_file"]),
        task=dict(dry_run=True,start=False,handover_enabled=False,platform_landing_enabled=False,
                  feature_tracking_enabled=True,control_mode="companion_attitude",**paths))


@pytest.mark.parametrize("group,key,value",[
    ("control","dry_run",False),("control","allow_arming",True),
    ("control","allow_hardware_actions",True),("task","start",True),
    ("task","handover_enabled",True),("task","dry_run",False),
])
def test_deployed_candidate_rejects_live_or_automatic_start(tmp_path,group,key,value):
    c=config(tmp_path);c[group][key]=value
    p=tmp_path/"candidate.yaml";p.write_text(yaml.safe_dump(c))
    with pytest.raises(ValueError,match="candidate_"):
        loadCandidate(p)


def test_candidate_config_loads_only_existing_files_and_conservative_limit(tmp_path):
    c=config(tmp_path);p=tmp_path/"candidate.yaml"
    p.write_text(yaml.safe_dump(c));assert loadCandidate(p)==c
    c["max_accuracy_m"]=.4;p.write_text(yaml.safe_dump(c))
    with pytest.raises(ValueError,match="accuracy"):loadCandidate(p)


def test_hardware_dry_run_bridge_switches_without_injecting_failures(monkeypatch):
    clock=NS(now=10.)
    monkeypatch.setattr("boom_birds_bringup.feature_ekf_bridge.time.monotonic",lambda:clock.now)
    visual=NS(fix=LandingFix(1000.,(.1,-.1,0.),.52,"features"))
    tracker=NS(ranges=[(1000.,.5)],poses=[(1000.,np.zeros(3),np.eye(3))],
        rangeMount=np.diag([1.,-1.,-1.,1.]),
        profile={"guidance":dict(range_timeout_s=.2,telemetry_timeout_s=.2,
                                 range_min_m=.02,range_max_m=5.,max_tilt_rad=.35)})
    tracker.rangeMount[2,3]=-.02
    params={"dry_run":True,"start":False}
    mission=NS(handover=False,get_parameter=lambda k:NS(value=params[k]),
        fsm=NS(state=State.LANDING,phase="TAG_DESCEND"),tracker=tracker,
        featureFix=NS(evaluate=lambda now:visual.fix),landingInputs=lambda now:True,
        rosNow=lambda:1000.+clock.now-10.)
    control=NS(backend=NS(dry_run=True),core=NS(),
        _vioFeedback=((0.,0.,.5),(0.,0.,0.),(0.,0.,0.,1.),10.),
        attitude_controller=NS(offset=0.),_publishExecution=lambda out:None)
    outputs=[]
    def send(desired):
        outputs.append(control._vioFeedback);return True
    control.core.outputHandler=send
    current=NS(sample=EkfReferenceSample(10.,(10.,20.),True,False,.1,0,0,-.5,0.,True,.1))
    reader=NS(snapshot=lambda:(current.sample,(.1,.2)))
    bridge=DryRunFeatureEkfBridge(mission,control,reader)
    assert mission.featureFix.evaluate(1000.).source=="features"
    clock.now=10.1
    tracker.ranges.clear()
    current.sample=EkfReferenceSample(10.1,(10.01,20.02),True,False,.1,0,0,-.49,.1,True,.1)
    control._vioFeedback=((9.,8.,7.),(0.,0.,4.),(0.,0.,0.,1.),10.1)
    visual.fix=None
    result=mission.featureFix.evaluate(1000.1)
    assert result.source=="ekf" and result.height==pytest.approx(.51)
    assert control.core.outputHandler(None)
    assert outputs[-1][0]==pytest.approx((.02,.01,.49))
    assert outputs[-1][1]==pytest.approx((.2,.1,-.1))
    assert bridge.report()["live_control_enabled"] is False
    control.backend.dry_run=False
    with pytest.raises(ValueError,match="live_not_verified"):control.core.outputHandler(None)
