from dataclasses import replace
import pytest
from boom_birds_control.landing_ekf_height import LandingEkfHeight
from boom_birds_control.landing_ekf_reference import EkfReferenceSample


def height():
    return LandingEkfHeight(max_age_s=.2,max_pair_skew_s=.05,max_accuracy_m=.4,
        max_duration_s=12.,max_speed_m_s=1.,max_height_m=2.)


def sample(t=1.,z=-.5,**kw):
    return replace(EkfReferenceSample(t,(0.,0.),True,False,.1,0,0,
        z,.1,True,.15),**kw)


def test_ekf_height_latches_after_range_loss_and_preserves_ground_reference():
    h=height()
    assert h.step(1.,sample(),1.,.52)==.52
    assert h.step(1.1,sample(1.1,-.49))==pytest.approx(.51)
    assert h.mode=="ekf"
    assert h.step(1.2,sample(1.2,-.48),1.2,1.4)==pytest.approx(.50)
    assert h.mode=="ekf"


@pytest.mark.parametrize("changes",[
    {"vertical_position_valid":False},{"vertical_accuracy_m":.5},{"vertical_accuracy_m":0.},
    {"reset_counter":1},{"epoch":1},{"position_ned_z":float("nan")},
    {"velocity_ned_z":2.},{"position_ned_z":0.},
])
def test_invalid_vertical_state_never_returns_height(changes):
    h=height()
    assert h.step(1.,sample(),1.,.52)==.52
    assert h.step(1.1,sample(1.1,**changes)) is None
    assert h.mode=="failed"
    assert h.step(1.2,sample(1.2),1.2,.5) is None


def test_no_ground_anchor_cannot_estimate_height():
    h=height()
    assert h.step(1.,sample()) is None
    assert h.reason=="height_anchor_missing"
