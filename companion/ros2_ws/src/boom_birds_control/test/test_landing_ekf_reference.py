from dataclasses import replace
import pytest
from boom_birds_control.landing_ekf_reference import (
    EkfReferenceSample, LandingEkfReference, LandingReferenceSelector, VisualReference)


def reference():
    return LandingEkfReference(max_age_s=.2, max_pair_skew_s=.05, max_accuracy_m=.15,
        max_duration_s=12., max_displacement_m=.5, max_speed_m_s=1.)


def sample(t=1., position=(10., 20.), **changes):
    return replace(EkfReferenceSample(t, position, True, False, .03, 4, 5), **changes)


def visual(t=1., error=(.1, -.2)):
    return VisualReference(t, error)


def test_selector_latches_ekf_for_remainder_even_if_vision_returns():
    selector = LandingReferenceSelector(reference())
    assert selector.step(1., visual(), sample()) == (.1, -.2)
    assert selector.step(1.1, None, sample(1.1, (10.02, 19.97))) == pytest.approx((.08, -.17))
    assert selector.mode == "ekf"
    assert selector.step(1.2, visual(1.2, (.4, .4)), sample(1.2, (10.03, 19.96))) == pytest.approx((.07, -.16))
    assert selector.mode == "ekf"
    # More than the previous .3 second coast window, with no height threshold.
    assert selector.step(2., None, sample(2., (10.03, 19.96))) == pytest.approx((.07, -.16))
    assert selector.mode == "ekf"


@pytest.mark.parametrize("change,reason", [
    ({"horizontal_position_valid":False}, "estimator_unavailable"),
    ({"constant_position_mode":True}, "estimator_unavailable"),
    ({"position_accuracy_m":.2}, "position_accuracy"),
    ({"position_accuracy_m":0.}, "position_accuracy"),
    ({"epoch":5}, "estimator_reset"),
    ({"reset_counter":6}, "estimator_reset"),
    ({"position_ned_xy":(float("nan"), 20.)}, "sample_invalid"),
    ({"position_ned_xy":(11., 20.)}, "position_jump"),
])
def test_loss_of_evidence_revokes_anchor(change, reason):
    selector = LandingReferenceSelector(reference())
    selector.step(1., visual(), sample())
    assert selector.step(1.1, None, sample(1.1, **change)) is None
    assert selector.reason == reason and selector.mode == "failed"
    assert selector.step(1.2, visual(1.2), sample(1.2)) is None


def test_seed_requires_synchronized_visual_and_estimator():
    ref = reference()
    assert not ref.seed(1., visual(.9), sample())
    assert ref.reason == "visual_pair_invalid"
    assert not ref.seed(1., visual(), sample(horizontal_position_valid=False))
    assert ref.reason == "estimator_unavailable"


def test_new_unpaired_visual_does_not_leave_a_stale_anchor():
    selector = LandingReferenceSelector(reference())
    selector.step(1., visual(), sample())
    assert selector.step(1.1, visual(1.1), None) == (.1, -.2)
    assert selector.step(1.2, None, sample(1.2)) is None
    assert selector.mode == "failed"


def test_stale_visual_triggers_switch_and_sticky_failure_requires_reset():
    selector = LandingReferenceSelector(reference())
    selector.step(1., visual(), sample())
    assert selector.step(1.21, visual(), sample(1.21)) == pytest.approx((.1, -.2))
    assert selector.mode == "ekf"
    assert selector.step(1.3, None, sample(1.3, reset_counter=6)) is None
    selector.reset()
    assert selector.step(1.4, visual(1.4), sample(1.4)) == (.1, -.2)


def test_freshness_reordering_and_reference_lifetime():
    ref = reference()
    assert ref.seed(1., visual(), sample())
    assert ref.update(1.1, sample(.85)) is None
    assert ref.reason == "position_stale"
    assert ref.seed(1., visual(), sample())
    assert ref.update(1.1, sample(.99)) is None
    assert ref.reason == "sample_reordered"
    assert ref.seed(1., visual(), sample())
    assert ref.update(13.1, sample(13.1)) is None
    assert ref.reason == "reference_expired"


def test_no_new_sample_may_move():
    ref = reference()
    assert ref.seed(1., visual(), sample())
    assert ref.update(1.05, sample(1., (10.01, 20.))) is None
    assert ref.reason == "position_jump"


@pytest.mark.parametrize("loss_height", [.441, .145])
def test_synthetic_xy_reference_continues_to_touchdown(loss_height):
    # Perfect simulated EKF and range; tests the reference math, not aircraft dynamics.
    selector = LandingReferenceSelector(reference())
    t, x, y, height = 1., .04, -.03, loss_height
    assert selector.step(t, visual(t, (-x, -y)), sample(t, (x, y))) == (-x, -y)
    while height > 0:
        t += .05
        error = selector.step(t, None, sample(t, (x, y)))
        assert selector.mode == "ekf" and error == pytest.approx((-x, -y))
        x += .6*error[0]*.05
        y += .6*error[1]*.05
        height = max(0., height-.08*.05)
    assert abs(x) < .04 and abs(y) < .03
