from dataclasses import replace
import json
import threading
import time
import pytest
from test_platform_landing import profile, inputs
from boom_birds_control.platform_executor import PlatformExecutor
from boom_birds_control.platform_landing import HandoffFeedback
from boom_birds_control.platform_runtime import SegmentedTrace
from boom_birds_control.px4_backend import FakePx4Backend


def make(release):
    backend = FakePx4Backend()
    backend.connect()
    p = profile()
    p["runtime"]["history_capacity"] = 32
    ex = PlatformExecutor(backend, p, test_only=True, revoke_navigation=lambda: True,
                          release_compute=release, native_land=lambda why: True, disarm=lambda: True)
    ex.request()
    return ex


def tick(ex, t, fault=None):
    o, r, f = inputs(t)
    if fault:
        o, r, f = fault(o, r, f)
    ack = None
    if ex.core.last_sent_sequence:
        seq = ex.core.last_sent_sequence
        ack = HandoffFeedback(t, ex.core.token, seq, f.epoch, True, False,
                              ex.core.sent_history[seq][1], "TEST_ONLY_FAKE_PX4")
    return ex.tick(t, o, r, f, ack, navigation_ready=True)


def test_service_late_then_releases_once_and_history_is_bounded():
    calls = []
    ex = make(lambda token: calls.append(token) or len(calls) >= 3)
    for i in range(150):
        tick(ex, i*.02)
    assert ex.release_verified and ex.core.released
    assert len(calls) == 3 and len(set(calls)) == 1
    assert len(ex.history) == 32 and ex.history_evicted == 118


def test_async_wait_does_not_resend_and_rejects_wrong_token():
    calls = []
    ex = make(lambda token: calls.append(token))
    for i in range(35):
        tick(ex, i*.02)
    assert ex.release_state == "WAIT_RESULT" and len(calls) == 1
    assert not ex.release_result("old", True)
    assert ex.release_result(ex.core.token, True)
    tick(ex, .72)
    assert ex.core.released


@pytest.mark.parametrize("cancel", ["manual", "epoch", "timeout"])
def test_late_async_success_does_not_confirm_cancelled_or_timed_out_release(cancel):
    ex = make(lambda token: None)
    for i in range(30):
        tick(ex, i*.02)
    token = ex.core.token
    if cancel == "manual":
        ex.cancel()
    elif cancel == "epoch":
        tick(ex, .62, lambda o, r, f: (o, r, replace(f, epoch=1)))
    else:
        for i in range(30, 150):
            tick(ex, i*.02)
        assert ex.release_state == "FAILED"
    assert not ex.release_result(token, True)
    assert not ex.release_verified and not ex.core.released


def test_unavailable_service_attempt_limit_preserves_vio():
    calls = []
    ex = make(lambda token: calls.append(token) or False)
    for i in range(150):
        tick(ex, i*.02)
    assert ex.release_state == "FAILED"
    assert len(calls) == ex.runtime.release_max_attempts
    assert not ex.core.released


def test_segmented_writer_and_invalid_record_are_observable(tmp_path):
    writer = SegmentedTrace(tmp_path/"trace", queue_capacity=2, segment_records=2)
    # Never make the control producer wait for a saturated writer.
    for i in range(1000):
        writer.put(dict(i=i))
    assert writer.close()
    lines = [json.loads(line) for p in sorted((tmp_path/"trace").glob("*.jsonl"))
             for line in p.read_text().splitlines()]
    assert len(lines) == writer.written
    assert writer.dropped + writer.written == 1000
    assert all(len(p.read_text().splitlines()) <= 2 for p in (tmp_path/"trace").glob("*.jsonl"))
    assert not writer.put(dict(i=1001))
    bad = SegmentedTrace(tmp_path/"bad")
    bad.put(dict(value=float("nan")))
    assert bad.close() and bad.errors == 1


def test_native_land_completion_requires_fresh_landed_and_disarmed_feedback():
    ex = make(lambda token: True)
    ex.cancel()
    o, r, f = inputs(1.)
    assert ex.tick(1., o, r, replace(f, landed=True, armed=True)).state == "NATIVE_LAND"
    assert ex.tick(2., o, r, replace(f, landed=True, armed=False)).state == "NATIVE_LAND"
    assert ex.tick(2.02, o, r, replace(f, stamp=2.02, landed=True, armed=False)).state == "COMPLETE"

def test_detector_rejects_nonfinite_solver_candidates(monkeypatch):
    import cv2
    import numpy as np
    from boom_birds_sensing.platform_observation import BoardDetector
    detector = BoardDetector(profile(), test_only=True)
    monkeypatch.setattr(cv2, "solvePnPGeneric", lambda *a, **k: (1, [np.full((3, 1), np.nan)], [np.ones((3, 1))], None))
    out = detector.estimate([(7, np.array([[100.,100.],[200.,100.],[200.,200.],[100.,200.]]))], 1.)
    assert not out.valid and out.reason == "pnp_failed"

@pytest.mark.parametrize("fault", ["wrong_id", "stale", "future", "pose_jump", "range_jump",
                                  "reordered", "tag_loss", "epoch", "offboard", "estimator"])
@pytest.mark.parametrize("at_tick", [2, 8, 30])
def test_faults_at_acquire_prepare_and_descent_never_retain_downward_velocity(fault, at_tick):
    from dataclasses import replace
    ex = make(lambda token: True)
    for i in range(at_tick):
        tick(ex, i*.02)
    now = at_tick*.02
    def inject(o, r, f):
        if fault == "wrong_id": o = replace(o, tag_ids=(999,))
        elif fault == "stale": o = replace(o, stamp=now-1.)
        elif fault == "future": o = replace(o, stamp=now+1.)
        elif fault == "pose_jump":
            p = [list(row) for row in o.T_body_platform]
            p[0][3] = 2.
            o = replace(o, T_body_platform=tuple(map(tuple,p)))
        elif fault == "range_jump": r = replace(r, distance_m=r.distance_m+1.)
        elif fault == "reordered": o = replace(o, stamp=max(0., now-.03))
        elif fault == "tag_loss": o = replace(o, valid=False)
        elif fault == "epoch": f = replace(f, epoch=1)
        elif fault == "offboard": f = replace(f, offboard=False)
        elif fault == "estimator": f = replace(f, velocity_estimator_valid=False)
        return o, r, f
    result = tick(ex, now, inject)
    # 准入期间已改变的 epoch 可作为新基线；接管后必须失效。
    if fault in ("epoch", "offboard") and at_tick == 2:
        return
    assert result.velocity_ned is None or result.velocity_ned[2] <= 0
    assert not result.descent_permitted

def test_release_receipt_during_tag_loss_updates_resource_state_without_descent():
    ex = make(lambda token: None)
    for i in range(30):
        tick(ex, i*.02)
    assert ex.release_result(ex.core.token, True)
    result = tick(ex, .62, lambda o,r,f: (replace(o, valid=False),r,f))
    assert ex.core.released and not result.vio_required and not result.release_compute
    assert not result.descent_permitted and result.velocity_ned[2] == 0
