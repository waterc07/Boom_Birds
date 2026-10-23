import importlib.util
from pathlib import Path
import json
import pytest

p = Path(__file__).resolve().parents[1] / "analyze_occupancy_log.py"
spec = importlib.util.spec_from_file_location("ego_occupancy_analysis", p)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def record(**kwargs):
    d = dict(finite=True, in_map=True, position=[0., 0., 0.], raw=0,
             inflated=0, raw_unknown=True, virtual_ceiling=False,
             contributors=[], contributors_total=0, contributors_truncated=False)
    d.update(kwargs)
    return d


def line(d):
    return "[WARN] BB_OCCUPANCY reason=initial_bspline_collision data=" + json.dumps(d)


def test_unknown_is_not_free():
    assert m.classify(record()) == "unknown_without_inflation"
    assert m.classify(record(raw_unknown=False)) == "known_free"


def test_outside_and_nonfinite_are_not_obstacles():
    assert m.classify(record(in_map=False)) == "outside_map"
    assert m.classify(dict(finite=False, in_map=False)) == "nonfinite_input"


def test_raw_ceiling_and_unexplained_inflation():
    assert m.classify(record(raw=1, inflated=1)) == "raw_occupied"
    assert m.classify(record(inflated=1, virtual_ceiling=True)) == "virtual_ceiling"
    assert m.classify(record(inflated=1)) == "inflated_without_current_contributor"


def test_neighbor_contributors_and_truncation():
    c = dict(index=[1, 2, 3], center=[.1, .2, .3], log_odds=2.)
    assert m.classify(record(inflated=1, contributors=[c], contributors_total=1)) == "inflated_from_neighbors"
    assert m.classify(record(inflated=1, contributors=[c]*16, contributors_total=27, contributors_truncated=True)) == "inflated_from_neighbors"


@pytest.mark.parametrize("lines", [[], ["map_ready=false"], ["BB_OCCUPANCY broken"],
                                    [line(record()).replace('true', 'NaN', 1)]])
def test_absent_or_malformed_fails_closed(lines):
    with pytest.raises(ValueError): m.analyze(lines)


def test_bad_source_count_rejected():
    with pytest.raises(ValueError): m.classify(record(contributors_total=1))


def test_summary_and_bounded_input():
    result = m.analyze(["unrelated", line(record()), line(record(raw=1))])
    assert result["record_count"] == 2
    assert result["records"][0]["line"] == 2
    with pytest.raises(ValueError): m.analyze([line(record())]*2, maxRecords=1)
