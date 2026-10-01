import numpy as np
import pytest
from boom_birds_sim.scene_reachability import GridSpec, check_reachability
from boom_birds_sim.forest_scenes import load_profile


SPEC = GridSpec(0.1, (2.0, 2.0, 2.0), 0.0, 0.1, 1.9)
START, GOAL = (-0.7, 0, 0.8), (0.7, 0, 0.8)


def test_open_scene_connects_exact_goal():
    result = check_reachability([[0, 0.8, 0.8]], START, GOAL, SPEC, 0.3)
    assert result["verdict"] == "REACHABLE"
    assert result["reachable_6"] and result["reachable_26"]


def test_solid_wall_disconnects_free_endpoints():
    wall = np.array([[0, y, z] for y in np.arange(-1, 1, 0.1)
                     for z in np.arange(0, 2, 0.1)])
    result = check_reachability(wall, START, GOAL, SPEC, 0.3)
    assert result["start_free"] and result["goal_free"]
    assert result["verdict"] == "DISCONNECTED"
    assert not result["tolerance_reachable_26"]


def test_occupied_final_goal_is_not_relocated_to_tolerance_ball():
    result = check_reachability([GOAL], START, GOAL, SPEC, 0.3)
    assert result["verdict"] == "GOAL_OCCUPIED"


def test_inflation_closes_a_gap():
    ys = np.arange(-1, 1, 0.1)
    wall = np.array([[0, y, z] for y in ys if abs(y) > 0.12
                     for z in np.arange(0, 2, 0.1)])
    clear = GridSpec(0.1, (2, 2, 2), 0, 0, 1.9)
    assert check_reachability(wall, START, GOAL, clear, 0.3)["verdict"] == "REACHABLE"
    blocked = GridSpec(0.1, (2, 2, 2), 0, 0.3, 1.9)
    assert check_reachability(wall, START, GOAL, blocked, 0.3)["verdict"] == "DISCONNECTED"


def test_ceiling_prevents_flying_over_wall():
    wall = np.array([[0, y, z] for y in np.arange(-1, 1, 0.1)
                     for z in np.arange(0, 1.3, 0.1)])
    assert check_reachability(wall, START, GOAL, SPEC, 0.3)["verdict"] == "REACHABLE"
    low = GridSpec(0.1, (2, 2, 2), 0, 0.1, 1.4)
    assert check_reachability(wall, START, GOAL, low, 0.3)["verdict"] == "DISCONNECTED"


@pytest.mark.parametrize("cloud", [[], [[float("nan"), 0, 0]], [[0, 0]]])
def test_invalid_cloud_is_not_free_space(cloud):
    with pytest.raises(ValueError):
        check_reachability(cloud, START, GOAL, SPEC, 0.3)


def test_reference_profile_does_not_inherit_dense_geometry():
    ref, dense = load_profile("reference_30m"), load_profile("dense")
    assert ref == dict(forest_obs_num=20, forest_circle_num=20,
                       forest_x_size=40.0, forest_center_x=0.0)
    assert dense["forest_obs_num"] == dense["forest_circle_num"] == 250
    with pytest.raises(ValueError):
        load_profile("unknown")

def test_diagonal_contact_is_not_admitted_as_a_corridor():
    spec = GridSpec(1, (2, 2, 1), 0, 0, -1)
    result = check_reachability([[-0.5, 0.5, 0.5], [0.5, -0.5, 0.5]],
                                [-0.5, -0.5, 0.5], [0.5, 0.5, 0.5], spec, 0)
    assert result["verdict"] == "CORNER_ONLY"
    assert result["reachable_26"] and not result["reachable_6"]


def test_outside_start_is_an_error():
    with pytest.raises(ValueError, match="地图外"):
        check_reachability([[0, 0, 0]], [-2, 0, 0.8], GOAL, SPEC, 0.3)

def test_scene_recorder_keeps_invalid_points_for_admission(tmp_path):
    import importlib.util
    from pathlib import Path
    from types import SimpleNamespace
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    from std_msgs.msg import Header
    tool = Path(__file__).resolve().parents[3] / "tools/sih_record.py"
    module_spec = importlib.util.spec_from_file_location("sih_record_admission_test", tool)
    recorder = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(recorder)
    target = SimpleNamespace(cloud_saved=False, out_dir=tmp_path)
    msg = create_cloud_xyz32(Header(frame_id="global"), [[float("nan"), 0., 1.], [1., 2., 3.]])
    recorder.Recorder._on_cloud(target, msg)
    saved = np.load(tmp_path / "scene_cloud.npy")
    assert saved.shape == (2, 3)
    assert np.isnan(saved[0, 0])
    assert saved[1].tolist() == [1., 2., 3.]
    with pytest.raises(ValueError):
        check_reachability(saved, [0, 0, 1], [1, 0, 1],
                           GridSpec(.1, (4., 4., 3.), 0., .1, 2.5), .3)
