from pathlib import Path
import importlib.util
import cv2
import numpy as np
import pytest
import yaml
from launch import LaunchContext
from launch_ros.utilities import evaluate_parameters

from boom_birds_bringup.compute_profile import prepare_mono_vio, raw_geometry, openvins_parameters
from boom_birds_bringup.hardware_profile import validate_profile, read_yaml
from test_hardware_profile import bundle


def mono_bundle(tmp_path):
    args = bundle(tmp_path)
    original = read_yaml(args["vio_config_file"])
    original["init_dyn_min_rec_cond"] = "1e-12"
    Path(args["vio_config_file"]).write_text(yaml.safe_dump(original))
    config = prepare_mono_vio(args["vio_config_file"], tmp_path / "mono", 320, 240)
    args["vio_config_file"] = str(config)
    return args


def test_mono_removes_cam1_preserves_metric_extrinsics_and_scales_rays(tmp_path):
    args = mono_bundle(tmp_path)
    config = Path(args["vio_config_file"])
    chain = read_yaml(config.parent / "kalibr_imucam_chain.yaml")
    assert list(chain) == ["cam0"]
    fs = cv2.FileStorage(str(config.parent / "kalibr_imucam_chain.yaml"), cv2.FILE_STORAGE_READ)
    assert fs.isOpened()
    assert fs.getNode("cam0").getNode("T_imu_cam").size() == 4
    fs.release()
    assert chain["cam0"]["resolution"] == [320, 240]
    assert chain["cam0"]["intrinsics"] == [200., 200., 160., 120.]
    assert chain["cam0"]["T_imu_cam"] == np.eye(4).tolist()
    assert read_yaml(config)["init_dyn_min_rec_cond"] == 1e-12
    assert validate_profile(**args)["hardware_verified"] is False
    with pytest.raises(ValueError, match="只允许 dry-run"):
        validate_profile(**args, live=True)
    assert raw_geometry(config, 1280, 480) == {"raw_output_scale": .5, "raw_decode_divisor": 2}
    params = openvins_parameters(config)
    assert params["max_cameras"] == 1 and params["use_stereo"] is False
    assert params["max_slam"] == 0 and params["num_pts"] == 100
    with pytest.raises(FileExistsError):
        prepare_mono_vio(tmp_path / "estimator.yaml", config.parent, 320, 240)


def test_mono_rejects_unscaled_intrinsics_and_nonuniform_image_scaling(tmp_path):
    args = mono_bundle(tmp_path)
    config = Path(args["vio_config_file"])
    with pytest.raises(ValueError, match="等比例"):
        raw_geometry(config, 1280, 960)
    chain_path = config.parent / "kalibr_imucam_chain.yaml"
    chain = read_yaml(chain_path)
    chain["cam0"]["intrinsics"][0] *= 2
    chain_path.write_text(yaml.safe_dump(chain))
    with pytest.raises(ValueError, match="内参不同"):
        validate_profile(**args)


def test_synthetic_marker_survives_derivation(tmp_path):
    args = bundle(tmp_path)
    source = Path(args["vio_config_file"])
    source.write_text("# TEST-ONLY\n" + source.read_text())
    args["vio_config_file"] = str(prepare_mono_vio(source, tmp_path / "derived"))
    with pytest.raises(ValueError, match="合成配置"):
        validate_profile(**args)


def test_mono_launch_passes_one_camera_and_budget_to_real_node(tmp_path, monkeypatch):
    path = Path(__file__).resolve().parents[1] / "launch/attitude_hardware.launch.py"
    spec = importlib.util.spec_from_file_location("mono_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    share = module.get_package_share_directory
    monkeypatch.setattr(module, "get_package_share_directory", lambda name:
        str(tmp_path / name) if name == "ego_planner" else share(name))
    ctx = LaunchContext()
    ctx.launch_configurations.update(mono_bundle(tmp_path))
    ctx.launch_configurations.update(camera_device="/dev/test", capture_width="1280", capture_height="480",
        capture_fps="60", output_scale="1.0", compute_profile="mono_budget", shared_decode="auto", dry_run="true",
        allow_arming="false", allow_non_loopback="false", allow_hardware_actions="false", hardware_validation_note="")
    actions = module.nodes(ctx)
    # 采集/深度合并后仍向估计器传入单目覆盖。
    params = evaluate_parameters(ctx, actions[3]._Node__parameters)[0]
    assert params["max_cameras"] == 1 and params["use_stereo"] is False
    assert params["num_opencv_threads"] == 1 and params["max_clones"] == 6
    source_parameters = evaluate_parameters(ctx, actions[2]._Node__parameters)
    assert source_parameters[0].name == "stereo_camera.yaml"
    source = source_parameters[-1]
    assert source["raw_output_scale"] == .5
    assert source["raw_decode_divisor"] == 2
    assert dict(actions[5].launch_arguments)["skip_pixel"] == "2"
    assert dict(actions[5].launch_arguments)["resolution"] == "0.2"

    assert actions[2]._Node__node_executable == "shared_stereo_depth"
    assert source["publish_mjpeg"] is False
    ctx.launch_configurations["shared_decode"] = "false"
    split_actions = module.nodes(ctx)
    split_source = evaluate_parameters(ctx, split_actions[2]._Node__parameters)[-1]
    assert split_source["publish_mjpeg"] is True
    assert split_actions[3]._Node__node_executable == "depth_node"
