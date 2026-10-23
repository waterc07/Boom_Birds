from pathlib import Path
import importlib.util
import cv2
import numpy as np
import pytest
import yaml
from launch import LaunchContext
from launch_ros.utilities import evaluate_parameters

from boom_birds_bringup.compute_profile import prepareMonoVio, rawGeometry, openvinsParameters
from boom_birds_bringup.hardware_profile import validateProfile, readYaml
from test_hardware_profile import bundle


def monoBundle(tmp_path):
    args = bundle(tmp_path)
    original = readYaml(args["vio_config_file"])
    original["init_dyn_min_rec_cond"] = "1e-12"
    Path(args["vio_config_file"]).write_text(yaml.safe_dump(original))
    config = prepareMonoVio(args["vio_config_file"], tmp_path / "mono", 320, 240)
    args["vio_config_file"] = str(config)
    return args


def test_mono_removes_cam1_preserves_metric_extrinsics_and_scales_rays(tmp_path):
    args = monoBundle(tmp_path)
    config = Path(args["vio_config_file"])
    chain = readYaml(config.parent / "kalibr_imucam_chain.yaml")
    assert list(chain) == ["cam0"]
    fs = cv2.FileStorage(str(config.parent / "kalibr_imucam_chain.yaml"), cv2.FILE_STORAGE_READ)
    assert fs.isOpened()
    assert fs.getNode("cam0").getNode("T_imu_cam").size() == 4
    fs.release()
    assert chain["cam0"]["resolution"] == [320, 240]
    assert chain["cam0"]["intrinsics"] == [200., 200., 160., 120.]
    assert chain["cam0"]["T_imu_cam"] == np.eye(4).tolist()
    assert readYaml(config)["init_dyn_min_rec_cond"] == 1e-12
    assert validateProfile(**args)["hardware_verified"] is False
    with pytest.raises(ValueError, match="只允许 dry-run"):
        validateProfile(**args, live=True)
    assert rawGeometry(config, 1280, 480) == {"raw_output_scale": .5, "raw_decode_divisor": 2}
    params = openvinsParameters(config)
    assert params["max_cameras"] == 1 and params["use_stereo"] is False
    assert params["max_slam"] == 0 and params["num_pts"] == 100
    with pytest.raises(FileExistsError):
        prepareMonoVio(tmp_path / "estimator.yaml", config.parent, 320, 240)


def test_mono_rejects_unscaled_intrinsics_and_nonuniform_image_scaling(tmp_path):
    args = monoBundle(tmp_path)
    config = Path(args["vio_config_file"])
    with pytest.raises(ValueError, match="等比例"):
        rawGeometry(config, 1280, 960)
    chainPath = config.parent / "kalibr_imucam_chain.yaml"
    chain = readYaml(chainPath)
    chain["cam0"]["intrinsics"][0] *= 2
    chainPath.write_text(yaml.safe_dump(chain))
    with pytest.raises(ValueError, match="内参不同"):
        validateProfile(**args)


def test_synthetic_marker_survives_derivation(tmp_path):
    args = bundle(tmp_path)
    source = Path(args["vio_config_file"])
    source.write_text("# TEST-ONLY\n" + source.read_text())
    args["vio_config_file"] = str(prepareMonoVio(source, tmp_path / "derived"))
    with pytest.raises(ValueError, match="合成配置"):
        validateProfile(**args)


def test_mono_launch_passes_one_camera_and_budget_to_real_node(tmp_path, monkeypatch):
    path = Path(__file__).resolve().parents[1] / "launch/attitude_hardware.launch.py"
    spec = importlib.util.spec_from_file_location("mono_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    share = module.get_package_share_directory
    monkeypatch.setattr(module, "get_package_share_directory", lambda name:
        str(tmp_path / name) if name == "ego_planner" else share(name))
    ctx = LaunchContext()
    ctx.launch_configurations.update(monoBundle(tmp_path))
    ctx.launch_configurations.update(camera_device="/dev/test", capture_width="1280", capture_height="480",
        capture_fps="60", output_scale="1.0", compute_profile="mono_budget", shared_decode="auto", dry_run="true",
        allow_arming="false", allow_non_loopback="false", allow_hardware_actions="false", hardware_validation_note="")
    actions = module.nodes(ctx)
    # 采集/深度合并后仍向估计器传入单目覆盖。
    params = evaluate_parameters(ctx, actions[3]._Node__parameters)[0]
    assert params["max_cameras"] == 1 and params["use_stereo"] is False
    assert params["num_opencv_threads"] == 1 and params["max_clones"] == 6
    sourceParameters = evaluate_parameters(ctx, actions[2]._Node__parameters)
    assert sourceParameters[0].name == "stereo_camera.yaml"
    source = sourceParameters[-1]
    assert source["raw_output_scale"] == .5
    assert source["raw_decode_divisor"] == 2
    assert dict(actions[5].launch_arguments)["skip_pixel"] == "2"
    assert dict(actions[5].launch_arguments)["resolution"] == "0.2"
    assert dict(actions[5].launch_arguments)["map_min_z"] == "-0.1"
    ctx.launch_configurations["map_min_z"] = "-0.5"
    shifted = module.nodes(ctx)
    assert dict(shifted[5].launch_arguments)["map_min_z"] == "-0.5"
    assert dict(shifted[5].launch_arguments)["map_size_z"] == "6.0"

    assert actions[2]._Node__node_executable == "shared_stereo_depth"
    assert source["publish_mjpeg"] is False
    ctx.launch_configurations["shared_decode"] = "false"
    splitActions = module.nodes(ctx)
    splitSource = evaluate_parameters(ctx, splitActions[2]._Node__parameters)[-1]
    assert splitSource["publish_mjpeg"] is True
    assert splitActions[3]._Node__node_executable == "depth_node"


@pytest.mark.parametrize("bounds", [
    {"map_min_z": "nan"}, {"map_size_z": "0"},
    {"virtual_ceil_z": "inf"}, {"virtual_ceil_z": "7"},
])
def test_invalid_map_bounds_fail_before_hardware_profile_or_node_creation(bounds):
    path = Path(__file__).resolve().parents[1]/"launch/attitude_hardware.launch.py"
    spec = importlib.util.spec_from_file_location("invalid_bounds_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ctx = LaunchContext()
    ctx.launch_configurations.update(bounds)
    with pytest.raises(ValueError, match="map_min_z"):
        module.nodes(ctx)


def test_upright_profile_and_mono_derivation_preserve_pixel_offset(tmp_path):
    from stereo_depth.orientation import convertCalibration, convertCameraChain
    args=bundle(tmp_path)
    npz=Path(args["calibration_file"])
    with np.load(npz) as data:
        converted=convertCalibration(data)
    np.savez(npz,**converted)
    cameraPath=tmp_path/"camera.yaml"
    chain=convertCameraChain(readYaml(cameraPath))
    cameraPath.write_text(yaml.safe_dump(chain))
    extrPath=Path(args["extrinsics_file"])
    extr=readYaml(extrPath)
    extr["T_I_C0"]=chain["cam0"]["T_imu_cam"]
    extrPath.write_text(yaml.safe_dump(extr))
    assert validateProfile(**args)["hardware_verified"] is False
    derived=prepareMonoVio(args["vio_config_file"],tmp_path/"upright-mono",320,240)
    args["vio_config_file"]=str(derived)
    assert validateProfile(**args)["hardware_verified"] is False
    cam=readYaml(derived.parent/"kalibr_imucam_chain.yaml")["cam0"]
    assert cam["intrinsics"]==[200.,200.,159.,119.]
    cam.pop("image_orientation")
    (derived.parent/"kalibr_imucam_chain.yaml").write_text(yaml.safe_dump({"cam0":cam}))
    with pytest.raises(ValueError,match="图像方向"):
        validateProfile(**args)
