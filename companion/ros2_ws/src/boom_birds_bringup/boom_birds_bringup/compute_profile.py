"""固定硬件的单目 VIO 算力配置；标定从已有文件派生。"""
from copy import deepcopy
from pathlib import Path
import hashlib
import json
import math
import re
import shutil
import yaml

from boom_birds_bringup.hardware_profile import read_yaml

VIO_BUDGET = {
    "max_cameras": 1, "use_stereo": False, "use_klt": True,
    "num_pts": 100, "max_clones": 6, "max_slam": 0,
    "max_msckf_in_update": 20, "num_opencv_threads": 1,
    # 图像源限 20 Hz；留时间戳抖动余量，避免二次限频落到约 10 Hz。
    "track_frequency": 30.0, "downsample_cameras": False,
    "use_aruco": False, "save_total_state": False,
    "record_timing_information": False,
}


class _OpenCVDumper(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):
        return super().increase_indent(flow, False)


def opencv_yaml(value):
    # OpenCV FileStorage 不接受 PyYAML 默认的 indentless sequence。
    return yaml.dump(value, Dumper=_OpenCVDumper, sort_keys=False, default_flow_style=None)


def raw_geometry(vio_config_file, capture_width, capture_height):
    """由 cam0 尺寸决定原图缩放；不改变镜头、采集模式或物理基线。"""
    config = read_yaml(vio_config_file)
    chain = read_yaml(Path(vio_config_file).parent / config["relative_config_imucam"])
    width, height = chain["cam0"]["resolution"]
    if capture_width <= 0 or capture_width % 2 or capture_height <= 0:
        raise ValueError("采集拼接尺寸必须为正，宽度必须为偶数")
    scale = float(width) / (capture_width // 2)
    if (not math.isfinite(scale) or not 0 < scale <= 1
            or width != int(width) or height != int(height)
            or not math.isclose(float(height) / capture_height, scale, abs_tol=1e-9)):
        raise ValueError("VIO 尺寸必须是采集每目尺寸的等比例缩小")
    divisor = next(d for d in (4, 2, 1) if scale * d <= 1 and capture_width % (2*d) == 0 and capture_height % d == 0)
    return {"raw_output_scale": scale, "raw_decode_divisor": divisor}


def openvins_parameters(vio_config_file, budget=True):
    config = read_yaml(vio_config_file)
    if budget and (config.get("max_cameras") != 1 or config.get("use_stereo") is not False):
        raise ValueError("mono_budget 需要单目配置；先运行 prepare_mono_vio.py")
    # 仅覆盖布尔值和已知类型参数；其余数值由 OpenVINS 按目标类型读取。
    params = {k: v for k, v in config.items() if isinstance(v, bool)}
    params["max_cameras"] = int(config["max_cameras"])
    if budget:
        params.update(VIO_BUDGET)
    params["config_path"] = str(Path(vio_config_file).resolve())
    params["verbosity"] = "WARNING"
    return params


def prepare_mono_vio(source, output, width=640, height=480):
    """生成独立目录，保留 cam0/IMU 标定和来源，不覆盖已有文件。"""
    source, output = Path(source).resolve(), Path(output).resolve()
    config = read_yaml(source)
    # PyYAML 把无小数点的科学计数法当字符串，OpenCV 则把它读作数字。
    for key, value in config.items():
        if isinstance(value, str) and re.fullmatch(r"[-+]?\d+(?:\.\d*)?[eE][-+]?\d+", value):
            config[key] = float(value)
    camera_path = source.parent / config["relative_config_imucam"]
    imu_path = source.parent / config["relative_config_imu"]
    chain = read_yaml(camera_path)
    cam = deepcopy(chain["cam0"])
    old_w, old_h = cam["resolution"]
    sx, sy = width / old_w, height / old_h
    if width < 1 or height < 1 or not 0 < sx <= 1 or not math.isclose(sx, sy, abs_tol=1e-9):
        raise ValueError("单目尺寸必须是原 cam0 的等比例缩小")
    cam["intrinsics"] = [float(x) * sx for x in cam["intrinsics"]]
    cam["resolution"] = [width, height]
    cam["cam_overlaps"] = []
    config.update(VIO_BUDGET)
    config["relative_config_imucam"] = "kalibr_imucam_chain.yaml"
    config["relative_config_imu"] = "kalibr_imu_chain.yaml"
    inputs = (source, camera_path, imu_path)
    provenance = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    # 合成输入只用于软件测试；派生配置不能丢掉原有 TEST-ONLY 标记。
    test_only = any("TEST-ONLY" in p.read_text() or "SYNTHETIC" in p.read_text() or "synthetic" in str(p) for p in inputs)
    config["boombirds_test_only"] = test_only
    header = "%YAML:1.0\n# " + ("TEST-ONLY synthetic input" if test_only else "Derived calibration; accuracy not revalidated") + "\n"
    output.mkdir(parents=True, exist_ok=False)
    for name, value in (("estimator_config.yaml", config), ("kalibr_imucam_chain.yaml", {"cam0": cam})):
        (output / name).write_text(header + opencv_yaml(value), encoding="utf-8")
    shutil.copyfile(imu_path, output / "kalibr_imu_chain.yaml")
    (output / "source_manifest.json").write_text(json.dumps({"inputs": provenance, "quality": "NOT RUN", "budget": "NOT RUN"}, indent=2) + "\n")
    return output / "estimator_config.yaml"
