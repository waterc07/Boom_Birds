"""匹配分辨率、标定缩放和输出尺度必须对应，默认路径保持320x240。"""

from dataclasses import replace
import numpy as np
import pytest
from stereo_depth.core import Config, StereoProcessor
from boom_birds_sensing.depth_core import rectified_camera_info
from boom_birds_sensing.camera_geometry import scaled_camera_info


def test_full_resolution_metric_depth_and_publish_intrinsics(tmp_path):
    rng = np.random.default_rng(12)
    left = rng.integers(0, 256, (480, 640), dtype=np.uint8)
    right = np.zeros_like(left)
    right[:, :-12] = left[:, 12:]
    path = tmp_path / "stereo.npz"
    k = np.array([[420.0, 0.0, 319.5], [0.0, 420.0, 239.5], [0.0, 0.0, 1.0]])
    np.savez(
        path,
        image_size=[640, 480],
        K1=k,
        K2=k,
        D1=np.zeros(5),
        D2=np.zeros(5),
        R=np.eye(3),
        T=[-0.0676718, 0.0, 0.0],
    )
    base = Config(calibration=path, num_disparities=64, threads=1)
    with pytest.raises(ValueError, match="experimental"):
        StereoProcessor(replace(base, matching_size=(640, 480)))
    for size in ((320, 240), (640, 480)):
        processor = StereoProcessor(
            replace(
                base,
                matching_size=size,
                experimental=size != (320, 240),
                sgbm_block_size=3 if size == (640, 480) else 5,
            )
        )
        _, _, _, depth, valid, _ = processor.process_pair(left, right, with_xyz=False)
        assert depth.shape == size[::-1]
        roi = depth[size[1] // 4 : size[1] * 3 // 4, size[0] // 3 : size[0] * 3 // 4]
        assert np.isfinite(roi).mean() > 0.95
        assert abs(np.nanmedian(roi) - 420 * 0.0676718 / 12) < 0.02
        assert np.isnan(depth[~valid]).all()
        info = rectified_camera_info(processor)
        info = scaled_camera_info(info, 320 / size[0])
        assert (info["width"], info["height"]) == (320, 240)
        assert abs(info["fx"] - 210) < 1e-6
