"""深度计算与发布前契约转换：复用 stereo_depth 的 StereoProcessor。

设计约束（来自任务契约）：
- 算法只有一份：几何与匹配全部调用 depth_preview.StereoProcessor，本模块不做第二套实现。
- 公共深度契约保留 NaN 无效值；写入 ROS 消息时转成 0.0（无回波），并把无效掩码显式交给下游。
- 主 XYZ 输出保持 H×W 像素对应关系，无效点三分量为 NaN、is_dense=false；
  紧凑点云只是附加输出，不能替代主输出。
- 相机内参必须来自同一标定的重投影矩阵（P1），不能另算一套，否则地图与深度不自洽。
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from depth_preview import DEPTH_SIZE, Config, StereoProcessor

from .frames import compose, make_transform


@dataclass(frozen=True)
class DepthResult:
    """一帧深度结果；数组发布后不再修改。"""

    rectified_a: np.ndarray
    rectified_b: np.ndarray
    disparity: np.ndarray
    xyz: np.ndarray          # (H, W, 3) float32，米，无效像素三通道为 NaN
    valid: np.ndarray        # (H, W) bool
    timings: dict

    @property
    def depth(self) -> np.ndarray:
        return self.xyz[:, :, 2]


def make_processor(calibration_path: str) -> StereoProcessor:
    """构造 StereoProcessor；缺标定文件必须显式失败，不允许静默使用占位值。"""
    calib = Path(calibration_path)
    if not calib.is_file():
        raise FileNotFoundError(f"标定文件不存在：{calib}")
    return StereoProcessor(Config(calibration=calib))


def process_stitched(processor: StereoProcessor, stitched_bgr: np.ndarray) -> DepthResult:
    """对已解码的左右拼接 BGR 图做校正 + 双向 SGBM + 重投影。

    返回的主 XYZ 保持 DEPTH_SIZE 的 H×W 结构，无效像素为 NaN。
    """
    a, b = processor.rectify_image(stitched_bgr)
    gray_a = cv2_cvt_gray(a)
    gray_b = cv2_cvt_gray(b)
    t0 = time.monotonic()
    disparity_a = processor.matcher_a.compute(gray_a, gray_b).astype(np.float32) / 16
    disparity_b = processor.matcher_b.compute(gray_b, gray_a).astype(np.float32) / 16
    t1 = time.monotonic()
    xyz, valid = processor.reconstruct(disparity_a, disparity_b)
    t2 = time.monotonic()
    timings = {
        "match_ms": (t1 - t0) * 1e3,
        "post_ms": (t2 - t1) * 1e3,
        "compute_ms": (t2 - t0) * 1e3,
    }
    return DepthResult(a, b, disparity_a, xyz, valid, timings)


def cv2_cvt_gray(bgr: np.ndarray) -> np.ndarray:
    import cv2

    return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)


def annotate_validity(depth_m: np.ndarray, valid: np.ndarray, max_depth_m: float, min_depth_m: float):
    """计算最终的「有效观测」掩码与发布用深度。

    - match_valid: 上游一致性检查给出的匹配有效掩码
    - over_range : 匹配有效但 z > max_depth_m（经过验证的超量程观测）
    - invalid    : 非有限 / z <= min_depth_m / 超量程 → 发布为 0.0（无回波）
    本轮不从匹配失败推断自由空间：invalid 与 over_range 在发布值上同为 0.0，
    但掩码分开统计，便于诊断与后续策略调整。
    """
    d = np.asarray(depth_m, dtype=np.float32)
    match_valid = np.asarray(valid, dtype=bool)
    finite = np.isfinite(d)
    too_near = ~finite | (d <= float(min_depth_m))
    over_range = match_valid & finite & (d > float(max_depth_m))
    invalid = too_near | over_range | ~match_valid
    out = d.copy()
    out[invalid] = 0.0
    return out, invalid, over_range


def xyz_to_structured(xyz: np.ndarray, valid: np.ndarray):
    """H×W 的 XYZ → 结构化点云字段（保持像素对应关系，无效点保留 NaN）。

    返回 (points, is_dense)，points 按行优先展开为 (H*W,)，is_dense 恒为 False。
    """
    xyz = np.asarray(xyz, dtype=np.float32)
    valid = np.asarray(valid, dtype=bool)
    if xyz.shape[:2] != valid.shape:
        raise ValueError("valid 掩码形状必须与 XYZ 前两维一致")
    h, w = valid.shape
    flat = xyz.reshape(h * w, 3)
    pts = np.zeros(h * w, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4")])
    pts["x"] = flat[:, 0]
    pts["y"] = flat[:, 1]
    pts["z"] = flat[:, 2]
    bad = ~valid.reshape(h * w)
    pts["x"][bad] = np.nan
    pts["y"][bad] = np.nan
    pts["z"][bad] = np.nan
    return pts, False


def xyz_to_compact(xyz: np.ndarray, valid: np.ndarray):
    """仅有效点的紧凑点云（附加输出，不替代主点云）。"""
    xyz = np.asarray(xyz, dtype=np.float32)
    valid = np.asarray(valid, dtype=bool)
    flat = xyz.reshape(-1, 3)[valid.reshape(-1)]
    pts = np.zeros(flat.shape[0], dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4")])
    if flat.shape[0]:
        pts["x"], pts["y"], pts["z"] = flat[:, 0], flat[:, 1], flat[:, 2]
    return pts, True


def make_rect_transform(processor: StereoProcessor) -> np.ndarray:
    """T_C0_Crect：原图相机系 ← 校正相机系；旋转为 R1 的**转置**。

    推导（2026-09-22 用 OpenCV 的校正查找表做了严格往返实验）：

    1. cv2.initUndistortRectifyMap(K1, D1, R1, P1) 生成的 map_a 把**校正像素**映射回
       **原图像素**，因此 map_a 就是 OpenCV 定义的「校正 → 原图」真值。
    2. 取校正像素 → map_a 得原图像素 → 用 undistortPoints 得到原图归一化射线 →
       分别按两种假设投回校正像素：
         rect = R1 · raw    → 误差 3e-6 px（3 个采样点，含畸变的真实标定）
         rect = R1ᵀ · raw   → 误差 7~9 px
       零畸变合成标定（非单位 R1）上同样：R1 误差 5e-6 px，R1ᵀ 误差 13 px。
       结论：**rect = R1 · raw**，即 R_Crect_C0 = R1。
    3. 因此从校正系变到原图系（本函数语义）的旋转是 R_C0_Crect = R1ᵀ。
       同一关系的另一种写法：raw = R1ᵀ · rect。

    早期版本错误地写成 R1（把「校正→原图」当成 R1），在 R1 偏离单位阵时会让地图方向
    偏掉约 2×1.13°（真机标定），或用合成大旋转时偏差十几像素；已修正。
    回归测试 test_rect_chain.py 用 OpenCV 的 map_a 往返与解析值双重断言：
    任何一侧转置错误都会失败。
    """
    return make_transform(processor.r1.T, np.zeros(3))


def camera_rect_transform(processor: StereoProcessor, T_w_i: np.ndarray, T_i_c0: np.ndarray) -> np.ndarray:
    """T_W_Crect = T_W_I · T_I_C0 · T_C0_Crect（T_A_B 表示把 B 系变到 A 系）。"""
    return compose(T_w_i, T_i_c0, make_rect_transform(processor))


def rectified_camera_info(processor: StereoProcessor, frame_id: str = "cam0_rect") -> dict:
    """由标定的 P1 推导 320×240 深度图的内参与投影矩阵。

    StereoProcessor 用「每目标定分辨率 → DEPTH_SIZE」缩放后的 K 调用 stereoRectify，
    因此 P1 的 fx/fy/cx/cy 已经是 DEPTH_SIZE 尺度下、alpha=0 的取值，
    可直接给 EGO 的 grid_map/{fx,fy,cx,cy} 使用。基线沿用同一标定的 |T|（不随分辨率缩放）。
    """
    return {
        "width": int(DEPTH_SIZE[0]),
        "height": int(DEPTH_SIZE[1]),
        "fx": float(processor.p1[0, 0]),
        "fy": float(processor.p1[1, 1]),
        "cx": float(processor.p1[0, 2]),
        "cy": float(processor.p1[1, 2]),
        "baseline_m": float(processor.baseline_m),
        "frame_id": frame_id,
    }
