"""原始双目发布尺寸；不改变采集模式或物理基线。"""
import math

import cv2


def scaled_raw_size(width, height, scale):
    scale = float(scale)
    if not math.isfinite(scale) or not 0 < scale <= 1:
        raise ValueError("raw_output_scale must be finite and in (0, 1]")
    if width <= 0 or height <= 0:
        raise ValueError("raw image dimensions must be positive")
    size = (round(width * scale), round(height * scale))
    if min(size) < 1 or not math.isclose(size[0] / width, size[1] / height, abs_tol=1e-9):
        raise ValueError("raw_output_scale must preserve aspect ratio at integer dimensions")
    return size


def resize_raw_pair(left, right, scale):
    if left.shape != right.shape or left.ndim != 2:
        raise ValueError("raw stereo pair must have matching mono8 dimensions")
    size = scaled_raw_size(left.shape[1], left.shape[0], scale)
    if size == (left.shape[1], left.shape[0]):
        return left, right
    return (cv2.resize(left, size, interpolation=cv2.INTER_AREA),
            cv2.resize(right, size, interpolation=cv2.INTER_AREA))
