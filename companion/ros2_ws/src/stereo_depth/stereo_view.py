"""装机查看方向；算法输入和光学坐标不在此变换。"""
import cv2
import numpy as np


def rotate_tiles(image, tiles=1, rotation_deg=180):
    """逐块旋转，保持 A/B 或图像/深度的槽位顺序。"""
    if rotation_deg not in (0, 180):
        raise ValueError("preview rotation must be 0 or 180")
    if tiles < 1 or image.shape[1] % tiles:
        raise ValueError("image width must be divisible by tiles")
    if rotation_deg == 0:
        return image.copy()
    return np.concatenate([cv2.rotate(part, cv2.ROTATE_180)
                           for part in np.split(image, tiles, axis=1)], axis=1)
