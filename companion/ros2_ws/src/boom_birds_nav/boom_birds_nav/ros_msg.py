"""ROS 消息构造小工具（纯函数为主，便于离线测试）。

PointCloud2 直接按结构化 dtype 打包，避免逐点 Python 循环；
对 NaN 不做任何替换，主点云必须保留无效点的 NaN 与 H×W 对应关系。
"""

from __future__ import annotations

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def cloud2_from_structured(points, header: Header, is_dense: bool) -> PointCloud2:
    """由结构化 (x,y,z) float32 数组构造 PointCloud2。"""
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = int(points.shape[0])
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.is_dense = bool(is_dense)
    msg.data = points.tobytes()
    return msg


def cloud2_xyz_hw(xyz: np.ndarray, valid: np.ndarray, header: Header) -> PointCloud2:
    """按 H×W 展开的 PointCloud2（height=H, width=W, is_dense=False）。

    逐行展开保持像素对应关系：index = v * W + u。
    无效点三分量为 NaN。
    """
    xyz = np.asarray(xyz, dtype=np.float32)
    valid = np.asarray(valid, dtype=bool)
    h, w = valid.shape
    data = xyz.reshape(h * w, 3).copy()
    bad = ~valid.reshape(h * w)
    data[bad] = np.nan
    from .depth_core import xyz_to_structured

    pts, is_dense = xyz_to_structured(data.reshape(h, w, 3), valid)
    msg = cloud2_from_structured(pts, header, is_dense)
    msg.height = int(h)
    msg.width = int(w)
    msg.row_step = 12 * int(w)
    return msg
