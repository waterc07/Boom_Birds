# ******************************************************************************
# @file           : native.py
# @brief          : 校验数组类型与尺寸后，通过 ctypes 调用 C++ 双目深度内核。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

"""ctypes ABI；数组类型与尺寸在进入 C++ 前检查。"""

import ctypes as ct
from pathlib import Path
import numpy as np


class NativeKernels:
    """加载 C++ 共享库，并将 NumPy 数组传给对应的计算内核。"""

    def __init__(self, library=""):
        # 未指定路径时，从已安装的 ROS 2 包中查找默认共享库。
        if not library:
            from ament_index_python.packages import get_package_prefix

            library = (
                Path(get_package_prefix("stereo_depth"))
                / "lib/stereo_depth/libstereo_depth_kernels.so"
            )
        self.path = str(Path(library).resolve(strict=True))
        self.lib = ct.CDLL(self.path)
        # 声明 C 函数的参数和返回类型，避免 ctypes 按默认类型解释指针。
        ptr = ct.c_void_p
        self.lib.bb_has_neon.restype = ct.c_int
        self.lib.bb_reconstruct.argtypes = [
            ptr,
            ptr,
            ct.c_int,
            ct.c_int,
            ct.c_int,
            ptr,
            ptr,
            ptr,
            ptr,
            ct.c_int,
        ]
        self.lib.bb_reconstruct.restype = ct.c_int
        self.lib.bb_census.argtypes = [ptr, ptr, ct.c_int, ct.c_int, ct.c_int, ptr, ptr, ptr]
        self.lib.bb_census.restype = ct.c_int
        self.hasNeon = bool(self.lib.bb_has_neon())

    def reconstruct(self, a, b, q, lut, n, xyz):
        """根据左右 Q4 视差图重建深度或 XYZ，返回结果和有效像素掩码。

        q 为 4×4 重投影矩阵，lut 将 Q4 视差映射为深度，n 为视差档数。
        xyz 为真时返回三通道坐标，否则只返回深度。
        """
        # C++ 内核按连续的二维 int16 数组读取左右视差。
        if a.dtype != np.int16 or b.dtype != np.int16 or a.shape != b.shape or a.ndim != 2:
            raise ValueError("native reconstruction requires equal int16 Q4 arrays")
        a, b = np.ascontiguousarray(a), np.ascontiguousarray(b)
        q = np.ascontiguousarray(q, dtype=np.float64)
        lut = np.ascontiguousarray(lut, dtype=np.float32)
        if q.shape != (4, 4) or lut.size < n * 16:
            raise ValueError("invalid Q/LUT")
        # 分配输出缓冲区，由 C++ 直接写入；掩码先用 uint8 表示。
        h, w = a.shape
        out = np.empty((h, w, 3) if xyz else (h, w), np.float32)
        mask = np.empty((h, w), np.uint8)
        rc = self.lib.bb_reconstruct(
            a.ctypes.data,
            b.ctypes.data,
            w,
            h,
            n,
            q.ctypes.data,
            lut.ctypes.data,
            out.ctypes.data,
            mask.ctypes.data,
            int(xyz),
        )
        if rc:
            raise RuntimeError(f"native reconstruction failed: {rc}")
        return out, mask.astype(bool)

    def census(self, a, b, n, bounds=None):
        """对左右 8 位灰度图做 Census 匹配，返回 Q4 格式的视差图。

        bounds 可选，包含每个像素的最小和最大候选视差数组。
        """
        if a.ndim != 2 or a.shape != b.shape or a.dtype != np.uint8 or b.dtype != np.uint8:
            raise ValueError("Census requires equal mono8 images")
        a, b = np.ascontiguousarray(a), np.ascontiguousarray(b)
        lo = hi = None
        # 没有搜索范围时传空指针，内核会搜索完整的视差范围。
        if bounds is not None:
            lo, hi = [np.ascontiguousarray(v, dtype=np.int16) for v in bounds]
            if lo.shape != a.shape or hi.shape != a.shape or np.any(lo > hi):
                raise ValueError("invalid Census bounds")
        out = np.empty(a.shape, np.int16)
        rc = self.lib.bb_census(
            a.ctypes.data,
            b.ctypes.data,
            a.shape[1],
            a.shape[0],
            n,
            None if lo is None else lo.ctypes.data,
            None if hi is None else hi.ctypes.data,
            out.ctypes.data,
        )
        if rc:
            raise RuntimeError(f"Census failed: {rc}")
        return out


    def unpackRaw10(self,payload,width,height,lut):
        """解包 RAW10 图像，并用 1024 项查找表转换为 8 位图像。"""
        # 校验图像尺寸及数据长度，并由总长度推算每行字节数（含填充）。
        if (type(width) is not int or type(height) is not int or not 4<=width<=16384
                or width%4 or not 2<=height<=16384 or len(payload)%height):
            raise ValueError("raw10_native_layout")
        stride=len(payload)//height
        if not width*5//4<=stride<=width*5//4+256:raise ValueError("raw10_native_stride")
        # 每个 10 位像素值都需要一个 uint8 查找表条目。
        table=np.asarray(lut)
        if table.shape!=(1024,) or table.dtype!=np.uint8:
            raise ValueError("raw10_native_lut")
        table=np.ascontiguousarray(table)
        raw=np.frombuffer(payload,np.uint8)
        output=np.empty((height,width),np.uint8)
        # 声明 RAW10 内核的 ABI，然后把输入与输出缓冲区的地址传给 C++。
        function=self.lib.bb_unpack_raw10
        function.argtypes=[ct.c_void_p,ct.c_size_t,ct.c_int,ct.c_int,ct.c_int,ct.c_void_p,ct.c_void_p]
        function.restype=ct.c_int
        result=function(raw.ctypes.data,raw.size,width,height,stride,table.ctypes.data,output.ctypes.data)
        if result:raise RuntimeError("raw10_native_failed:"+str(result))
        return output
