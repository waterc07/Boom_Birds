"""ctypes ABI；数组类型与尺寸在进入 C++ 前检查。"""

import ctypes as ct
from pathlib import Path
import numpy as np


class NativeKernels:
    def __init__(self, library=""):
        if not library:
            from ament_index_python.packages import get_package_prefix

            library = (
                Path(get_package_prefix("stereo_depth"))
                / "lib/stereo_depth/libstereo_depth_kernels.so"
            )
        self.path = str(Path(library).resolve(strict=True))
        self.lib = ct.CDLL(self.path)
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
        self.has_neon = bool(self.lib.bb_has_neon())

    def reconstruct(self, a, b, q, lut, n, xyz):
        if a.dtype != np.int16 or b.dtype != np.int16 or a.shape != b.shape or a.ndim != 2:
            raise ValueError("native reconstruction requires equal int16 Q4 arrays")
        a, b = np.ascontiguousarray(a), np.ascontiguousarray(b)
        q = np.ascontiguousarray(q, dtype=np.float64)
        lut = np.ascontiguousarray(lut, dtype=np.float32)
        if q.shape != (4, 4) or lut.size < n * 16:
            raise ValueError("invalid Q/LUT")
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
        if a.ndim != 2 or a.shape != b.shape or a.dtype != np.uint8 or b.dtype != np.uint8:
            raise ValueError("Census requires equal mono8 images")
        a, b = np.ascontiguousarray(a), np.ascontiguousarray(b)
        lo = hi = None
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
