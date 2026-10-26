"""双目几何与匹配；导入不打开设备。"""

from dataclasses import dataclass
from pathlib import Path
import time
from .orientation import SENSOR, imageOrientation, cameraMatrix, orientStitched

import cv2
import numpy as np


def _resolveRoot():
    """源码内查找默认标定；安装后使用 ament share，与输出目录无关。"""
    here = Path(__file__).resolve().parent.parent
    if (here / "calibration").is_dir():
        return here
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("stereo_depth"))
    except Exception:
        return here


ROOT = _resolveRoot()
DEPTH_SIZE = (320, 240)  # 每目计算分辨率，不能仅改此值而忽略标定尺度。
PREVIEW_RANGE_M = (0.15, 3.0) #仅对预览图生效，并非算法的有效深度范围


@dataclass(frozen=True)
class Config:
    """集中定义运行参数；匹配尺度与标定同步缩放。"""

    device: str = "/dev/video0"  # V4L2 设备节点；编号随接入顺序变化
    port: int = 8081  # 预览HTTP 端口
    threads: int = 4  # OpenCV 线程数（cv2.setNumThreads），
    reduced_decode: bool = True  # 半分辨率 JPEG 解码；关闭即完整解码后再缩小

    calibration: Path = ROOT / "calibration/upright_20261022/candidate.npz"
    output_dir: Path = Path.home() / "boombirds-data/stereo_depth/depth_outputs"

    # 量程：两者联动，最终档数见 selectNumDisparities()。
    num_disparities: int = 96  # 视差搜索档数，0：按 required_min_depth_m 选择最小的 16 倍数
    required_min_depth_m: float = 0.0  # 最近有效距离（米）；0 表示不约束

    # 性能档
    fixed_maps: bool = False  # convertMaps 生成 CV_16SC2/16UC1 映射表
    reuse_buffers: bool = False  # 开启后返回图会在下次处理时被覆盖
    postprocess: str = "numpy"  # numpy / lut / native
    matcher: str = "sgbm"  # sgbm / census / coarse_census / elas

    # 实验总闸：关闭时拒绝任何改变默认行为的参数组合。
    experimental: bool = False  # 非 sgbm 匹配器、640x480、窗口 3/7 都要求为 true
    native_library: str = ""  # 空：从已安装包加载 libstereo_depth_kernels.so
    elas_library: str = ""  # ELAS 包装库绝对路径；不自动下载、构建或回退
    rknn_model: str = ""  # 必须与 rknn_manifest 同时提供
    rknn_manifest: str = ""  # JSON 合约与模型 SHA256，加载前校验

    matching_size: tuple[int, int] = DEPTH_SIZE  # 默认 320x240；其他值需 experimental
    sgbm_block_size: int = 5  # 窗口 3/7 需 experimental；P1/P2 按窗口面积同步


@dataclass(frozen=True)
class CapturedFrame:
    sequence: int
    received_at: float  # 主机完成取帧的单调时钟时间，不是传感器曝光时间。
    packet: np.ndarray


@dataclass(frozen=True)
class DepthFrame:

    source: CapturedFrame
    rectified: np.ndarray
    disparity: np.ndarray
    xyz: np.ndarray
    valid: np.ndarray
    preview: np.ndarray
    timings: dict

    @property
    def depth(self):
        return self.xyz[:, :, 2]


class StereoProcessor:
    """只负责几何与图像运算，可用已保存的 JPEG 离线调用，不依赖相机和 HTTP。"""

    def __init__(self, config):
        self.config = config
        self.depth_size = tuple(config.matching_size)
        if self.depth_size not in (DEPTH_SIZE, (640, 480)):
            raise ValueError("matching_size must be 320x240 or 640x480")
        if self.depth_size != DEPTH_SIZE and not config.experimental:
            raise ValueError("full-resolution matching requires experimental=true")
        if config.postprocess not in ("numpy", "lut", "native"):
            raise ValueError("unknown postprocess")
        if config.matcher not in ("sgbm", "census", "coarse_census", "elas"):
            raise ValueError("unknown matcher")
        if (config.matcher != "sgbm" or config.rknn_model) and not config.experimental:
            raise ValueError("alternative matchers/RKNN require experimental=true")
        if bool(config.rknn_model) != bool(config.rknn_manifest):
            raise ValueError("RKNN model and manifest must be supplied together")
        if config.sgbm_block_size not in (3, 5, 7):
            raise ValueError("sgbm_block_size must be 3, 5 or 7")
        if config.sgbm_block_size != 5 and not config.experimental:
            raise ValueError("changed SGBM block size requires experimental=true")
        self._scratch = {}
        self._prior = None
        self._frameStamp = None
        self.refiner = None
        self.alternative = None
        self.native = None
        if not config.calibration.is_file():
            raise FileNotFoundError(
                f"标定文件不存在：{config.calibration}；请从树莓派取回标定目录、重新标定，"
                "或用 --calibration 指定已有的 candidate.npz"
            )
        with np.load(config.calibration) as archive:
            calibration = {key: archive[key].copy() for key in archive.files}
        self.imageOrientation = imageOrientation(calibration)
        self.calibration = calibration
        imageSize = tuple(int(v) for v in calibration["image_size"])
        if len(imageSize) != 2 or min(imageSize) < 240 or any(v % 2 for v in imageSize):
            raise ValueError("标定图像尺寸必须为有效偶数像素尺寸")
        self.capture_size = (imageSize[0] * 2, imageSize[1])
        self.baseline_m = float(np.linalg.norm(calibration["T"]))
        if not np.isfinite(self.baseline_m) or not 0.001 < self.baseline_m < 1:
            raise ValueError("标定基线必须以米为单位且处于合理范围")
        translation = calibration["T"].reshape(3)
        if translation[0] >= 0 or abs(translation[1]) > abs(translation[0]) * 0.2:
            raise ValueError("标定不符合 A→B 正视差水平双目，请检查左右顺序")

        # 按标定的原始每目尺寸缩放内参；采集也使用该模式，避免假定不同模式同视场。
        # T 的单位仍是米，不能随图像尺寸一起缩放。
        kA = cameraMatrix(calibration, 1, self.depth_size)
        kB = cameraMatrix(calibration, 2, self.depth_size)
        rA, rB, pA, pB, self.q, _, _ = cv2.stereoRectify(
            kA,
            calibration["D1"],
            kB,
            calibration["D2"],
            self.depth_size,
            calibration["R"],
            calibration["T"],
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=0,
        )
        # R1 将原图光学系旋转到校正光学系；位姿链 T_C0_Crect 使用 R1.T。
        # P1 为匹配分辨率、alpha=0 的投影内参；ROS CameraInfo 复用它。
        self.r1, self.r2 = rA, rB
        self.p1, self.p2 = pA, pB
        self.map_a = cv2.initUndistortRectifyMap(
            kA, calibration["D1"], rA, pA, self.depth_size, cv2.CV_32FC1
        )
        self.mapB = cv2.initUndistortRectifyMap(
            kB, calibration["D2"], rB, pB, self.depth_size, cv2.CV_32FC1
        )
        if config.fixed_maps:
            self.map_a = cv2.convertMaps(*self.map_a, cv2.CV_16SC2)
            self.mapB = cv2.convertMaps(*self.mapB, cv2.CV_16SC2)
        if not np.isfinite(self.q).all():
            raise ValueError("标定 Q 必须有限")
        self.num_disparities = selectNumDisparities(
            self.q, config.num_disparities, config.required_min_depth_m
        )
        self.nearestDepthM = float(
            self.q[2, 3] / (self.q[3, 2] * (self.num_disparities - 1) + self.q[3, 3])
        )
        # 水平 stereoRectify 的 Q；其他结构仍走完整重投影。
        self._zOnlySupported = bool(
            np.all(self.q[2, :3] == 0) and np.all(self.q[3, :2] == 0) and np.all(self.q[:2, 2] == 0)
        )
        if config.postprocess != "numpy" and not self._zOnlySupported:
            raise ValueError("LUT/native postprocess requires horizontal Q")
        d = np.arange(self.num_disparities * 16, dtype=np.float64) / 16
        with np.errstate(divide="ignore", invalid="ignore"):
            self._zLut = (self.q[2, 3] / (self.q[3, 2] * d + self.q[3, 3])).astype(np.float32)
        if config.postprocess == "native" or config.matcher in ("census", "coarse_census"):
            from .native import NativeKernels

            self.native = NativeKernels(config.native_library)
        if config.matcher in ("census", "coarse_census"):
            from .experiments import CensusMatcher

            self.alternative = CensusMatcher(
                self.native, self.num_disparities, coarse=config.matcher == "coarse_census"
            )
        elif config.matcher == "elas":
            from .experiments import ElasMatcher

            self.alternative = ElasMatcher(config.elas_library, self.num_disparities)
        if config.rknn_model:
            from .experiments import RknnResidualRefiner

            self.refiner = RknnResidualRefiner(
                config.rknn_model, config.rknn_manifest, self.num_disparities
            )
        parameters = dict(
            numDisparities=self.num_disparities,
            blockSize=config.sgbm_block_size,
            P1=8 * config.sgbm_block_size**2,
            P2=32 * config.sgbm_block_size**2,
            disp12MaxDiff=1,
            uniquenessRatio=12,
            speckleWindowSize=80,
            speckleRange=2,
            preFilterCap=31,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        self.matcherA = cv2.StereoSGBM_create(minDisparity=0, **parameters)
        self.matcherB = cv2.StereoSGBM_create(minDisparity=1 - self.num_disparities, **parameters)
        self.rows, self.cols = np.indices(self.depth_size[::-1], dtype=np.int32)

    def _buffer(self, name, shape, dtype):
        if not self.config.reuse_buffers:
            return None
        key = (name, tuple(shape), np.dtype(dtype).str)
        if key not in self._scratch:
            self._scratch[key] = np.empty(shape, dtype=dtype)
        return self._scratch[key]

    def _remap(self, image, maps, name):
        shape = (*self.depth_size[::-1], *image.shape[2:])
        return cv2.remap(image, *maps, cv2.INTER_LINEAR, dst=self._buffer(name, shape, image.dtype))

    def setVioPrior(self, prior, frame_stamp):
        if not self.config.experimental or self.config.matcher not in ("census", "coarse_census"):
            raise ValueError("VIO search prior requires experimental Census")
        self._prior, self._frameStamp = prior, float(frame_stamp)

    def close(self):
        if self.refiner is not None:
            self.refiner.close()
            self.refiner = None

    def rectify(self, packet):
        """半尺寸 JPEG 解码可省去完整解码；完整解码选项用于对照。"""
        mode = cv2.IMREAD_REDUCED_COLOR_2 if self.config.reduced_decode else cv2.IMREAD_COLOR
        image = cv2.imdecode(packet.reshape(-1), mode)
        divisor = 2 if self.config.reduced_decode else 1
        expected = (self.capture_size[1] // divisor, self.capture_size[0] // divisor)
        if image is None or image.shape[:2] != expected:
            raise RuntimeError("JPEG 尺寸不符合预期，请检查相机输出模式")
        image = orientStitched(image, SENSOR, self.imageOrientation)
        return self._rectifyResized(image)

    def rectifyImage(self, image):
        """对已解码、方向与标定一致的拼接图做校正；ROS 输入不得再次旋转。

        入参 image 是完整的左右拼接灰度或 BGR 数组（宽 = 2 × 每目宽）。
        与 rectify() 的唯一区别是本方法不经过 JPEG 解码，尺寸/缩放/校正路径完全一致，
        避免 ROS 侧为复用而重新编码 JPEG。
        """
        if (
            image is None
            or image.dtype != np.uint8
            or not (image.ndim == 2 or (image.ndim == 3 and image.shape[2] == 3))
        ):
            raise ValueError("rectify_image 需要灰度或 BGR 拼接图")
        return self._rectifyResized(image)

    def _rectifyResized(self, image):
        """把任意尺寸的拼接 BGR 图缩放到匹配分辨率并做极线校正，返回 (a, b)。"""
        if image.shape[:2] != (self.depth_size[1], self.depth_size[0] * 2):
            image = cv2.resize(
                image,
                (self.depth_size[0] * 2, self.depth_size[1]),
                interpolation=cv2.INTER_AREA,
                dst=self._buffer(
                    "stitched",
                    (self.depth_size[1], self.depth_size[0] * 2, *image.shape[2:]),
                    image.dtype,
                ),
            )
        a = self._remap(image[:, : self.depth_size[0]], self.map_a, "a")
        b = self._remap(image[:, self.depth_size[0] :], self.mapB, "b")
        return a, b

    def reconstruct(self, disparityA, disparityB, *, with_xyz=True):
        """执行左右一致性检查，输出校正 A 目光学系 XYZ 或 Z（米）。"""
        # A 目横坐标 u 对应 B 目 u-d；B→A 视差符号与 A→B 相反。
        targetCols = np.rint(self.cols - disparityA).astype(int)
        inside = (targetCols >= 0) & (targetCols < self.depth_size[0])
        sampledB = disparityB[self.rows, np.clip(targetCols, 0, self.depth_size[0] - 1)]
        if with_xyz or not self._zOnlySupported:
            points = cv2.reprojectImageTo3D(disparityA, self.q)
            finite = np.isfinite(points).all(axis=2)
            z = points[:, :, 2]
            output = points if with_xyz else z.copy()
        else:
            # 用同一 Q 的齐次除法直接求 Z，不分配 H×W×3 数组。
            denominator = self.q[3, 2] * disparityA.astype(np.float64) + self.q[3, 3]
            with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
                z = (self.q[2, 3] / denominator).astype(np.float32)
            finite = np.isfinite(z)
            output = z
        valid = (
            inside
            & (disparityA > 0)
            & (disparityA < self.num_disparities - 1)
            & (sampledB > -self.num_disparities)
            & (np.abs(disparityA + sampledB) <= 1)
            & finite
            & (z > 0)
        )
        output[~valid] = np.nan
        return output, valid

    def reconstructQ4(self, disparityA, disparityB, *, with_xyz=True):
        if disparityA.dtype != np.int16 or disparityB.dtype != np.int16:
            raise ValueError("Q4 disparity must be int16")
        if disparityA.shape != self.depth_size[::-1] or disparityB.shape != disparityA.shape:
            raise ValueError("Q4 disparity dimensions must match matching_size")
        if self.config.postprocess == "numpy":
            return self.reconstruct(
                disparityA.astype(np.float32) / 16,
                disparityB.astype(np.float32) / 16,
                with_xyz=with_xyz,
            )
        if self.config.postprocess == "native":
            return self.native.reconstruct(
                disparityA, disparityB, self.q, self._zLut, self.num_disparities, with_xyz
            )
        targetCols = np.rint(self.cols - disparityA.astype(np.float32) / 16).astype(np.int32)
        inside = (targetCols >= 0) & (targetCols < self.depth_size[0])
        sampledB = disparityB[self.rows, np.clip(targetCols, 0, self.depth_size[0] - 1)]
        z = self._zLut[np.clip(disparityA, 0, len(self._zLut) - 1)].copy()
        valid = (
            inside
            & (disparityA > 0)
            & (disparityA < 16 * (self.num_disparities - 1))
            & (sampledB > -16 * self.num_disparities)
            & (np.abs(disparityA.astype(np.int32) + sampledB) <= 16)
            & np.isfinite(z)
            & (z > 0)
        )
        if with_xyz:
            output = cv2.reprojectImageTo3D(disparityA.astype(np.float32) / 16, self.q)
            valid &= np.isfinite(output).all(axis=2)
            output[:, :, 2] = z
        else:
            output = z
        output[~valid] = np.nan
        return output, valid

    def processImage(self, image, *, with_xyz=True):
        """已解码左右拼接图 → 校正图、视差、XYZ、有效掩码、耗时。"""
        a, b = self.rectifyImage(image)
        return self.matchRectified(a, b, with_xyz=with_xyz)

    def processPair(self, left, right, *, with_xyz=True):
        if (
            left.ndim != 2
            or left.shape != right.shape
            or left.dtype != np.uint8
            or right.dtype != np.uint8
        ):
            raise ValueError("共享输入必须是同尺寸灰度双目")
        if left.shape != self.depth_size[::-1]:
            left = cv2.resize(
                left,
                self.depth_size,
                interpolation=cv2.INTER_AREA,
                dst=self._buffer("left", self.depth_size[::-1], left.dtype),
            )
            right = cv2.resize(
                right,
                self.depth_size,
                interpolation=cv2.INTER_AREA,
                dst=self._buffer("right", self.depth_size[::-1], right.dtype),
            )
        a = self._remap(left, self.map_a, "a")
        b = self._remap(right, self.mapB, "b")
        return self.matchRectified(a, b, with_xyz=with_xyz)

    def matchRectified(self, a, b, *, with_xyz=True):
        if (
            a.shape != b.shape
            or a.shape[:2] != self.depth_size[::-1]
            or a.dtype != np.uint8
            or b.dtype != np.uint8
            or not (a.ndim == 2 or (a.ndim == 3 and a.shape[2] == 3))
        ):
            raise ValueError("rectified pair must match matching_size with mono8/BGR8")
        start = time.monotonic()
        grayA = (
            a
            if a.ndim == 2
            else cv2.cvtColor(
                a, cv2.COLOR_BGR2GRAY, dst=self._buffer("gray_a", self.depth_size[::-1], np.uint8)
            )
        )
        grayB = (
            b
            if b.ndim == 2
            else cv2.cvtColor(
                b, cv2.COLOR_BGR2GRAY, dst=self._buffer("gray_b", self.depth_size[::-1], np.uint8)
            )
        )
        if self.alternative is None:
            rawA = self.matcherA.compute(
                grayA, grayB, self._buffer("disp_a", self.depth_size[::-1], np.int16)
            )
            rawB = self.matcherB.compute(
                grayB, grayA, self._buffer("disp_b", self.depth_size[::-1], np.int16)
            )
        else:
            prior = (
                None
                if self._prior is None
                else self._prior.forFrame(grayA.shape, self._frameStamp)
            )
            rawA, rawB = self.alternative.computePair(grayA, grayB, prior)
        self._prior = None  # 每帧必须重新交付带时间戳的先验。
        if self.refiner is not None:
            rawA = self.refiner.refine(grayA, grayB, rawA)
            rb = self.refiner.refine(
                np.fliplr(grayB).copy(), np.fliplr(grayA).copy(), -np.fliplr(rawB).copy()
            )
            rawB = -np.fliplr(rb).copy()
        matched = time.monotonic()
        output, valid = self.reconstructQ4(rawA, rawB, with_xyz=with_xyz)
        disparityA = rawA.astype(np.float32) / 16
        # scratch 只能内部复用，返回的帧快照不能在下次处理时被覆盖。
        if self.config.reuse_buffers:
            a, b = a.copy(), b.copy()
        end = time.monotonic()
        return (
            a,
            b,
            disparityA,
            output,
            valid,
            {
                "match_ms": (matched - start) * 1e3,
                "post_ms": (end - matched) * 1e3,
                "compute_ms": (end - start) * 1e3,
            },
        )


def selectNumDisparities(q, requested=96, required_min_depth_m=0.0):
    """保留 d < N-1 的一致性门限；自动范围不得超过原 96 档。"""
    if isinstance(requested, bool) or int(requested) != requested:
        raise ValueError("num_disparities 必须为整数")
    requested = int(requested)
    minimum = float(required_min_depth_m)
    if not np.isfinite(minimum) or minimum < 0:
        raise ValueError("required_min_depth_m 必须有限且非负")
    if requested == 0 and minimum == 0:
        raise ValueError("自动视差范围需要 required_min_depth_m > 0")
    if minimum:
        if not (np.all(q[2, :3] == 0) and np.all(q[3, :2] == 0) and q[2, 3] > 0 and q[3, 2] > 0):
            raise ValueError("自动量程检查需要正视差水平 Q")
        disparity = (q[2, 3] / minimum - q[3, 3]) / q[3, 2]
        if not np.isfinite(disparity) or disparity <= 0:
            raise ValueError("required_min_depth_m 对应视差无效")
        needed = max(16, int(np.ceil((np.floor(disparity) + 2) / 16)) * 16)
        if requested == 0:
            if needed > 96:
                raise ValueError("最近探测距离需要超过 96 视差，不能自动缩窄")
            requested = needed
        elif requested < needed:
            raise ValueError(f"num_disparities={requested} 无法覆盖 {minimum} m，至少需要 {needed}")
    if requested < 16 or requested >= DEPTH_SIZE[0] or requested % 16:
        raise ValueError("num_disparities 必须是 16 的倍数且小于图像宽度")
    return requested
