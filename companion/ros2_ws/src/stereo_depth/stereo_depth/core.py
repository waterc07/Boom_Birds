"""双目几何与匹配；导入不打开设备。"""
from dataclasses import dataclass
from pathlib import Path
import time
import cv2
import numpy as np

def _resolve_root():
    """标定与输出目录的根。

    默认是当前文件所在目录（仓库开发布局，行为与既有版本一致）。
    安装到 ROS 2 share 目录后，源码目录不可写也不再位于仓库内，此时回退到
    ament share 路径，保证已安装包不依赖当前工作目录。
    """
    here = Path(__file__).resolve().parent.parent
    if (here / "calibration").is_dir():
        return here
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("stereo_depth"))
    except Exception:
        return here


ROOT = _resolve_root()
DEPTH_SIZE = (320, 240)    # 每目计算分辨率，不能仅改此值而忽略标定尺度。
PREVIEW_RANGE_M = (0.15, 3.0)


@dataclass(frozen=True)
class Config:
    """集中定义运行参数；分辨率固定，避免误用不匹配的标定。"""

    device: str = "/dev/video0"
    port: int = 8081
    threads: int = 4
    reduced_decode: bool = True
    calibration: Path = ROOT / "calibration/live_20260916_210120_642136/candidate.npz"
    output_dir: Path = ROOT / "depth_outputs"
    num_disparities: int = 96  # 0：按 required_min_depth_m 选择最小的 16 倍数
    required_min_depth_m: float = 0.0


@dataclass(frozen=True)
class CapturedFrame:
    sequence: int
    received_at: float  # 主机完成取帧的单调时钟时间，不是传感器曝光时间。
    packet: np.ndarray


@dataclass(frozen=True)
class DepthFrame:
    """发布后不再修改内部数组，HTTP 保存时可安全读取同一帧的完整快照。"""

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
        if not config.calibration.is_file():
            raise FileNotFoundError(
                f"标定文件不存在：{config.calibration}；请从树莓派取回标定目录、重新标定，"
                "或用 --calibration 指定已有的 candidate.npz")
        with np.load(config.calibration) as archive:
            calibration = {key: archive[key].copy() for key in archive.files}
        image_size = tuple(int(v) for v in calibration["image_size"])
        if len(image_size) != 2 or min(image_size) < 240 or any(v % 2 for v in image_size):
            raise ValueError("标定图像尺寸必须为有效偶数像素尺寸")
        self.capture_size = (image_size[0] * 2, image_size[1])
        self.baseline_m = float(np.linalg.norm(calibration["T"]))
        if not np.isfinite(self.baseline_m) or not .001 < self.baseline_m < 1:
            raise ValueError("标定基线必须以米为单位且处于合理范围")
        translation = calibration["T"].reshape(3)
        if translation[0] >= 0 or abs(translation[1]) > abs(translation[0]) * .2:
            raise ValueError("标定不符合 A→B 正视差水平双目，请检查左右顺序")

        # 按标定的原始每目尺寸缩放内参；采集也使用该模式，避免假定不同模式同视场。
        # T 的单位仍是米，不能随图像尺寸一起缩放。
        k_a = calibration["K1"].copy()
        k_b = calibration["K2"].copy()
        for k in (k_a, k_b):
            k[0, :] *= DEPTH_SIZE[0] / image_size[0]
            k[1, :] *= DEPTH_SIZE[1] / image_size[1]
        r_a, r_b, p_a, p_b, self.q, _, _ = cv2.stereoRectify(
            k_a, calibration["D1"], k_b, calibration["D2"], DEPTH_SIZE,
            calibration["R"], calibration["T"],
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
        )
        # R1 将原图光学系旋转到校正光学系；位姿链 T_C0_Crect 使用 R1.T。
        # P1 为 DEPTH_SIZE、alpha=0 的投影内参；ROS CameraInfo 复用它。
        self.r1, self.r2 = r_a, r_b
        self.p1, self.p2 = p_a, p_b
        self.map_a = cv2.initUndistortRectifyMap(
            k_a, calibration["D1"], r_a, p_a, DEPTH_SIZE, cv2.CV_32FC1)
        self.map_b = cv2.initUndistortRectifyMap(
            k_b, calibration["D2"], r_b, p_b, DEPTH_SIZE, cv2.CV_32FC1)
        if not np.isfinite(self.q).all():
            raise ValueError("标定 Q 必须有限")
        self.num_disparities = select_num_disparities(
            self.q, config.num_disparities, config.required_min_depth_m)
        self.nearest_depth_m = float(self.q[2, 3] / (
            self.q[3, 2] * (self.num_disparities - 1) + self.q[3, 3]))
        # 水平 stereoRectify 的 Q；其他结构仍走完整重投影。
        self._z_only_supported = bool(
            np.all(self.q[2, :3] == 0) and np.all(self.q[3, :2] == 0)
            and np.all(self.q[:2, 2] == 0))
        parameters = dict(
            numDisparities=self.num_disparities, blockSize=5,
            P1=8 * 25, P2=32 * 25, disp12MaxDiff=1,
            uniquenessRatio=12, speckleWindowSize=80, speckleRange=2,
            preFilterCap=31, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        self.matcher_a = cv2.StereoSGBM_create(minDisparity=0, **parameters)
        self.matcher_b = cv2.StereoSGBM_create(
            minDisparity=1 - self.num_disparities, **parameters)
        self.rows, self.cols = np.indices(DEPTH_SIZE[::-1], dtype=np.int32)

    def rectify(self, packet):
        """半尺寸 JPEG 解码可省去完整解码；完整解码选项用于对照。"""
        mode = cv2.IMREAD_REDUCED_COLOR_2 if self.config.reduced_decode else cv2.IMREAD_COLOR
        image = cv2.imdecode(packet.reshape(-1), mode)
        divisor = 2 if self.config.reduced_decode else 1
        expected = (self.capture_size[1] // divisor, self.capture_size[0] // divisor)
        if image is None or image.shape[:2] != expected:
            raise RuntimeError("JPEG 尺寸不符合预期，请检查相机输出模式")
        return self._rectify_resized(image)

    def rectify_image(self, image):
        """对已解码的 BGR 拼接图做校正；供 ROS 2 节点复用同一套几何运算。

        入参 image 是完整的左右拼接灰度或 BGR 数组（宽 = 2 × 每目宽）。
        与 rectify() 的唯一区别是本方法不经过 JPEG 解码，尺寸/缩放/校正路径完全一致，
        避免 ROS 侧为复用而重新编码 JPEG。
        """
        if image is None or not (image.ndim == 2 or (image.ndim == 3 and image.shape[2] == 3)):
            raise ValueError("rectify_image 需要灰度或 BGR 拼接图")
        return self._rectify_resized(image)

    def _rectify_resized(self, image):
        """把任意尺寸的拼接 BGR 图缩放到 DEPTH_SIZE 并做极线校正，返回 (a, b)。"""
        if image.shape[:2] != (DEPTH_SIZE[1], DEPTH_SIZE[0] * 2):
            image = cv2.resize(image, (640, 240), interpolation=cv2.INTER_AREA)
        a = cv2.remap(image[:, :320], *self.map_a, cv2.INTER_LINEAR)
        b = cv2.remap(image[:, 320:], *self.map_b, cv2.INTER_LINEAR)
        return a, b

    def reconstruct(self, disparity_a, disparity_b, *, with_xyz=True):
        """执行左右一致性检查，输出校正 A 目光学系 XYZ 或 Z（米）。"""
        # A 目横坐标 u 对应 B 目 u-d；B→A 视差符号与 A→B 相反。
        target_cols = np.rint(self.cols - disparity_a).astype(int)
        inside = (target_cols >= 0) & (target_cols < DEPTH_SIZE[0])
        sampled_b = disparity_b[self.rows, np.clip(target_cols, 0, DEPTH_SIZE[0] - 1)]
        if with_xyz or not self._z_only_supported:
            points = cv2.reprojectImageTo3D(disparity_a, self.q)
            finite = np.isfinite(points).all(axis=2)
            z = points[:, :, 2]
            output = points if with_xyz else z.copy()
        else:
            # 用同一 Q 的齐次除法直接求 Z，不分配 H×W×3 数组。
            denominator = self.q[3, 2] * disparity_a.astype(np.float64) + self.q[3, 3]
            with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
                z = (self.q[2, 3] / denominator).astype(np.float32)
            finite = np.isfinite(z)
            output = z
        valid = (
            inside & (disparity_a > 0) & (disparity_a < self.num_disparities - 1)
            & (sampled_b > -self.num_disparities)
            & (np.abs(disparity_a + sampled_b) <= 1)
            & finite & (z > 0)
        )
        output[~valid] = np.nan
        return output, valid

    def process_image(self, image, *, with_xyz=True):
        """已解码左右拼接图 → 校正图、视差、XYZ、有效掩码、耗时。"""
        a, b = self.rectify_image(image)
        return self.match_rectified(a, b, with_xyz=with_xyz)

    def process_pair(self, left, right, *, with_xyz=True):
        if left.ndim != 2 or left.shape != right.shape:
            raise ValueError("共享输入必须是同尺寸灰度双目")
        if left.shape != DEPTH_SIZE[::-1]:
            left = cv2.resize(left, DEPTH_SIZE, interpolation=cv2.INTER_AREA)
            right = cv2.resize(right, DEPTH_SIZE, interpolation=cv2.INTER_AREA)
        a = cv2.remap(left, *self.map_a, cv2.INTER_LINEAR)
        b = cv2.remap(right, *self.map_b, cv2.INTER_LINEAR)
        return self.match_rectified(a, b, with_xyz=with_xyz)

    def match_rectified(self, a, b, *, with_xyz=True):
        start = time.monotonic()
        gray_a = a if a.ndim == 2 else cv2.cvtColor(a, cv2.COLOR_BGR2GRAY)
        gray_b = b if b.ndim == 2 else cv2.cvtColor(b, cv2.COLOR_BGR2GRAY)
        # SGBM 返回 4 位小数的定点视差；除以 16 后才能交给米制 Q 矩阵重投影。
        disparity_a = self.matcher_a.compute(gray_a, gray_b).astype(np.float32) / 16
        disparity_b = self.matcher_b.compute(gray_b, gray_a).astype(np.float32) / 16
        matched = time.monotonic()
        output, valid = self.reconstruct(disparity_a, disparity_b, with_xyz=with_xyz)
        end = time.monotonic()
        return a, b, disparity_a, output, valid, {
            "match_ms": (matched-start)*1e3, "post_ms": (end-matched)*1e3,
            "compute_ms": (end-start)*1e3,
        }


def select_num_disparities(q, requested=96, required_min_depth_m=0.0):
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
        if not (np.all(q[2, :3] == 0) and np.all(q[3, :2] == 0)
                and q[2, 3] > 0 and q[3, 2] > 0):
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
