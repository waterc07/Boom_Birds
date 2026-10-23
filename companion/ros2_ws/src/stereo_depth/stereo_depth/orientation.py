"""倒装双目的图像方向与内参缩放；输入方向必须和标定一起传递。"""
import cv2
import numpy as np

SENSOR = "sensor_ab"
UPRIGHT = "upright_ba_rot180"
ROTATION = np.diag([-1., -1., 1.])


def imageOrientation(calibration):
    value = str(np.asarray(calibration.get("image_orientation", SENSOR)).item())
    if value not in (SENSOR, UPRIGHT):
        raise ValueError("unknown stereo image_orientation: " + value)
    return value


def scaleMatrix(k, sourceSize, targetSize, offset=0.):
    """offset=1 表示先缩放旧传感器内参，再执行 u'=W-1-u。"""
    source = np.asarray(sourceSize, dtype=float)
    target = np.asarray(targetSize, dtype=float)
    if source.shape != (2,) or target.shape != (2,) or not np.isfinite([source, target]).all() or min(*source, *target) <= 0:
        raise ValueError("camera dimensions must be finite and positive")
    factors = target / source
    if not np.isclose(factors[0], factors[1], atol=1e-9, rtol=0):
        raise ValueError("camera scaling must preserve aspect ratio")
    offset = float(offset)
    if offset not in (0., 1.):
        raise ValueError("intrinsics_scale_offset must be 0 or 1")
    result = np.array(k, dtype=float, copy=True)
    result[0] *= factors[0]
    result[1] *= factors[1]
    result[0, 2] += (factors[0] - 1) * offset
    result[1, 2] += (factors[1] - 1) * offset
    return result


def cameraMatrix(calibration, side, size):
    return scaleMatrix(calibration["K" + str(side)], calibration["image_size"], size,
                        calibration.get("intrinsics_scale_offset", 0.))


def orientPair(left, right, source=SENSOR, target=UPRIGHT):
    if source not in (SENSOR, UPRIGHT) or target not in (SENSOR, UPRIGHT):
        raise ValueError("unknown stereo input/output orientation")
    if left.shape != right.shape or left.ndim not in (2, 3):
        raise ValueError("stereo pair dimensions must match")
    if source == target:
        return left, right
    return cv2.rotate(right, cv2.ROTATE_180), cv2.rotate(left, cv2.ROTATE_180)


def orientStitched(image, source=SENSOR, target=UPRIGHT):
    if image.ndim not in (2, 3) or image.shape[1] % 2:
        raise ValueError("stitched image must contain two equal-width eyes")
    if source == target:
        # 校验方向名，避免未知标记静默通过。
        orientPair(image[:, :image.shape[1]//2], image[:, image.shape[1]//2:], source, target)
        return image
    a, b = np.split(image, 2, axis=1)
    return np.hstack(orientPair(a, b, source, target))


def convertCalibration(data):
    """传感器 A/B 标定 → B/A 逐目旋转 180°；不接受重复转换。"""
    if imageOrientation(data) != SENSOR:
        raise ValueError("calibration is already upright")
    out = {k: np.array(v, copy=True) for k, v in data.items()}
    w, h = (int(v) for v in data["image_size"])
    for new, old in ((1, 2), (2, 1)):
        k = np.array(data[f"K{old}"], dtype=float, copy=True)
        if k.shape != (3, 3) or k[0, 1] != 0 or k[1, 0] != 0:
            raise ValueError("conversion requires zero-skew pinhole intrinsics")
        k[0, 2], k[1, 2] = w - 1 - k[0, 2], h - 1 - k[1, 2]
        d = np.array(data[f"D{old}"], dtype=float, copy=True)
        if d.size not in (4, 5):
            raise ValueError("conversion supports only 4/5-term radtan")
        d.reshape(-1)[2:4] *= -1
        out[f"K{new}"], out[f"D{new}"] = k, d
    r = np.asarray(data["R"])
    t = np.asarray(data["T"]).reshape(3)
    out["R"] = ROTATION @ r.T @ ROTATION
    out["T"] = (-ROTATION @ r.T @ t)[:, None]
    values = cv2.stereoRectify(out["K1"], out["D1"], out["K2"], out["D2"],
                              (w, h), out["R"], out["T"],
                              flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    out.update(zip(("R1", "R2", "P1", "P2", "Q"), values[:5]))
    tx, ty, tz = out["T"].reshape(3)
    out["E"] = np.array([[0, -tz, ty], [tz, 0, -tx], [-ty, tx, 0.]]) @ out["R"]
    out["F"] = np.linalg.inv(out["K2"]).T @ out["E"] @ np.linalg.inv(out["K1"])
    out["image_orientation"] = np.array(UPRIGHT)
    out["intrinsics_scale_offset"] = np.array(1.)
    return out

def convertCameraChain(source):
    """两目 pinhole/radtan 链换序并旋转；时间偏移随物理传感器保留。"""
    from copy import deepcopy
    if set(source) != {"cam0", "cam1"}:
        raise ValueError("conversion requires both original cameras")
    h = np.diag([-1., -1., 1., 1.])
    result = {}
    for new, old in ((0, 1), (1, 0)):
        cam = deepcopy(source[f"cam{old}"])
        if cam.get("image_orientation", SENSOR) != SENSOR:
            raise ValueError("camera chain is already upright")
        if cam["camera_model"] != "pinhole" or cam["distortion_model"] != "radtan":
            raise ValueError("conversion requires pinhole/radtan cameras")
        fx, fy, cx, cy = cam["intrinsics"]
        w, height = cam["resolution"]
        cam["intrinsics"] = [fx, fy, w-1-cx, height-1-cy]
        d = np.array(cam["distortion_coeffs"], dtype=float)
        if d.size not in (4, 5):
            raise ValueError("conversion supports only 4/5-term radtan")
        d[2:4] *= -1
        cam["distortion_coeffs"] = d.tolist()
        if "T_cam_imu" in cam:
            cam["T_cam_imu"] = (h @ np.asarray(cam["T_cam_imu"])).tolist()
        if "T_imu_cam" in cam:
            cam["T_imu_cam"] = (np.asarray(cam["T_imu_cam"]) @ h).tolist()
        cam.pop("T_cn_cnm1", None)
        cam["rostopic"] = source[f"cam{new}"]["rostopic"]
        cam["cam_overlaps"] = [1-new]
        cam["image_orientation"] = UPRIGHT
        cam["intrinsics_scale_offset"] = 1.
        result[f"cam{new}"] = cam
    def camFromImu(cam):
        return np.asarray(cam["T_cam_imu"]) if "T_cam_imu" in cam else np.linalg.inv(cam["T_imu_cam"])
    relative = camFromImu(result["cam1"]) @ np.linalg.inv(camFromImu(result["cam0"]))
    result["cam1"]["T_cn_cnm1"] = relative.tolist()
    return result
