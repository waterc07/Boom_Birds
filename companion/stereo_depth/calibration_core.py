"""视频标定的纯几何部分；不打开相机，不依赖 HTTP。

角点始终为原始每目像素坐标，物点单位为米。训练与留出帧分开，
留出误差仅作几何检查，不等同于独立距离精度验收。
"""
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass(frozen=True)
class Board:
    columns: int = 11
    rows: int = 8
    square_m: float = 0.020

    @property
    def pattern(self):
        return self.columns, self.rows

    def objects(self):
        points = np.zeros((self.columns * self.rows, 3), np.float32)
        points[:, :2] = np.mgrid[:self.columns, :self.rows].T.reshape(-1, 2)
        return points * self.square_m


def detect(gray, board, thorough=True):
    """小图搜索，全分辨率亚像素细化；返回与原图匹配的角点。"""
    scale = min(1.0, 640 / gray.shape[1])
    small = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    found, points = cv2.findChessboardCorners(
        small, board.pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_FAST_CHECK)
    if not found and thorough:
        found, points = cv2.findChessboardCornersSB(small, board.pattern)
    if not found:
        return None
    points = np.ascontiguousarray(points / scale, dtype=np.float32)
    cv2.cornerSubPix(gray, points, (7, 7), (-1, -1),
                     (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, .001))
    return points


def align_pair(a, b, board):
    """消除矩形棋盘检测的 180° 编号歧义；不改变坐标或翻转图像。"""
    a, b = a.copy(), b.copy()
    va = a[board.columns - 1, 0] - a[0, 0]
    vb = b[board.columns - 1, 0] - b[0, 0]
    if np.dot(va, vb) < 0:
        b = b[::-1].copy()
    # 固定 A 的主方向，使相邻检测不会仅因编号反向而被判为运动。
    if va[np.argmax(np.abs(va))] < 0:
        a, b = a[::-1].copy(), b[::-1].copy()
    return a, b


def descriptor(points, size, board):
    p = points.reshape(board.rows, board.columns, 2)
    tl, tr, bl, br = p[0, 0], p[0, -1], p[-1, 0], p[-1, -1]
    top, bottom = np.linalg.norm(tr - tl), np.linalg.norm(br - bl)
    left, right = np.linalg.norm(bl - tl), np.linalg.norm(br - tr)
    area = abs(cv2.contourArea(np.float32([tl, tr, br, bl]))) / np.prod(size)
    roll = np.arctan2((tr - tl)[1], (tr - tl)[0])
    aspect = np.log(max((top + bottom) / max(left + right, 1), .01)
                    / ((board.columns - 1) / (board.rows - 1)))
    return np.array([*points.reshape(-1, 2).mean(axis=0) / size, np.sqrt(area),
                     roll, (top - bottom) / max(top + bottom, 1),
                     (left - right) / max(left + right, 1), aspect])


def is_novel(pair, samples, size, board):
    now = np.concatenate([descriptor(p, size, board) for p in pair])
    # 各维容差：位置、大小、平面转角、透视及长宽比。
    tolerance = np.tile([.085, .085, .06, .18, .07, .07, .13], 2)
    for old in samples:
        before = np.concatenate([descriptor(p, size, board) for p in old])
        delta = now - before
        delta[[3, 10]] = (delta[[3, 10]] + np.pi) % (2 * np.pi) - np.pi
        if np.max(np.abs(delta) / tolerance) < 1:
            return False
    return True


def coverage(samples, size, board):
    grids, centers = [], []
    for eye in range(2):
        grid, center = np.zeros((3, 4), int), np.zeros((3, 3), int)
        for pair in samples:
            pts = pair[eye].reshape(-1, 2) / size
            bins = np.clip((pts * [4, 3]).astype(int), [0, 0], [3, 2])
            for x, y in np.unique(bins, axis=0):
                grid[y, x] += 1
            x, y = np.clip((pts.mean(axis=0) * 3).astype(int), 0, 2)
            center[y, x] += 1
        grids.append(grid.tolist())
        centers.append(center.tolist())
    desc = [descriptor(p[0], size, board) for p in samples]
    scales = [d[2] for d in desc]
    tilted = int(sum(max(abs(d[4]), abs(d[5])) > .07 or abs(d[6]) > .18 for d in desc))
    ratio = max(scales) / max(min(scales), 1e-6) if scales else 1
    roll_span = float(np.ptp([d[3] for d in desc])) if desc else 0
    missing = []
    if len(samples) < 24:
        missing.append(f"还需 {24 - len(samples)} 组不同姿态（至少 24 组）")
    if any(np.count_nonzero(g) < 9 for g in grids):
        missing.append("移向画面四边与四角，完整棋盘仍须同时出现在两目")
    if any(np.count_nonzero(g) < 5 for g in centers):
        missing.append("让棋盘中心分布到更多区域：左、右、上、下")
    if ratio < 1.5:
        missing.append("改变距离，补充较近和较远的棋盘")
    if tilted < 6:
        missing.append("补充左右转面、上下俯仰的倾斜姿态，不能只平移或平面旋转")
    if roll_span < .35:
        missing.append("在画面内轻转棋盘，补充不同方向")
    return dict(grids=grids, centers=centers, count=len(samples), tilted=tilted,
                scale_ratio=round(float(ratio), 2), roll_span_deg=round(float(np.degrees(roll_span)), 1),
                missing=missing, ready=not missing)


def pair_quality(grays, pair, size, board):
    """边缘锐度采用 OpenCV 棋盘黑白过渡宽度；阈值是采集启发式。"""
    widths, reasons = [], []
    for label, gray, pts in zip(('A', 'B'), grays, pair):
        p = pts.reshape(-1, 2)
        spacing = np.median(np.linalg.norm(np.diff(p.reshape(board.rows, board.columns, 2), axis=1), axis=2))
        if spacing < 12:
            reasons.append(f"{label} 目棋盘太小，请靠近一些")
        margin = min(float(p.min()), float((np.array(size) - 1 - p).min()))
        if margin < max(12, spacing * .8):
            reasons.append(f"{label} 目棋盘靠边或外边框不完整，请略向内移")
        sharpness = float(cv2.estimateChessboardSharpness(gray, board.pattern, pts)[0][0])
        widths.append(round(sharpness, 2) if np.isfinite(sharpness) else None)
        if not np.isfinite(sharpness) or sharpness > 3.5:
            reasons.append(f"{label} 目边缘模糊，请调焦、保持静止或增加照明")
    return widths, reasons


def epipolar_errors(pair, model, size):
    rect = cv2.stereoRectify(model['K1'], model['D1'], model['K2'], model['D2'], size,
                           model['R'], model['T'], flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    a = cv2.undistortPoints(pair[0], model['K1'], model['D1'], R=rect[0], P=rect[2])
    b = cv2.undistortPoints(pair[1], model['K2'], model['D2'], R=rect[1], P=rect[3])
    return np.abs(a[:, 0, 1] - b[:, 0, 1])


def calibrate(samples, size, board, progress=lambda message: None):
    """固定留出 20% 姿态；拒绝非有限/退化解；不强行缩放到旧的 65 mm。"""
    if len(samples) < 24:
        raise ValueError('至少需要 24 组有效双目角点')
    test_ids = list(range(4, len(samples), 5))
    train_ids = [i for i in range(len(samples)) if i not in test_ids]
    obj = board.objects()
    objects = [obj.copy() for _ in train_ids]
    images = [[samples[i][eye] for i in train_ids] for eye in range(2)]
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-8)
    mono = []
    for eye in range(2):
        progress(f"正在计算 {'A' if eye == 0 else 'B'} 目内参和畸变…")
        mono.append(cv2.calibrateCamera(objects, images[eye], size, None, None, criteria=criteria))
    progress('正在计算双目相对旋转、平移与校正矩阵…')
    rms, k1, d1, k2, d2, r, t, e, f = cv2.stereoCalibrate(
        objects, images[0], images[1], mono[0][1], mono[0][2], mono[1][1], mono[1][2],
        size, criteria=criteria, flags=cv2.CALIB_FIX_INTRINSIC)
    model = dict(K1=k1, D1=d1, K2=k2, D2=d2, R=r, T=t, E=e, F=f,
                 image_size=np.array(size), square_size_m=np.array(board.square_m))
    if not all(np.isfinite(v).all() for v in model.values()):
        raise ValueError('标定返回非有限值，请重新采集更多有效姿态')
    baseline = float(np.linalg.norm(t))
    if baseline <= 1e-5 or baseline > 1:
        raise ValueError('基线估计退化或超过 1 m，请检查棋盘尺寸与角点对应')
    for k in (k1, k2):
        if min(k[0, 0], k[1, 1]) <= 0 or not (-size[0] < k[0, 2] < 2 * size[0]
                                              and -size[1] < k[1, 2] < 2 * size[1]):
            raise ValueError('内参解明显不合理，请重新采集')
    r1, r2, p1, p2, q, roi1, roi2 = cv2.stereoRectify(
        k1, d1, k2, d2, size, r, t, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    model.update(R1=r1, R2=r2, P1=p1, P2=p2, Q=q)
    errors = np.concatenate([epipolar_errors(samples[i], model, size) for i in test_ids])
    holdout_mono = [[], []]
    for eye, (k, d) in enumerate(((k1, d1), (k2, d2))):
        for i in test_ids:
            ok, rv, tv = cv2.solvePnP(obj, samples[i][eye], k, d)
            if not ok:
                raise ValueError('留出帧姿态求解失败')
            projected = cv2.projectPoints(obj, rv, tv, k, d)[0]
            holdout_mono[eye].append(float(np.sqrt(np.mean(np.sum((projected - samples[i][eye])**2, axis=2)))))
    warnings = []
    if max(mono[0][0], mono[1][0], rms) > 1.0:
        warnings.append('训练重投影 RMS 超过 1 px')
    if np.percentile(errors, 95) > 1.5:
        warnings.append('留出帧垂直误差 P95 超过 1.5 px')
    if max(max(v) for v in holdout_mono) > 1.5:
        warnings.append('部分留出帧单目重投影 RMS 超过 1.5 px')
    if abs(baseline - .065) > .0065:
        warnings.append('估计基线与历史 65 mm 相差超过 10%，请核对打印比例和安装尺寸')
    if t[0, 0] >= 0 or abs(t[1, 0]) > abs(t[0, 0]) * .2:
        warnings.append('双目顺序或安装方向不符合当前正视差水平深度程序')
    warnings.extend(coverage(samples, size, board)['missing'])
    report = dict(status='CANDIDATE_NEEDS_REVIEW' if warnings else 'CANDIDATE_NOT_METRIC_VALIDATED',
                  image_size=list(size), board_inner_corners=list(board.pattern), square_mm=board.square_m * 1000,
                  baseline_mm=baseline * 1000, mono_rms_px=[mono[0][0], mono[1][0]], stereo_rms_px=rms,
                  holdout_vertical_p95_px=float(np.percentile(errors, 95)),
                  holdout_vertical_median_px=float(np.median(errors)), holdout_mono_rms_px=holdout_mono,
                  train_indices=train_ids, holdout_indices=test_ids, warnings=warnings,
                  coverage=coverage(samples, size, board),
                  threshold_note='阈值为原始分辨率下的工程筛查，不是距离精度或飞行验收',
                  scale_note='方格 20 mm 决定米制尺度；未按旧基线强制缩放',
                  valid_roi=[list(roi1), list(roi2)])
    return model, report
