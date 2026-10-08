"""单目内参求解；角点使用完整 RAW 解码图的像素坐标。"""
import cv2
import numpy as np
from calibration_core import Board, coverage as shared_coverage


def coverage(samples, size, board):
    return shared_coverage(samples, size, board, eyes=1)


def pose_error(points, model, board):
    ok, rv, tv = cv2.solvePnP(board.objects(), points, model['K'], model['D'])
    if not ok or not np.isfinite(tv).all() or tv[2, 0] <= 0:
        raise ValueError('棋盘姿态求解失败')
    projected = cv2.projectPoints(board.objects(), rv, tv, model['K'], model['D'])[0]
    return float(np.sqrt(np.mean(np.sum((projected - points) ** 2, axis=2))))


def calibrate(samples, size, board):
    if len(samples) < 24:
        raise ValueError('样本数量或位置/距离/倾角覆盖不足')
    for sample in samples:
        p = sample[0]
        if p.shape != (board.columns * board.rows, 1, 2) or not np.isfinite(p).all():
            raise ValueError('角点维度或数值不合法')
        if np.any(p < 0) or np.any(p.reshape(-1, 2) >= np.array(size)):
            raise ValueError('角点超出原始图像')
    if not coverage(samples, size, board)['ready']:
        raise ValueError('位置/距离/倾角覆盖不足')
    # 每五组留出一组，不参与 K/D 拟合；留出姿态仍由 PnP 估计，仅检查重投影。
    # 这不是独立距离真值或相机安装外参验收。
    holdout = list(range(4, len(samples), 5))
    train = [i for i in range(len(samples)) if i not in holdout]
    rms, k, d, _, _ = cv2.calibrateCamera(
        [board.objects() for _ in train], [samples[i][0] for i in train],
        size, None, None,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-8))
    if not np.isfinite(rms) or not np.isfinite(k).all() or not np.isfinite(d).all():
        raise ValueError('求解返回非有限值')
    if min(k[0, 0], k[1, 1]) <= 0 or not (0 < k[0, 2] < size[0] and 0 < k[1, 2] < size[1]):
        raise ValueError('内参明显退化')
    model = dict(K=k, D=d, image_size=np.array(size), square_size_m=board.square_m)
    errors = [pose_error(samples[i][0], model, board) for i in holdout]
    warnings = []
    if rms > 1:
        warnings.append('训练重投影 RMS 超过 1 px')
    if max(errors) > 1.5:
        warnings.append('留出姿态重投影 RMS 超过 1.5 px')
    report = dict(status='CANDIDATE_NEEDS_REVIEW' if warnings else 'CANDIDATE_NOT_METRIC_VALIDATED',
                  camera_model='pinhole', distortion_model='plumb_bob',
                  image_size=list(size), board_inner_corners=list(board.pattern),
                  square_mm=board.square_m * 1000, K=k.tolist(), D=d.reshape(-1).tolist(),
                  train_rms_px=float(rms), holdout_rms_px=errors,
                  train_indices=train, holdout_indices=holdout, warnings=warnings,
                  coverage=coverage(samples, size, board),
                  limitation='重投影筛查；未验证距离精度、相机安装外参、时间同步或飞行')
    return model, report
