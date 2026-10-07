"""TEST-ONLY local-feature continuation; output is a pixel target, not a board pose."""
from dataclasses import dataclass, asdict
import math
import cv2
import numpy as np


@dataclass(frozen=True)
class TrackConfig:
    capture_min_m: float = .5
    capture_max_m: float = 1.5
    roi_radius_px: int = 65
    max_points: int = 60
    min_points: int = 8
    max_gap_s: float = .15
    max_identity_age_s: float = 8.
    max_fb_px: float = 1.
    ransac_px: float = 2.
    min_inlier_ratio: float = .7

    def __post_init__(self):
        for name, value in asdict(self).items():
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(name)
        for name in ("roi_radius_px", "max_points", "min_points"):
            if type(getattr(self, name)) is not int:
                raise ValueError(name)
        if self.capture_min_m >= self.capture_max_m or self.min_points < 4 or self.max_points < self.min_points or self.min_inlier_ratio > 1:
            raise ValueError("track_limits")


class BoardFeatureTrack:
    """Identity may only be seeded by a valid decoded board; loss requires new decoding."""
    def __init__(self, detector, config=None):
        if not detector.test_only:
            raise ValueError("feature_track_requires_test_only")
        self.detector = detector
        self.cfg = config or TrackConfig()
        self.reset()

    def reset(self):
        self.previous = self.points = self.center = None
        self.stamp = self.identity_stamp = None
        self.ids = ()

    def invalid(self, stamp, reason):
        self.reset()
        return dict(stamp=stamp, valid=False, reason=reason, test_only=True)

    def seed(self, image, observation, distance_m):
        c = self.cfg
        if (not observation.valid or observation.board != self.detector.board["name"]
                or not observation.tag_ids or any(i not in self.detector.tags for i in observation.tag_ids)):
            return self.invalid(observation.stamp, "identity_unconfirmed")
        if not math.isfinite(distance_m) or not c.capture_min_m <= distance_m <= c.capture_max_m:
            return self.invalid(observation.stamp, "outside_capture_height")
        if image is None or image.shape != tuple(reversed(self.detector.camera["image_size"])) or image.dtype != np.uint8:
            return self.invalid(observation.stamp, "image_invalid")
        pose = np.linalg.inv(self.detector.T_B_C) @ observation.pose()
        target = np.asarray(self.detector.board["landing_point_m"], float).reshape(1, 3)
        center = cv2.projectPoints(target, cv2.Rodrigues(pose[:3, :3])[0],
                                  pose[:3, 3], self.detector.K, self.detector.D)[0].reshape(2)
        mask = np.zeros_like(image)
        cv2.circle(mask, tuple(np.rint(center).astype(int)), c.roi_radius_px, 255, -1)
        points = cv2.goodFeaturesToTrack(image, c.max_points, .02, 5, mask=mask)
        if points is None or len(points) < c.min_points:
            return self.invalid(observation.stamp, "insufficient_center_features")
        self.previous, self.points, self.center = image.copy(), points, center
        self.stamp = self.identity_stamp = observation.stamp
        self.ids = observation.tag_ids
        return self.output(observation.stamp, "decoded", len(points), 1., 0.)

    def output(self, stamp, source, count, ratio, fb):
        return dict(stamp=stamp, valid=True, reason="ok", test_only=True,
                    board=self.detector.board["name"], inherited_tag_ids=list(self.ids),
                    identity_stamp=self.identity_stamp, identity_age_s=stamp-self.identity_stamp,
                    landing_pixel=self.center.tolist(), feature_count=count,
                    inlier_ratio=ratio, forward_backward_px=fb, source=source,
                    frame="camera_pixels", metric_pose_available=False)

    def update(self, image, stamp):
        c = self.cfg
        if self.previous is None:
            return self.invalid(stamp, "decode_required")
        if not math.isfinite(stamp) or not 0 < stamp-self.stamp <= c.max_gap_s:
            return self.invalid(stamp, "sample_gap")
        if stamp-self.identity_stamp > c.max_identity_age_s:
            return self.invalid(stamp, "identity_expired")
        if image is None or image.shape != self.previous.shape or image.dtype != np.uint8:
            return self.invalid(stamp, "image_invalid")
        next_points, status, _ = cv2.calcOpticalFlowPyrLK(self.previous, image, self.points, None)
        if next_points is None:
            return self.invalid(stamp, "flow_failed")
        back, back_status, _ = cv2.calcOpticalFlowPyrLK(image, self.previous, next_points, None)
        if back is None:
            return self.invalid(stamp, "flow_failed")
        fb = np.linalg.norm(back-self.points, axis=2).ravel()
        keep = (status.ravel() == 1) & (back_status.ravel() == 1) & np.isfinite(fb) & (fb <= c.max_fb_px)
        previous = self.points.reshape(-1, 2)[keep]
        current = next_points.reshape(-1, 2)[keep]
        if len(current) < c.min_points:
            return self.invalid(stamp, "feature_loss")
        h, inliers = cv2.findHomography(previous, current, cv2.RANSAC, c.ransac_px)
        if h is None or inliers is None or not np.isfinite(h).all():
            return self.invalid(stamp, "homography_failed")
        good = inliers.ravel().astype(bool)
        ratio = float(good.mean())
        if good.sum() < c.min_points or ratio < c.min_inlier_ratio:
            return self.invalid(stamp, "inliers_insufficient")
        hull_area = cv2.contourArea(cv2.convexHull(current[good].astype(np.float32)))
        if hull_area < 25:
            return self.invalid(stamp, "features_degenerate")
        center = cv2.perspectiveTransform(self.center.astype(np.float32).reshape(1, 1, 2), h).reshape(2)
        height, width = image.shape
        if not np.isfinite(center).all() or not (0 <= center[0] < width and 0 <= center[1] < height):
            return self.invalid(stamp, "target_outside_image")
        self.previous, self.points, self.center = image.copy(), current[good].astype(np.float32).reshape(-1, 1, 2), center
        self.stamp = stamp
        return self.output(stamp, "local_features", int(good.sum()), ratio, float(fb[keep].max()))
