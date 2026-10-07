"""Downward AprilTag board detection and planar PnP."""
import math
import numpy as np
import cv2
from boom_birds_control.platform_model import (
    FAMILIES, transform, BoardObservation, validate_profile)

class BoardDetector:
    def __init__(self, profile, *, test_only=False):
        self.profile = profile
        self.test_only = test_only
        self.validate(profile, test_only=test_only)
        self.board = profile["board"]
        self.camera = profile["camera"]
        self.K = np.asarray(self.camera["K"], float)
        self.D = np.asarray(self.camera["D"], float)
        self.T_B_C = transform(self.camera["T_body_camera"])
        self.dictionary = cv2.aruco.getPredefinedDictionary(FAMILIES[self.board["family"]])
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.tags = {tag["id"]: tag for tag in self.board["tags"]}

    validate = staticmethod(validate_profile)

    @staticmethod
    def tag_corners(tag):
        # Printed marker: TL, TR, BR, BL; platform/tag +z faces the aircraft.
        s = tag["size_m"] / 2
        points = np.array([[-s, s, 0, 1], [s, s, 0, 1],
                           [s, -s, 0, 1], [-s, -s, 0, 1]])
        return (transform(tag["T_platform_tag"]) @ points.T).T[:, :3]

    def observe(self, image, stamp, source="image"):
        if not math.isfinite(stamp): raise ValueError("sample_stamp")
        if image is None or image.shape[:2] != tuple(reversed(self.camera["image_size"])):
            return self.invalid(stamp, "image_size", source)
        corners, ids, rejected = cv2.aruco.detectMarkers(
            image, self.dictionary, parameters=self.parameters)
        detections = [] if ids is None else [(int(i), c.reshape(4, 2))
                                             for i, c in zip(ids.ravel(), corners)]
        return self.estimate(detections, stamp, source, len(rejected))

    def invalid(self, stamp, reason, source="image", ids=()):
        return BoardObservation(stamp, self.board["name"], False, reason,
                                tag_ids=tuple(ids), source=source)

    def estimate(self, detections, stamp, source="corners", rejected=0):
        if not math.isfinite(stamp): raise ValueError("sample_stamp")
        ids = [i for i, _ in detections]
        if len(set(ids)) != len(ids): return self.invalid(stamp, "duplicate_detection", source, ids)
        known = [(i, np.asarray(c, float)) for i, c in detections if i in self.tags]
        if len(known) < self.profile["quality"]["min_tags"]:
            return self.invalid(stamp, "board_not_visible", source, ids)
        if any(c.shape != (4, 2) or not np.isfinite(c).all() for _, c in known):
            return self.invalid(stamp, "corner_invalid", source, ids)
        objects = np.concatenate([self.tag_corners(self.tags[i]) for i, _ in known])
        pixels = np.concatenate([c for _, c in known])
        edge = min(np.linalg.norm(c - np.roll(c, 1, axis=0), axis=1).min() for _, c in known)
        if edge < self.profile["quality"]["min_edge_px"]:
            return self.invalid(stamp, "tag_too_small", source, ids)
        result = cv2.solvePnPGeneric(objects, pixels, self.K, self.D, flags=cv2.SOLVEPNP_IPPE)
        solutions = []
        for rvec, tvec in zip(result[1], result[2]):
            r = cv2.Rodrigues(rvec)[0]
            if np.min((r @ objects.T + tvec).T[:, 2]) <= 0: continue
            # Reject the back of the configured plate.
            if float(r[:, 2] @ (-tvec.ravel())) <= 0: continue
            projected = cv2.projectPoints(objects, rvec, tvec, self.K, self.D)[0].reshape(-1, 2)
            error = float(np.sqrt(np.mean(np.sum((projected - pixels)**2, axis=1))))
            solutions.append((error, r, tvec.ravel()))
        solutions.sort(key=lambda item: item[0])
        if not solutions: return self.invalid(stamp, "pnp_failed", source, ids)
        error, r, t = solutions[0]
        ratio = solutions[1][0] / max(error, 1e-6) if len(solutions) > 1 else 1e6
        # Near-identical solutions are equivalent; compare their actual geometry.
        distinct = len(solutions) > 1 and (np.linalg.norm(solutions[1][2] - t) > .01
                    or np.linalg.norm(solutions[1][1] - r) > .05)
        q = self.profile["quality"]
        if error > q["max_reprojection_px"]:
            return self.invalid(stamp, "reprojection", source, ids)
        if distinct and ratio < q["min_ambiguity_ratio"]:
            return self.invalid(stamp, "planar_ambiguity", source, ids)
        pose = np.eye(4); pose[:3, :3] = r; pose[:3, 3] = t
        # PnP 输出平台→相机；安装外参左乘后，控制器收到平台→机体。
        pose = self.T_B_C @ pose
        return BoardObservation(stamp, self.board["name"], True, "ok",
            tuple(tuple(float(x) for x in row) for row in pose), tuple(i for i, _ in known),
            error, ratio, float(edge), source,
            quality={"rejected_candidates": rejected, "unknown_ids": [i for i in ids if i not in self.tags],
                     "synthetic": self.profile["synthetic"], "planar_distinct": bool(distinct), "detector": "OpenCV_AprilTag_dictionary"})
