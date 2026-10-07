"""Downward AprilTag board observation. T_A_B maps B coordinates into A."""
from dataclasses import dataclass, field
import math
import numpy as np
import cv2

FAMILIES = {"tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
            "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
            "tag36h10": cv2.aruco.DICT_APRILTAG_36h10,
            "tag36h11": cv2.aruco.DICT_APRILTAG_36h11}


def transform(value, name="transform"):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1., atol=1e-6)):
        raise ValueError(name)
    return t


@dataclass(frozen=True)
class BoardObservation:
    stamp: float
    board: str
    valid: bool
    reason: str
    T_body_platform: tuple = ()
    tag_ids: tuple = ()
    reprojection_px: float = 0.
    ambiguity_ratio: float = 0.
    min_edge_px: float = 0.
    source: str = "image"
    frame: str = "body_flu"
    quality: dict = field(default_factory=dict)

    def pose(self):
        if not self.valid or self.frame != "body_flu":
            raise ValueError("invalid_observation")
        return transform(self.T_body_platform, "observation_pose")


def validate_profile(p, *, test_only=False):
    if p.get("schema") != 1 or type(p.get("synthetic")) is not bool:
        raise ValueError("profile_schema")
    if p["synthetic"] and not test_only:
        raise ValueError("synthetic_profile_requires_test_only")
    for part in ("camera", "board", "guidance"):
        if part not in p: raise ValueError("missing_" + part)
    camera, board = p["camera"], p["board"]
    k = np.asarray(camera["K"], float)
    d = np.asarray(camera["D"], float)
    size = camera["image_size"]
    if (k.shape != (3, 3) or not np.isfinite(k).all() or k[0, 0] <= 0
            or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1])
            or d.shape not in ((4,), (5,)) or not np.isfinite(d).all()
            or len(size) != 2 or any(type(x) is not int or x <= 0 for x in size)):
        raise ValueError("camera_intrinsics")
    transform(camera["T_body_camera"], "camera_extrinsics")
    if board["family"] not in FAMILIES or not board["name"] or not board["tags"]:
        raise ValueError("board_family_or_layout")
    ids = []
    dictionary = cv2.aruco.getPredefinedDictionary(FAMILIES[board["family"]])
    for tag in board["tags"]:
        if (type(tag["id"]) is not int or not 0 <= tag["id"] < len(dictionary.bytesList)
                or not math.isfinite(tag["size_m"]) or tag["size_m"] <= 0):
            raise ValueError("tag_id_or_size")
        ids.append(tag["id"])
        t = transform(tag["T_platform_tag"], "tag_layout")
        if not np.allclose(t[2], [0, 0, 1, 0], atol=1e-6):
            raise ValueError("board_must_be_planar_z_up")
    if len(ids) != len(set(ids)): raise ValueError("duplicate_tag_id")
    landing = np.asarray(board["landing_point_m"], float)
    if landing.shape != (3,) or not np.isfinite(landing).all() or abs(landing[2]) > 1e-6:
        raise ValueError("landing_point")
    if not math.isfinite(board["target_yaw_rad"]): raise ValueError("target_yaw")
    q = p["quality"]
    for name in ("max_reprojection_px", "min_edge_px", "min_ambiguity_ratio"):
        if not math.isfinite(q[name]) or q[name] <= 0: raise ValueError(name)
    if type(q["min_tags"]) is not int or not 1 <= q["min_tags"] <= len(ids):
        raise ValueError("min_tags")
    if not test_only:
        required = ("intrinsics", "camera_extrinsics", "board_measurement", "sampling_time",
                    "range_sensor", "px4_velocity_estimator", "near_ground")
        evidence = p.get("evidence", {})
        from pathlib import Path
        if p.get("hardware_verified") is not True or any(
                not isinstance(evidence.get(key), str) or not evidence[key]
                or not Path(evidence[key]).is_file() for key in required):
            raise ValueError("hardware_evidence_missing")
