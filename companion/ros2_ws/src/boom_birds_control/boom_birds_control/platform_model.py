# ******************************************************************************
# @file           : platform_model.py
# @brief          : 定义下视 AprilTag 降落板的观测数据与位姿约定。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

"""下视 AprilTag 降落板的观测数据与配置契约。

T_A_B 表示 B→A；T_body_platform 为平台→机体 FLU，板面 z=0、+z 朝向飞行器。
"""
from dataclasses import dataclass, field
import math

import numpy as np
import cv2

FAMILIES = {"tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
            "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
            "tag36h10": cv2.aruco.DICT_APRILTAG_36h10,
            "tag36h11": cv2.aruco.DICT_APRILTAG_36h11}


def transform(value, name="transform"):
    """校验并返回刚体、右手系的 4×4 变换 T_A_B（B→A）；失败抛 ValueError(name)。"""
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1., atol=1e-6)):
        raise ValueError(name)
    return t


@dataclass(frozen=True)
class BoardObservation:
    """一帧板观测；无效帧也保留结构，供状态和日志解释原因。

    T_body_platform 为平台→机体，板面 z=0、+z 朝向飞行器。
    """

    stamp: float  # 采样时刻（秒）
    board: str
    valid: bool
    reason: str
    T_body_platform: tuple = ()  # 平台→机体；无效观测为空
    tag_ids: tuple = ()  # 参与解算的已知 Tag ID
    reprojection_px: float = 0.  # PnP 重投影 RMS 误差（px）
    ambiguity_ratio: float = 0.  # 次优/最优解误差比，越大越不歧义
    min_edge_px: float = 0.  # 最小 Tag 图像边长（px）
    source: str = "image"
    frame: str = "body_flu"
    quality: dict = field(default_factory=dict)

    def pose(self):
        if not self.valid or self.frame != "body_flu":
            raise ValueError("invalid_observation")
        return transform(self.T_body_platform, "observation_pose")


def validateProfile(p, *, test_only=False):
    """校验降落配置；失败抛 ValueError(错误码)。test_only 时跳过硬件证据检查。"""
    if p.get("schema") != 1 or type(p.get("synthetic")) is not bool:
        raise ValueError("profile_schema")
    from .flight_trial import flightTrialApproved
    trial = flightTrialApproved(p)
    # 合成数据只允许 test_only 或显式试飞许可。
    if p["synthetic"] and not test_only and not trial:
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
        # Tag 必须共面于平台 z=0 且 +z 朝上；size_m 是印刷黑边边长。
        t = transform(tag["T_platform_tag"], "tag_layout")
        if not np.allclose(t[2], [0, 0, 1, 0], atol=1e-6):
            raise ValueError("board_must_be_planar_z_up")
    if len(ids) != len(set(ids)): raise ValueError("duplicate_tag_id")
    # 落点在平台系板面上；target_yaw_rad 为平台系绕 +z 的目标机头朝向。
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
        # 七类生产证据文件必须存在；显式试飞许可可替代。
        required = ("intrinsics", "camera_extrinsics", "board_measurement", "sampling_time",
                    "range_sensor", "px4_velocity_estimator", "near_ground")
        evidence = p.get("evidence", {})
        from pathlib import Path
        if not trial and (p.get("hardware_verified") is not True or any(
                not isinstance(evidence.get(key), str) or not evidence[key]
                or not Path(evidence[key]).is_file() for key in required)):
            raise ValueError("hardware_evidence_missing")
