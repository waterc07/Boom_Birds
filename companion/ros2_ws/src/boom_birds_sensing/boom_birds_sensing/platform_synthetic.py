"""TEST-ONLY：配置板在合成相机中的投影，不能作真实标定证据。"""
import cv2
import numpy as np
from .platform_observation import BoardDetector


def render_board(detector, body_pose):
    camera_pose = np.linalg.inv(detector.T_B_C) @ body_pose
    rvec = cv2.Rodrigues(camera_pose[:3, :3])[0]
    width, height = detector.camera["image_size"]
    image = np.full((height, width), 255, np.uint8)
    for tag in detector.board["tags"]:
        points = detector.tag_corners(tag)
        if np.min((camera_pose[:3, :3] @ points.T + camera_pose[:3, 3, None]).T[:, 2]) <= 0:
            continue
        pixels = cv2.projectPoints(points, rvec, camera_pose[:3, 3], detector.K, detector.D)[0].reshape(4, 2)
        marker = cv2.aruco.drawMarker(detector.dictionary, tag["id"], 160)
        homography = cv2.getPerspectiveTransform(
            np.float32([[0, 0], [159, 0], [159, 159], [0, 159]]), np.float32(pixels))
        image = np.minimum(image, cv2.warpPerspective(marker, homography, (width, height), borderValue=255))
    return image
