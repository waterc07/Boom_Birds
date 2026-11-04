"""VIO 机体里程计到 MAVROS ODOMETRY；只供速度融合，不融合位置/高度/yaw。"""
from copy import deepcopy
import math
import numpy as np
from .attitude_control import finiteVector, rotation


def externalVisionOdometry(message, now, *, worldFrame="global", bodyFrame="body", maxAge=.2):
    stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
    if not math.isfinite(stamp) or stamp<=0 or not 0<=now-stamp<=maxAge:
        raise ValueError("external_vision_stale_or_future")
    if message.header.frame_id!=worldFrame or message.child_frame_id!=bodyFrame:
        raise ValueError("external_vision_frames")
    p=message.pose.pose.position;q=message.pose.pose.orientation
    finiteVector((p.x,p.y,p.z),3);rotation((q.x,q.y,q.z,q.w))
    for vector in (message.twist.twist.linear,message.twist.twist.angular):
        finiteVector((vector.x,vector.y,vector.z),3)
    result=deepcopy(message)
    # VIO 世界系只要求 z 向上；LOCAL_FRD 不表示磁北。twist 已为机体 FLU。
    result.header.frame_id="odom";result.child_frame_id="base_link"
    # 当前 pose_adapter 的 twist covariance 沿用 pose covariance，单位不适用于速度。
    # 这里显式采用未标定噪声假设，不能作为实测置信度；仅允许 EKF2_EV_CTRL=4。
    result.pose.covariance=np.diag((.04,.04,.04,.04,.04,.04)).reshape(-1).tolist()
    result.twist.covariance=np.diag((.04,.04,.04,1.,1.,1.)).reshape(-1).tolist()
    return result
