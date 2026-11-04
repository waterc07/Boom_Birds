"""Depth/CameraInfo/VIO pairing for the task-local corridor check."""
from collections import deque
from dataclasses import replace
import math
import time
import numpy as np
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from boom_birds_control.attitude_control import rotation
from boom_birds_control.runtime_config import DEFAULTS
from .task_depth_guard import DepthCorridor, executionDepthEnvelope


class TaskDepthBridge:
    def __init__(self, node, config):
        self.node,self.config = node,config
        self.guard = DepthCorridor(config)
        self.cameras = deque(maxlen=100)
        self.infos = deque(maxlen=16)
        self.pending = None
        self.bridge = CvBridge()
        node.declare_parameter("depth_roi", False)
        node.declare_parameter("depth_roi_speed_limit_m_s", 1.0)
        self.roiPublisher = None
        self.roiSpeedLimit = float(node.get_parameter("depth_roi_speed_limit_m_s").value)
        if not math.isfinite(self.roiSpeedLimit) or self.roiSpeedLimit <= 0:
            raise ValueError("depth_roi_speed_limit_m_s")
        if node.get_parameter("depth_roi").value:
            self.roiPublisher = node.create_publisher(
                PolygonStamped, DEFAULTS.depth_topic + "/execution_envelope", 1)
        node.create_subscription(PoseStamped, DEFAULTS.camera_pose_topic, self.onCamera,50)
        node.create_subscription(CameraInfo, DEFAULTS.camera_info_topic, self.onInfo,10)
        node.create_subscription(Image, DEFAULTS.depth_topic, self.onDepth,1)

    def onCamera(self, msg):
        p,q = msg.pose.position,msg.pose.orientation
        try:
            if msg.header.frame_id != self.node.config.world_frame:
                raise ValueError("depth_camera_world_frame")
            t = np.eye(4)
            t[:3,:3] = rotation((q.x,q.y,q.z,q.w))
            t[:3,3] = (p.x,p.y,p.z)
            if not np.isfinite(t).all():
                raise ValueError("depth_camera_pose")
            stamp = self.node.stamp(msg)
            if self.cameras and stamp <= self.cameras[-1][0]:
                self.cameras.clear()
                raise ValueError("depth_camera_reordered")
            self.cameras.append((stamp,t))
            self.publishDemand(stamp, t, msg.header.stamp)
        except ValueError:
            self.guard.invalidate("depth_camera_pose")

    def publishDemand(self, stamp, worldCamera, rosStamp):
        if self.roiPublisher is None:
            return
        # 空包络要求接收端全幅回退；不沿用上一条有效直行需求。
        msg = PolygonStamped()
        msg.header.stamp, msg.header.frame_id = rosStamp, DEFAULTS.camera_frame
        node = self.node
        poses = node.tracker.poses
        if poses:
            body = min(poses, key=lambda item: abs(item[0] - stamp))
            now = node.rosNow()
            if (abs(body[0] - stamp) <= self.config.pair_tolerance_s
                    and 0 <= now - stamp <= self.config.depth_timeout_s
                    and 0 <= now - body[0] <= self.config.depth_timeout_s
                    and self._forwardDemand(body[2])):
                worldBody = np.eye(4)
                worldBody[:3, :3], worldBody[:3, 3] = body[2], body[1]
                try:
                    points = executionDepthEnvelope(
                        self.config, np.linalg.inv(worldBody) @ worldCamera, self.roiSpeedLimit)
                    msg.polygon.points = [Point32(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                                          for p in points]
                except (ValueError, np.linalg.LinAlgError):
                    pass
        self.roiPublisher.publish(msg)

    def _forwardDemand(self, worldBodyRotation):
        from .lifecycle import State
        fsm = self.node.fsm
        if (fsm.state != State.EXECUTING or fsm.avoidance.active
                or fsm.stageIndex < 0 or fsm.phase == "TAG_ACQUIRE"
                or abs(fsm.segmentYawGoal - fsm.segmentYawStart) > 1e-6
                or self.node.control is None):
            return False
        delta = worldBodyRotation.T @ (np.asarray(fsm.segmentGoal) - fsm.segmentStart)
        v = self.node.control.control_velocity_world
        velocity = worldBodyRotation.T @ np.array([v.x, v.y, v.z])
        return bool(np.isfinite(delta).all() and np.isfinite(velocity).all()
            and delta[0] >= 0 and np.max(np.abs(delta[1:])) <= 1e-6
            and velocity[0] >= -1e-6 and np.linalg.norm(velocity) <= self.roiSpeedLimit
            and np.max(np.abs(velocity[1:])) <= 1e-6
            and fsm.task["max_speed_m_s"] <= self.roiSpeedLimit)

    def onInfo(self, msg):
        k = np.asarray(msg.k,float).reshape(3,3)
        p = np.asarray(msg.p,float).reshape(3,4)
        if (msg.header.frame_id != DEFAULTS.camera_frame or not np.isfinite(k).all()
                or not np.isfinite(p).all() or not np.allclose(p[:,:3],k)
                or not np.allclose(p[:,3],0.) or msg.width <= 0 or msg.height <= 0):
            self.infos.clear()
            self.guard.invalidate("depth_camera_info")
            return
        self.infos.append((self.node.stamp(msg),msg.width,msg.height,k))

    def onDepth(self, msg):
        if msg.encoding != "32FC1" or msg.header.frame_id != DEFAULTS.camera_frame:
            self.pending = None
            self.guard.invalidate("depth_encoding_or_frame")
            return
        self.pending = msg

    def evaluate(self):
        node,c = self.node,self.config
        now = node.rosNow()
        msg = self.pending
        if msg is not None:
            stamp = node.stamp(msg)
            if not 0 <= now-stamp <= c.depth_timeout_s:
                self.guard.invalidate("depth_stale_or_future")
                self.pending = None
            elif not self.infos or not self.cameras or not node.tracker.poses:
                self.guard.invalidate("depth_pair_missing")
            else:
                info = min(self.infos,key=lambda item:abs(item[0]-stamp))
                camera = min(self.cameras,key=lambda item:abs(item[0]-stamp))
                body = min(node.tracker.poses,key=lambda item:abs(item[0]-stamp))
                difference = max(abs(item[0]-stamp) for item in (info,camera,body))
                if difference > c.pair_tolerance_s:
                    self.guard.invalidate("depth_pair_missing")
                elif not all(0 <= now-item[0] <= c.depth_timeout_s for item in (info,camera,body)):
                    self.guard.invalidate("depth_pair_stale")
                elif (msg.width,msg.height) != info[1:3]:
                    self.guard.invalidate("depth_info_size")
                    self.pending = None
                else:
                    worldBody = np.eye(4)
                    worldBody[:3,:3],worldBody[:3,3] = body[2],body[1]
                    bodyCamera = np.linalg.inv(worldBody) @ camera[1]
                    status = node.control
                    speed = node.fsm.task["max_speed_m_s"]
                    if status is not None:
                        v = status.control_velocity_world
                        speed = max(speed,math.hypot(v.x,v.y))
                    try:
                        depth = self.bridge.imgmsg_to_cv2(msg,desired_encoding="32FC1")
                        ground = node.fsm.groundZ if node.fsm.groundZ is not None else float(body[1][2])
                        self.guard.observe(depth,stamp,info[3],bodyCamera,worldBody,groundZ=ground,speed=speed)
                    except (ValueError,TypeError,RuntimeError):
                        self.guard.invalidate("depth_wire_invalid")
                    self.pending = None
        report = self.guard.fresh(now)
        return replace(report,stamp=time.monotonic()-(now-report.stamp))
