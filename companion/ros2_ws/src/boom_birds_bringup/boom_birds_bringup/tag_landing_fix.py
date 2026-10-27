"""Pair Tag, VIO and range samples without renewing their sample times."""
from collections import deque
import math
import numpy as np
from boom_birds_control.attitude_control import rotation
from boom_birds_control.platform_model import validateProfile, transform
from .tag_flight_task import LandingFix


class TagLandingFix:
    def __init__(self, profile, *, testOnly=False, pairTolerance=.03):
        validateProfile(profile, test_only=testOnly)
        from boom_birds_control.platform_landing import LandingConfig
        LandingConfig(**profile["guidance"])
        self.profile = profile
        self.testOnly = bool(testOnly)
        self.pairTolerance = pairTolerance
        self.poses = deque(maxlen=100)
        self.ranges = deque(maxlen=100)
        self.lastStamp = None
        self.lastTarget = None
        self.fix = None
        self.reason = "tag_missing"
        self.rangeMount = transform(profile["range"]["T_body_sensor"])
        if not np.allclose(self.rangeMount[:3, 2], [0.,0.,-1.], atol=1e-6):
            raise ValueError("range_beam_must_point_body_down")
        self.lastRange = self.lastBodyRotation = None
        self.pending = None
        self.generation = 0

    def clear(self):
        self.generation += 1
        self.poses.clear()
        self.ranges.clear()
        self.lastStamp = self.lastTarget = self.fix = self.pending = None
        self.lastRange = self.lastBodyRotation = None
        self.reason = "reset"

    def addPose(self, stamp, position, quaternion):
        if (not math.isfinite(stamp) or len(position) != 3 or not np.isfinite(position).all()):
            raise ValueError("tag_vio_pose")
        r = rotation(quaternion)
        if self.poses and stamp <= self.poses[-1][0]:
            self.clear()
            raise ValueError("tag_vio_clock_reset")
        self.poses.append((stamp, np.asarray(position), r))

    def addRange(self, stamp, distance, minimum, maximum, frame):
        valid = (frame == "range_sensor" and all(math.isfinite(v) for v in (stamp, distance, minimum, maximum))
                 and 0 <= minimum < maximum and minimum <= distance <= maximum)
        if not valid or self.ranges and stamp <= self.ranges[-1][0]:
            self.fix = None
            self.ranges.clear()
            self.reason = "tag_range_invalid"
            return
        self.ranges.append((stamp, distance))

    def submit(self, observation):
        self.pending = observation
        # An invalid/new image must not leave a previously accepted fix active.
        self.fix = None

    def evaluate(self, now):
        o = self.pending
        g, q = self.profile["guidance"], self.profile["quality"]
        def reject(reason):
            self.fix, self.reason = None, reason
            return None
        if o is None or type(o.valid) is not bool or not o.valid:
            return reject("tag_missing")
        if not math.isfinite(o.stamp) or not 0 <= now-o.stamp <= g["observation_timeout_s"]:
            return reject("tag_stale")
        if self.lastStamp is not None and o.stamp < self.lastStamp:
            return reject("tag_reordered")
        if o.board != self.profile["board"]["name"] or o.frame != "body_flu":
            return reject("tag_identity")
        ids = o.tag_ids
        known = {tag["id"] for tag in self.profile["board"]["tags"]}
        if (not isinstance(ids, (list, tuple)) or any(type(i) is not int for i in ids)
                or len(ids) < q["min_tags"] or len(set(ids)) != len(ids) or not set(ids) <= known):
            return reject("tag_identity")
        if (not isinstance(o.quality, dict) or type(o.quality.get("planar_distinct", True)) is not bool
                or any(type(v) not in (int, float) or not math.isfinite(v)
                       for v in (o.reprojection_px, o.ambiguity_ratio, o.min_edge_px))
                or not 0 <= o.reprojection_px <= q["max_reprojection_px"]
                or o.min_edge_px < q["min_edge_px"]
                or o.quality.get("planar_distinct", True) and o.ambiguity_ratio < q["min_ambiguity_ratio"]):
            return reject("tag_quality")
        try:
            bodyBoard = o.pose()
        except (ValueError, TypeError):
            return reject("tag_pose")
        if not self.poses or not self.ranges:
            return reject("tag_pair_missing")
        pose = min(self.poses, key=lambda item: abs(item[0]-o.stamp))
        ranging = min(self.ranges, key=lambda item: abs(item[0]-o.stamp))
        if max(abs(pose[0]-o.stamp), abs(ranging[0]-o.stamp)) > self.pairTolerance:
            return reject("tag_pair_missing")
        if (not 0 <= now-pose[0] <= g["telemetry_timeout_s"]
                or not 0 <= now-ranging[0] <= g["range_timeout_s"]):
            return reject("tag_pair_stale")
        _, position, r = pose
        distance = ranging[1]
        if not g["range_min_m"] <= distance <= g["range_max_m"]:
            return reject("tag_range_blind")
        if r[2, 2] < math.cos(g["max_tilt_rad"]):
            return reject("tag_vehicle_tilt")
        if float((r @ bodyBoard[:3, 2])[2]) < math.cos(g["max_tilt_rad"]):
            return reject("tag_board_tilt")
        point = (bodyBoard @ [*self.profile["board"]["landing_point_m"], 1.])[:3]
        target = position+r @ point
        height = -float((r @ (self.rangeMount[:3, 3]+self.rangeMount[:3, 2]*distance))[2])
        if height <= 0 or abs(height-(position[2]-target[2])) > g["range_pose_tolerance_m"]:
            return reject("tag_range_pose_disagreement")
        if (self.lastTarget is not None and o.stamp != self.lastStamp
                and o.stamp-self.lastStamp <= g["observation_timeout_s"]):
            if np.linalg.norm(target-self.lastTarget) > g["max_pose_jump_m"]:
                return reject("tag_target_jump")
            if abs(distance-self.lastRange) > g["max_range_jump_m"]:
                return reject("tag_range_jump")
            angle = math.acos(float(np.clip((np.trace(self.lastBodyRotation.T @ bodyBoard[:3,:3])-1)/2, -1., 1.)))
            if angle > g["max_rotation_jump_rad"]:
                return reject("tag_rotation_jump")
        self.lastRange = distance
        self.lastBodyRotation = bodyBoard[:3,:3].copy()
        self.lastStamp, self.lastTarget = o.stamp, target.copy()
        self.fix = LandingFix(o.stamp, tuple(target), height, tag_ids=tuple(o.tag_ids))
        self.reason = "ok"
        return self.fix
