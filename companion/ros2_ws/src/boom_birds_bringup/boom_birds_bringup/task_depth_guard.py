"""Forward depth corridor; unknown samples never authorize translation."""
from dataclasses import dataclass
import math
from pathlib import Path
import numpy as np
import yaml
from boom_birds_control.platform_model import transform


@dataclass(frozen=True)
class ObstacleConfig:
    stop_distance_m: float = .7
    emergency_distance_m: float = .25
    half_width_m: float = .2
    half_height_m: float = .18
    ground_clearance_m: float = .05
    ground_tolerance_m: float = .02
    depth_timeout_s: float = .3
    pair_tolerance_s: float = .03
    min_depth_m: float = .2
    max_depth_m: float = 5.
    reaction_s: float = .3
    braking_accel_m_s2: float = .3
    body_front_m: float = .15
    clearance_m: float = .25
    brake_timeout_s: float = 2.
    settle_s: float = .3
    observation_s: float = .3
    observe_timeout_s: float = 1.5
    stage_timeout_s: float = 3.
    detour_side_m: float = .4
    detour_forward_m: float = .2
    pass_margin_m: float = .3
    breadcrumb_spacing_m: float = .1
    breadcrumb_tolerance_m: float = .025
    breadcrumb_capacity: int = 256

    def __post_init__(self):
        if any(type(v) not in (int,float) or not math.isfinite(v) or v <= 0 for v in self.__dict__.values()):
            raise ValueError("obstacle_positive_values")
        if (type(self.breadcrumb_capacity) is not int or self.breadcrumb_capacity < 8
                or self.emergency_distance_m >= self.stop_distance_m
                or self.min_depth_m >= self.max_depth_m
                or self.pair_tolerance_s >= self.depth_timeout_s
                or self.ground_tolerance_m >= self.ground_clearance_m
                or self.observation_s >= self.observe_timeout_s):
            raise ValueError("obstacle_config_bounds")

    def distance(self, speed):
        return max(self.stop_distance_m, self.body_front_m+self.clearance_m
                   + speed*self.reaction_s+speed*speed/(2*self.braking_accel_m_s2))


def loadObstacleConfig(path, *, live=False):
    p = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(p,dict) or p.get("schema") != 1:
        raise ValueError("obstacle_profile_schema")
    config = ObstacleConfig(**p["guard"])
    if live:
        required = ("body_envelope","stopping_distance","depth_geometry","depth_latency")
        evidence = p.get("evidence",{})
        if p.get("hardware_verified") is not True or any(
                not isinstance(evidence.get(k),str) or not Path(evidence[k]).is_file() for k in required):
            raise ValueError("obstacle_hardware_evidence_missing")
    return config


@dataclass(frozen=True)
class ObstacleReport:
    stamp: float
    state: str
    reason: str
    nearest_m: float | None = None
    emergency: bool = False


class DepthCorridor:
    def __init__(self, config):
        self.config = config
        self.lastStamp = None
        self.lastRejected = None
        self.report = ObstacleReport(-math.inf,"UNKNOWN","depth_missing")

    def invalidate(self, reason):
        self.report = ObstacleReport(-math.inf,"UNKNOWN",reason)
        return self.report

    def fresh(self, now):
        r = self.report
        if not math.isfinite(now) or not 0 <= now-r.stamp <= self.config.depth_timeout_s:
            return ObstacleReport(r.stamp,"UNKNOWN","depth_stale")
        return r

    def observe(self, depth, stamp, intrinsics, bodyCamera, bodyWorld, *, groundZ, speed):
        c = self.config
        if (not math.isfinite(stamp) or self.lastStamp is not None and stamp <= self.lastStamp):
            return self.invalidate("depth_reordered")
        self.lastStamp = stamp
        def result(state,reason,nearest=None,emergency=False):
            self.report = ObstacleReport(stamp,state,reason,nearest,emergency)
            return self.report
        try:
            tc = transform(bodyCamera)
            tw = transform(bodyWorld)
        except (ValueError,TypeError):
            return result("UNKNOWN","depth_transform")
        k = np.asarray(intrinsics,dtype=float)
        z = np.asarray(depth)
        if (z.ndim != 2 or not z.size or z.size > 4_000_000 or z.dtype.kind != "f"
                or k.shape != (3,3) or not np.isfinite(k).all() or min(k[0,0],k[1,1]) <= 0
                or not np.allclose(k[2],[0,0,1]) or abs(k[0,1])+abs(k[1,0]) > 1e-8
                or not all(math.isfinite(v) for v in (groundZ,speed)) or speed < 0):
            return result("UNKNOWN","depth_geometry")
        if tc[0,2] < .8:
            return result("UNKNOWN","depth_camera_not_forward")
        distance = c.distance(speed)
        # Project the full inflated body cross-section at the stopping distance.
        if tw[2,2] <= .8:
            return result("UNKNOWN","depth_vehicle_tilt")
        corners = []
        for y in (-c.half_width_m,c.half_width_m):
            floorZ = (groundZ-c.ground_clearance_m+c.ground_tolerance_m
                      -tw[2,3]-tw[2,0]*distance-tw[2,1]*y)/tw[2,2]
            bottom = max(-c.half_height_m,floorZ)
            if bottom >= c.half_height_m:
                return result("UNKNOWN","corridor_below_floor")
            corners.extend(([distance,y,bottom],[distance,y,c.half_height_m]))
        corners = np.asarray(corners)
        cameraCorners = (corners-tc[:3,3]) @ tc[:3,:3]
        if np.any(cameraCorners[:,2] <= 0):
            return result("UNKNOWN","corridor_outside_fov")
        uv = cameraCorners @ k.T
        uv = uv[:,:2]/uv[:,2:]
        width,height = z.shape[1],z.shape[0]
        x0,y0 = np.floor(uv.min(axis=0)).astype(int)
        x1,y1 = np.ceil(uv.max(axis=0)).astype(int)
        if x0 < 0 or y0 < 0 or x1 >= width or y1 >= height or x1 <= x0 or y1 <= y0:
            return result("UNKNOWN","corridor_outside_fov")
        rows,cols = np.indices(z.shape)
        rays = np.stack(((cols-k[0,2])/k[0,0],(rows-k[1,2])/k[1,1],np.ones_like(z)),axis=-1)
        finite = np.isfinite(z) & (z > 0)
        points = (rays*np.where(finite,z,0)[...,None]) @ tc[:3,:3].T+tc[:3,3]
        worldHeight = points @ tw[2,:3]+tw[2,3]
        floor = worldHeight <= groundZ-c.ground_clearance_m+c.ground_tolerance_m
        corridor = (finite & ~floor & (points[...,0] > 0) & (points[...,0] <= distance)
                    & (np.abs(points[...,1]) <= c.half_width_m)
                    & (np.abs(points[...,2]) <= c.half_height_m))
        if corridor.any():
            nearest = float(points[...,0][corridor].min())
            return result("BLOCKED","obstacle_in_corridor",nearest,nearest <= c.emergency_distance_m)
        roi = z[y0:y1+1,x0:x1+1]
        rr,cc = np.mgrid[y0:y1+1,x0:x1+1]
        polygon = uv[[0,1,3,2]]
        signs = []
        for start,end in zip(polygon,np.roll(polygon,-1,axis=0)):
            edge = end-start
            signs.append(edge[0]*(rr-start[1])-edge[1]*(cc-start[0]))
        signs = np.stack(signs)
        required = np.all(signs >= -1e-6,axis=0) | np.all(signs <= 1e-6,axis=0)
        good = np.isfinite(roi) & (roi >= c.min_depth_m) & (roi <= c.max_depth_m)
        # Only the projected corridor polygon is required; its bounding box includes floor pixels.
        if not required.any() or not good[required].all():
            finiteRoi = roi[required & np.isfinite(roi)]
            self.lastRejected = dict(stamp=stamp,invalid_pixels=int((required & ~good).sum()),
                required_pixels=int(required.sum()),minimum_m=float(finiteRoi.min()) if finiteRoi.size else None,
                maximum_m=float(finiteRoi.max()) if finiteRoi.size else None,body_height_m=float(tw[2,3]-groundZ))
            return result("UNKNOWN","corridor_depth_unknown")
        return result("CLEAR","corridor_clear")
