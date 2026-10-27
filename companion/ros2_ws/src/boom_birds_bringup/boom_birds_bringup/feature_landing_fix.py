"""Range/attitude validation of decoded-plane and continued-feature pixel targets."""
from dataclasses import dataclass
import math
import cv2
import numpy as np
from boom_birds_control.attitude_control import wrap
from boom_birds_sensing.platform_feature_track import LocalFeatureObservation,TrackConfig
from boom_birds_sensing.platform_observation import BoardDetector
from .tag_flight_task import LandingFix


@dataclass(frozen=True)
class MetricFeatureConfig:
    max_axis_scale_error: float = .15
    max_axis_nonorthogonality: float = .2
    max_metric_drift_m: float = .1
    max_yaw_drift_rad: float = .15
    max_seed_point_correction_m: float = .01

    def __post_init__(self):
        if any(type(x) not in (int,float) or not math.isfinite(x) or x<=0 for x in self.__dict__.values()):
            raise ValueError("metric_feature_limits")
        if max(self.max_axis_scale_error,self.max_axis_nonorthogonality)>=1:
            raise ValueError("metric_feature_geometry_limits")


class FeatureLandingFix:
    def __init__(self,tracker):
        self.tracker=tracker
        self.profile=tracker.profile
        self.trackConfig=TrackConfig(**self.profile.get("feature_tracking",{}))
        self.config=MetricFeatureConfig(**self.profile.get("feature_geometry",{}))
        self.K=np.array(self.profile["camera"]["K"],float)
        self.mount=np.array(self.profile["camera"]["T_body_camera"],float)
        self.tags={t["id"]:t for t in self.profile["board"]["tags"]}
        self.generation=getattr(tracker,"generation",0)
        self.reset()

    def reset(self):
        self.pending=self.anchor=self.fix=None
        self.lastStamp=self.lastRange=None
        self.reason="feature_missing"
        self.pnpCandidateCount=0
        self.pnpReprojection=None

    def submit(self,observation):
        self.pending=observation
        self.fix=None
        if not observation.valid:
            self.anchor=None
            self.reason=observation.reason

    def evaluate(self,now):
        def reject(reason):
            self.fix=None;self.reason=reason
            return None
        if getattr(self.tracker,"generation",0)!=self.generation:
            self.generation=self.tracker.generation
            self.reset()
            return reject("feature_frame_reset")
        f=self.pending
        if f is None or type(f.valid) is not bool or not f.valid:return reject("feature_missing")
        c,g,q=self.trackConfig,self.profile["guidance"],self.profile["quality"]
        scalar=(now,f.stamp,f.identity_stamp,f.identity_age_s,f.inlier_ratio,
                f.forward_backward_px,f.reference_span_m)
        if any(type(x) not in (int,float) or not math.isfinite(x) for x in scalar):
            return reject("feature_wire_invalid")
        if not 0<=now-f.stamp<=g["observation_timeout_s"]:return reject("feature_stale")
        if self.lastStamp is not None and f.stamp<self.lastStamp:return reject("feature_reordered")
        if (f.kind!="board_local_features_v1" or f.frame!="camera_pixels"
                or f.pixel_geometry!="undistorted" or f.metric_pose_available is not False
                or f.board!=self.profile["board"]["name"] or type(f.test_only) is not bool
                or f.test_only!=self.tracker.testOnly):
            return reject("feature_contract")
        ids=f.inherited_tag_ids
        if (not isinstance(ids,list) or not ids or len(ids)>len(self.tags)
                or any(type(i) is not int or i not in self.tags for i in ids)
                or len(set(ids))!=len(ids)):
            return reject("feature_identity")
        age=f.stamp-f.identity_stamp
        if (not 0<=now-f.identity_stamp<=c.max_identity_age_s or age<0
                or abs(f.identity_age_s-age)>1e-5):return reject("feature_identity_expired")
        if (type(f.feature_count) is not int or not c.min_points<=f.feature_count<=c.max_points
                or not c.min_inlier_ratio<=f.inlier_ratio<=1 or not 0<=f.forward_backward_px<=c.max_fb_px):
            return reject("feature_quality")
        refs=np.asarray(f.reference_pixels,float)
        pixel=np.asarray(f.landing_pixel,float)
        expectedSpan=min(t["size_m"] for t in self.tags.values())/2
        width,height=self.profile["camera"]["image_size"]
        if (refs.shape!=(3,2) or pixel.shape!=(2,) or not np.isfinite(refs).all()
                or not np.allclose(refs[0],pixel,atol=1e-5)
                or abs(f.reference_span_m-expectedSpan)>1e-8
                or not (0<=pixel[0]<width and 0<=pixel[1]<height)):
            return reject("feature_pixel_geometry")
        if not self.tracker.poses or not self.tracker.ranges:return reject("feature_pair_missing")
        pose=min(self.tracker.poses,key=lambda x:abs(x[0]-f.stamp))
        ranging=min(self.tracker.ranges,key=lambda x:abs(x[0]-f.stamp))
        if max(abs(x[0]-f.stamp) for x in (pose,ranging))>self.tracker.pairTolerance:
            return reject("feature_pair_missing")
        if any(not 0<=now-x[0]<=g["telemetry_timeout_s" if x is pose else "range_timeout_s"] for x in (pose,ranging)):
            return reject("feature_pair_stale")
        _,position,rotation=pose
        distance=ranging[1]
        if not g["range_min_m"]<=distance<=g["range_max_m"]:return reject("feature_range_blind")
        if rotation[2,2]<math.cos(g["max_tilt_rad"]):return reject("feature_vehicle_tilt")
        vector=rotation@(self.tracker.rangeMount[:3,3]+self.tracker.rangeMount[:3,2]*distance)
        planeZ=float(position[2]+vector[2]);heightAbove=-float(vector[2])
        if heightAbove<=0:return reject("feature_range_direction")
        if self.lastStamp is not None and f.stamp!=self.lastStamp and f.stamp-self.lastStamp<=g["range_timeout_s"]:
            if self.lastRange is not None and abs(distance-self.lastRange)>g["max_range_jump_m"]:
                return reject("feature_range_jump")
        cameraOrigin=position+rotation@self.mount[:3,3]
        cameraRotation=rotation@self.mount[:3,:3]
        def intersect(pixels):
            pixels=np.asarray(pixels,float).reshape(-1,2)
            rays=np.column_stack((pixels,np.ones(len(pixels))))@np.linalg.inv(self.K).T
            rays=rays@cameraRotation.T
            if not np.isfinite(rays).all() or np.any(rays[:,2]>=-.1):
                raise ValueError("feature_plane_ray")
            distances=(planeZ-cameraOrigin[2])/rays[:,2]
            if np.any(distances<=0) or np.any(distances>g["range_max_m"]*2):
                raise ValueError("feature_plane_intersection")
            return cameraOrigin+distances[:,None]*rays
        try:
            current=intersect(refs)
        except ValueError as exc:return reject(str(exc))
        span=f.reference_span_m
        xAxis,yAxis=(current[1]-current[0])/span,(current[2]-current[0])/span
        scaleError=max(abs(np.linalg.norm(xAxis)-1),abs(np.linalg.norm(yAxis)-1))
        if (scaleError>self.config.max_axis_scale_error
                or abs(xAxis@yAxis)>self.config.max_axis_nonorthogonality
                or np.cross(xAxis,yAxis)[2]<=0):
            return reject("feature_yaw_unobservable")
        yaw=math.atan2(xAxis[1],xAxis[0])
        objectsTracked=np.asarray(f.object_points_m,float)
        pixelsTracked=np.asarray(f.image_points,float)
        pointIds=f.point_ids
        if (objectsTracked.shape!=(f.feature_count,3) or pixelsTracked.shape!=(f.feature_count,2)
                or not np.isfinite(objectsTracked).all() or not np.isfinite(pixelsTracked).all()
                or not ((pixelsTracked[:,0]>=0)&(pixelsTracked[:,0]<width)&
                        (pixelsTracked[:,1]>=0)&(pixelsTracked[:,1]<height)).all()
                or not np.allclose(objectsTracked[:,2],0.,atol=1e-9)
                or not isinstance(pointIds,list) or len(pointIds)!=f.feature_count
                or any(type(i) is not int or not 0<=i<c.max_points for i in pointIds)
                or len(set(pointIds))!=len(pointIds)):
            return reject("feature_point_correspondences")
        provisionalObjects=objectsTracked.copy()
        if f.source=="local_features":
            if (self.anchor is None or f.identity_stamp!=self.anchor["stamp"]
                    or tuple(ids)!=self.anchor["ids"]):
                return reject("feature_decode_required")
            for identity,point in zip(pointIds,objectsTracked):
                if identity not in self.anchor["objects"] or not np.allclose(point,self.anchor["source_objects"][identity],atol=1e-9):
                    return reject("feature_point_identity_changed")
            objectsTracked=np.array([self.anchor["objects"][identity] for identity in pointIds])
        elif f.source=="decoded_tag":
            try:
                seedRotation,seedTranslation=self.decodedPlane(f,intersect,position,rotation,planeZ)
                worldFeatures=intersect(pixelsTracked)
            except ValueError as exc:return reject(str(exc))
            calibrated=(worldFeatures[:,:2]-seedTranslation)@seedRotation
            objectsTracked=np.column_stack((calibrated,np.zeros(len(calibrated))))
            if np.max(np.linalg.norm(objectsTracked-provisionalObjects,axis=1))>self.config.max_seed_point_correction_m:
                return reject("feature_seed_metric_correction")
        else:
            return reject("feature_source")
        if np.linalg.eigvalsh(np.cov(objectsTracked[:,:2].T)).min()<(min(t["size_m"] for t in self.tags.values())*.03)**2:
            return reject("feature_object_points_degenerate")
        try:
            observedWorld=intersect(pixelsTracked)
        except ValueError as exc:return reject(str(exc))
        objectXY=objectsTracked[:,:2]
        centeredObjects=objectXY-objectXY.mean(axis=0)
        centeredWorld=observedWorld[:,:2]-observedWorld[:,:2].mean(axis=0)
        u,_,vt=np.linalg.svd(centeredObjects.T@centeredWorld)
        trackedRotation=vt.T@u.T
        if np.linalg.det(trackedRotation)<=0:return reject("feature_reflected_board")
        trackedTranslation=observedWorld[:,:2].mean(axis=0)-trackedRotation@objectXY.mean(axis=0)
        fittedTracked=np.column_stack((objectXY@trackedRotation.T+trackedTranslation,np.full(len(objectXY),planeZ)))
        trackedCamera=((fittedTracked-position)@rotation-self.mount[:3,3])@self.mount[:3,:3]
        if np.any(trackedCamera[:,2]<=0):return reject("feature_behind_camera")
        projected=trackedCamera@self.K.T
        projected=projected[:,:2]/projected[:,2,None]
        constrainedResidual=float(np.sqrt(np.mean(np.sum((projected-pixelsTracked)**2,axis=1))))
        if constrainedResidual>q["max_reprojection_px"]:return reject("feature_constrained_pnp_reprojection")
        targetXY=trackedRotation@np.asarray(self.profile["board"]["landing_point_m"][:2])+trackedTranslation
        trackedTarget=np.array([*targetXY,planeZ])
        trackedYaw=math.atan2(trackedRotation[1,0],trackedRotation[0,0])
        if (np.linalg.norm(current[0]-trackedTarget)>.02
                or abs(wrap(yaw-trackedYaw))>self.config.max_yaw_drift_rad):
            return reject("feature_pnp_reference_disagreement")
        try:
            result=cv2.solvePnPGeneric(np.ascontiguousarray(objectsTracked),
                np.ascontiguousarray(pixelsTracked),self.K,np.zeros(len(self.profile["camera"]["D"]),dtype=float),
                flags=cv2.SOLVEPNP_IPPE)
        except cv2.error:return reject("feature_pnp_failed")
        compatible=[]
        for rvec,tvec in zip(result[1],result[2]):
            if not np.isfinite(rvec).all() or not np.isfinite(tvec).all():continue
            cameraR=cv2.Rodrigues(rvec)[0]
            if np.any((objectsTracked@cameraR.T+tvec.ravel())[:,2]<=0):continue
            worldR=cameraRotation@cameraR
            worldOrigin=cameraOrigin+cameraRotation@tvec.ravel()
            worldTarget=worldOrigin+worldR@np.asarray(self.profile["board"]["landing_point_m"])
            candidateYaw=math.atan2(worldR[1,0],worldR[0,0])
            pixels=cv2.projectPoints(objectsTracked,rvec,tvec,self.K,
                np.zeros(len(self.profile["camera"]["D"]),dtype=float))[0].reshape(-1,2)
            error=float(np.sqrt(np.mean(np.sum((pixels-pixelsTracked)**2,axis=1))))
            if (error<=q["max_reprojection_px"] and worldR[2,2]>=math.cos(g["max_tilt_rad"])
                    and np.linalg.norm(worldTarget-trackedTarget)<=self.config.max_metric_drift_m
                    and abs(wrap(candidateYaw-trackedYaw))<=self.config.max_yaw_drift_rad):
                compatible.append(error)
        self.pnpCandidateCount=len(compatible)
        if not compatible:return reject("feature_pnp_prior_disagreement")
        self.pnpReprojection=min(compatible)
        current[0]=trackedTarget
        yaw=trackedYaw
        if f.source=="decoded_tag":
            if abs(f.stamp-f.identity_stamp)>1e-8:return reject("feature_decode_stamp")
            if not c.capture_min_m<=heightAbove<=c.capture_max_m:
                return reject("feature_outside_capture_height")
            try:
                orient,translation=self.decodedPlane(f,intersect,position,rotation,planeZ)
            except ValueError as exc:return reject(str(exc))
            anchorYaw=math.atan2(orient[1,0],orient[0,0])
            target=orient@np.asarray(self.profile["board"]["landing_point_m"][:2])+translation
            if (np.linalg.norm(current[0,:2]-target)>.02
                    or abs(wrap(yaw-anchorYaw))>self.config.max_yaw_drift_rad):
                return reject("feature_decode_reference")
            self.anchor=dict(stamp=f.identity_stamp,ids=tuple(ids),target=current[0].copy(),
                planeZ=planeZ,yaw=anchorYaw,objects={i:p.copy() for i,p in zip(pointIds,objectsTracked)},
                source_objects={i:p.copy() for i,p in zip(pointIds,provisionalObjects)})
            source="resolved_tag"
        elif f.source=="local_features":
            if (self.anchor is None or f.identity_stamp!=self.anchor["stamp"]
                    or tuple(ids)!=self.anchor["ids"]):
                return reject("feature_decode_required")
            if (abs(planeZ-self.anchor["planeZ"])>g["range_pose_tolerance_m"]
                    or np.linalg.norm(current[0]-self.anchor["target"])>self.config.max_metric_drift_m):
                return reject("feature_metric_drift")
            if abs(wrap(yaw-self.anchor["yaw"]))>self.config.max_yaw_drift_rad:
                return reject("feature_yaw_drift")
            source="features"
        else:return reject("feature_source")
        self.lastStamp,self.lastRange=f.stamp,distance
        self.fix=LandingFix(f.stamp,tuple(current[0]),heightAbove,source,f.identity_stamp,True,tuple(ids))
        self.reason="range_attitude_constrained_pnp" if source=="resolved_tag" else "local_feature_pnp"
        return self.fix


    def decodedPlane(self,f,intersect,position,rotation,planeZ):
        ids=f.inherited_tag_ids;q=self.profile["quality"]
        if not isinstance(f.seed_tag_pixels,list) or len(f.seed_tag_pixels)!=len(ids):
            raise ValueError("feature_decode_corners")
        objects=[];pixels=[];decoded=[]
        for entry in f.seed_tag_pixels:
            if not isinstance(entry,dict) or set(entry)!={"id","corners"}:
                raise ValueError("feature_decode_corners")
            identity=entry["id"]
            if type(identity) is not int or identity not in self.tags:raise ValueError("feature_identity")
            corners=np.asarray(entry["corners"],float)
            if corners.shape!=(4,2) or not np.isfinite(corners).all():
                raise ValueError("feature_decode_corners")
            if np.linalg.norm(corners-np.roll(corners,1,axis=0),axis=1).min()<q["min_edge_px"]:
                raise ValueError("feature_tag_too_small")
            decoded.append(identity)
            objects.extend(BoardDetector.tagCorners(self.tags[identity])[:,:2])
            pixels.extend(corners)
        if len(set(decoded))!=len(decoded) or set(decoded)!=set(ids):raise ValueError("feature_identity")
        objects,pixels=np.asarray(objects),np.asarray(pixels)
        world=intersect(pixels)
        u,_,vt=np.linalg.svd((objects-objects.mean(axis=0)).T@
            (world[:,:2]-world[:,:2].mean(axis=0)))
        orient=vt.T@u.T
        if np.linalg.det(orient)<=0:raise ValueError("feature_reflected_board")
        translation=world[:,:2].mean(axis=0)-orient@objects.mean(axis=0)
        fitted=np.column_stack((objects@orient.T+translation,np.full(len(objects),planeZ)))
        camera=((fitted-position)@rotation-self.mount[:3,3])@self.mount[:3,:3]
        if np.any(camera[:,2]<=0):raise ValueError("feature_decode_behind_camera")
        projected=camera@self.K.T
        projected=projected[:,:2]/projected[:,2,None]
        residual=float(np.sqrt(np.mean(np.sum((projected-pixels)**2,axis=1))))
        if residual>q["max_reprojection_px"]:raise ValueError("feature_range_attitude_reprojection")
        return orient,translation
