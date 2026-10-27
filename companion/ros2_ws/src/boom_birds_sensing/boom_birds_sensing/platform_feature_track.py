"""Identity-bounded planar feature continuation in undistorted camera pixels."""
from dataclasses import dataclass, asdict, field
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
    min_axis_spread_px: float = 3.

    def __post_init__(self):
        for name,value in asdict(self).items():
            if type(value) not in (int,float) or not math.isfinite(value) or value<=0:
                raise ValueError(name)
        for name in ("roi_radius_px","max_points","min_points"):
            if type(getattr(self,name)) is not int:raise ValueError(name)
        if (self.capture_min_m>=self.capture_max_m or self.min_points<4
                or self.max_points<self.min_points or self.min_inlier_ratio>1):
            raise ValueError("track_limits")


@dataclass(frozen=True)
class LocalFeatureObservation:
    stamp: float
    valid: bool
    reason: str
    board: str = ""
    inherited_tag_ids: list = field(default_factory=list)
    identity_stamp: float = 0.
    identity_age_s: float = 0.
    landing_pixel: list = field(default_factory=list)
    reference_pixels: list = field(default_factory=list)
    reference_span_m: float = 0.
    seed_tag_pixels: list = field(default_factory=list)
    object_points_m: list = field(default_factory=list)
    image_points: list = field(default_factory=list)
    point_ids: list = field(default_factory=list)
    feature_count: int = 0
    inlier_ratio: float = 0.
    forward_backward_px: float = 0.
    source: str = ""
    test_only: bool = False
    kind: str = "board_local_features_v1"
    frame: str = "camera_pixels"
    pixel_geometry: str = "undistorted"
    metric_pose_available: bool = False


class BoardFeatureTrack:
    def __init__(self,detector,config=None):
        self.detector=detector
        self.cfg=config or TrackConfig()
        self.reset()

    def reset(self):
        self.previous=self.points=self.center=self.references=None
        self.stamp=self.identity_stamp=None
        self.ids=()
        self.span=0.
        self.seedTags=[]
        self.objects=None
        self.pointIds=[]

    def invalid(self,stamp,reason):
        self.reset()
        return asdict(LocalFeatureObservation(stamp,False,reason,
            board=self.detector.board["name"],test_only=self.detector.test_only))

    def undistort(self,image):
        if (image is None or image.shape!=tuple(reversed(self.detector.camera["image_size"]))
                or image.dtype!=np.uint8):raise ValueError("image_invalid")
        return cv2.undistort(image,self.detector.K,self.detector.D)

    def referenceObjects(self):
        target=np.asarray(self.detector.board["landing_point_m"],float)
        self.span=min(tag["size_m"] for tag in self.detector.board["tags"])/2
        return np.array([target,target+[self.span,0,0],target+[0,self.span,0]])

    def begin(self,image,stamp,ids,pixels,references,seedTags,planeHomography):
        c=self.cfg
        center=references[0]
        mask=np.zeros_like(image)
        cv2.circle(mask,tuple(np.rint(center).astype(int)),c.roi_radius_px,255,-1)
        if pixels is not None:
            planeMask=np.zeros_like(mask)
            inner=self.detector.dictionary.markerSize/(self.detector.dictionary.markerSize+2)
            support=center+(np.asarray(pixels)-center)*inner
            cv2.fillConvexPoly(planeMask,cv2.convexHull(np.float32(support)).astype(np.int32),255)
            mask=cv2.bitwise_and(mask,planeMask)
        points=cv2.goodFeaturesToTrack(image,c.max_points,.02,5,mask=mask)
        if points is None or len(points)<c.min_points:
            return self.invalid(stamp,"insufficient_center_features")
        points=self.refine(image,points)
        if not self.supports(points.reshape(-1,2),center):
            return self.invalid(stamp,"features_degenerate")
        try:
            local=cv2.perspectiveTransform(points,np.linalg.inv(planeHomography)).reshape(-1,2)
        except (ValueError,np.linalg.LinAlgError,cv2.error):
            return self.invalid(stamp,"seed_plane_mapping")
        if not np.isfinite(local).all():return self.invalid(stamp,"seed_plane_mapping")
        self.objects=np.column_stack((local,np.zeros(len(local))))
        self.pointIds=list(range(len(local)))
        self.previous,self.points,self.center=image.copy(),points,center.copy()
        self.references=np.array(references,float)
        self.stamp=self.identity_stamp=stamp
        self.ids=tuple(ids);self.seedTags=seedTags
        return self.output(stamp,"decoded_tag" if seedTags else "decoded",len(points),1.,0.)

    @staticmethod
    def refine(image,points):
        points=points.copy().reshape(-1,1,2)
        height,width=image.shape
        p=points.reshape(-1,2)
        inside=np.isfinite(p).all(axis=1)&(p[:,0]>=4)&(p[:,0]<width-4)&(p[:,1]>=4)&(p[:,1]<height-4)
        if inside.any():
            refined=p[inside].astype(np.float32).reshape(-1,1,2)
            cv2.cornerSubPix(image,refined,(3,3),(-1,-1),
                (cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,20,.01))
            points[inside]=refined
        return points

    def supports(self,points,center):
        if len(points)<self.cfg.min_points:return False
        hull=cv2.convexHull(points.astype(np.float32))
        if cv2.contourArea(hull)<25:return False
        spread=np.linalg.eigvalsh(np.cov(points.T))
        return (spread.min()>=self.cfg.min_axis_spread_px**2
            and cv2.pointPolygonTest(hull,tuple(map(float,center)),False)>=0)

    def seed(self,image,observation,distance_m):
        if (not observation.valid or observation.board!=self.detector.board["name"]
                or not observation.tag_ids or any(i not in self.detector.tags for i in observation.tag_ids)):
            return self.invalid(observation.stamp,"identity_unconfirmed")
        if not math.isfinite(distance_m) or not self.cfg.capture_min_m<=distance_m<=self.cfg.capture_max_m:
            return self.invalid(observation.stamp,"outside_capture_height")
        try:image=self.undistort(image)
        except ValueError:return self.invalid(observation.stamp,"image_invalid")
        pose=np.linalg.inv(self.detector.T_B_C)@observation.pose()
        refs=cv2.projectPoints(self.referenceObjects(),cv2.Rodrigues(pose[:3,:3])[0],
            pose[:3,3],self.detector.K,np.zeros_like(self.detector.D))[0].reshape(-1,2)
        homography=self.detector.K@np.column_stack((pose[:3,0],pose[:3,1],pose[:3,3]))
        return self.begin(image,observation.stamp,observation.tag_ids,None,refs,[],homography)

    def seedDecoded(self,image,detections,stamp):
        """Decode confirms identity; the consumer must independently validate range/attitude geometry."""
        if not math.isfinite(stamp):return self.invalid(stamp,"sample_stamp")
        if len({i for i,_ in detections})!=len(detections):
            return self.invalid(stamp,"duplicate_detection")
        known=[]
        for identity,pixels in detections:
            if identity not in self.detector.tags:continue
            raw=np.asarray(pixels,float)
            if raw.shape!=(4,2) or not np.isfinite(raw).all():
                return self.invalid(stamp,"corner_invalid")
            edge=float(np.linalg.norm(raw-np.roll(raw,1,axis=0),axis=1).min())
            if edge<self.detector.profile["quality"]["min_edge_px"]:continue
            tag=self.detector.tags[identity]
            approximateHeight=(self.detector.K[0,0]+self.detector.K[1,1])/2*tag["size_m"]/edge
            known.append((identity,raw,approximateHeight))
        if not known:return None
        apparent=float(np.median([item[2] for item in known]))
        if not self.cfg.capture_min_m<=apparent<=self.cfg.capture_max_m:return None
        if len(known)<self.detector.profile["quality"]["min_tags"]:return None
        objects=[];pixels=[];seedTags=[]
        for identity,raw,_ in known:
            undistorted=cv2.undistortPoints(raw.reshape(-1,1,2),self.detector.K,
                self.detector.D,P=self.detector.K).reshape(-1,2)
            objects.extend(self.detector.tagCorners(self.detector.tags[identity])[:,:2])
            pixels.extend(undistorted)
            seedTags.append(dict(id=int(identity),corners=undistorted.tolist()))
        objects,pixels=np.asarray(objects),np.asarray(pixels)
        h,_=cv2.findHomography(objects,pixels,0)
        if h is None or not np.isfinite(h).all():return self.invalid(stamp,"seed_homography")
        fit=cv2.perspectiveTransform(objects.astype(np.float32).reshape(-1,1,2),h).reshape(-1,2)
        residual=float(np.sqrt(np.mean(np.sum((fit-pixels)**2,axis=1))))
        if residual>self.detector.profile["quality"]["max_reprojection_px"]:
            return self.invalid(stamp,"seed_reprojection")
        refs=cv2.perspectiveTransform(self.referenceObjects()[:,:2].astype(np.float32).reshape(-1,1,2),h).reshape(-1,2)
        try:image=self.undistort(image)
        except ValueError:return self.invalid(stamp,"image_invalid")
        return self.begin(image,stamp,[i for i,_,_ in known],pixels,refs,seedTags,h)

    def output(self,stamp,source,count,ratio,fb):
        return asdict(LocalFeatureObservation(stamp,True,"ok",
            board=self.detector.board["name"],inherited_tag_ids=list(self.ids),
            identity_stamp=self.identity_stamp,identity_age_s=stamp-self.identity_stamp,
            landing_pixel=self.center.tolist(),reference_pixels=self.references.tolist(),
            reference_span_m=self.span,seed_tag_pixels=self.seedTags,
            object_points_m=self.objects.tolist(),image_points=self.points.reshape(-1,2).tolist(),
            point_ids=list(self.pointIds),feature_count=count,
            inlier_ratio=ratio,forward_backward_px=fb,source=source,test_only=self.detector.test_only))

    def update(self,image,stamp):
        c=self.cfg
        if self.previous is None:return self.invalid(stamp,"decode_required")
        if not math.isfinite(stamp) or not 0<stamp-self.stamp<=c.max_gap_s:
            return self.invalid(stamp,"sample_gap")
        if stamp-self.identity_stamp>c.max_identity_age_s:return self.invalid(stamp,"identity_expired")
        try:image=self.undistort(image)
        except ValueError:return self.invalid(stamp,"image_invalid")
        forward,status,_=cv2.calcOpticalFlowPyrLK(self.previous,image,self.points,None)
        if forward is None:return self.invalid(stamp,"flow_failed")
        forward=self.refine(image,forward)
        back,backStatus,_=cv2.calcOpticalFlowPyrLK(image,self.previous,forward,None)
        if back is None:return self.invalid(stamp,"flow_failed")
        fb=np.linalg.norm(back-self.points,axis=2).ravel()
        candidate=forward.reshape(-1,2);height,width=image.shape
        visible=np.isfinite(candidate).all(axis=1)&(candidate[:,0]>=0)&(candidate[:,0]<width)&(candidate[:,1]>=0)&(candidate[:,1]<height)
        keep=(status.ravel()==1)&(backStatus.ravel()==1)&np.isfinite(fb)&(fb<=c.max_fb_px)&visible
        previous=self.points.reshape(-1,2)[keep];current=forward.reshape(-1,2)[keep]
        if len(current)<c.min_points:return self.invalid(stamp,"feature_loss")
        h,inliers=cv2.findHomography(previous,current,cv2.RANSAC,c.ransac_px)
        if h is None or inliers is None or not np.isfinite(h).all() or np.linalg.cond(h)>1e8:
            return self.invalid(stamp,"homography_failed")
        good=inliers.ravel().astype(bool);ratio=float(good.mean())
        if good.sum()<c.min_points or ratio<c.min_inlier_ratio:
            return self.invalid(stamp,"inliers_insufficient")
        refs=cv2.perspectiveTransform(self.references.astype(np.float32).reshape(-1,1,2),h).reshape(-1,2)
        center=refs[0]
        height,width=image.shape
        if (not np.isfinite(refs).all() or not (0<=center[0]<width and 0<=center[1]<height)):
            return self.invalid(stamp,"target_outside_image")
        if not self.supports(current[good],center):return self.invalid(stamp,"features_degenerate")
        self.objects=self.objects[keep][good]
        self.pointIds=np.asarray(self.pointIds)[keep][good].tolist()
        self.previous=image.copy();self.points=current[good].astype(np.float32).reshape(-1,1,2)
        self.center,self.references,self.stamp=center,refs,stamp
        return self.output(stamp,"local_features",int(good.sum()),ratio,float(fb[keep].max()))
