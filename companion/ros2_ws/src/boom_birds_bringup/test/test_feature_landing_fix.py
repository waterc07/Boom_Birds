from dataclasses import replace
from pathlib import Path
import math
import cv2
import numpy as np
import pytest
import yaml
from boom_birds_bringup.tag_landing_fix import TagLandingFix
from boom_birds_bringup.feature_landing_fix import FeatureLandingFix
from boom_birds_sensing.platform_feature_track import BoardFeatureTrack,LocalFeatureObservation
from boom_birds_sensing.platform_observation import BoardDetector
from tools.sensor_replay.platform_replay import renderBoard

ROOT=Path(__file__).parents[2]


def setup():
    profile=yaml.safe_load((ROOT/"boom_birds_sim/config/platform_landing_70mm_test.yaml").read_text())
    detector=BoardDetector(profile,test_only=True)
    tracker=TagLandingFix(profile,testOnly=True)
    return profile,detector,BoardFeatureTrack(detector),tracker,FeatureLandingFix(tracker)


def sample(detector,tracker,stamp,height=.65,x=0.,yaw=0.):
    r=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
    p=np.array([x,0.,height])
    pose=np.eye(4);pose[:3,:3]=r.T;pose[:3,3]=-r.T@p
    image=renderBoard(detector,pose)
    tracker.addPose(stamp,p,(0.,0.,math.sin(yaw/2),math.cos(yaw/2)))
    tracker.addRange(stamp,height-.02,.04,5.,"range_sensor")
    return image,pose


def seeded(x=0.,yaw=0.):
    profile,detector,features,tracker,metric=setup()
    image,pose=sample(detector,tracker,10.,x=x,yaw=yaw)
    detections,_=detector.findCorners(image)
    wire=features.seedDecoded(image,detections,10.)
    assert wire and wire["valid"],wire
    observation=LocalFeatureObservation(**wire)
    metric.submit(observation)
    fix=metric.evaluate(10.01)
    assert fix is not None,metric.reason
    return profile,detector,features,tracker,metric,observation,fix


def test_single_70mm_decoded_plane_is_resolved_with_fresh_range_and_attitude():
    _,_,_,_,metric,_,fix=seeded()
    assert fix.source=="resolved_tag"
    assert np.linalg.norm(fix.target)<.005
    assert fix.height==pytest.approx(.65)
    assert metric.reason=="range_attitude_constrained_pnp"


@pytest.mark.parametrize("x,yaw",[(0.,0.),(-.01,math.pi)])
def test_continued_pixels_follow_descent_with_original_identity_stamp(x,yaw):
    _,detector,features,tracker,metric,initial,_=seeded(x,yaw)
    continued=0
    for index in range(1,69):
        now=10.+index*.05;height=.65-index*.007
        image,pose=sample(detector,tracker,now,height,x,yaw)
        if height<.34:
            raw,_=detector.findCorners(image)
            # Remove the marker border; preserve the inner texture.
            center=features.center
            radius=500*.07/(height-.02)*.4
            # Projected target from known scene, used only to form this test's mask.
            cp=np.linalg.inv(detector.T_B_C)@pose
            target=cv2.projectPoints(np.zeros((1,3)),cv2.Rodrigues(cp[:3,:3])[0],
                cp[:3,3],detector.K,detector.D)[0].reshape(2)
            mask=np.full_like(image,255);pixelX,pixelY=np.rint(target).astype(int);rad=round(radius)
            mask[max(0,pixelY-rad):min(480,pixelY+rad+1),max(0,pixelX-rad):min(640,pixelX+rad+1)]=image[max(0,pixelY-rad):min(480,pixelY+rad+1),max(0,pixelX-rad):min(640,pixelX+rad+1)]
            image=mask
        wire=features.update(image,now)
        assert wire["valid"],(index,height,wire)
        metric.submit(LocalFeatureObservation(**wire))
        fix=metric.evaluate(now+.01)
        assert fix is not None,(index,height,metric.reason)
        assert fix.source=="features" and fix.identity_stamp==initial.identity_stamp
        assert np.linalg.norm(fix.target)<.015
        continued+=1
    assert continued==68


@pytest.mark.parametrize("failure",["old_image","old_identity","wrong_id","bad_range","no_pose","yaw_geometry","unconfirmed_identity","wrong_frame"])
def test_feature_permit_rejects_invalid_inputs(failure):
    _,_,_,tracker,metric,observation,_=seeded()
    now=10.05
    tracker.addPose(now,(0.,0.,.65),(0.,0.,0.,1.))
    tracker.addRange(now,.63,.04,5.,"range_sensor")
    item=replace(observation,stamp=now,identity_age_s=.05,source="local_features")
    if failure=="old_image":now=10.3
    elif failure=="old_identity":item=replace(item,identity_stamp=1.,identity_age_s=9.05)
    elif failure=="wrong_id":item=replace(item,inherited_tag_ids=[8])
    elif failure=="bad_range":tracker.ranges.clear()
    elif failure=="no_pose":tracker.poses.clear()
    elif failure=="yaw_geometry":item=replace(item,reference_pixels=[item.reference_pixels[0]]*3)
    elif failure=="unconfirmed_identity":metric.anchor=None
    elif failure=="wrong_frame":item=replace(item,pixel_geometry="distorted")
    metric.submit(item)
    assert metric.evaluate(now) is None


def test_invalid_feature_frame_requires_a_new_decoded_identity():
    _,_,_,_,metric,observation,_=seeded()
    metric.submit(replace(observation,valid=False,reason="feature_loss"))
    assert metric.evaluate(10.01) is None
    metric.submit(replace(observation,stamp=10.02,identity_age_s=.02,source="local_features"))
    assert metric.evaluate(10.03) is None


@pytest.mark.parametrize("fault",["relabel","metric_shift","collapse","identity_refresh"])
def test_pnp_rejects_correspondence_or_identity_changes(fault):
    _,_,_,tracker,metric,item,_=seeded()
    stamp=10.05
    tracker.addPose(stamp,(0.,0.,.65),(0.,0.,0.,1.))
    tracker.addRange(stamp,.63,.04,5.,"range_sensor")
    item=replace(item,stamp=stamp,identity_age_s=.05,source="local_features")
    if fault=="relabel":item=replace(item,point_ids=[i+1000 for i in item.point_ids])
    elif fault=="metric_shift":
        points=np.array(item.object_points_m);points[:,0]+=.05
        item=replace(item,object_points_m=points.tolist())
    elif fault=="collapse":item=replace(item,image_points=[item.image_points[0]]*item.feature_count)
    elif fault=="identity_refresh":item=replace(item,identity_stamp=stamp,identity_age_s=0.)
    metric.submit(item)
    assert metric.evaluate(stamp+.01) is None
