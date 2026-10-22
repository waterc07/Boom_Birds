"""倒装输入、派生标定和三条深度入口的一致性；不连接设备。"""
from pathlib import Path
from types import SimpleNamespace
import array
import cv2
import numpy as np
import pytest
from stereo_depth.core import Config, StereoProcessor
from stereo_depth.orientation import (SENSOR, UPRIGHT, ROTATION, camera_matrix,
    convert_calibration, convert_camera_chain, image_orientation, orient_pair, orient_stitched, scale_matrix)


def sensor_calibration():
    # 非零切向畸变、偏心主点及非平行光轴，避免只覆盖理想平行双目。
    return dict(K1=np.array([[650.,0.,603.],[0.,651.,481.],[0.,0.,1.]]),
                K2=np.array([[648.,0.,688.],[0.,649.,473.],[0.,0.,1.]]),
                D1=np.array([.03,-.02,.001,-.002,.003]),
                D2=np.array([.02,-.01,-.003,.004,-.002]),
                image_size=np.array([1280,960]),
                R=cv2.Rodrigues(np.array([.003,-.005,.001]))[0],
                T=np.array([[-.067],[.0001],[-.0002]]))


@pytest.mark.parametrize("size",[(1280,960),(640,480),(320,240)])
def test_distorted_projections_and_q_reconstruction_preserve_geometry(size):
    old=sensor_calibration()
    new=convert_calibration(old)
    rng=np.random.default_rng(1022)
    a=rng.uniform([-.2,-.15,.5],[.2,.15,8.],(10000,3))
    b=a@old["R"].T+old["T"].reshape(3)
    left,right=b@ROTATION,a@ROTATION
    def project(x,k,d):
        return cv2.projectPoints(x,np.zeros(3),np.zeros(3),k,d)[0].reshape(-1,2)
    for old_i,new_i,x,y in ((2,1,b,left),(1,2,a,right)):
        original=project(x,camera_matrix(old,old_i,size),old[f"D{old_i}"])
        actual=project(y,camera_matrix(new,new_i,size),new[f"D{new_i}"])
        np.testing.assert_allclose(actual,np.array(size)-1-original,atol=2e-12,rtol=0)
    r1,r2,p1,p2,q,*_=cv2.stereoRectify(camera_matrix(new,1,size),new["D1"],
        camera_matrix(new,2,size),new["D2"],size,new["R"],new["T"],
        flags=cv2.CALIB_ZERO_DISPARITY,alpha=0)
    rect_l,rect_r=left@r1.T,right@r2.T
    uv1,uv2=project(rect_l,p1[:,:3],np.zeros(5)),project(rect_r,p2[:,:3],np.zeros(5))
    disp=uv1[:,0]-uv2[:,0]
    assert np.all(disp>0)
    np.testing.assert_allclose(uv1[:,1],uv2[:,1],atol=1e-8,rtol=0)
    homogeneous=np.column_stack((uv1,disp,np.ones(len(a))))@q.T
    np.testing.assert_allclose(homogeneous[:,:3]/homogeneous[:,3,None],rect_l,atol=1e-8,rtol=0)
    np.testing.assert_allclose(np.linalg.norm(new["T"]),np.linalg.norm(old["T"]),atol=1e-15)
    with pytest.raises(ValueError,match="already upright"):
        convert_calibration(new)


def test_sensor_identity_and_orientation_markers_prevent_double_conversion():
    a=np.arange(48,dtype=np.uint8).reshape(6,8)
    b=a+100
    left,right=orient_pair(a,b)
    np.testing.assert_array_equal(left,b[::-1,::-1])
    np.testing.assert_array_equal(right,a[::-1,::-1])
    assert orient_pair(left,right,UPRIGHT,UPRIGHT)[0] is left
    np.testing.assert_array_equal(orient_pair(left,right,UPRIGHT,SENSOR)[0],a)
    with pytest.raises(ValueError):
        orient_pair(a,b,"unknown",UPRIGHT)
    with pytest.raises(ValueError):
        image_orientation(dict(image_orientation="unknown"))


@pytest.mark.parametrize("size",[(640,480),(320,240)])
def test_pixel_offset_is_composable_and_differs_from_naive_scaling(size):
    new=convert_calibration(sensor_calibration())
    direct=camera_matrix(new,1,size)
    via_640=scale_matrix(camera_matrix(new,1,(640,480)),(640,480),size,1.)
    np.testing.assert_allclose(direct,via_640,atol=1e-12,rtol=0)
    naive=new["K1"].copy()
    naive[:2]*=size[0]/1280
    assert abs(direct[0,2]-naive[0,2])>.4


@pytest.mark.parametrize("full_decode",[False,True])
def test_cli_jpeg_and_shared_gray_paths_apply_orientation_exactly_once(tmp_path,full_decode):
    converted=convert_calibration(sensor_calibration())
    path=tmp_path/"upright.npz"
    np.savez(path,**converted)
    processor=StereoProcessor(Config(calibration=path,reduced_decode=not full_decode))
    rng=np.random.default_rng(30)
    sensor=rng.integers(0,256,(960,2560),dtype=np.uint8)
    ok,jpeg=cv2.imencode(".jpg",sensor)
    assert ok
    mode=cv2.IMREAD_GRAYSCALE if full_decode else cv2.IMREAD_REDUCED_GRAYSCALE_2
    decoded=cv2.imdecode(jpeg,mode)
    upright=orient_stitched(decoded,SENSOR,UPRIGHT)
    # 灰度 JPEG 的 BGR 解码三通道相同；比较实际校正像素而非只看标记。
    cli=processor.rectify(jpeg)
    shared=processor.process_pair(*np.split(upright,2,axis=1))
    for a,b in zip(cli,shared[:2]):
        np.testing.assert_array_equal(cv2.cvtColor(a,cv2.COLOR_BGR2GRAY),b)
    pair=processor.process_pair(*np.split(upright,2,axis=1))
    stitched=processor.process_image(upright)
    for a,b in zip(pair[:5],stitched[:5]):
        np.testing.assert_array_equal(a,b)


def test_both_imu_extrinsics_and_time_offsets_follow_physical_sensor():
    source={}
    for i in (0,1):
        t=np.eye(4);t[:3,:3]=cv2.Rodrigues(np.array([.02,.01,-.01]))[0]
        t[0,3]=-.067*i
        source[f"cam{i}"]=dict(camera_model="pinhole",distortion_model="radtan",
            intrinsics=[320.,321.,300.+i*40,240.],distortion_coeffs=[.03,-.02,.001,-.002],
            resolution=[640,480],T_cam_imu=t.tolist(),timeshift_cam_imu=-.007-i*.00002,
            rostopic=f"/cam{i}/image_raw",cam_overlaps=[1-i])
    new=convert_camera_chain(source)
    points=np.array([[.1,.2,1.,1.],[-.2,.1,3.,1.]])
    for new_i,old_i in ((0,1),(1,0)):
        old=source[f"cam{old_i}"];cam=new[f"cam{new_i}"]
        before=points@np.asarray(old["T_cam_imu"]).T
        after=points@np.asarray(cam["T_cam_imu"]).T
        np.testing.assert_allclose(after[:,:3],before[:,:3]@ROTATION,atol=1e-12)
        assert cam["timeshift_cam_imu"]==old["timeshift_cam_imu"]
        assert cam["rostopic"]==source[f"cam{new_i}"]["rostopic"]


@pytest.mark.parametrize("orientation",[SENSOR,UPRIGHT])
def test_compressed_depth_path_obeys_transport_orientation(tmp_path,orientation):
    from sensor_msgs.msg import CompressedImage
    from boom_birds_sensing.depth_node import DepthNode
    path=tmp_path/"upright.npz"
    np.savez(path,**convert_calibration(sensor_calibration()))
    processor=StereoProcessor(Config(calibration=path))
    rng=np.random.default_rng(37)
    image=rng.integers(0,256,(480,1280),dtype=np.uint8)
    image=orient_stitched(image,SENSOR,orientation)
    ok,packet=cv2.imencode(".jpg",image)
    assert ok
    msg=CompressedImage()
    msg.format="jpeg; boombirds_orientation="+orientation
    msg.data=array.array("B",packet.tobytes())
    node=DepthNode.__new__(DepthNode)
    node.processor=processor;node.prior_buffer=None;node.mjpeg_decode_divisor=1
    node._needs_xyz=lambda:True
    captured=[]
    node._publish_result=lambda result,stamp:captured.append(result)
    node.on_packet(msg)
    decoded=cv2.imdecode(packet,cv2.IMREAD_GRAYSCALE)
    expected=processor.process_image(orient_stitched(decoded,orientation,UPRIGHT))
    np.testing.assert_array_equal(captured[0].depth,expected[3][:,:,2])


def test_real_default_is_upright():
    p=StereoProcessor(Config())
    assert p.image_orientation==UPRIGHT
    assert p.calibration["intrinsics_scale_offset"]==1


class Publisher:
    def __init__(self): self.messages=[]
    def publish(self,msg): self.messages.append(msg)
    def get_subscription_count(self): return 1


@pytest.mark.parametrize("input_orientation",[SENSOR,UPRIGHT])
def test_source_raw_compressed_and_shared_sink_share_upright_coordinates(input_orientation):
    from cv_bridge import CvBridge
    from boom_birds_sensing.stereo_source import StereoSourceNode, raw_camera_info
    node=StereoSourceNode.__new__(StereoSourceNode)
    node.calibration=convert_calibration(sensor_calibration())
    node.input_orientation=input_orientation
    node.raw_output_scale=.5;node.raw_decode_divisor=1
    node.frame_left,node.frame_right,node.frame_stitched="cam0","cam1","cam0"
    node.bridge=CvBridge();node.counters={"frames_published":0}
    params=dict(publish_mjpeg=True,publish_stitched=True,publish_raw_without_subscribers=True)
    node.get_parameter=lambda key:SimpleNamespace(value=params[key])
    a=np.tile(np.arange(64,dtype=np.uint8),(48,1))
    b=a+100
    sensor_a,sensor_b=orient_pair(a,b,SENSOR,input_orientation)
    ok,jpeg=cv2.imencode(".jpg",np.hstack([sensor_a,sensor_b]))
    assert ok
    frame=SimpleNamespace(capture_ros_s=123.25,stitched=jpeg.tobytes(),stitched_width=128,height=48)
    node._take_frame=lambda:frame
    node.frame_source=SimpleNamespace(decode=lambda frame:(sensor_a,sensor_b))
    for name in ("pub_left","pub_right","pub_stitched","pub_mjpeg"):
        setattr(node,name,Publisher())
    shared=[];infos=[]
    node.decoded_sink=lambda left,right,stamp:shared.append((left,right,stamp))
    node._publish_camera_info=lambda stamp,w,h:infos.append((stamp,w,h))
    node._tick_captured()
    expected=orient_pair(a,b,SENSOR,UPRIGHT)
    for actual,target in zip(shared[0][:2],expected):
        np.testing.assert_array_equal(actual,target)
    for publisher,target in zip((node.pub_left,node.pub_right),expected):
        msg=publisher.messages[0]
        np.testing.assert_array_equal(node.bridge.imgmsg_to_cv2(msg),cv2.resize(target,(32,24),interpolation=cv2.INTER_AREA))
        assert msg.header.stamp==shared[0][2]
    compressed=node.pub_mjpeg.messages[0]
    assert compressed.format.endswith(UPRIGHT)
    decoded=cv2.imdecode(np.frombuffer(bytes(compressed.data),np.uint8),cv2.IMREAD_GRAYSCALE)
    np.testing.assert_allclose(decoded,np.hstack(expected),atol=2,rtol=0)
    info,baseline,_=raw_camera_info(node.calibration,"left","cam0",320,240)
    k=camera_matrix(node.calibration,1,(320,240))
    np.testing.assert_allclose(info.k,k.reshape(-1),atol=1e-12)
