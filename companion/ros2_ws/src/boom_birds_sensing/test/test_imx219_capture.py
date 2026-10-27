from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import cv2
import numpy as np
import pytest
import yaml
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from boom_birds_sensing.imx219_capture import decodeRaw10,GAMMA_LUT,Imx219Media
from boom_birds_sensing.downward_source import nativeGeometry,DownwardSource


def packRaw(raw,padding=0):
    h,w=raw.shape
    groups=raw.reshape(h,w//4,4).astype(np.uint16)
    packed=np.empty((h,w//4,5),np.uint8)
    packed[:,:,:4]=groups>>2
    packed[:,:,4]=np.bitwise_or.reduce((groups&3)<<np.array([0,2,4,6],np.uint16),axis=2)
    rows=np.pad(packed.reshape(h,w*5//4),((0,0),(0,padding)))
    return rows.tobytes()


@pytest.mark.parametrize("padding",[0,14,32])
def test_raw10_padding_and_low_bits_preserve_calibration_pixels(padding):
    raw=(np.arange(8*12).reshape(8,12)*11%1024).astype(np.uint16)
    expected=cv2.cvtColor(GAMMA_LUT[raw],cv2.COLOR_BayerRGGB2GRAY)
    assert np.array_equal(decodeRaw10(packRaw(raw,padding),12,8),expected)


@pytest.mark.parametrize("width,height,payload",[(11,8,b"0"*128),(12,0,b""),(12,8,b"0"*121),(12,8,b"0"*64),(12,8,b"0"*8000)])
def test_raw10_rejects_unsupported_payload_layout(width,height,payload):
    with pytest.raises(ValueError):decodeRaw10(payload,width,height)


def camera(width=16,height=12):
    return dict(image_size=[width,height],K=[[10.,0.,7.5],[0.,10.,5.5],[0.,0.,1.]],D=[0.,0.,0.,0.,0.])


def test_explicit_half_scale_matches_intrinsics_and_distortion():
    native=camera();working=deepcopy(native);working["image_size"]=[8,6]
    k=np.array(native["K"]);k[:2,:]*=.5;working["K"]=k.tolist()
    assert nativeGeometry(working,native)==(16,12,.5)


@pytest.mark.parametrize("fault",["unequal_scale","wrong_k","wrong_d","float_size","upsample"])
def test_native_geometry_refuses_unmatched_camera_parameters(fault):
    native=camera();working=deepcopy(native)
    if fault=="unequal_scale":working["image_size"]=[8,12]
    elif fault=="wrong_k":working["K"][0][0]+=1
    elif fault=="wrong_d":working["D"][0]=.1
    elif fault=="float_size":native["image_size"][0]=16.
    else:working["image_size"]=[32,24]
    with pytest.raises(ValueError):nativeGeometry(working,native)


class Media:
    def __init__(self,fail=False,flip=False):
        self.calls=[];self.fail=fail;self.flip=flip;self.changed=False
    def __call__(self,args):
        self.calls.append(args)
        if args[0]=="v4l2-ctl":
            if "--get-fmt-video" in args:return "Width/Height : 640/480\nPixel Format : 'YUYV'"
            if "--get-ctrl=horizontal_flip,vertical_flip,test_pattern" in args:
                return "horizontal_flip: "+str(int(self.flip))+"\nvertical_flip: 0\ntest_pattern: 0"
            return ""
        if "-p" in args:return 'imx219 11-0010 -> "rp1-cfe-csi2_ch0":0 []'
        if "-e" in args:return "/dev/video0" if args[-1]=="rp1-cfe-csi2_ch0" else "/dev/v4l-subdev2"
        if "--get-v4l2" in args:
            if self.changed:return "[fmt:SRGGB10_1X10/1640x1232 field:none crop:(8,8)/3280x2464]"
            return "[stream:0 fmt:SRGGB10_1X10/640x480 field:none]"
        if "-V" in args and "1640x1232" in args[-1]:
            self.changed=True
            if self.fail and '"csi2":0' in args[-1]:
                self.fail=False
                raise RuntimeError("injected_media_error")
        return ""


def test_media_restores_original_formats_and_link_after_capture():
    runner=Media();media=Imx219Media("/dev/video0","/dev/media0","imx219 11-0010",runner)
    media.configure(1640,1232);media.close()
    restores=[a[-1] for a in runner.calls if "-V" in a and "640x480" in a[-1]]
    assert len(restores)==3
    assert runner.calls[-1][-1]=='"csi2":4 -> "rp1-cfe-csi2_ch0":0 [0]'
    assert any("--set-fmt-video=width=640,height=480,pixelformat=YUYV" in a for a in runner.calls)


def test_partial_media_configuration_failure_can_be_rolled_back():
    runner=Media(fail=True);media=Imx219Media("/dev/video0","/dev/media0","imx219 11-0010",runner)
    with pytest.raises(RuntimeError,match="injected"):media.configure(1640,1232)
    media.close()
    assert len([a for a in runner.calls if "-V" in a and "640x480" in a[-1]])==3


def test_media_refuses_flips_and_wrong_sensor_mode():
    media=Imx219Media("/dev/video0","/dev/media0","imx219 11-0010",Media(flip=True))
    with pytest.raises(ValueError,match="flip"):media.configure(1640,1232)
    media.close()
    with pytest.raises(ValueError,match="calibrated"):media.configure(820,616)


def test_resized_image_and_camera_info_keep_driver_sample_time(tmp_path):
    native=camera();working=deepcopy(native);working["image_size"]=[8,6]
    k=np.array(native["K"]);k[:2,:]*=.5;working["K"]=k.tolist()
    for name,value in (("native",native),("working",working)):
        (tmp_path/(name+".yaml")).write_text(yaml.safe_dump(dict(synthetic=False,camera=value)))
    context=rclpy.context.Context();rclpy.init(context=context)
    node=DownwardSource(context=context,parameter_overrides=[
        Parameter("config_file",value=str(tmp_path/"working.yaml")),
        Parameter("native_config_file",value=str(tmp_path/"native.yaml")),
        Parameter("pixel_format",value="GREY"),Parameter("start_capture",value=False)])
    images=[];infos=[]
    node.pub=NS(publish=images.append);node.infoPub=NS(publish=infos.append)
    frame=NS(capture_ros_s=100.,capture_uncertainty_s=.001,stitched=bytes(range(192)),
             header=lambda:Header(stamp=Time(sec=100,nanosec=0)))
    try:
        with patch.object(node,"get_clock",return_value=NS(now=lambda:NS(nanoseconds=100010000000))):
            node.publishFrame(frame)
        assert (images[0].width,images[0].height)==(8,6)
        assert images[0].header.stamp==infos[0].header.stamp==Time(sec=100,nanosec=0)
        assert np.array_equal(infos[0].k,np.asarray(working["K"]).ravel())
    finally:node.destroy_node();context.shutdown()

def test_busy_imx219_stops_before_media_or_capture_changes(tmp_path):
    from unittest.mock import Mock
    profile=tmp_path/"camera.yaml"
    profile.write_text(yaml.safe_dump(dict(synthetic=False,camera=camera())))
    context=rclpy.context.Context();rclpy.init(context=context)
    node=DownwardSource(context=context,parameter_overrides=[
        Parameter("config_file",value=str(profile)),Parameter("start_capture",value=False)])
    source=NS(open=Mock(),close=Mock());media=NS(configure=Mock(),close=Mock())
    node.source=source;node.media=media
    try:
        with patch("boom_birds_sensing.downward_source.subprocess.run",return_value=NS(returncode=0)):
            node.capture()
        assert node.error=="imx219_device_in_use_or_unreadable"
        source.open.assert_not_called();media.configure.assert_not_called()
        source.close.assert_called_once();media.close.assert_called_once()
        assert node.published==0
    finally:node.destroy_node();context.shutdown()


@pytest.mark.parametrize("reason",["downward_sample_age","downward_sample_age_after_decode"])
def test_overdue_frame_is_dropped_without_refreshing_or_stopping_fresh_frames(reason):
    from unittest.mock import Mock
    node=NS(publishFrame=Mock(side_effect=[ValueError(reason),None]),droppedStale=0)
    DownwardSource.processFrame(node,object())
    assert node.droppedStale==1
    DownwardSource.processFrame(node,object())
    assert node.publishFrame.call_count==2 and node.droppedStale==1


def test_clock_failure_is_not_treated_as_a_droppable_late_frame():
    from unittest.mock import Mock
    node=NS(publishFrame=Mock(side_effect=ValueError("downward_ros_clock_changed")),droppedStale=0)
    with pytest.raises(ValueError,match="clock"):DownwardSource.processFrame(node,object())
    assert node.droppedStale==0


@pytest.fixture(scope="module")
def nativeKernel(tmp_path_factory):
    import subprocess
    from stereo_depth.native import NativeKernels
    library=tmp_path_factory.mktemp("raw10-native")/"kernels.so"
    source=Path(__file__).parents[2]/"stereo_depth/native/kernels.cpp"
    subprocess.run(["g++","-std=c++17","-O2","-shared","-fPIC",str(source),"-o",str(library)],check=True)
    return NativeKernels(str(library))


@pytest.mark.parametrize("width,height,padding",[(12,8,0),(1640,1232,14)])
def test_native_raw10_is_pixel_identical_to_reference(nativeKernel,width,height,padding):
    raw=np.random.default_rng(41).integers(0,1024,(height,width),dtype=np.uint16)
    payload=packRaw(raw,padding)
    assert np.array_equal(decodeRaw10(payload,width,height,nativeKernel),
                          decodeRaw10(payload,width,height))


def test_native_raw10_rejects_incorrect_sizes_before_access(nativeKernel):
    with pytest.raises(ValueError):nativeKernel.unpackRaw10(bytes(120),12,8,GAMMA_LUT[:-1])
    with pytest.raises(ValueError):nativeKernel.unpackRaw10(bytes(119),12,8,GAMMA_LUT)
    # The C ABI also checks null and layout arguments if called without the wrapper.
    nativeKernel.unpackRaw10(bytes(120),12,8,GAMMA_LUT)
    assert nativeKernel.lib.bb_unpack_raw10(None,120,12,8,15,None,None)==-1
