from types import SimpleNamespace as NS
import math
import struct
from pathlib import Path
import cv2
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from boom_birds_sensing.mavros_range import MavrosRangeDecoder
from boom_birds_sensing.downward_source import decodeMono,DownwardSource


def raw(boot=100000,identity=2,orientation=25,current=63,quality=0,fov=0.):
    data=struct.pack("<IHHHBBBB",boot,4,500,current,0,identity,orientation,0)
    data+=struct.pack("<ff4fB",fov,0.,0.,0.,0.,0.,quality)
    padded=data+b"\0"*((-len(data))%8)
    return NS(msgid=132,framing_status=1,sysid=1,compid=1,len=len(data),
              payload64=struct.unpack("<"+"Q"*(len(padded)//8),padded))


def ready(remote=100_000_000_000):
    decoder=MavrosRangeDecoder(sensorId=2)
    for i in range(5):
        sync=NS(header=NS(stamp=NS(sec=1000,nanosec=0)),remote_timestamp_ns=remote+i,
                round_trip_time_ms=2.,observed_offset_ns=1_000_000_000_000-remote,
                estimated_offset_ns=1_000_000_000_000-remote)
        decoder.onSync(sync,100.,1000.)
    return decoder


def test_range_uses_fcu_boot_sample_time_and_explicit_sensor_selection():
    decoder=MavrosRangeDecoder()
    assert decoder.parse(raw()) is None
    assert decoder.reason=="range_sensor_selection_required"
    assert decoder.candidates["2"]["orientation"]==25
    decoder=ready()
    item=decoder.map(raw(boot=99950),nowMono=100.,nowRos=1000.,connected=True)
    assert item.stamp==pytest.approx(999.95)
    assert item.distance_m==pytest.approx(.63)
    assert decoder.map(raw(boot=99950),nowMono=100.,nowRos=1000.,connected=True) is None
    assert decoder.reason=="range_nonmonotonic"


@pytest.mark.parametrize("fault",["id","orientation","distance","quality","fov","sysid","crc","stale","future","clock","disconnect","unsynced"])
def test_range_rejects_invalid_measurements(fault):
    decoder=ready()
    opts=dict(id=dict(identity=3),orientation=dict(orientation=0),distance=dict(current=0),
              quality=dict(quality=1),fov=dict(fov=math.nan),stale=dict(boot=99800),
              future=dict(boot=100030))
    msg=raw(**opts.get(fault,{}))
    if fault=="sysid":msg.sysid=2
    if fault=="crc":msg.framing_status=0
    if fault=="unsynced":decoder=MavrosRangeDecoder(sensorId=2)
    assert decoder.map(msg,nowMono=100.,nowRos=1000.1 if fault=="clock" else 1000.,
                       connected=fault!="disconnect") is None


def test_range_boot_millisecond_wrap_and_truncated_core():
    remote=((1<<32)+100)*1000000
    decoder=ready(remote)
    item=decoder.map(raw(boot=50),nowMono=100.,nowRos=1000.,connected=True)
    assert item.stamp==pytest.approx(999.95)
    msg=raw();msg.len=13
    assert ready().parse(msg) is not None


@pytest.mark.parametrize("fmt",["MJPG","GREY","YUYV"])
def test_downward_decodes_whole_frame_without_stereo_split(fmt):
    gray=np.tile(np.arange(64,dtype=np.uint8),(48,1))
    if fmt=="MJPG":payload=cv2.imencode(".jpg",gray)[1].tobytes()
    elif fmt=="GREY":payload=gray.tobytes()
    else:
        packed=np.empty((48,64,2),np.uint8);packed[:,:,0]=gray;packed[:,:,1]=128
        payload=packed.tobytes()
    result=decodeMono(payload,64,48,fmt)
    assert result.shape==(48,64)
    assert np.max(np.abs(result.astype(float)-gray))<=1
    with pytest.raises(ValueError):
        decodeMono(payload,32,48,fmt)


def test_downward_start_disabled_does_not_open_a_camera_and_checks_sample_time():
    profile=Path(__file__).parents[2]/"boom_birds_sim/config/platform_landing_70mm_test.yaml"
    context=rclpy.context.Context();rclpy.init(context=context)
    def forbidden(**kwargs):raise AssertionError("camera opened")
    node=DownwardSource(sourceFactory=forbidden,context=context,parameter_overrides=[
        Parameter("config_file",value=str(profile)),Parameter("start_capture",value=False)])
    try:
        assert node.source is None
        now=node.get_clock().now().nanoseconds*1e-9
        with pytest.raises(ValueError,match="downward_sample_age"):
            node.publishFrame(NS(capture_ros_s=now-.3,capture_uncertainty_s=.001))
        with pytest.raises(ValueError,match="downward_clock_uncertainty"):
            node.publishFrame(NS(capture_ros_s=now,capture_uncertainty_s=.02))
    finally:
        node.destroy_node();context.shutdown()

def test_range_node_publishes_selected_measurement_with_original_stamp():
    import time
    from builtin_interfaces.msg import Time
    from std_msgs.msg import Header
    from boom_birds_sensing.mavros_range_node import MavrosRangeNode
    context=rclpy.context.Context();rclpy.init(context=context)
    node=MavrosRangeNode(context=context,parameter_overrides=[Parameter("sensor_id",value=2),
        Parameter("stream_rate_hz",value=0.)])
    samples=[];node.pub=NS(publish=samples.append)
    try:
        now=node.get_clock().now().nanoseconds
        stamp=Time(sec=now//1000000000,nanosec=now%1000000000)
        node.onState(NS(header=Header(stamp=stamp),connected=True))
        offset=now-100_000_000_000
        for i in range(5):
            node.onSync(NS(header=Header(stamp=stamp),remote_timestamp_ns=100_000_000_000+i,
                round_trip_time_ms=2.,observed_offset_ns=offset,estimated_offset_ns=offset))
        node.onRaw(raw(boot=99950))
        assert len(samples)==1
        msg=samples[0]
        assert msg.header.frame_id=="range_sensor"
        assert msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9==pytest.approx(now*1e-9-.05,abs=1e-5)
        assert msg.range==pytest.approx(.63)
        node.onRaw(raw(boot=99800))
        assert len(samples)==1
        assert node.decoder.reason=="range_sample_age"
    finally:
        node.destroy_node();context.shutdown()


def test_downward_rejects_uncertain_frame_then_accepts_fresh_frame():
    from std_msgs.msg import Header
    profile=Path(__file__).parents[2]/"boom_birds_sim/config/platform_landing_70mm_test.yaml"
    context=rclpy.context.Context();rclpy.init(context=context)
    node=DownwardSource(context=context,parameter_overrides=[
        Parameter("config_file",value=str(profile)),Parameter("start_capture",value=False),
        Parameter("pixel_format",value="GREY")])
    samples=[];infos=[]
    node.pub=NS(publish=samples.append);node.infoPub=NS(publish=infos.append)
    try:
        now=node.get_clock().now().nanoseconds*1e-9
        node.processFrame(NS(capture_ros_s=now,capture_uncertainty_s=.02))
        assert node.droppedUncertain==1 and not samples
        def fresh():
            return NS(capture_ros_s=node.get_clock().now().nanoseconds*1e-9,
                capture_uncertainty_s=.0001,stitched=bytes(node.width*node.height),
                header=lambda:Header(stamp=node.get_clock().now().to_msg()))
        node.processFrame(fresh())
        assert node.published==1 and len(samples)==len(infos)==1
        node.timebase.offset_jump_count=1
        with pytest.raises(ValueError,match="downward_ros_clock_changed"):
            node.processFrame(fresh())
        assert node.published==1
    finally:
        node.destroy_node();context.shutdown()
