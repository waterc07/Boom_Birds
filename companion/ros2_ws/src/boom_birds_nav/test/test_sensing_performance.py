import threading
from unittest.mock import Mock
import numpy as np
import pytest
from std_msgs.msg import Header
from boom_birds_sensing.stereo_source import StereoSourceNode
from boom_birds_sensing.depth_core import makeProcessor, processStitched
from boom_birds_sensing.ros_msg import cloud2XyzHw
from boom_birds_sim.synthetic import defaultScene, renderStereo, writeSynthCalibration

def test_gray_stereo_matches_repeated_bgr(tmp_path):
    path=tmp_path/"cal.npz"
    writeSynthCalibration(str(path))
    processor=makeProcessor(str(path))
    left,right,_=renderStereo(defaultScene())
    gray=np.hstack([left,right])
    bgr=np.repeat(gray[:,:,None],3,axis=2)
    expected=processStitched(processor,bgr)
    actual=processStitched(processor,gray)
    np.testing.assert_array_equal(actual.valid,expected.valid)
    np.testing.assert_array_equal(actual.disparity,expected.disparity)
    np.testing.assert_array_equal(actual.xyz,expected.xyz)

@pytest.mark.parametrize("mode,expected,remaining,dropped",[
    ("v4l2",3,[],2),("replay",1,[2,3],0),
])
def test_capture_latest_and_replay_fifo(mode,expected,remaining,dropped):
    node=StereoSourceNode.__new__(StereoSourceNode)
    node.mode=mode;node._queue=[1,2,3];node._queueLock=threading.Lock()
    node.counters={"dropped_backlog":0}
    assert node._takeFrame()==expected
    assert node._queue==remaining
    assert node.counters["dropped_backlog"]==dropped

def test_cloud_payload_layout_and_nan():
    xyz=np.arange(18,dtype=np.float32).reshape(2,3,3)
    valid=np.array([[True,False,True],[False,True,True]])
    msg=cloud2XyzHw(xyz,valid,Header())
    data=np.frombuffer(msg.data,dtype="<f4").reshape(2,3,3)
    np.testing.assert_array_equal(data[valid],xyz[valid])
    assert np.isnan(data[~valid]).all()
    assert (msg.height,msg.width,msg.row_step,msg.point_step)==(2,3,36,12)
    assert not msg.is_dense

from boom_birds_sensing.depth_node import DepthNode
from boom_birds_sensing.frame_worker import LatestFrameWorker

def _workerNode():
    node=DepthNode.__new__(DepthNode)
    node.worker=None
    node.processor=Mock(spec=["close"])
    node.processingPeriod=0.0
    return node

def test_depth_worker_replaces_pending_pair_and_stops():
    node=_workerNode()
    entered=threading.Event();release=threading.Event();finished=threading.Event()
    seen=[]
    def process(left,right):
        seen.append((left,right))
        if left==1:
            entered.set()
            assert release.wait(2)
        else:
            finished.set()
    node.onPair=process
    node.worker=LatestFrameWorker(node._processInput, fatalErrors=True)
    try:
        node.queuePair(1,1)
        assert entered.wait(2)
        node.queuePair(2,2);node.queuePair(3,3)
        assert node.droppedProcessing==1
        release.set()
        assert finished.wait(2)
    finally:
        release.set();node.shutdown()
    assert seen==[(1,1),(3,3)]
    node.processor.close.assert_called_once()
    assert not node.worker.thread.is_alive()
    node.queuePair(4,4)
    assert not node.worker.snapshot()["pending"]

def test_depth_worker_failure_is_visible_to_next_input():
    node=_workerNode()
    def fail(*args):
        raise ValueError("processor failed")
    node.onPair=fail
    node.worker=LatestFrameWorker(node._processInput, fatalErrors=True)
    node.queuePair(1,1)
    node.worker.thread.join(2)
    with pytest.raises(RuntimeError,match="depth worker failed"):
        node.queuePair(2,2)
    node.shutdown()
    node.processor.close.assert_called_once()

def test_exact_pairing_rejects_nearby_stamps():
    import message_filters
    from sensor_msgs.msg import Image
    left=message_filters.SimpleFilter();right=message_filters.SimpleFilter()
    sync=message_filters.TimeSynchronizer([left,right],queue_size=2)
    received=[]
    sync.registerCallback(lambda a,b:received.append((a.header.stamp.nanosec,b.header.stamp.nanosec)))
    def image(ns):
        m=Image();m.header.stamp.sec=1;m.header.stamp.nanosec=ns;return m
    left.signalMessage(image(0));right.signalMessage(image(10000000))
    assert received==[]
    right.signalMessage(image(0))
    assert received==[(0,0)]

def test_mjpeg_matches_raw_pair_and_preserves_stamp(tmp_path):
    import cv2
    from cv_bridge import CvBridge
    from sensor_msgs.msg import CompressedImage
    import array
    path=tmp_path/"cal.npz";writeSynthCalibration(str(path))
    left,right,_=renderStereo(defaultScene())
    ok,packet=cv2.imencode(".jpg",np.hstack([left,right]))
    assert ok
    mono=cv2.imdecode(packet,cv2.IMREAD_GRAYSCALE)
    node=DepthNode.__new__(DepthNode);node.processor=makeProcessor(str(path))
    node.priorBuffer=None
    node.bridge=CvBridge();captured=[]
    node.mjpeg_decode_divisor=1
    node._needsXyz=lambda:True
    node._publishResult=lambda result,stamp:captured.append((result,stamp))
    header=Header();header.stamp.sec=123;header.stamp.nanosec=456
    l=node.bridge.cv2_to_imgmsg(mono[:,:mono.shape[1]//2],encoding="mono8",header=header)
    r=node.bridge.cv2_to_imgmsg(mono[:,mono.shape[1]//2:],encoding="mono8",header=header)
    node.onPair(l,r)
    msg=CompressedImage();msg.header=header;msg.format="jpeg"
    data=array.array("B");data.frombytes(packet.tobytes());msg.data=data
    node.onPacket(msg)
    np.testing.assert_array_equal(captured[0][0].valid,captured[1][0].valid)
    np.testing.assert_array_equal(captured[0][0].xyz,captured[1][0].xyz)
    assert captured[0][1]==captured[1][1]==header.stamp

def test_invalid_mjpeg_is_rejected():
    from sensor_msgs.msg import CompressedImage
    node=DepthNode.__new__(DepthNode)
    msg=CompressedImage();msg.format="png"
    with pytest.raises(ValueError,match="must be JPEG"):
        node.onPacket(msg)
    msg.format="jpeg"
    with pytest.raises(ValueError,match="invalid stitched JPEG"):
        node.onPacket(msg)

@pytest.mark.parametrize("module,extra",[
    ("stereo_source",["-p","mode:=replay","-p","path:={recording}"]),
    ("depth_node",["-p","process_latest_only:=true"]),
])
def test_sigint_with_inherited_ignore_exits_zero(tmp_path,module,extra):
    import os,subprocess,sys,time,signal,pathlib
    repo=pathlib.Path(__file__).resolve().parents[5]
    cal=repo/"companion/ros2_ws/src/stereo_depth/test/fixtures/calibration/live_20260916_210120_642136/candidate.npz"
    recording=pathlib.Path(__file__).parent/"recordings/stereo.png"
    args=[s.format(recording=recording) for s in extra]
    script=("import signal;signal.signal(signal.SIGINT,signal.SIG_IGN);"
            "from boom_birds_sensing."+module+" import main;main()")
    env=os.environ.copy();env["ROS_DOMAIN_ID"]="188";env["ROS_LOCALHOST_ONLY"]="1"
    log=tmp_path/(module+".log")
    with log.open("w") as f:
        proc=subprocess.Popen([sys.executable,"-c",script,"--ros-args",
                               "-p","calibration_file:="+str(cal),*args],
                              stdout=f,stderr=subprocess.STDOUT,env=env)
        try:
            deadline=time.monotonic()+10
            while time.monotonic()<deadline:
                text=log.read_text()
                if "stereo_source " in text or "\u6df1\u5ea6\u5185\u53c2" in text:
                    break
                assert proc.poll() is None,text
                time.sleep(.1)
            else:pytest.fail(log.read_text())
            time.sleep(.2)
            proc.send_signal(signal.SIGINT)
            assert proc.wait(timeout=5)==0,log.read_text()
        finally:
            if proc.poll() is None:
                proc.kill();proc.wait()
