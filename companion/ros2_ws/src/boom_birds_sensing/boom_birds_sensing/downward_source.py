"""Downward mono V4L2 ingress using the existing driver timestamp path."""
import json
import math
from pathlib import Path
import threading
import subprocess
import time
import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from .camera_timestamp import V4L2_PIX_FMT_MJPEG,V4L2_PIX_FMT_YUYV,V4L2_PIX_FMT_GREY
from .stereo_capture import V4L2FrameSource
from .timebase import RosTimeBase
from .frame_worker import LatestFrameWorker
from .imx219_capture import RAW10_FORMAT,decodeRaw10,Imx219Media


FORMATS={"MJPG":V4L2_PIX_FMT_MJPEG,"YUYV":V4L2_PIX_FMT_YUYV,"GREY":V4L2_PIX_FMT_GREY,"pRAA":RAW10_FORMAT}


def decodeMono(payload,width,height,pixelFormat):
    raw=np.frombuffer(payload,np.uint8)
    if pixelFormat=="pRAA":
        image=decodeRaw10(payload,width,height)
    elif pixelFormat=="MJPG":
        image=cv2.imdecode(raw,cv2.IMREAD_GRAYSCALE)
    elif pixelFormat=="GREY":
        if raw.size!=width*height:raise ValueError("downward_payload_stride_or_size")
        image=raw.reshape(height,width).copy()
    elif pixelFormat=="YUYV":
        if raw.size!=width*height*2:raise ValueError("downward_payload_stride_or_size")
        image=cv2.cvtColor(raw.reshape(height,width,2),cv2.COLOR_YUV2GRAY_YUYV)
    else:raise ValueError("downward_pixel_format")
    if image is None or image.shape!=(height,width):raise ValueError("downward_decoded_size")
    return image


def nativeGeometry(camera,native):
    width,height=camera["image_size"];nativeWidth,nativeHeight=native["image_size"]
    if any(type(x) is not int or x<=0 for x in (width,height,nativeWidth,nativeHeight)):
        raise ValueError("downward_integer_image_size")
    scale=width/nativeWidth
    if not .25<=scale<=1 or not math.isclose(height/nativeHeight,scale,abs_tol=1e-9):
        raise ValueError("downward_requires_uniform_native_scale")
    k=np.asarray(native["K"],float);d=np.asarray(native["D"],float)
    if (k.shape!=(3,3) or not np.isfinite(k).all() or min(k[0,0],k[1,1])<=0
            or not np.allclose(k[2],[0,0,1]) or not np.isfinite(d).all()):
        raise ValueError("downward_native_intrinsics")
    expected=k.copy();expected[:2,:]*=scale
    if not np.allclose(np.asarray(camera["K"]),expected,atol=1e-6,rtol=0):
        raise ValueError("downward_scaled_intrinsics")
    if d.shape!=np.asarray(camera["D"]).shape or not np.allclose(camera["D"],d,atol=1e-12,rtol=0):
        raise ValueError("downward_native_distortion")
    return nativeWidth,nativeHeight,scale


class DownwardSource(Node):
    def __init__(self,sourceFactory=V4L2FrameSource,**kwargs):
        super().__init__("boom_birds_downward_source",**kwargs)
        defaults=dict(device="",config_file="",pixel_format="MJPG",capture_fps=30,
            publish_rate_hz=20.,capture_buffers=4,max_frame_age_s=.15,
            max_timestamp_uncertainty_s=.005,image_topic="/boom_birds/downward/image",
            frame_id="downward_optical",start_capture=True,native_config_file="",
            media_device="",sensor_entity="",raw10_native_library="",opencv_threads=1)
        for key,value in defaults.items():self.declare_parameter(key,value)
        get=lambda key:self.get_parameter(key).value
        threads=int(get("opencv_threads"))
        if not 1<=threads<=4:raise ValueError("downward_opencv_threads")
        cv2.setNumThreads(threads)
        self.rawKernel=None
        if get("raw10_native_library"):
            from stereo_depth.native import NativeKernels
            self.rawKernel=NativeKernels(str(get("raw10_native_library")))
            if not hasattr(self.rawKernel.lib,"bb_unpack_raw10"):raise ValueError("raw10_native_abi_missing")
        profile=yaml.safe_load(Path(str(get("config_file"))).read_text())
        camera=profile["camera"]
        self.width,self.height=map(int,camera["image_size"])
        self.K=np.asarray(camera["K"],float);self.D=np.asarray(camera["D"],float)
        if (min(self.width,self.height)<=0 or self.K.shape!=(3,3) or self.D.shape not in ((4,),(5,))
                or not np.isfinite(self.K).all() or not np.isfinite(self.D).all()
                or min(self.K[0,0],self.K[1,1])<=0):raise ValueError("downward_camera_profile")
        nativeProfile=(yaml.safe_load(Path(str(get("native_config_file"))).read_text())
                       if get("native_config_file") else profile)
        self.nativeWidth,self.nativeHeight,self.imageScale=nativeGeometry(camera,nativeProfile["camera"])
        self.media=None
        self.pixelFormat=str(get("pixel_format"))
        if self.pixelFormat not in FORMATS:raise ValueError("downward_pixel_format")
        fps=int(get("capture_fps"));self.rate=float(get("publish_rate_hz"))
        self.maxAge=float(get("max_frame_age_s"));self.maxUncertainty=float(get("max_timestamp_uncertainty_s"))
        if (not 1<=fps<=120 or not math.isfinite(self.rate) or not 0<self.rate<=fps
                or not 0<self.maxAge<=.15 or not 0<self.maxUncertainty<=.03):
            raise ValueError("downward_capture_limits")
        if get("start_capture") and (not str(get("device")) or nativeProfile.get("synthetic") is not False):
            raise ValueError("downward_requires_real_profile_and_explicit_device")
        if self.pixelFormat=="pRAA":
            if nativeProfile["camera"].get("sensor")!="IMX219" or (self.nativeWidth,self.nativeHeight)!=(1640,1232):
                raise ValueError("raw10_requires_native_imx219_profile")
            if get("start_capture"):
                if not get("media_device") or not get("sensor_entity"):
                    raise ValueError("raw10_requires_explicit_media_and_sensor")
                self.media=Imx219Media(str(get("device")),str(get("media_device")),str(get("sensor_entity")))
        self.frameId=str(get("frame_id"))
        self.bridge=CvBridge();self.stop=threading.Event();self.thread=None;self.worker=None
        self.droppedUncertain=0
        self.frames=self.published=self.droppedStale=0;self.error="";self.lastCapture=None;self.lastUncertainty=None
        self.pub=self.create_publisher(Image,str(get("image_topic")),qos_profile_sensor_data)
        self.infoPub=self.create_publisher(CameraInfo,str(get("image_topic"))+"/camera_info",qos_profile_sensor_data)
        self.stats=self.create_publisher(String,"/boom_birds/downward/status",10)
        self.create_timer(.5,self.report)
        self.timebase=RosTimeBase(self.get_clock())
        self.source=None
        if get("start_capture"):
            self.source=sourceFactory(device=str(get("device")),width=self.nativeWidth,height=self.nativeHeight,
                fps=fps,pixelFormat=FORMATS[self.pixelFormat],buffer_count=int(get("capture_buffers")),
                timebase=self.timebase,allow_realtime=False,timeout_s=.3)
            self.worker=LatestFrameWorker(self.processFrame,processingPeriod=1/self.rate,fatalErrors=True,name="downward-decode")
            self.thread=threading.Thread(target=self.capture,name="downward-v4l2",daemon=True)
            self.thread.start()

    def publishFrame(self,frame):
        now=self.get_clock().now().nanoseconds*1e-9
        if not 0<=now-frame.capture_ros_s<=self.maxAge:raise ValueError("downward_sample_age")
        if (not math.isfinite(frame.capture_uncertainty_s)
                or not 0<=frame.capture_uncertainty_s<=self.maxUncertainty):
            raise ValueError("downward_clock_uncertainty")
        if self.timebase.offset_jump_count:raise ValueError("downward_ros_clock_changed")
        image=(decodeRaw10(frame.stitched,self.nativeWidth,self.nativeHeight,self.rawKernel)
               if self.pixelFormat=="pRAA" else
               decodeMono(frame.stitched,self.nativeWidth,self.nativeHeight,self.pixelFormat))
        if self.imageScale!=1.:
            image=cv2.resize(image,(self.width,self.height),interpolation=cv2.INTER_AREA)
        now=self.get_clock().now().nanoseconds*1e-9
        if not 0<=now-frame.capture_ros_s<=self.maxAge:raise ValueError("downward_sample_age_after_decode")
        msg=self.bridge.cv2_to_imgmsg(image,encoding="mono8")
        msg.header=frame.header();msg.header.frame_id=self.frameId
        info=CameraInfo();info.header=msg.header;info.width=self.width;info.height=self.height
        info.distortion_model="plumb_bob";info.k=self.K.ravel().tolist();info.d=self.D.tolist()
        info.r=np.eye(3).ravel().tolist()
        info.p=np.column_stack((self.K,np.zeros(3))).ravel().tolist()
        if not self.stop.is_set() and self.context.ok():
            self.pub.publish(msg);self.infoPub.publish(info)
            self.published+=1;self.lastCapture=frame.capture_ros_s
            self.lastUncertainty=frame.capture_uncertainty_s

    def processFrame(self,frame):
        try:self.publishFrame(frame)
        except ValueError as exc:
            if str(exc) == "downward_clock_uncertainty":
                self.droppedUncertain+=1
            elif str(exc) in ("downward_sample_age","downward_sample_age_after_decode"):
                self.droppedStale+=1
            else:
                raise

    def capture(self):
        try:
            if self.media is not None:
                owner=subprocess.run(["fuser",str(self.get_parameter("device").value)],capture_output=True)
                if owner.returncode!=1:raise RuntimeError("imx219_device_in_use_or_unreadable")
                self.media.configure(self.nativeWidth,self.nativeHeight)
            self.source.open()
            while not self.stop.is_set() and self.context.ok():
                frame=self.source.nextFrame();self.frames+=1
                if self.worker is not None:self.worker.submit(frame)
                else:self.processFrame(frame)
        except Exception as exc:
            self.error=str(self.worker.error) if self.worker is not None and self.worker.error is not None else str(exc)
            if self.context.ok():self.get_logger().error("downward capture stopped: "+str(exc))
        finally:
            if self.worker is not None:self.worker.close()
            if self.source is not None:self.source.close()
            if self.media is not None:
                try:self.media.close()
                except Exception as exc:
                    self.error=str(exc)
                    if self.context.ok():self.get_logger().error(self.error)

    def report(self):
        self.stats.publish(String(data=json.dumps(dict(frames=self.frames,published=self.published,
            device=self.get_parameter("device").value,pixel_format=self.pixelFormat,
            dropped_stale=self.droppedStale,dropped_uncertain=self.droppedUncertain,worker=None if self.worker is None else self.worker.snapshot(),
            image_size=[self.width,self.height],native_image_size=[self.nativeWidth,self.nativeHeight],
            image_scale=self.imageScale,native_raw10_library=None if self.rawKernel is None else self.rawKernel.path,
            opencv_threads=cv2.getNumThreads(),error=self.error,
            last_capture_ros_s=self.lastCapture,timestamp_uncertainty_s=self.lastUncertainty,
            sample_time_source="V4L2_driver_to_ROS",exposure_offset_verified=False),allow_nan=False)))

    def destroy_node(self):
        self.stop.set()
        if self.worker is not None:self.worker.close()
        if self.thread is not None:
            self.thread.join(timeout=2.)
            if self.thread.is_alive():self.get_logger().error("downward capture thread did not stop")
        return super().destroy_node()


def main():
    rclpy.init();node=None
    try:
        node=DownwardSource()
        rclpy.spin(node)
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
