"""IMX219 RAW10 decoding and reversible CSI media configuration."""
from pathlib import Path
import re
import subprocess
import cv2
import numpy as np


RAW10_FORMAT=int.from_bytes(b"pRAA","little")
GAMMA_LUT=np.uint8(np.power(np.clip((np.arange(1024)-64)/(1023-64),0,1),1/2.2)*255)


def decodeRaw10(payload,width,height,kernel=None):
    if width%4 or height<=0 or len(payload)%height:raise ValueError("raw10_layout")
    stride=len(payload)//height
    if not width*5//4<=stride<=width*5//4+256:raise ValueError("raw10_stride")
    if kernel is not None:
        return cv2.cvtColor(kernel.unpackRaw10(payload,width,height,GAMMA_LUT),cv2.COLOR_BayerRGGB2GRAY)
    groups=np.frombuffer(payload,np.uint8).reshape(height,stride)[:,:width*5//4].reshape(height,width//4,5)
    raw=(groups[:,:,:4].astype(np.uint16)<<2)|((groups[:,:,4,None].astype(np.uint16)>>np.array([0,2,4,6],np.uint16))&3)
    bayer=GAMMA_LUT[raw.reshape(height,width)]
    return cv2.cvtColor(bayer,cv2.COLOR_BayerRGGB2GRAY)


class Imx219Media:
    def __init__(self,device,media,sensor,runner=None):
        self.device,self.media,self.sensor=device,media,sensor
        self.runner=runner or self.run
        self.previous=[]
        self.linkEnabled=None
        self.videoFormat=None

    @staticmethod
    def run(args):
        result=subprocess.run(args,capture_output=True,text=True,timeout=8)
        if result.returncode:raise RuntimeError("media_command_failed:"+str(args)+":"+result.stderr.strip())
        return result.stdout

    def command(self,*args):
        return self.runner(["media-ctl","-d",self.media,*args])

    def configure(self,width,height):
        if (width,height)!=(1640,1232):raise ValueError("imx219_requires_calibrated_1640x1232_mode")
        topology=self.command("-p")
        if "imx219" not in self.sensor.lower() or self.sensor not in topology:
            raise ValueError("imx219_sensor_entity")
        route=self.command("-e","rp1-cfe-csi2_ch0").strip()
        if Path(route).resolve()!=Path(self.device).resolve():raise ValueError("imx219_capture_device")
        link=re.search(r'-> "rp1-cfe-csi2_ch0":0 \[([^]]*)\]',topology)
        if link is None:raise ValueError("imx219_csi_route")
        self.linkEnabled="ENABLED" in link[1]
        for entity,pad in ((self.sensor,0),("csi2",0),("csi2",4)):
            text=self.command("--get-v4l2",f'"{entity}":{pad}')
            match=re.search(r"fmt:([A-Za-z0-9_]+)/([0-9]+)x([0-9]+)",text)
            if match is None:raise ValueError("imx219_original_format")
            code,w,h=match.groups()
            self.previous.append((entity,pad,f"fmt:{code}/{w}x{h} field:none"))
        subdev=self.command("-e",self.sensor).strip()
        controls=self.runner(["v4l2-ctl","-d",subdev,"--get-ctrl=horizontal_flip,vertical_flip,test_pattern"])
        if any(re.search(r"\b"+name+r":\s*0\b",controls) is None
               for name in ("horizontal_flip","vertical_flip","test_pattern")):
            raise ValueError("imx219_flip_or_test_pattern")
        fmt=self.runner(["v4l2-ctl","-d",self.device,"--get-fmt-video"])
        size=re.search(r"Width/Height\s*:\s*(\d+)/(\d+)",fmt)
        pixel=re.search(r"Pixel Format\s*:\s*'([^']+)'",fmt)
        if size is None or pixel is None:raise ValueError("imx219_original_video_format")
        self.videoFormat=(*size.groups(),pixel[1])
        self.command("-l",'"csi2":4 -> "rp1-cfe-csi2_ch0":0 [1]')
        for entity,pad,_ in self.previous:
            self.command("-V",f'"{entity}":{pad} [fmt:SRGGB10_1X10/{width}x{height} field:none]')
        current=self.command("--get-v4l2",f'"{self.sensor}":0')
        if "SRGGB10_1X10/1640x1232" not in current or "crop:(8,8)/3280x2464" not in current.replace(" ",""):
            raise ValueError("imx219_calibrated_crop")

    def close(self):
        errors=[]
        for entity,pad,fmt in self.previous:
            try:self.command("-V",f'"{entity}":{pad} [{fmt}]')
            except Exception as exc:errors.append(str(exc))
        if self.videoFormat is not None:
            w,h,pixel=self.videoFormat
            try:self.runner(["v4l2-ctl","-d",self.device,f"--set-fmt-video=width={w},height={h},pixelformat={pixel}"])
            except Exception as exc:errors.append(str(exc))
        if self.linkEnabled is not None:
            try:self.command("-l",f'"csi2":4 -> "rp1-cfe-csi2_ch0":0 [{int(self.linkEnabled)}]')
            except Exception as exc:errors.append(str(exc))
        self.previous=[];self.linkEnabled=self.videoFormat=None
        if errors:raise RuntimeError("imx219_restore_failed:"+repr(errors))
