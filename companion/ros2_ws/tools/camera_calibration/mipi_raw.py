"""Pi 5 IMX219 的 pRAA 采集，不经过 ISP，不连接 ROS 或飞控。"""
import os
import re
import select
import subprocess
import time
import cv2
import numpy as np


def run(*args):
    return subprocess.run(args, check=True, capture_output=True, text=True, timeout=8).stdout


def unpack_raw10(payload, width, height, stride):
    if width % 4 or stride < width * 5 // 4 or len(payload) != stride * height:
        raise ValueError('RAW10 长度或行步幅不符')
    rows = np.frombuffer(payload, np.uint8).reshape(height, stride)
    # 每 5 字节编码 4 像素：前 4 字节为高 8 位，第 5 字节依次存各像素低 2 位。
    # 只解包有效行宽，stride 尾部填充不参与像素。
    groups = rows[:, :width * 5 // 4].reshape(height, width // 4, 5)
    pixels = (groups[:, :, :4].astype(np.uint16) << 2) | ((groups[:, :, 4, None].astype(np.uint16) >> np.array([0, 2, 4, 6], dtype=np.uint16)) & 3)
    return pixels.reshape(height, width).astype(np.uint16)


GAMMA_LUT = np.uint8(np.power(np.clip((np.arange(1024)-64)/(1023-64), 0, 1), 1/2.2)*255)


def demosaic(payload, width, height, stride):
    raw = unpack_raw10(payload, width, height, stride)
    # 固定黑电平和 gamma，无几何缩放；使用显式 RGGB 命名避免两字母 Bayer 别名歧义。
    bayer = GAMMA_LUT[raw]
    return cv2.cvtColor(bayer, cv2.COLOR_BayerRGGB2BGR)


class RawCamera:
    def __init__(self, args):
        self.args = args
        self.process = None
        self.metadata = {}
        self.pending = bytearray()

    def configure(self):
        a = self.args
        if 'imx219' not in run('media-ctl', '-d', a.media, '-p'):
            raise ValueError('指定 media 设备不是 IMX219 管线')
        owned = subprocess.run(['fuser', a.device], capture_output=True)
        if owned.returncode == 0:
            raise ValueError('MIPI 相机已被其他进程占用')
        run('media-ctl', '-d', a.media, '-l', '"csi2":4 -> "rp1-cfe-csi2_ch0":0 [1]')
        for entity, pad in [('imx219 11-0010', 0), ('csi2', 0), ('csi2', 4)]:
            run('media-ctl', '-d', a.media, '-V',
                f'"{entity}":{pad} [fmt:SRGGB10_1X10/{a.width}x{a.height} field:none]')
        run('v4l2-ctl', '-d', a.subdev, '--set-ctrl=horizontal_flip=0,vertical_flip=0,test_pattern=0')
        self.set_exposure(a.exposure, a.gain)
        run('v4l2-ctl', '-d', a.device,
            f'--set-fmt-video=width={a.width},height={a.height},pixelformat=pRAA')
        fmt = run('v4l2-ctl', '-d', a.device, '--get-fmt-video')
        dims = re.search(r'Width/Height\s*:\s*(\d+)/(\d+)', fmt)
        stride = re.search(r'Bytes per Line\s*:\s*(\d+)', fmt)
        image_bytes = re.search(r'Size Image\s*:\s*(\d+)', fmt)
        if not dims or tuple(map(int, dims.groups())) != (a.width, a.height) or "'pRAA'" not in fmt:
            raise ValueError('驱动格式回读与请求不符')
        self.stride = int(stride[1])
        self.frame_bytes = int(image_bytes[1])
        if self.frame_bytes != self.stride * a.height:
            raise ValueError('暂不支持该 RAW 缓冲布局')
        topology = run('media-ctl', '-d', a.media, '-p')
        sensor = topology.split('- entity 16: imx219', 1)[-1].split('- entity 18:', 1)[0]
        if 'crop:(8,8)/3280x2464' not in sensor or 'SRGGB10_1X10/1640x1232' not in sensor:
            raise ValueError('传感器裁剪或模式不符，拒绝标定')
        self.metadata.update(format=fmt, topology=topology,
                             device=a.device, subdev=a.subdev, media=a.media,
                             sensor='IMX219', image_size=[a.width, a.height],
                             timestamp_source='host_monotonic_after_pipe_read_not_exposure',
                             processing='RAW10 RGGB; black=64; gamma=2.2; no ISP, resize or flip')
        self.process = subprocess.Popen(
            ['v4l2-ctl', '-d', a.device, '--stream-mmap=4', '--stream-to=-'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)

    def set_exposure(self, exposure, gain):
        if not 4 <= exposure <= 1600 or not 0 <= gain <= 232:
            raise ValueError('曝光必须为 4..1600 行；模拟增益为 0..232')
        run('v4l2-ctl', '-d', self.args.subdev,
            f'--set-ctrl=exposure={exposure},analogue_gain={gain},digital_gain=256')
        controls = run('v4l2-ctl', '-d', self.args.subdev, '--get-ctrl=exposure,analogue_gain,digital_gain')
        if not all(f'{key}: {value}' in controls for key, value in
                   [('exposure', exposure), ('analogue_gain', gain), ('digital_gain', 256)]):
            raise ValueError('曝光/增益回读不符')
        self.metadata['controls'] = controls

    def read(self, stop):
        deadline = time.monotonic() + 3
        fd = self.process.stdout.fileno()
        while len(self.pending) < self.frame_bytes:
            if stop.is_set():
                return None
            if time.monotonic() > deadline:
                raise RuntimeError('RAW 取帧超过 3 秒')
            if select.select([fd], [], [], .2)[0]:
                chunk = os.read(fd, min(1024 * 1024, self.frame_bytes - len(self.pending)))
                if not chunk:
                    raise RuntimeError('V4L2 采集进程退出')
                self.pending.extend(chunk)
        result = bytes(self.pending)
        self.pending.clear()
        return result

    def close(self):
        if self.process is not None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
            self.process.stdout.close()
            self.process = None
