"""Separate ROS service for caller-owned VIO/stereo child processes."""
import json
import time
import math
from types import SimpleNamespace
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .compute_resources import ComputeResources


class ComputeReleaseNode(Node):
    def __init__(self, vio_processes, stereo_processes, expected_session, *,
                 confirmation_timeout_s=.5, **kwargs):
        super().__init__("compute_release_manager",**kwargs)
        if confirmation_timeout_s<=0:raise ValueError("confirmation_timeout")
        # 仅接受调用者持有的子进程；服务不能按进程名终止其它任务。
        self.resources=ComputeResources(vio_processes,stereo_processes)
        self.expected_session=expected_session
        self.timeout=confirmation_timeout_s
        self.received=None
        self.pending=None
        self.epoch=None
        self.last_sequence=-1
        self.create_subscription(String,"/boom_birds/platform/status",self.on_status,10)
        self.create_service(Trigger,"/boom_birds/compute/release",self.release)

    def on_status(self,msg):
        try:
            out=json.loads(msg.data)
            if (out["session"]!=self.expected_session() or out["release_compute"] is not True
                    or out["vio_required"] is not True or out["owner"]!="platform"
                    or out["state"] not in ("ALIGN","DESCEND","FLARE","TOUCHDOWN")
                    or not isinstance(out["token"],str) or not out["token"]):
                self.pending=None
                return
            now = time.monotonic()
            stamp, sequence, epoch = out["stamp_monotonic"], out["sequence"], out["fcu_epoch"]
            if (type(stamp) not in (int, float) or not math.isfinite(stamp)
                    or not 0 <= now-stamp <= self.timeout
                    or type(sequence) is not int or sequence <= self.last_sequence
                    or type(epoch) is not int or epoch < 0
                    or self.epoch is not None and epoch != self.epoch):
                self.pending=None
                return
            self.resources.confirm(SimpleNamespace(release_compute=True,token=out["token"]))
            self.epoch, self.last_sequence = epoch, sequence
            self.pending=out["token"]
            self.received=stamp
        except (ValueError,KeyError,TypeError):self.pending=None

    def release(self,request,response):
        response.success=False
        response.message="fresh_confirmed_platform_lease_required"
        if self.pending is not None and self.received is not None and 0<=time.monotonic()-self.received<=self.timeout:
            response.success=self.resources.release(self.pending)
            response.message="owned_processes_stopped" if response.success else "owned_process_still_running"
        return response
