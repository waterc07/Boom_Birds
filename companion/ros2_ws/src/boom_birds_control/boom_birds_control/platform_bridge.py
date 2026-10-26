"""Optional Px4Interface bridge; software/SIH only until a hardware feedback adapter exists."""
from dataclasses import asdict
import json
import math
from pathlib import Path
import time
import uuid
import yaml
from std_msgs.msg import String
from sensor_msgs.msg import Range
from std_srvs.srv import Trigger
from .platform_model import BoardObservation
from .platform_executor import PlatformExecutor, RuntimeOptions, SegmentedTrace
from .platform_landing import FlightSample, RangeSample


class PlatformBridge:
    def __init__(self,node,path,test_only):
        self.node=node
        self.producer = str(uuid.uuid4())
        self.status_sequence = 0
        profile=yaml.safe_load(Path(path).read_text())
        # Validate calibration before any backend connection.
        from .platform_landing import PlatformLanding
        PlatformLanding(profile,test_only=test_only)
        from .sih_guard import verifySihProcess
        pid=int(node.get_parameter("sih_pid").value)
        self.sih=verifySihProcess(pid)
        if not test_only:
            raise ValueError("hardware_velocity_confirmation_adapter_not_verified")
        if not self.sih and not node.get_parameter("dry_run").value and node.get_parameter("backend").value!="fake":
            raise ValueError("platform_test_only_requires_dry_run_or_local_sih")
        node.backend.allow_platform_handoff=True
        self.observation=self.range=None
        self.releaseService=node.create_client(Trigger,"/boom_birds/compute/release")
        self.releaseFuture=None
        self.releaseFutureToken=None
        node.declare_parameter("platform_trace_directory", "")
        options = RuntimeOptions(profile)
        tracePath = str(node.get_parameter("platform_trace_directory").value)
        trace = SegmentedTrace(tracePath, queueCapacity=options.trace_queue_capacity,
                               segmentRecords=options.trace_segment_records) if tracePath else None
        self.executor=PlatformExecutor(node.backend,profile,test_only=test_only,trace=trace,
            revokeNavigation=self.revoke,release_compute=self.release,
            nativeLand=lambda reason:node.backend.set_mode("auto:land"),disarm=node.backend.disarm)
        self.feedbackReader=None
        if self.sih:
            from .platform_sih_feedback import SihVelocityFeedback
            self.feedbackReader=SihVelocityFeedback(pid,self.executor.core)
        self.pub=node.create_publisher(String,"/boom_birds/platform/status",10)
        node.create_subscription(String,"/boom_birds/platform/observation",self.onObservation,10)
        node.create_subscription(Range,"/boom_birds/platform/range",self.onRange,10)
        node.create_subscription(String,"/boom_birds/platform/request",self.request,10)
        self.output=None

    @property
    def ownsOutput(self):return self.executor.owner!="navigation"

    def revoke(self):
        self.node._cmd=None
        self.node.ingress.cancel("platform_handoff")
        return True

    def release(self,token):
        if self.executor.core.confirmed and self.releaseService.service_is_ready():
            self.releaseFuture=self.releaseService.call_async(Trigger.Request())
            self.releaseFutureToken=token
            return None
        return False

    def onObservation(self,msg):
        try:
            item=json.loads(msg.data)
            if (not isinstance(item, dict) or type(item.get("valid")) is not bool
                    or any(type(item.get(k, 0.)) not in (int, float) or not math.isfinite(item.get(k, 0.))
                           for k in ("stamp", "reprojection_px", "ambiguity_ratio", "min_edge_px"))
                    or not isinstance(item.get("tag_ids", []), (list, tuple))
                    or any(type(i) is not int or i < 0 for i in item.get("tag_ids", []))
                    or not isinstance(item.get("quality", {}), dict)
                    or type(item.get("quality", {}).get("planar_distinct", True)) is not bool):
                raise ValueError("observation_wire_schema")
            # 按原 ROS 采样时间计算年龄，再映射到本机单调时钟；排队耗时不能在接收时清零。
            age=self.node.get_clock().now().nanoseconds*1e-9-float(item["stamp"])
            item["stamp"]=time.monotonic()-age
            self.observation=BoardObservation(**item)
            if self.observation.valid:
                self.observation.pose()
        except (ValueError,KeyError,TypeError):
            self.observation=None

    def onRange(self,msg):
        stamp=self.node._observationTime(msg)
        if stamp is None:self.range=None
        else:self.range=RangeSample(stamp,msg.range,
            msg.min_range<=msg.range<=msg.max_range and msg.header.frame_id=="range_sensor")

    def request(self,msg):
        try:
            item=json.loads(msg.data)
            if not self.node.ingress.session or item.get("session")!=self.node.ingress.session:
                raise ValueError("platform_request_old_session")
            if item["action"]=="cancel":
                self.executor.cancel("manual_cancel")
            elif item["action"] in ("land","takeoff"):
                # 同会话重复请求幂等；已有交接不能被另一种 intent 覆盖。
                if self.executor.core.state != "NAVIGATION" and self.executor.core.intent == item["action"]:
                    return
                self.executor.request(item["action"])
            else:raise ValueError("unknown_platform_action")
        except (ValueError,KeyError,TypeError) as exc:self.node.get_logger().error(str(exc))

    def tick(self,now):
        state=self.node.backend.readVehicleState()
        c=self.executor.core.cfg
        ack,valid=self.feedbackReader.snapshot() if self.feedbackReader else (None,False)
        age=state.attitude_age_s
        positionAge=state.position_age_s
        if (age is None or positionAge is None or age>c.telemetry_timeout_s
                or positionAge>c.telemetry_timeout_s or state.velocity_ned_m_s is None
                or any(x is None for x in (state.roll_rad,state.pitch_rad,state.yaw_rad))):
            flight=None
        else:
            landed=(state.landed_state==1 if state.landed_age_s is not None
                    and state.landed_age_s<=1.5 else None)
            flight=FlightSample(now-max(age,positionAge),state.roll_rad,state.pitch_rad,state.yaw_rad,
                state.velocity_ned_m_s,state.connected,state.is_offboard,valid,landed,
                state.armed,(state.restart_epoch << 32) | state.frame_reset_epoch)
        from .px4_failsafe import SignalId
        diagnostic=self.node.core.monitor.report()
        navReady=self.node._positionAllowedByAlignment()[0]
        for signal in (SignalId.VIO_POSE,SignalId.IMU,SignalId.CAMERA):
            stamp=diagnostic["last_seen"].get(signal.value)
            timeout=diagnostic["timeouts"].get(signal.value,0.)
            navReady=navReady and stamp is not None and 0 <= now-stamp <= timeout
        self.output=self.executor.tick(now,self.observation,self.range,flight,ack,
                                       navigationReady=navReady)
        # 本周期先处理 epoch、故障和释放超时，再接受异步结果。
        if self.releaseFuture is not None and self.releaseFuture.done():
            try:
                success = self.releaseFuture.result().success
            except Exception:
                success = False
            self.executor.releaseResult(self.releaseFutureToken, success)
            self.releaseFuture=None
            self.releaseFutureToken=None
        self.status_sequence += 1
        self.pub.publish(String(data=json.dumps(dict(**asdict(self.output),
            producer=self.producer, status_sequence=self.status_sequence,
            stamp_monotonic=now, fcu_epoch=self.executor.core.epoch,
            release_state=self.executor.release_state, release_attempts=self.executor.release_attempts,
            history_evicted=self.executor.history_evicted,
            session=self.node.ingress.session,owner=self.executor.owner,release_verified=self.executor.release_verified,
            test_only=True,feedback_source="local_SIH_uORB" if self.sih else "unavailable"),allow_nan=False)))
        return self.ownsOutput

    def close(self):
        if self.feedbackReader:self.feedbackReader.close()
        self.executor.close()
