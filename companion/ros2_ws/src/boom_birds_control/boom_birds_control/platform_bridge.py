"""Optional Px4Interface bridge; software/SIH only until a hardware feedback adapter exists."""
from dataclasses import asdict
import json
from pathlib import Path
import time
import yaml
from std_msgs.msg import String
from sensor_msgs.msg import Range
from std_srvs.srv import Trigger
from .platform_model import BoardObservation
from .platform_executor import PlatformExecutor
from .platform_landing import FlightSample, RangeSample


class PlatformBridge:
    def __init__(self,node,path,test_only):
        self.node=node
        profile=yaml.safe_load(Path(path).read_text())
        # Validate calibration before any backend connection.
        from .platform_landing import PlatformLanding
        PlatformLanding(profile,test_only=test_only)
        from .sih_guard import verify_sih_process
        pid=int(node.get_parameter("sih_pid").value)
        self.sih=verify_sih_process(pid)
        if not test_only:
            raise ValueError("hardware_velocity_confirmation_adapter_not_verified")
        if not self.sih and not node.get_parameter("dry_run").value and node.get_parameter("backend").value!="fake":
            raise ValueError("platform_test_only_requires_dry_run_or_local_sih")
        node.backend.allow_platform_handoff=True
        self.observation=self.range=None
        self.release_service=node.create_client(Trigger,"/boom_birds/compute/release")
        self.release_future=None
        self.executor=PlatformExecutor(node.backend,profile,test_only=test_only,
            revoke_navigation=self.revoke,release_compute=self.release,
            native_land=lambda reason:node.backend.set_mode("auto:land"),disarm=node.backend.disarm)
        self.feedback_reader=None
        if self.sih:
            from .platform_sih_feedback import SihVelocityFeedback
            self.feedback_reader=SihVelocityFeedback(pid,self.executor.core)
        self.pub=node.create_publisher(String,"/boom_birds/platform/status",10)
        node.create_subscription(String,"/boom_birds/platform/observation",self.on_observation,10)
        node.create_subscription(Range,"/boom_birds/platform/range",self.on_range,10)
        node.create_subscription(String,"/boom_birds/platform/request",self.request,10)
        self.output=None

    @property
    def owns_output(self):return self.executor.owner!="navigation"

    def revoke(self):
        self.node._cmd=None
        self.node.ingress.cancel("platform_handoff")
        return True

    def release(self,token):
        if self.executor.core.confirmed and self.release_service.service_is_ready():
            self.release_future=self.release_service.call_async(Trigger.Request())
        return False

    def on_observation(self,msg):
        try:
            item=json.loads(msg.data)
            # Image observation retains sample time in the ROS domain on the wire.
            age=self.node.get_clock().now().nanoseconds*1e-9-float(item["stamp"])
            item["stamp"]=time.monotonic()-age
            self.observation=BoardObservation(**item)
        except (ValueError,KeyError,TypeError):
            self.observation=None

    def on_range(self,msg):
        stamp=self.node._observation_time(msg)
        if stamp is None:self.range=None
        else:self.range=RangeSample(stamp,msg.range,
            msg.min_range<=msg.range<=msg.max_range and msg.header.frame_id=="range_sensor")

    def request(self,msg):
        try:
            item=json.loads(msg.data)
            if item["action"]=="cancel":self.executor.cancel("manual_cancel")
            elif item["action"] in ("land","takeoff"):
                if not self.node.ingress.session or item.get("session")!=self.node.ingress.session:
                    raise ValueError("platform_request_old_session")
                self.executor.request(item["action"])
            else:raise ValueError("unknown_platform_action")
        except (ValueError,KeyError,TypeError) as exc:self.node.get_logger().error(str(exc))

    def tick(self,now):
        state=self.node.backend.read_vehicle_state()
        c=self.executor.core.cfg
        ack,valid=self.feedback_reader.snapshot() if self.feedback_reader else (None,False)
        age=state.attitude_age_s
        position_age=state.position_age_s
        if (age is None or position_age is None or age>c.telemetry_timeout_s
                or position_age>c.telemetry_timeout_s or state.velocity_ned_m_s is None
                or any(x is None for x in (state.roll_rad,state.pitch_rad,state.yaw_rad))):
            flight=None
        else:
            landed=(state.landed_state==1 if state.landed_age_s is not None
                    and state.landed_age_s<=1.5 else None)
            flight=FlightSample(now-max(age,position_age),state.roll_rad,state.pitch_rad,state.yaw_rad,
                state.velocity_ned_m_s,state.connected,state.is_offboard,valid,landed,
                state.armed,state.restart_epoch)
        if self.release_future is not None and self.release_future.done():
            try:self.executor.release_verified=self.release_future.result().success
            except Exception:self.executor.release_verified=False
            self.release_future=None
        from .px4_failsafe import SignalId
        diagnostic=self.node.core.monitor.report()
        nav_ready=self.node._position_allowed_by_alignment()[0]
        for signal in (SignalId.VIO_POSE,SignalId.IMU,SignalId.CAMERA):
            stamp=diagnostic["last_seen"].get(signal.value)
            timeout=diagnostic["timeouts"].get(signal.value,0.)
            nav_ready=nav_ready and stamp is not None and 0 <= now-stamp <= timeout
        self.output=self.executor.tick(now,self.observation,self.range,flight,ack,
                                       navigation_ready=nav_ready)
        self.pub.publish(String(data=json.dumps(dict(**asdict(self.output),
            session=self.node.ingress.session,owner=self.executor.owner,release_verified=self.executor.release_verified,
            test_only=True,feedback_source="local_SIH_uORB" if self.sih else "unavailable"),allow_nan=False)))
        return self.owns_output

    def close(self):
        if self.feedback_reader:self.feedback_reader.close()
