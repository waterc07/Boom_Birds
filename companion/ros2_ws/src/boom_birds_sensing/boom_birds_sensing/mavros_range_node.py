"""PX4/MAVROS DISTANCE_SENSOR ingress; requests telemetry only."""
from collections import deque
from dataclasses import asdict
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import String
from mavros_msgs.msg import Mavlink,State,TimesyncStatus
from mavros_msgs.srv import CommandLong
from .mavros_range import MavrosRangeDecoder


class MavrosRangeNode(Node):
    def __init__(self,**kwargs):
        super().__init__("boom_birds_mavros_range",**kwargs)
        defaults=dict(mavros_namespace="/mavros",raw_topic="/uas1/mavlink_source",
            range_topic="/boom_birds/platform/range",sensor_id=-1,target_system=1,
            target_component=1,stream_rate_hz=50.,max_sample_age_s=.15)
        for key,value in defaults.items():self.declare_parameter(key,value)
        get=lambda key:self.get_parameter(key).value
        rate=float(get("stream_rate_hz"))
        if not 0<=rate<=100:raise ValueError("range_stream_rate")
        self.rate=rate
        self.decoder=MavrosRangeDecoder(int(get("sensor_id")),int(get("target_system")),
            int(get("target_component")),float(get("max_sample_age_s")))
        self.pending=deque(maxlen=32)
        self.connected=False;self.stateAt=None;self.future=None;self.requested=False
        self.published=0;self.lastPublished=None;self.lastRequest=-float("inf")
        ns=str(get("mavros_namespace")).rstrip("/")
        self.pub=self.create_publisher(Range,str(get("range_topic")),qos_profile_sensor_data)
        self.stats=self.create_publisher(String,"/boom_birds/platform/range_status",10)
        self.create_subscription(State,ns+"/state",self.onState,qos_profile_sensor_data)
        self.create_subscription(TimesyncStatus,ns+"/timesync_status",self.onSync,qos_profile_sensor_data)
        self.create_subscription(Mavlink,str(get("raw_topic")),self.onRaw,
            QoSProfile(depth=1000,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.command=self.create_client(CommandLong,ns+"/cmd/command")
        self.create_timer(.005,self.drain)
        self.create_timer(.2,self.report)

    def healthy(self,now):
        return self.connected and self.stateAt is not None and 0<=now-self.stateAt<=3.

    def onState(self,msg):
        now=time.monotonic();ros=self.get_clock().now().nanoseconds*1e-9
        age=ros-msg.header.stamp.sec-msg.header.stamp.nanosec*1e-9
        self.connected=bool(msg.connected) and 0<=age<=3.
        self.stateAt=now-max(0.,age)
        if not self.connected:
            self.decoder.gate.reset("heartbeat_stale")
            self.decoder.lastStamp=None;self.pending.clear();self.requested=False

    def onSync(self,msg):
        before=self.decoder.gate.restarts
        self.decoder.onSync(msg,time.monotonic(),self.get_clock().now().nanoseconds*1e-9)
        if before!=self.decoder.gate.restarts or self.decoder.gate.lastSync is None:
            self.pending.clear();self.requested=False

    def onRaw(self,msg):
        now=time.monotonic();ros=self.get_clock().now().nanoseconds*1e-9
        reading=self.decoder.map(msg,nowMono=now,nowRos=ros,connected=self.healthy(now))
        if reading is None:return
        if len(self.pending)==self.pending.maxlen:
            self.decoder.reject("range_queue_overflow");return
        self.pending.append(reading)
        self.drain()

    def drain(self):
        now=time.monotonic();ros=self.get_clock().now().nanoseconds*1e-9
        gate=self.decoder.gate
        if not self.healthy(now) or gate.lastSync is None or not 0<=now-gate.lastSync<=gate.sync_timeout_s:
            self.pending.clear();return
        while self.pending:
            reading=self.pending[0]
            if reading.stamp>ros:return
            self.pending.popleft()
            if ros-reading.stamp>self.decoder.maxAge:
                self.decoder.reject("range_publish_stale");continue
            if self.lastPublished is not None and reading.stamp<=self.lastPublished:
                self.decoder.reject("range_publish_nonmonotonic");continue
            msg=Range()
            ns=round(reading.stamp*1e9)
            msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(ns,1000000000)
            msg.header.frame_id="range_sensor"
            msg.radiation_type=Range.INFRARED
            msg.field_of_view=float(reading.field_of_view)
            msg.min_range=float(reading.minimum_m);msg.max_range=float(reading.maximum_m)
            msg.range=float(reading.distance_m)
            self.pub.publish(msg);self.published+=1;self.lastPublished=reading.stamp

    def report(self):
        now=time.monotonic()
        if self.rate>0 and self.healthy(now) and not self.requested and self.future is None and now-self.lastRequest>5 and self.command.service_is_ready():
            self.lastRequest=now
            self.future=self.command.call_async(CommandLong.Request(command=511,param1=132.,param2=1e6/self.rate))
            def done(future):
                self.future=None
                try:self.requested=bool(future.result().success)
                except Exception:self.requested=False
            self.future.add_done_callback(done)
        self.stats.publish(String(data=json.dumps(dict(connected=self.healthy(now),
            selected_sensor_id=self.decoder.sensorId,candidates=list(self.decoder.candidates.values()),
            reason=self.decoder.reason,published=self.published,rejected=self.decoder.rejected,
            timesync_count=self.decoder.gate.count,stream_request_accepted=self.requested,
            sample_time_source="PX4_boot_timesync"),allow_nan=False)))


def main():
    rclpy.init();node=None
    try:
        node=MavrosRangeNode()
        rclpy.spin(node)
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
