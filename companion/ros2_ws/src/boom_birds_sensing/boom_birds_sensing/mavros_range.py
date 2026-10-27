"""DISTANCE_SENSOR decoding with PX4 boot-clock mapping; no receipt-time fallback."""
from dataclasses import dataclass
import math
import struct
from boom_birds_control.mavros_clock import ImuGate


@dataclass(frozen=True)
class RangeReading:
    stamp: float
    minimum_m: float
    maximum_m: float
    distance_m: float
    field_of_view: float
    sensor_id: int


class MavrosRangeDecoder:
    def __init__(self,sensorId=-1,systemId=1,componentId=1,maxAge=.15,maxFuture=.02):
        if not -1<=sensorId<=255 or not 1<=systemId<=255 or not 1<=componentId<=255:
            raise ValueError("range_source_selection")
        if not 0<maxAge<=1 or not 0<=maxFuture<=.02:raise ValueError("range_time_limits")
        self.sensorId,self.systemId,self.componentId=sensorId,systemId,componentId
        self.maxAge,self.maxFuture=maxAge,maxFuture
        self.gate=ImuGate(max_sample_age_s=maxAge,hold_last_sync_on_rtt=True)
        self.lastStamp=None
        self.reason="range_missing"
        self.candidates={}
        self.rejected=self.accepted=0

    def reject(self,reason):
        self.reason=reason;self.rejected+=1
        return None

    def onSync(self,msg,nowMono,nowRos):
        restarts=self.gate.restarts
        self.gate.onSync(msg,nowMono,nowRos)
        if restarts!=self.gate.restarts or self.gate.lastSync is None:self.lastStamp=None

    def parse(self,raw):
        if raw.msgid!=132 or raw.framing_status!=1:return None
        if raw.sysid!=self.systemId or raw.compid!=self.componentId:return None
        data=b"".join(struct.pack("<Q",int(v)) for v in raw.payload64)[:int(raw.len)]
        if not 13<=len(data)<=39:
            self.reject("range_payload_length");return None
        boot,minimum,maximum,current,kind,identity,orientation,covariance=struct.unpack("<IHHHBBBB",data[:14].ljust(14,b"\0"))
        self.candidates[str(identity)]=dict(id=identity,sensor_type=kind,orientation=orientation)
        if self.sensorId<0:return self.reject("range_sensor_selection_required")
        if identity!=self.sensorId:return None
        if orientation!=25:return self.reject("range_sensor_not_downward")
        if kind not in (0,1,2,3):return self.reject("range_sensor_type_unknown")
        if not 0<minimum<maximum or not minimum<=current<=maximum:
            return self.reject("range_outside_sensor_limits")
        fov=struct.unpack("<f",data[14:18])[0] if len(data)>=18 else 0.
        if not math.isfinite(fov) or not 0<=fov<=math.pi:return self.reject("range_fov_invalid")
        if len(data)>=39 and data[38]==1:return self.reject("range_signal_invalid")
        return boot,minimum*.01,maximum*.01,current*.01,fov,identity

    def map(self,raw,*,nowMono,nowRos,connected):
        values=self.parse(raw)
        if values is None:return None
        g=self.gate
        if not connected:return self.reject("range_heartbeat_stale")
        if g.lastSync is None or not 0<=nowMono-g.lastSync<=g.sync_timeout_s:
            return self.reject("range_timesync_stale")
        if g.count<g.converge_samples or g.lastRemote is None:return self.reject("range_clock_not_ready")
        if g.rosOffset is None or abs(nowRos-nowMono-g.rosOffset)>.05:
            g.reset("ros_clock_changed");return self.reject("range_ros_clock_changed")
        boot,minimum,maximum,current,fov,identity=values
        remoteMs=int(g.lastRemote)//1000000
        base=remoteMs//(1<<32)*(1<<32)
        extended=base+boot
        if extended-remoteMs>1<<31:extended-=1<<32
        elif remoteMs-extended>1<<31:extended+=1<<32
        stamp=(extended*1000000+g.offsetNs)*1e-9
        if not -self.maxFuture<=nowRos-stamp<=self.maxAge:return self.reject("range_sample_age")
        if self.lastStamp is not None and stamp<=self.lastStamp:return self.reject("range_nonmonotonic")
        self.lastStamp=stamp;self.reason="ready";self.accepted+=1
        return RangeReading(stamp,minimum,maximum,current,fov,identity)
