"""Read actual local SIH uORB controller mode and trajectory velocity. TEST-ONLY."""
import json
import math
import os
import re
from collections import deque
import subprocess
import threading
import time
from pathlib import Path
from .sih_guard import verifySihProcess
from .platform_landing import HandoffFeedback

#: 由独立探针进程写出的 uORB 快照文件；控制进程内不再派生 px4-listener 子进程。
PROBE_ENV = "BB_SIH_FEEDBACK_FILE"


def fields(text):
    age=re.search(r"timestamp:.*?\(([\d.]+) seconds ago\)",text)
    if age is None: raise ValueError("missing_uorb_timestamp")
    scalars=dict(re.findall(r"^\s*(\w+):\s*(True|False|[-+\d.e]+)\s*$",text,re.M))
    return scalars,float(age.group(1))


class SihVelocityFeedback:
    def __init__(self,pid,core,outdir=None,feedPath=None):
        if feedPath is None:
            feedPath=os.environ.get(PROBE_ENV,"") or None
        if feedPath is None and not verifySihProcess(pid):
            raise ValueError("local_sih_pid_required")
        self.pid,self.core=pid,core
        self.feedPath=Path(feedPath) if feedPath else None
        self.outdir=Path(outdir) if outdir else None
        if self.outdir: self.outdir.mkdir(parents=True,exist_ok=True)
        self.stop=threading.Event()
        self.lock=threading.Lock()
        self.latest=None
        self.estimatorValid=False
        self.raw=deque(maxlen=1000)
        self.rawEvicted=0
        target=self._runFile if self.feedPath is not None else self._run
        self.thread=threading.Thread(target=target,daemon=True)
        self.thread.start()

    def snapshot(self):
        with self.lock:return self.latest,self.estimatorValid

    def close(self):
        self.stop.set();self.thread.join(timeout=2.)
        if self.outdir:
            (self.outdir/"uorb-feedback.json").write_text(json.dumps(list(self.raw),indent=2))
            (self.outdir/"summary.json").write_text(json.dumps(dict(capacity=self.raw.maxlen, evicted=self.rawEvicted)))

    def _readSamples(self):
        try:
            payload=json.loads(self.feedPath.read_text())
            if not isinstance(payload, dict):
                return None
            stamp = payload.get("stamp")
            now = time.monotonic()
            if (type(stamp) not in (int, float) or not math.isfinite(stamp)
                    or not 0 <= now-stamp <= .2):
                return None
            samples=payload.get("samples")
            if not isinstance(samples,dict):
                return None
            if any(not isinstance(samples.get(topic),str) for topic in
                   ("offboard_control_mode","trajectory_setpoint","vehicle_control_mode","vehicle_local_position")):
                return None
            sampleStamps = payload.get("sample_stamps", {topic:stamp for topic in samples})
            if (not isinstance(sampleStamps, dict) or any(
                    type(sampleStamps.get(topic)) not in (int, float)
                    or not math.isfinite(sampleStamps[topic])
                    or not 0 <= now-sampleStamps[topic] <= .2 for topic in samples)):
                return None
            sampleEnds = payload.get("sample_ends", sampleStamps)
            if (not isinstance(sampleEnds, dict) or any(
                    type(sampleEnds.get(topic)) not in (int, float)
                    or not math.isfinite(sampleEnds[topic])
                    or not sampleStamps[topic] <= sampleEnds[topic] <= stamp for topic in samples)):
                return None
            return samples, stamp, sampleStamps, sampleEnds
        except (OSError,ValueError,TypeError):
            return None

    def _runFile(self):
        while not self.stop.is_set():
            samples=self._readSamples()
            if samples is None:
                with self.lock:
                    self.latest=None
                    self.estimatorValid=False
            else:
                try:
                    self._consume(samples[0], sampledAt=samples[1], sampleStamps=samples[2], sampleEnds=samples[3])
                except (ValueError,KeyError,TypeError):
                    with self.lock:
                        self.latest=None
                        self.estimatorValid=False
            self.stop.wait(.05)

    def _run(self):
        while not self.stop.is_set():
            localRead=False
            try:
                if not verifySihProcess(self.pid):raise ValueError("sih_process_changed")
                binary=Path("/proc")/str(self.pid)/"exe"
                binary=binary.resolve(strict=True)
                samples={}
                for topic in ("offboard_control_mode","trajectory_setpoint","vehicle_control_mode","vehicle_local_position"):
                    result=subprocess.run([str(binary.with_name("px4-listener")),topic,"-n","1"],
                        cwd=binary.parent.parent/"rootfs/0",capture_output=True,text=True,timeout=.4)
                    if result.returncode:raise ValueError("uorb_listener")
                    samples[topic]=result.stdout
                self._consume(samples,localRead=True)
            except (ValueError,OSError,subprocess.TimeoutExpired):
                with self.lock:
                    self.latest=None
                    if not localRead:self.estimatorValid=False
            self.stop.wait(.08)

    def _consume(self,samples,localRead=True,*,sampledAt=None,sampleStamps=None,sampleEnds=None):
        now = time.monotonic()
        sampleAge = 0. if sampledAt is None else max(0., now-sampledAt)
        ocm,age=fields(samples["offboard_control_mode"])
        ctrl,cage=fields(samples["vehicle_control_mode"])
        local,lage=fields(samples["vehicle_local_position"])
        # 姿态导航不会刷新 trajectory_setpoint。准入的速度估计有效性只取本机位置估计；
        # 速度交接回执另检验 OCM 和目标时间。
        age += sampleAge if sampleStamps is None else max(0., now-sampleStamps["offboard_control_mode"])
        lage += sampleAge if sampleStamps is None else max(0., now-sampleStamps["vehicle_local_position"])
        valid=(lage <= .2 and local.get("v_xy_valid")=="True" and local.get("v_z_valid")=="True")
        with self.lock:self.latest,self.estimatorValid=None,valid
        trajectory=samples["trajectory_setpoint"]
        _,tage=fields(trajectory)
        velocity=re.search(r"^\s*velocity:\s*\[([^]]+)\]",trajectory,re.M)
        if velocity is None:raise ValueError("missing_velocity")
        v=tuple(float(x) for x in velocity.group(1).split(","))
        # vehicle_control_mode 按事件发布；不以其旧时间戳判过期。
        # 连续输入 OCM、目标和速度估计须新鲜，控制标志仍须逐项成立。
        # 速度估计有效性只取本地位置（已在上面写入）；trajectory/OCM 过期只会
        # 使回执不可用，不能把可用的 PX4 速度估计一起判成不可用。
        tage += sampleAge if sampleStamps is None else max(0., now-sampleStamps["trajectory_setpoint"])
        self.diagnostic = dict(stamp=now, ocm_age_s=age, local_age_s=lage,
                               target_age_s=tage, estimator_valid=valid)
        if max(age,lage,tage) > .2:return
        active=(ocm.get("velocity")=="True" and ocm.get("position")=="False"
                and ocm.get("attitude")=="False"
                and ctrl.get("flag_control_velocity_enabled")=="True"
                and ctrl.get("flag_control_position_enabled")=="False"
                and ctrl.get("flag_control_offboard_enabled")=="True")
        # MAVLink has no application sequence echo. Correlate actual PX4 target
        # with the most recent sent velocity; the evidence is explicitly SIH-only.
        # CLI 输出在进程启动与退出之间生成；新鲜度取最早时刻，目标关联
        # 使用最晚时刻，避免把已经收到的变化目标估早到发送之前。
        targetAt = now-tage
        targetLatest = targetAt if sampleEnds is None else targetAt + (
            sampleEnds["trajectory_setpoint"]-sampleStamps["trajectory_setpoint"])
        matches=[seq for seq,(stamp,value) in list(self.core.sent_history.items())
                 if 0 <= targetLatest-stamp <= .3 and value is not None
                 and len(v)==3 and all(abs(a-b)<=.01 for a,b in zip(v,value))]
        self.diagnostic.update(active=active, matches=len(matches), echo=v)
        ack=None
        if active and valid and matches:
            ack=HandoffFeedback(targetLatest,self.core.token,max(matches),self.core.epoch,
                True,False,v,"PX4_OCM_AND_SETPOINT_ECHO")
        with self.lock:self.latest,self.estimatorValid=ack,valid
        if len(self.raw) == self.raw.maxlen:
            self.rawEvicted += 1
        self.raw.append(dict(stamp=now,active=active,valid=valid,samples=samples))
