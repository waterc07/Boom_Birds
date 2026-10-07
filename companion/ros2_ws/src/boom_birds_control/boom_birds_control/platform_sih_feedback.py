"""Read actual local SIH uORB controller mode and trajectory velocity. TEST-ONLY."""
import re
from collections import deque
import subprocess
import threading
import time
from pathlib import Path
from .sih_guard import verify_sih_process
from .platform_landing import HandoffFeedback


def fields(text):
    age=re.search(r"timestamp:.*?\(([\d.]+) seconds ago\)",text)
    if age is None: raise ValueError("missing_uorb_timestamp")
    scalars=dict(re.findall(r"^\s*(\w+):\s*(True|False|[-+\d.e]+)\s*$",text,re.M))
    return scalars,float(age.group(1))


class SihVelocityFeedback:
    def __init__(self,pid,core,outdir=None):
        if not verify_sih_process(pid): raise ValueError("local_sih_pid_required")
        self.pid,self.core=pid,core
        self.outdir=Path(outdir) if outdir else None
        if self.outdir: self.outdir.mkdir(parents=True,exist_ok=True)
        self.stop=threading.Event()
        self.lock=threading.Lock()
        self.latest=None
        self.estimator_valid=False
        self.raw=deque(maxlen=1000)
        self.raw_evicted=0
        self.thread=threading.Thread(target=self._run,daemon=True)
        self.thread.start()

    def snapshot(self):
        with self.lock:return self.latest,self.estimator_valid

    def close(self):
        self.stop.set();self.thread.join(timeout=2.)
        if self.outdir:
            import json
            (self.outdir/"uorb-feedback.json").write_text(json.dumps(list(self.raw),indent=2))
            (self.outdir/"summary.json").write_text(json.dumps(dict(capacity=self.raw.maxlen, evicted=self.raw_evicted)))

    def _run(self):
        while not self.stop.is_set():
            local_read=False
            try:
                if not verify_sih_process(self.pid):raise ValueError("sih_process_changed")
                binary=Path("/proc")/str(self.pid)/"exe"
                binary=binary.resolve(strict=True)
                samples={}
                for topic in ("offboard_control_mode","trajectory_setpoint","vehicle_control_mode","vehicle_local_position"):
                    result=subprocess.run([str(binary.with_name("px4-listener")),topic,"-n","1"],
                        cwd=binary.parent.parent/"rootfs/0",capture_output=True,text=True,timeout=.4)
                    if result.returncode:raise ValueError("uorb_listener")
                    samples[topic]=result.stdout
                ocm,age=fields(samples["offboard_control_mode"])
                ctrl,cage=fields(samples["vehicle_control_mode"])
                local,lage=fields(samples["vehicle_local_position"])
                # 姿态导航不会刷新 trajectory_setpoint。准入的速度估计有效性
                # 只取本机位置估计；速度交接回执另检验 OCM 和目标时间。
                valid=(lage <= .2 and local.get("v_xy_valid")=="True"
                       and local.get("v_z_valid")=="True")
                with self.lock:self.latest,self.estimator_valid=None,valid
                local_read=True
                trajectory=samples["trajectory_setpoint"]
                _,tage=fields(trajectory)
                velocity=re.search(r"^\s*velocity:\s*\[([^]]+)\]",trajectory,re.M)
                if velocity is None:raise ValueError("missing_velocity")
                v=tuple(float(x) for x in velocity.group(1).split(","))
                # vehicle_control_mode 按事件发布；不以其旧时间戳判过期。
                # 连续输入 OCM、目标和速度估计须新鲜，控制标志仍须逐项成立。
                if max(age,lage,tage)>.2:raise ValueError("uorb_stale")
                active=(ocm.get("velocity")=="True" and ocm.get("position")=="False"
                        and ocm.get("attitude")=="False"
                        and ctrl.get("flag_control_velocity_enabled")=="True"
                        and ctrl.get("flag_control_position_enabled")=="False"
                        and ctrl.get("flag_control_offboard_enabled")=="True")
                now=time.monotonic()
                # MAVLink has no application sequence echo. Correlate actual PX4 target
                # with the most recent sent velocity; the evidence is explicitly SIH-only.
                matches=[seq for seq,(stamp,value) in list(self.core.sent_history.items())
                         if 0 <= now-max(age,tage)-stamp <= .3 and value is not None
                         and len(v)==3 and all(abs(a-b)<=.01 for a,b in zip(v,value))]
                ack=None
                if active and valid and matches:
                    ack=HandoffFeedback(now-max(age,tage),self.core.token,max(matches),self.core.epoch,
                        True,False,v,"PX4_OCM_AND_SETPOINT_ECHO")
                with self.lock:self.latest,self.estimator_valid=ack,valid
                if len(self.raw) == self.raw.maxlen:
                    self.raw_evicted += 1
                self.raw.append(dict(stamp=now,active=active,valid=valid,samples=samples))
            except (ValueError,OSError,subprocess.TimeoutExpired):
                with self.lock:
                    self.latest=None
                    if not local_read:self.estimator_valid=False
            self.stop.wait(.08)
