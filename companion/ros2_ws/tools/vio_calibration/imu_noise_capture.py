"""飞控 HIGHRES_IMU 静止噪声采集；仅受限遥测，USB 供电。"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import queue
import signal
import threading
import time

from .vio_calibration import atomicJson, digest
from .vio_calibration_inputs import TelemetryLink


class NoiseLink(TelemetryLink):
    def __init__(self, device, out, jobs):
        from pymavlink.dialects.v20 import common as mav
        self.sampleParser = mav.MAVLink(None)
        self.sampleParser.robust_parsing = True
        self.jobs = jobs
        self.recording = False
        self.count = 0
        self.firstUs = self.lastUs = None
        self.max_gap_s = 0.0
        self.gapsOver30ms = 0
        super().__init__(device, out, checkBattery=False)

    def consume(self, data, forward=None):
        super().consume(data, forward)
        for msg in self.sampleParser.parse_buffer(data) or []:
            if (not self.recording or msg.get_type() != 'HIGHRES_IMU'
                    or msg.get_srcSystem() != 1 or msg.get_srcComponent() != 1):
                continue
            if msg.fields_updated & 63 != 63:
                raise RuntimeError('IMU six-axis fields incomplete')
            values = [msg.xgyro, msg.ygyro, msg.zgyro, msg.xacc, msg.yacc, msg.zacc]
            if not all(math.isfinite(v) for v in values):
                raise RuntimeError('IMU contains non-finite measurements')
            stamp = int(msg.time_usec)
            if stamp <= 0 or (self.lastUs is not None and stamp <= self.lastUs):
                raise RuntimeError('IMU time moved backwards or repeated')
            if self.lastUs is not None:
                dt = (stamp - self.lastUs) * 1e-6
                self.max_gap_s = max(self.max_gap_s, dt)
                self.gapsOver30ms += dt > .03
            if self.firstUs is None:
                self.firstUs = stamp
            self.lastUs = stamp
            self.count += 1
            temperature = msg.temperature if math.isfinite(msg.temperature) else ''
            self.jobs.put_nowait(('sample', [stamp, time.monotonic_ns(), *values, temperature]))


def capture(out, seconds=600, device='/dev/ttyAMA0'):
    out = Path(out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    jobs = queue.Queue(maxsize=4096)
    done = threading.Event()
    errors = []
    report = dict(status='PREPARING', requested_seconds=seconds, power_source='usb',
        battery_check_required=False, user_stationary_confirmed=True,
        source='PX4 calibrated HIGHRES_IMU, same six-axis values as VIO FRD input',
        timestamp_policy='original PX4 sample time_usec; no interpolation',
        hardware_verified=False)
    atomicJson(out/'status.json', report)

    def writer():
        try:
            with (out/'imu.csv').open('x', newline='') as stream:
                csvout = csv.writer(stream)
                csvout.writerow(['sample_us','received_mono_ns','wx','wy','wz','ax','ay','az','temperature_c'])
                lastFlush = time.monotonic()
                while not done.is_set() or not jobs.empty():
                    try:
                        kind, item = jobs.get(timeout=.1)
                    except queue.Empty:
                        continue
                    if kind == 'sample':
                        csvout.writerow(item)
                    else:
                        atomicJson(out/'status.json', item)
                    if time.monotonic()-lastFlush > 5:
                        stream.flush()
                        os.fsync(stream.fileno())
                        lastFlush = time.monotonic()
                stream.flush()
                os.fsync(stream.fileno())
        except BaseException as exc:
            errors.append(str(exc))

    thread = threading.Thread(target=writer)
    thread.start()
    link = None
    error = None
    old = signal.getsignal(signal.SIGTERM)
    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        link = NoiseLink(device, out/'link', jobs)
        link.configure()
        link.recording = True
        start = time.monotonic()
        lastStatus = 0.0
        print('RECORDING: keep aircraft stationary for '+str(seconds)+' seconds', flush=True)
        while time.monotonic()-start < seconds:
            link.poll(.05)
            if errors:
                raise RuntimeError(errors[0])
            elapsed = time.monotonic()-start
            if elapsed-lastStatus >= 5:
                duration = (link.lastUs-link.firstUs)*1e-6 if link.count > 1 else 0
                report.update(status='RECORDING', elapsed_s=round(elapsed,2),
                    samples=link.count, sensorHz=(link.count-1)/duration if duration else 0,
                    max_gap_s=link.max_gap_s, gapsOver30ms=link.gapsOver30ms)
                jobs.put_nowait(('status', dict(report)))
                lastStatus = elapsed
        if link.count < seconds*45:
            raise RuntimeError('Not enough IMU samples for requested duration')
    except BaseException as exc:
        error = str(exc) or 'capture interrupted'
    finally:
        if link:
            link.recording = False
            link.close()
            error = error or link.error
            if len(link.restored)!=len(link.modified) or not all(v is True for v in link.restored.values()):
                error = 'Telemetry interval restore not confirmed'
        done.set()
        thread.join(timeout=30)
        if thread.is_alive() or errors:
            error = error or 'writer failed: '+str(errors)
        signal.signal(signal.SIGTERM, old)
        if link:
            duration = (link.lastUs-link.firstUs)*1e-6 if link.count > 1 else 0
            report.update(samples=link.count, sample_duration_s=duration,
                sensorHz=(link.count-1)/duration if duration else 0,
                max_gap_s=link.max_gap_s, gapsOver30ms=link.gapsOver30ms)
        report.update(status='ABORTED' if error else 'CAPTURED_REQUIRES_ANALYSIS', error=error)
        if not thread.is_alive():
            report['csv_sha256'] = digest(out/'imu.csv')
        atomicJson(out/'status.json', report)
        print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
    return 1 if error else 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',required=True)
    parser.add_argument('--seconds',type=int,default=600)
    parser.add_argument('--device',default='/dev/ttyAMA0')
    args = parser.parse_args()
    if not 10 <= args.seconds <= 86400:
        parser.error('seconds must be between 10 and 86400')
    raise SystemExit(capture(args.out,args.seconds,args.device))