"""HIGHRES_IMU 原始时间戳数据的重叠 Allan deviation 与噪声候选。"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from .vio_calibration import atomic_json, digest, write_yaml


def overlapping_adev(values, rate, clusters):
    values = np.asarray(values, dtype=float)
    if values.ndim == 1:
        values = values[:, None]
    integrated = np.vstack((np.zeros((1, values.shape[1])), np.cumsum(values, axis=0)/rate))
    output = []
    for m in clusters:
        delta = integrated[2*m:] - 2*integrated[m:-m] + integrated[:-2*m]
        output.append(np.sqrt(np.mean(delta*delta,axis=0)/(2*(m/rate)**2)))
    return np.asarray(output)


def fit_component(tau, adev, samples, rate, target):
    lower, upper = (.2, 5.) if target < 0 else (5., min(60., samples/rate/20))
    eligible = np.where((tau >= lower) & (tau <= upper) & (adev > 0))[0]
    candidates = []
    for width in range(5, 13):
        for start in range(max(0,len(eligible)-width+1)):
            idx = eligible[start:start+width]
            if tau[idx[-1]]/tau[idx[0]] < 2:
                continue
            x,y = np.log(tau[idx]),np.log(adev[idx])
            slope,intercept = np.polyfit(x,y,1)
            total = np.sum((y-y.mean())**2)
            r2 = 1-np.sum((y-(slope*x+intercept))**2)/total if total else 0.
            if abs(slope-target) > .15 or r2 < .9:
                continue
            coefficient = np.median(adev[idx]*np.sqrt(tau[idx]) if target < 0
                else adev[idx]*np.sqrt(3/tau[idx]))
            candidates.append((abs(slope-target),-np.log(tau[idx[-1]]/tau[idx[0]]),
                dict(coefficient=float(coefficient), slope=float(slope), r2=float(r2),
                    tau_range_s=[float(tau[idx[0]]),float(tau[idx[-1]])])))
    return min(candidates,key=lambda item:item[:2])[2] if candidates else None


def analyze(source, out, warmup_s=30.):
    source,out = Path(source),Path(out)
    out.mkdir(parents=True,exist_ok=False)
    raw = np.genfromtxt(source,delimiter=',',names=True)
    stamps = np.atleast_1d(raw['sample_us']).astype(np.int64)
    values = np.column_stack([np.atleast_1d(raw[k]) for k in ('wx','wy','wz','ax','ay','az')])
    if len(stamps) < 1000 or not np.isfinite(values).all() or np.any(np.diff(stamps)<=0):
        raise ValueError('samples missing, non-finite or nonmonotonic')
    # 不跨越丢采样间隙积分，不插值；保留原始 CSV 并记录选中的连续段。
    dt = np.diff(stamps)*1e-6
    nominal = float(np.median(dt))
    starts = np.r_[0,np.flatnonzero(dt>1.5*nominal)+1]
    ends = np.r_[starts[1:],len(stamps)]
    longest = int(np.argmax(ends-starts))
    lo,hi = int(starts[longest]),int(ends[longest])
    lo = int(np.searchsorted(stamps, stamps[lo] + round(warmup_s*1e6)))
    if hi-lo < 1000:
        raise ValueError('continuous segment too short after warmup exclusion')
    used = values[lo:hi]
    rate = (hi-lo-1)/((stamps[hi-1]-stamps[lo])*1e-6)
    duration = (hi-lo-1)/rate
    width = max(1,round(rate*5))
    blocks = [used[i:i+width] for i in range(0,len(used)-width+1,width)]
    acc_means = np.asarray([b[:,3:].mean(axis=0) for b in blocks])
    drift = float(np.max(np.linalg.norm(acc_means-acc_means[0],axis=1)))
    gyro_excess = float(np.mean(np.linalg.norm(used[:,:3],axis=1)>.1))
    acc_std = max(float(np.max(np.std(b[:,3:],axis=0))) for b in blocks)
    stationary = drift <= .2 and gyro_excess <= .001 and acc_std <= .2
    clusters = np.unique(np.round(np.geomspace(1,len(used)//8,90)).astype(int))
    tau = clusters/rate
    adev = overlapping_adev(used,rate,clusters)
    half = len(used)//2
    halves = [overlapping_adev(used[:half],rate,clusters),
              overlapping_adev(used[-half:],rate,clusters)]
    fits = {}
    axes = ('wx','wy','wz','ax','ay','az')
    for i,axis in enumerate(axes):
        fits[axis] = {}
        for label,target in [('white_noise',-.5),('bias_random_walk',.5)]:
            fit = fit_component(tau,adev[:,i],len(used),rate,target)
            if fit:
                a,b=fit['tau_range_s']
                select=(tau>=a)&(tau<=b)
                estimates=[float(np.median(part[select,i]*np.sqrt(tau[select]) if target<0
                    else part[select,i]*np.sqrt(3/tau[select]))) for part in halves]
                ratio=max(estimates)/min(estimates) if min(estimates)>0 else float('inf')
                fit['half_record_coefficients']=estimates
                fit['half_record_ratio']=ratio
                if ratio > 2:
                    fit=None
            fits[axis][label]=fit
    def combined(names,label):
        entries=[fits[axis][label] for axis in names]
        return max(x['coefficient'] for x in entries) if all(entries) and stationary else None
    noise=dict(gyroscope_noise_density=combined(axes[:3],'white_noise'),
        gyroscope_random_walk=combined(axes[:3],'bias_random_walk'),
        accelerometer_noise_density=combined(axes[3:],'white_noise'),
        accelerometer_random_walk=combined(axes[3:],'bias_random_walk'),
        update_rate=float(rate),rostopic='/imu0',
        source=f'PX4 calibrated HIGHRES_IMU FRD; CSV {source}; sha256={digest(source)}; '
               f'continuous {duration:.3f}s, warmup excluded {warmup_s}s; overlapping Allan fit; '
               'short-record candidate requires review')
    complete=all(noise[k] is not None for k in ('gyroscope_noise_density','gyroscope_random_walk',
        'accelerometer_noise_density','accelerometer_random_walk'))
    report=dict(status='MEASURED_NOISE_CANDIDATE' if complete else 'PARTIAL_NOISE_ESTIMATE',
        source_csv=str(source),source_sha256=digest(source),raw_samples=len(stamps),
        used_sample_slice=[lo,hi],used_duration_s=float(duration),update_rate=float(rate),
        excluded_warmup_s=warmup_s,raw_max_gap_s=float(dt.max()),
        raw_gaps_over_1_5_period=int(np.sum(dt>1.5*nominal)),
        stationary_check=dict(passed=stationary,acceleration_mean_drift_m_s2=drift,
            gyro_over_0_1_rad_s_fraction=gyro_excess,max_accel_block_std_m_s2=acc_std,
            note='5 s window heuristics; user must keep sensor fixed'),
        axes=fits,kalibr_noise=noise,hardware_verified=False,
        note='Missing identifiable +1/2 slope remains null; no example values or interpolation.')
    with (out/'allan.csv').open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['tau_s',*axes])
        writer.writerows(np.column_stack((tau,adev)))
    write_yaml(out/('imu-noise.candidate.yaml' if complete else 'imu-noise.partial.yaml'),noise)
    atomic_json(out/'analysis.json',report)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--csv',required=True)
    p.add_argument('--out',required=True)
    args=p.parse_args()
    print(json.dumps(analyze(args.csv,args.out),ensure_ascii=False,indent=2))