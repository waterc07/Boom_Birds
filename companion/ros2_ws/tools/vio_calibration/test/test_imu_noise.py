import numpy as np
import pytest
from tools.vio_calibration.imu_noise_analysis import overlapping_adev, fit_component


def test_white_noise_density_units_and_no_random_walk_fit():
    rng=np.random.default_rng(17)
    rate=50.
    density=.002
    values=rng.normal(0,density*np.sqrt(rate),size=100000)
    clusters=np.array([25,50,100,150])
    adev=overlapping_adev(values,rate,clusters)[:,0]
    assert np.allclose(adev*np.sqrt(clusters/rate),density,rtol=.12)
    tau=np.geomspace(.2,60,70)
    assert fit_component(tau,density/np.sqrt(tau),100000,rate,.5) is None
    fit=fit_component(tau,density/np.sqrt(tau),100000,rate,-.5)
    assert fit['coefficient']==pytest.approx(density)


def test_bias_random_walk_units():
    rng=np.random.default_rng(7)
    rate=50.
    coefficient=.0001
    values=np.cumsum(rng.normal(0,coefficient/np.sqrt(rate),size=100000))
    tau=np.array([1.,3.,6.])
    adev=overlapping_adev(values,rate,(tau*rate).astype(int))[:,0]
    assert np.allclose(adev*np.sqrt(3/tau),coefficient,rtol=.15)


def test_capture_preserves_original_frd_and_rejects_duplicate(monkeypatch):
    import queue
    from types import SimpleNamespace as NS
    from tools.vio_calibration.imu_noise_capture import NoiseLink, TelemetryLink
    monkeypatch.setattr(TelemetryLink,'consume',lambda *args:None)
    msg=NS(get_type=lambda:'HIGHRES_IMU',get_srcSystem=lambda:1,get_srcComponent=lambda:1,
        fields_updated=63,time_usec=123456,xgyro=.01,ygyro=-.02,zgyro=.03,
        xacc=1.,yacc=2.,zacc=-9.81,temperature=30.)
    link=object.__new__(NoiseLink)
    link.sample_parser=NS(parse_buffer=lambda _: [msg])
    link.jobs=queue.Queue()
    link.recording=True
    link.count=0
    link.first_us=link.last_us=None
    link.max_gap_s=0.
    link.gaps_over_30ms=0
    link.consume(b'fixture')
    _,row=link.jobs.get_nowait()
    assert row[0]==123456
    assert row[2:8]==[.01,-.02,.03,1.,2.,-9.81]
    with pytest.raises(RuntimeError,match='backwards or repeated'):
        link.consume(b'fixture')

def test_warmup_uses_sensor_time_with_jittered_periods(tmp_path):
    import csv
    from tools.vio_calibration.imu_noise_analysis import analyze
    rng=np.random.default_rng(22)
    stamps=1_000_000+np.cumsum(np.resize([18000,22000,22000],6000))
    source=tmp_path/'imu.csv'
    with source.open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['sample_us','received_mono_ns','wx','wy','wz','ax','ay','az','temperature_c'])
        values=rng.normal(0,.001,(6000,6))
        values[:,5]-=9.81
        for stamp,row in zip(stamps,values):
            writer.writerow([int(stamp),int(stamp)*1000,*row,30.])
    report=analyze(source,tmp_path/'analysis')
    first,last=report['used_sample_slice']
    excluded=(stamps[first]-stamps[0])*1e-6
    assert 30<=excluded<30.03
    assert last==6000
