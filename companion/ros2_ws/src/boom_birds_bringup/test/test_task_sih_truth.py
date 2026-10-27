"""Decode only local SIH physical-truth packets; no PX4 process is started."""
from types import SimpleNamespace
import struct
import numpy as np
from tools.run_tag_flight_sih import PhysicsTruth


def reader():
    r=PhysicsTruth.__new__(PhysicsTruth)
    r.node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1000_000_000_000)))
    r.latest=r.origin=r.lastUsec=r.fault=None
    r.count=0
    return r


def packet(usec=1000,lat=473977420,lon=85455940,alt=488000,velocity=(0,0,0)):
    payload=struct.pack("<Q4f3f3i3h2H3h",usec,1.,0.,0.,0.,0.,0.,0.,lat,lon,alt,*velocity,0,0,0,0,0)
    payload=payload.rstrip(bytes(1))
    wire=payload.ljust((len(payload)+7)//8*8,bytes(1))
    return SimpleNamespace(msgid=115,sysid=1,compid=1,framing_status=1,len=len(payload),
        payload64=struct.unpack("<"+"Q"*(len(wire)//8),wire),
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1000,nanosec=0)))


def test_zero_truncated_stationary_packet_initializes_reference():
    r=reader();m=packet()
    assert m.len<50
    r.onRaw(m)
    assert r.count==1 and np.allclose(r.latest["p"],0.)
    assert np.allclose(r.latest["r"],np.eye(3))


def test_physical_units_and_axes():
    r=reader();r.onRaw(packet())
    r.onRaw(packet(usec=2000,lat=473977421,alt=488100,velocity=(100,200,-300)))
    assert r.count==2
    assert abs(r.latest["p"][0]-.011119492664)<1e-9
    assert r.latest["p"][2]==.1
    assert np.allclose(r.latest["v"],[1.,-2.,3.])


def test_reordered_truth_is_a_fault_and_foreign_source_is_ignored():
    r=reader();r.onRaw(packet())
    m=packet(usec=2000);m.sysid=2;r.onRaw(m)
    assert r.count==1
    r.onRaw(packet(usec=900))
    assert r.fault=="truth_clock_reset"
