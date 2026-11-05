import pytest
from boom_birds_bringup.runtime_network import DdsSocketBuffers


def test_socket_limits_are_raised_only_as_needed_and_restored():
    values={"net.core.rmem_max":212992,"net.core.wmem_max":8388608}
    writes=[]
    def write(k,v):values[k]=v;writes.append((k,v))
    lease=DdsSocketBuffers(read=values.get,write=write)
    lease.prepare()
    assert values["net.core.rmem_max"]==4194304 and len(writes)==1
    lease.restore()
    assert values["net.core.rmem_max"]==212992
    assert values["net.core.wmem_max"]==8388608


def test_socket_restore_does_not_overwrite_a_later_change():
    values={"net.core.rmem_max":212992,"net.core.wmem_max":4194304}
    lease=DdsSocketBuffers(read=values.get,write=lambda k,v:values.__setitem__(k,v))
    lease.prepare();values["net.core.rmem_max"]=8388608
    lease.restore()
    assert values["net.core.rmem_max"]==8388608


def test_partially_failed_socket_setup_can_be_rolled_back():
    values={"net.core.rmem_max":212992,"net.core.wmem_max":212992}
    def write(k,v):
        if k=="net.core.wmem_max":raise RuntimeError("not permitted")
        values[k]=v
    lease=DdsSocketBuffers(read=values.get,write=write)
    with pytest.raises(RuntimeError):lease.prepare()
    lease.restore()
    assert values["net.core.rmem_max"]==212992


@pytest.mark.parametrize("interactive",[False,True])
def test_sudo_password_prompt_is_only_allowed_on_operator_terminal(monkeypatch,interactive):
    from types import SimpleNamespace as NS
    from boom_birds_bringup import runtime_network as module
    calls=[]
    monkeypatch.setattr(module.os,'geteuid',lambda:1000)
    monkeypatch.setattr(module.sys,'stdin',NS(isatty=lambda:interactive))
    def run(command,**kwargs):
        calls.append(command)
        return NS(returncode=1 if '-n' in command else 0,stderr='sudo: a password is required')
    monkeypatch.setattr(module.subprocess,'run',run)
    if interactive:DdsSocketBuffers.writeValue('net.core.rmem_max',4194304)
    else:
        with pytest.raises(RuntimeError,match='password is required'):
            DdsSocketBuffers.writeValue('net.core.rmem_max',4194304)
    assert calls[0][:2]==['sudo','-n']
    assert len(calls)==(2 if interactive else 1)
