"""确认授权的本机 PX4 SIH 进程仍存活；不枚举设备或其它进程。"""
from pathlib import Path

def verify_sih_process(pid, proc=Path("/proc")):
    if type(pid) is not int or pid <= 0:
        return False
    try:
        root = proc / str(pid)
        exe = (root / "exe").resolve(strict=True)
        env = dict(part.split(b"=", 1) for part in (root / "environ").read_bytes().split(b"\0") if b"=" in part)
        return (str(exe).endswith("/build/px4_sitl_default/bin/px4") and
                env.get(b"PX4_SIM_MODEL") == b"sihsim_quadx" and
                env.get(b"PX4_SIMULATOR") == b"sihsim" and
                env.get(b"PX4_SYS_AUTOSTART") == b"10040")
    except (OSError, ValueError):
        return False
