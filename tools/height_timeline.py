from pathlib import Path
import numpy as np
from pyulog import ULog

LOGDIR = Path(__file__).resolve().parents[2] / "data" / "px4_logs"

for i in range(3):
    u = ULog(str(LOGDIR / f"log_{i}_UnknownDate.ulg"))
    t0 = u.start_timestamp / 1e6
    def g(name, multi=0): return u.get_dataset(name, multi).data
    lp, sp = g("vehicle_local_position"), g("vehicle_local_position_setpoint")
    mc, la = g("manual_control_setpoint"), g("vehicle_land_detected")
    fl, mo = g("estimator_status_flags"), g("actuator_motors")
    vs, cm = g("vehicle_status"), g("vehicle_control_mode")
    def near(d, field, t):
        j = np.argmin(np.abs(d["timestamp"] / 1e6 - t))
        return d[field][j]
    print(f"\nLOG {i}")
    for t in np.arange(np.ceil(t0), u.last_timestamp / 1e6, 1):
        if near(cm, "flag_armed", t):
            mmax = max(near(mo, f"control[{k}]", t) for k in range(4))
            print(f'{t-t0:4.0f}s z={near(lp,"z",t):6.2f} zsp={near(sp,"z",t):6.2f} '
                  f'rng={near(lp,"dist_bottom",t):5.2f} valid={near(lp,"dist_bottom_valid",t)} '
                  f'rngH={near(fl,"cs_rng_hgt",t)} thr={near(mc,"throttle",t):.2f} '
                  f'land={near(la,"landed",t)} mmax={mmax:.2f} nav={near(vs,"nav_state",t)} '
                  f'fail={near(vs,"failsafe",t)}')
