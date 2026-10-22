"""按 1 秒步长打印 ULog 高度、测距与油门时间线（只读）。"""
from pathlib import Path
import argparse

import numpy as np
from pyulog import ULog

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", type=Path, nargs="+")
    args = parser.parse_args()
    for index, path in enumerate(args.logs):
        name = path.name
        if not path.is_file():
            parser.error(f"未找到日志：{path}")
        u = ULog(str(path))
        t0 = u.start_timestamp / 1e6

        def g(dataset, multi=0):
            return u.get_dataset(dataset, multi).data

        lp, sp = g("vehicle_local_position"), g("vehicle_local_position_setpoint")
        mc, la = g("manual_control_setpoint"), g("vehicle_land_detected")
        fl, mo = g("estimator_status_flags"), g("actuator_motors")
        vs, cm = g("vehicle_status"), g("vehicle_control_mode")

        def near(d, field, t):
            j = np.argmin(np.abs(d["timestamp"] / 1e6 - t))
            return d[field][j]

        print(f"\nLOG {index} ({name})")
        for t in np.arange(np.ceil(t0), u.last_timestamp / 1e6, 1):
            if near(cm, "flag_armed", t):
                mmax = max(near(mo, f"control[{k}]", t) for k in range(4))
                print(f'{t-t0:4.0f}s z={near(lp,"z",t):6.2f} zsp={near(sp,"z",t):6.2f} '
                      f'rng={near(lp,"dist_bottom",t):5.2f} valid={near(lp,"dist_bottom_valid",t)} '
                      f'rngH={near(fl,"cs_rng_hgt",t)} thr={near(mc,"throttle",t):.2f} '
                      f'land={near(la,"landed",t)} mmax={mmax:.2f} nav={near(vs,"nav_state",t)} '
                      f'fail={near(vs,"failsafe",t)}')


if __name__ == "__main__":
    main()
