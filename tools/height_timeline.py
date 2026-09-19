"""按 1 秒步长打印 3 个历史 ULog 的高度、测距与油门时间线（只读）。

日志来自外层 data/px4_logs，按脚本自身位置定位，终端当前目录不影响结果。
用法：python3 tools/height_timeline.py（需要 numpy 与 pyulog）
"""
from pathlib import Path

import numpy as np
from pyulog import ULog

LOGDIR = Path(__file__).resolve().parents[2] / "data" / "px4_logs"
LOGS = [f"log_{i}_UnknownDate.ulg" for i in range(3)]


def main():
    for index, name in enumerate(LOGS):
        path = LOGDIR / name
        if not path.is_file():
            raise SystemExit(f"未找到日志：{path}；请核对 LOGDIR 或改用实际文件名")
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
