"""单帧自检入口：不依赖 ROS 运行时的深度链核心检查。

用途：在只有标定的情况下回答「这条链路是否按契约工作」：
    python3 -m boom_birds_nav.deep_checks --calibration <npz> [--synth]
输出：有效像素比例、分区域深度误差统计、无效值契约检查结果，并给出退出码。

注意：--synth 使用合成数据与合成标定（TEST-ONLY），结论只说明实现自洽，
不构成任何真机精度证据。
"""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
from pathlib import Path

import numpy as np


def run(calibration: str, synth: bool, max_depth: float, min_depth: float) -> dict:
    from .depth_core import annotate_validity, make_processor, process_stitched
    from .synthetic import default_scene, render_stereo, write_synth_calibration

    if synth:
        tmp = Path(tempfile.mkdtemp()) / "synth_candidate.npz"
        write_synth_calibration(str(tmp))
        calibration = str(tmp)
        scene = default_scene()
        left, right, depth_gt = render_stereo(scene)
        stitched = np.hstack([left, right])[:, :, None].repeat(3, axis=2)
    else:
        raise SystemExit("非合成模式需要真实采集帧；脱机自检请使用 --synth，或在测试中提供图像")

    processor = make_processor(calibration)
    result = process_stitched(processor, stitched)
    depth_pub, invalid, over_range = annotate_validity(result.depth, result.valid, max_depth, min_depth)

    out = {
        "calibration": calibration,
        "data": "synthetic (TEST-ONLY)",
        "valid_ratio": float(result.valid.mean()),
        "invalid_ratio": float(invalid.mean()),
        "over_range_pixels": int(over_range.sum()),
        "depth_published_zero_fraction": float((depth_pub == 0.0).mean()),
        "invalid_has_nan": bool(np.isnan(result.depth[~result.valid]).all()) if (~result.valid).any() else True,
        "published_has_nan": bool(np.isnan(depth_pub).any()),
        "intrinsics": {
            "fx": float(processor.p1[0, 0]),
            "fy": float(processor.p1[1, 1]),
            "cx": float(processor.p1[0, 2]),
            "cy": float(processor.p1[1, 2]),
            "baseline_m": float(processor.baseline_m),
        },
        "per_region": {},
    }
    m = 20
    core = np.zeros_like(result.valid)
    core[m:-m, m:-m] = True
    for name, expect in (("wall_3m", 3.0), ("obstacle_1_8m", 1.8)):
        sel = result.valid & core & (np.abs(depth_gt - expect) < 1e-3)
        if sel.sum() == 0:
            out["per_region"][name] = {"n": 0}
            continue
        err = np.abs(result.depth[sel] - expect)
        out["per_region"][name] = {
            "n": int(sel.sum()),
            "median_m": float(np.median(err)),
            "p95_m": float(np.percentile(err, 95)),
            "max_m": float(err.max()),
        }
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Boom_Birds 深度单帧自检（脱机）")
    ap.add_argument("--calibration", default="", help="标定 npz；--synth 时忽略")
    ap.add_argument("--synth", action="store_true", help="使用合成数据与合成标定（TEST-ONLY）")
    ap.add_argument("--max-depth", type=float, default=5.0)
    ap.add_argument("--min-depth", type=float, default=0.2)
    args = ap.parse_args(argv)

    if not args.synth and not args.calibration:
        print("需要 --calibration 或 --synth", file=sys.stderr)
        return 2
    report = run(args.calibration, args.synth, args.max_depth, args.min_depth)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    ok = report["published_has_nan"] is False and report["invalid_has_nan"] is True
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
