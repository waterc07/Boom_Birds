"""同输入分档回放；报告只覆盖运行机器，不推断 Pi/NPU 性能。"""

import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import platform
import resource
import time
import cv2
import numpy as np
from .core import Config, StereoProcessor
from .experiments import project_vio_prior

PROFILES = {
    "baseline": {},
    "fixed": {"fixed_maps": True},
    "reuse": {"reuse_buffers": True},
    "lut": {"postprocess": "lut"},
    "native": {"postprocess": "native"},
    "combined": {"fixed_maps": True, "reuse_buffers": True, "postprocess": "native"},
    "census": {"matcher": "census", "postprocess": "native", "experimental": True},
    "coarse": {"matcher": "coarse_census", "postprocess": "native", "experimental": True},
    "prior": {"matcher": "census", "postprocess": "native", "experimental": True},
    "elas": {"matcher": "elas", "experimental": True},
    "rknn": {"rknn_model": None, "experimental": True},
}


def load_frames(directory):
    frames = []
    for path in sorted(Path(directory).glob("*.npz")):
        with np.load(path, allow_pickle=False) as data:
            if not {"left", "right"} <= set(data.files):
                raise ValueError(f"missing stereo: {path}")
            a, b = data["left"].copy(), data["right"].copy()
            if a.ndim != 2 or a.shape != b.shape or a.dtype != np.uint8 or b.dtype != np.uint8:
                raise ValueError(f"not an equal mono8 pair: {path}")
            pose = data["T_w_crect"].copy() if "T_w_crect" in data else None
            stamp = float(data["stamp"]) if "stamp" in data else None
            frames.append(
                (a, b, pose, stamp, str(path), hashlib.sha256(path.read_bytes()).hexdigest())
            )
    if not frames:
        raise ValueError("no NPZ stereo frames")
    return frames


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--frames",
        required=True,
        help="NPZ: left/right mono8; prior additionally requires stamp/T_w_crect",
    )
    parser.add_argument("--calibration", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--profiles", default="baseline,fixed,reuse,lut,native,combined,census,coarse"
    )
    parser.add_argument("--threads", default="1,2,3")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--disparities", type=int, default=64)
    parser.add_argument("--native-library", default="")
    parser.add_argument("--elas-library", default="")
    parser.add_argument("--rknn-model", default="")
    parser.add_argument("--rknn-manifest", default="")
    parser.add_argument("--xyz", action="store_true")
    args = parser.parse_args(argv)
    profiles = args.profiles.split(",")
    threads = [int(v) for v in args.threads.split(",")]
    if args.repeats < 1 or any(v < 1 for v in threads) or any(v not in PROFILES for v in profiles):
        parser.error("invalid repeats/threads/profiles")
    frames = load_frames(args.frames)
    report = {
        "machine": platform.machine(),
        "platform": platform.platform(),
        "opencv": cv2.__version__,
        "opencv_build": cv2.getBuildInformation(),
        "hardware_acceptance": "NOT RUN",
        "input_sha256": [{"path": f[4], "sha256": f[5]} for f in frames],
        "calibration_sha256": hashlib.sha256(Path(args.calibration).read_bytes()).hexdigest(),
        "with_xyz": args.xyz,
        "variants": [],
    }
    base = Config(
        calibration=Path(args.calibration),
        num_disparities=args.disparities,
        native_library=args.native_library,
        elas_library=args.elas_library,
        rknn_model=args.rknn_model,
        rknn_manifest=args.rknn_manifest,
    )
    cv2.setNumThreads(1)
    reference = StereoProcessor(replace(base, rknn_model="", rknn_manifest=""))
    truth = [reference.process_pair(f[0], f[1], with_xyz=args.xyz) for f in frames]
    reference.close()
    order = [(p, t) for p in profiles for t in threads]
    np.random.default_rng(17).shuffle(order)
    for name, thread in order:
        record = {"profile": name, "threads": thread, "status": "PASS"}
        processor = None
        try:
            if name == "prior" and any(f[2] is None or f[3] is None for f in frames):
                raise ValueError("prior requires measured camera-frame poses and stamps")
            options = dict(PROFILES[name])
            options["rknn_model"] = args.rknn_model if name == "rknn" else ""
            options["rknn_manifest"] = args.rknn_manifest if name == "rknn" else ""
            if name == "rknn" and not args.rknn_model:
                raise ValueError("RKNN model not supplied")
            cv2.setNumThreads(thread)
            processor = StereoProcessor(replace(base, **options))
            # warm-up does not enter timing/quality aggregates
            processor.process_pair(frames[0][0], frames[0][1], with_xyz=args.xyz)
            if processor.alternative is not None and hasattr(processor.alternative, "frames"):
                processor.alternative.frames = 0
            totals = []
            matches = []
            posts = []
            priors = []
            valid = []
            disagreement = []
            differences = []
            relative = []
            guided = fallback = 0
            compute_cpu = 0.0
            for repeat in range(args.repeats):
                previous = None
                for index, (left, right, pose, stamp, *_) in enumerate(frames):
                    cpu_start = time.process_time()
                    start = time.perf_counter()
                    if name == "prior" and previous is not None:
                        old, old_pose, old_stamp = previous
                        prior = project_vio_prior(
                            old,
                            processor.p1,
                            processor.q,
                            np.linalg.inv(pose) @ old_pose,
                            old_stamp,
                            stamp,
                        )
                        processor.set_vio_prior(prior, stamp)
                    priors.append((time.perf_counter() - start) * 1000)
                    result = processor.process_pair(left, right, with_xyz=args.xyz)
                    totals.append((time.perf_counter() - start) * 1000)
                    compute_cpu += time.process_time() - cpu_start
                    matches.append(result[-1]["match_ms"])
                    posts.append(result[-1]["post_ms"])
                    depth = result[3][:, :, 2] if args.xyz else result[3]
                    ref = truth[index][3][:, :, 2] if args.xyz else truth[index][3]
                    mask = result[4]
                    common = mask & truth[index][4]
                    valid.append(float(mask.mean()))
                    disagreement.append(float((mask != truth[index][4]).mean()))
                    if common.any():
                        differences.extend(np.abs(depth[common] - ref[common]).tolist())
                        relative.extend(
                            (np.abs(depth[common] - ref[common]) / ref[common]).tolist()
                        )
                    if name == "prior":
                        previous = (depth.copy(), pose, stamp)
                    stats = getattr(processor.alternative, "stats", {})
                    guided += int(stats.get("guided", False))
                    fallback += int(stats.get("full_fallback", False))

            def summary(values):
                return (
                    {
                        "p50": float(np.percentile(values, 50)),
                        "p95": float(np.percentile(values, 95)),
                        "max": float(np.max(values)),
                    }
                    if values
                    else None
                )

            elapsed = sum(totals) / 1000
            record.update(
                total_ms=summary(totals),
                prior_ms=summary(priors),
                match_ms=summary(matches),
                post_ms=summary(posts),
                cpu_single_core_percent=compute_cpu / elapsed * 100,
                valid_ratio=float(np.mean(valid)),
                mask_disagreement=float(np.mean(disagreement)),
                depth_difference_m=summary(differences),
                relative_depth_difference=summary(relative),
                guided_frames=guided,
                full_fallback_frames=fallback,
                process_rss_highwater_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                neon=processor.native.has_neon if processor.native else None,
            )
        except (FileNotFoundError, ImportError, ValueError) as exc:
            record.update(status="BLOCKED", reason=str(exc))
        except Exception as exc:
            record.update(status="FAIL", reason=f"{type(exc).__name__}: {exc}")
        finally:
            if processor is not None:
                processor.close()
        report["variants"].append(record)
    Path(args.output).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(
        json.dumps(
            {
                "output": args.output,
                "status_counts": {
                    s: sum(v["status"] == s for v in report["variants"])
                    for s in ("PASS", "BLOCKED", "FAIL")
                },
            },
            ensure_ascii=False,
        )
    )
    return (
        1
        if any(v["status"] == "FAIL" for v in report["variants"])
        else (2 if any(v["status"] == "BLOCKED" for v in report["variants"]) else 0)
    )


if __name__ == "__main__":
    raise SystemExit(main())
