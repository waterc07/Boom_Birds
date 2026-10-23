"""汇总四组已完成回放；缺组合、不同配置或算法无视觉更新都不能通过。"""

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from .gazebo_fixture import MOTION_PROFILES
from .gazebo_evaluate import evaluate

EXPECTED = {
    (scene, motion) for scene in ("range_3m", "range_5m") for motion in ("nominal", "gentle")
}
GEOMETRY = (
    "width",
    "height",
    "fx",
    "fy",
    "cx",
    "cy",
    "baseline_m",
    "image_rate_hz",
    "imu_rate_hz",
    "camera_to_imu_rotation",
    "left_camera_position_in_imu_m",
    "seed",
    "sensor_noise",
)


def caseKey(source, manifest):
    """旧录制清单缺字段时，从实际保存的 world 核验场景和激励。"""
    world = ET.parse(Path(source) / "sensor_world.sdf").getroot()
    wall = world.find("./world/model[@name='front_wall']/pose")
    distance = float(wall.text.split()[0])
    scene = {3.2: "range_3m", 5.0: "range_5m"}[distance]
    plugin = world.find("./world/model[@name='bb_rig']/plugin[@name='boom_birds::SensorMotion']")
    amplitudes = [float(v) for v in plugin.findtext("amplitudes").split()]
    frequencies = [float(v) for v in plugin.findtext("frequencies").split()]
    profiles = [
        name
        for name, (a, f) in MOTION_PROFILES.items()
        if np.allclose(amplitudes, a, rtol=0, atol=1e-10)
        and np.allclose(frequencies, f, rtol=0, atol=1e-10)
    ]
    if len(profiles) != 1:
        raise ValueError("world uses unknown motion profile")
    motion = profiles[0]
    if manifest.get("scene", scene) != scene or manifest.get("motion_profile", motion) != motion:
        raise ValueError("manifest does not match saved world")
    return scene, motion


def summarize(runs):
    rows = []
    errors = []
    profiles = set()
    geometry = []
    gates = []
    seen = set()
    for source, out in runs:
        source, out = Path(source).resolve(), Path(out).resolve()
        manifest = json.loads((source / "manifest.json").read_text())
        replay = json.loads((out / "replay.json").read_text())
        processes = json.loads((out / "processes.json").read_text())
        evaluation = evaluate(source, out)
        key = caseKey(source, manifest)
        seen.add(key)
        geometry.append({k: manifest[k] for k in GEOMETRY})
        gates.append(evaluation["gates"])
        profiles.add(
            (
                replay.get("zupt_mode"),
                replay.get("feature_profile"),
                replay.get("depth_profile"),
                replay.get("sgbm_block_size"),
            )
        )
        config = out / "estimator_config.yaml"
        fingerprintOk = config.exists() and hashlib.sha256(
            config.read_bytes()
        ).hexdigest() == replay.get("estimator_config_sha256")
        calibrationOk = all(
            (out / name).read_bytes() == (source / name).read_bytes()
            for name in ("kalibr_imu_chain.yaml", "kalibr_imucam_chain.yaml")
        )
        exitOk = processes == {"openvins": 0, "depth": 0, "playback": 0}
        visualOk = evaluation.get("vio", {}).get("visual_updates_observed", False)
        passed = bool(
            evaluation["pass"]
            and exitOk
            and visualOk
            and fingerprintOk
            and calibrationOk
            and replay.get("truth_published") is False
        )
        rows.append(
            dict(
                scene=key[0],
                motion_profile=key[1],
                input=str(source),
                output=str(out),
                pass_case=passed,
                processes=processes,
                config_fingerprint_matches=fingerprintOk,
                calibration_matches_input=calibrationOk,
                evaluation=evaluation,
                replay=replay,
            )
        )
    if geometry and geometry[0]["sensor_noise"] != "disabled baseline":
        errors.append("matrix requires noiseless baseline")
    missing = EXPECTED - seen
    if missing:
        errors.append("missing combinations: " + repr(sorted(missing)))
    if seen - EXPECTED:
        errors.append("unexpected scene/motion combination")
    if len(profiles) != 1:
        errors.append("runs use different estimator/depth profiles")
    if any(v != geometry[0] for v in geometry[1:]):
        errors.append("sensor geometry, seed, noise or rates differ")
    if any(v != gates[0] for v in gates[1:]):
        errors.append("acceptance gates differ")
    return dict(
        scope="TEST_ONLY_noiseless_sensor_rig_desktop_replay",
        errors=errors,
        pass_matrix=bool(rows and not errors and all(r["pass_case"] for r in rows)),
        combinations=len(seen),
        runs=rows,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--run", nargs=2, action="append", required=True, metavar=("INPUT", "REPLAY")
    )
    parser.add_argument("--out", required=True, help="JSON report path")
    args = parser.parse_args()
    target = Path(args.out).resolve()
    if target.exists():
        parser.error("report must use a new path")
    report = summarize(args.run)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2))
    print(
        json.dumps(
            dict(pass_matrix=report["pass_matrix"], errors=report["errors"], report=str(target))
        )
    )
    return 0 if report["pass_matrix"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
