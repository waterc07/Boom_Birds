#!/usr/bin/env python3
"""TEST-ONLY cropped-marker continuation; no ROS, FCU or control commands."""
import argparse
from platform_evidence import fingerprints
from dataclasses import asdict
import json
from pathlib import Path
import cv2
import numpy as np
import yaml
from boom_birds_sensing.platform_feature_track import BoardFeatureTrack, TrackConfig
from boom_birds_sensing.platform_observation import BoardDetector
from boom_birds_sensing.platform_synthetic import render_board

ap = argparse.ArgumentParser()
ap.add_argument("--config", required=True)
ap.add_argument("--out", required=True)
args = ap.parse_args()
out = Path(args.out)
out.mkdir(parents=True, exist_ok=False)
profile = yaml.safe_load(Path(args.config).read_text())
detector = BoardDetector(profile, test_only=True)
cfg = TrackConfig(roi_radius_px=50)
tracker = BoardFeatureTrack(detector, cfg)
pose = np.eye(4)
pose[:3, :3] = cv2.Rodrigues(np.array([.08, .05, .1]))[0]
pose[2, 3] = -1.
image = render_board(detector, pose)
observation = detector.observe(image, 10.)
initial = tracker.seed(image, observation, 1.)
assert initial["valid"], initial
cv2.imwrite(str(out/"decoded.png"), image)
center = np.rint(initial["landing_pixel"]).astype(int)
x, y = center
partial = np.full_like(image, 255)
partial[y-60:y+61, x-60:x+61] = image[y-60:y+61, x-60:x+61]
rows = []
for frame in range(1, 61):
    dx, dy = frame*.25, -frame*.15
    moved = cv2.warpAffine(partial, np.float32([[1, 0, dx], [0, 1, dy]]),
                           (image.shape[1], image.shape[0]), borderValue=255)
    stamp = 10.+frame*.02
    decoded = detector.observe(moved, stamp)
    tracked = tracker.update(moved, stamp)
    error = None if not tracked["valid"] else float(np.linalg.norm(
        np.asarray(tracked["landing_pixel"])-np.asarray(initial["landing_pixel"])-[dx, dy]))
    rows.append(dict(decoded=decoded.valid, track=tracked, pixel_error=error))
    if frame in (1, 30, 60):
        cv2.imwrite(str(out/("partial-%02d.png"%frame)), moved)
loss = tracker.update(np.full_like(image, 255), 11.22)
assert all(not r["decoded"] and r["track"]["valid"] and r["pixel_error"] < 1. for r in rows), rows
assert not loss["valid"] and tracker.update(image, 11.24)["reason"] == "decode_required"
(out/"frames.jsonl").write_text("".join(json.dumps(r)+"\n" for r in rows))
(out/"report.json").write_text(json.dumps(dict(kind="SYNTHETIC_FEATURE_TRACK", result="PASS",
    config=profile, source_sha256=fingerprints(args.config), tracking=asdict(cfg), frames=len(rows), max_pixel_error=max(r["pixel_error"] for r in rows),
    loss=loss, control_integration="NOT RUN", metric_accuracy="NOT RUN"), indent=2))
print((out/"report.json").read_text())
