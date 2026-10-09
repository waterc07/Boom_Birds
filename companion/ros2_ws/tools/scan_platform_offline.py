#!/usr/bin/env python3
"""合成图像/配置与引导参数扫描。结果不作真实精度或 Pi 性能证据。"""
import argparse
from platform_evidence import load_profile
from platform_evidence import fingerprints
import copy
import itertools
import json
from pathlib import Path
import cv2
import numpy as np
import yaml
from boom_birds_control.platform_landing import PlatformLanding, RangeSample, FlightSample, HandoffFeedback, ned_from_flu
from boom_birds_control.platform_model import BoardObservation
from boom_birds_sensing.platform_observation import BoardDetector
from boom_birds_sensing.platform_replay import render_board


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    profile = load_profile(args.config)
    variants = {}
    for size in sorted({profile["board"]["tags"][0]["size_m"], .15, .30, .45}):
        p = copy.deepcopy(profile)
        p["board"]["tags"][0]["size_m"] = size
        variants[str(size)] = p
    mixed = copy.deepcopy(profile)
    mixed["board"]["tags"][0]["size_m"] = .30
    mixed["board"]["tags"][0]["T_platform_tag"][0][3] = .20
    tag = copy.deepcopy(profile["board"]["tags"][0])
    tag.update(id=8, size_m=.08)
    mixed["board"]["tags"].append(tag)
    variants["mixed_300_80"] = mixed
    rows = []
    image_samples = out/"image-samples"
    image_samples.mkdir()
    for name, p in variants.items():
        (out/(name+".yaml")).write_text(yaml.safe_dump(p, sort_keys=False))
        detector = BoardDetector(p, test_only=True)
        for height, offset, tilt, blur, occlusion in itertools.product(
                (.2, .35, .5, 1., 1.5, 2., 3., 4.), (0., .1, .3), (0., .15, .3), (0, 3), (0., .2)):
            rotation = ned_from_flu(0., tilt, 0.)
            truth = np.eye(4)
            truth[:3, :3] = rotation.T @ np.diag([1., -1., -1.])
            truth[:3, 3] = rotation.T @ np.array([offset, 0., height])
            image = render_board(detector, truth)
            if blur:
                image = cv2.GaussianBlur(image, (blur, blur), 0)
            if occlusion:
                # 明确的中心横条遮挡，记录位置；不把该比例解释为所有遮挡模式。
                image[240:240+int(480*occlusion), :] = 255
            observation = detector.observe(image, 1., "TEST_ONLY_SCAN")
            error = np.linalg.norm(observation.pose()[:3, 3]-truth[:3, 3]) if observation.valid else None
            rows.append(dict(layout=name, height_m=height, offset_m=offset, tilt_rad=tilt,
                             blur_kernel=blur, center_occlusion_fraction=occlusion,
                             valid=observation.valid, reason=observation.reason, translation_error_m=error,
                             tag_ids=observation.tag_ids, min_edge_px=observation.min_edge_px))
            if offset == tilt == occlusion == blur == 0 and height in (.2, .5, 1.5, 3.):
                cv2.imwrite(str(image_samples/(name+"-"+str(height)+".png")), image)
    # 同一投影输入，改变解析内参/外参以量化敏感性，不合成“真实标定”。
    sensitivity = []
    detector = BoardDetector(profile, test_only=True)
    truth = np.eye(4)
    truth[:3, 3] = [.1, 0., -1.5]
    image = render_board(detector, truth)
    for focal_scale, mount_dx in itertools.product((.97, 1., 1.03), (-.02, 0., .02)):
        p = copy.deepcopy(profile)
        p["camera"]["K"][0][0] *= focal_scale
        p["camera"]["K"][1][1] *= focal_scale
        p["camera"]["T_body_camera"][0][3] += mount_dx
        result = BoardDetector(p, test_only=True).observe(image, 1., "TEST_ONLY_CALIBRATION_PERTURBATION")
        sensitivity.append(dict(focal_scale=focal_scale, mount_dx_m=mount_dx, valid=result.valid,
                                translation_error_m=float(np.linalg.norm(result.pose()[:3, 3]-truth[:3, 3])) if result.valid else None))
    # 受控一阶速度响应模型；逐样本延迟，保持原下降许可闸门。
    guidance = []
    for gain, delay, initial_error in itertools.product((.4, .6, .8), (0., .08, .18), (.1, .3)):
        p = copy.deepcopy(profile)
        p["guidance"]["kp_xy"] = gain
        core = PlatformLanding(p, test_only=True)
        core.request()
        position = np.array([initial_error, 0., 1.5])
        velocity = np.zeros(3)
        queue = []
        commands = []
        aligned_at = None
        sign_changes = 0
        previous = position[0]
        unsafe = 0
        for i in range(1200):
            now = i*.02
            pose = np.eye(4)
            pose[:3, 3] = [position[0], 0., -position[2]]
            observation = BoardObservation(now, p["board"]["name"], True, "model", tuple(map(tuple, pose)), (7,), .1, 5., 100.)
            queue.append((observation, RangeSample(now, position[2]-.02)))
            slot = max(0, len(queue)-1-int(round(delay/.02)))
            o, r = queue[slot]
            flight = FlightSample(now, 0., 0., 0., tuple(velocity), True, True, True)
            ack = None
            if core.last_sent_sequence:
                seq = core.last_sent_sequence
                ack = HandoffFeedback(now, core.token, seq, 0, True, False, core.sent_history[seq][1], "TEST_ONLY_FAKE_PX4")
            result = core.step(now, o, r, flight, ack, navigation_ready=True, navigation_revoked=True)
            if result.velocity_ned is not None:
                core.note_sent(result, now, True)
                cmd = np.asarray(result.velocity_ned)
                unsafe += int(cmd[2] > 0 and not result.descent_permitted)
                commands.append(cmd.tolist())
                # PX4 仿真以外的简化模型：速度时间常数 0.12 s。
                velocity += (cmd-velocity)*(.02/.12)
                position[0] -= velocity[0]*.02
                position[2] = max(.06, position[2]-velocity[2]*.02)
            if previous*position[0] < 0:
                sign_changes += 1
            previous = position[0]
            if result.descent_permitted and aligned_at is None:
                aligned_at = now
            if result.state == "NATIVE_LAND":
                break
        guidance.append(dict(kp_xy=gain, delay_s=delay, initial_error_m=initial_error,
                             first_descent_s=aligned_at, final_xy_m=abs(position[0]),
                             sign_changes=sign_changes, final_state=core.state,
                             unauthorized_descent=unsafe))
    # 打印尺寸取配置中的黑框边长；小板居中放在 A4，禁止固定输出 300 mm。
    tag = profile["board"]["tags"][0]
    edge = tag["size_m"]*1000.
    grid = detector.dictionary.markerSize+2
    quiet = edge/grid
    page = (210., 297.) if edge+2*quiet <= 200. else (edge+2*quiet,)*2
    x0, y0 = (page[0]-edge)/2, (page[1]-edge)/2
    cell = edge/grid
    marker = cv2.aruco.drawMarker(detector.dictionary, tag["id"], grid)
    cells = [f'<rect x="{x0+c*cell:g}" y="{y0+r*cell:g}" width="{cell:g}" height="{cell:g}" fill="black"/>'
             for r in range(grid) for c in range(grid) if marker[r, c] == 0]
    filename = f'{profile["board"]["family"]}-id{tag["id"]}-black{edge:g}mm'+("-a4" if page == (210., 297.) else "")+".svg"
    (out/filename).write_text(
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{page[0]:g}mm" height="{page[1]:g}mm" viewBox="0 0 {page[0]:g} {page[1]:g}">'
        f'<rect width="{page[0]:g}" height="{page[1]:g}" fill="white"/>'+"".join(cells)+'</svg>')
    print_tag = dict(family=profile["board"]["family"], id=tag["id"],
                     black_outer_edge_mm=edge, quiet_margin_mm=quiet, svg_page_mm=page, svg=filename)
    summaries = {}
    for name in variants:
        subset = [r for r in rows if r["layout"] == name]
        clear = [r for r in subset if r["offset_m"] == r["tilt_rad"] == r["center_occlusion_fraction"] == r["blur_kernel"] == 0]
        summaries[name] = dict(cases=len(subset), valid=sum(r["valid"] for r in subset),
                               centered_clear_valid_heights_m=[r["height_m"] for r in clear if r["valid"]],
                               invalid_reasons={reason: sum(r["reason"] == reason for r in subset) for reason in set(r["reason"] for r in subset if not r["valid"])})
    (out/"images.jsonl").write_text("".join(json.dumps(r)+"\n" for r in rows))
    (out/"guidance.json").write_text(json.dumps(guidance, indent=2))
    (out/"sensitivity.json").write_text(json.dumps(sensitivity, indent=2))
    report = dict(kind="SYNTHETIC_SCAN", config=profile, image_cases=len(rows),
                  layouts=summaries, calibration_cases=len(sensitivity), guidance_cases=len(guidance),
                  unauthorized_descent=sum(r["unauthorized_descent"] for r in guidance),
                  tag=print_tag,
                  limitation="synthetic pinhole/radtan and simple velocity model; no hardware accuracy or fusion proof")
    report["source_sha256"] = fingerprints(args.config)
    (out/"report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    return int(report["unauthorized_descent"] != 0)

if __name__ == "__main__":
    raise SystemExit(main())
