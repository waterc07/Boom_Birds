"""离线比较 Gazebo 真值与算法输出；只对齐初始航向/平移，不拟合尺度。"""

import argparse
import json
import re
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from cv_bridge import CvBridge
from .gazebo_capture import LEFT, TRUTH, DEPTH_TRUTH, ns
from .gazebo_replay import POSE, DEPTH, ODOM, GATES
from .gazebo_fixture import check_imu_truth


def read_messages(path, selected):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {
        t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in selected
    }
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    while reader.has_next():
        topic, data, _ = reader.read_next()
        yield topic, deserialize_message(data, types[topic])


def pose_row(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return [ns(msg.header.stamp) * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w]


def pose_errors(truth, estimate):
    """时间插值后，用第一条姿态定航向、第一条位置定平移。"""
    truth = np.asarray(truth, float)
    estimate = np.asarray(estimate, float)
    estimate = estimate[(estimate[:, 0] >= truth[0, 0]) & (estimate[:, 0] <= truth[-1, 0])]
    if len(estimate) < 2:
        raise ValueError("insufficient overlapping poses")
    t = estimate[:, 0]
    positions = np.column_stack([np.interp(t, truth[:, 0], truth[:, i]) for i in (1, 2, 3)])
    true_rotation = Slerp(truth[:, 0], Rotation.from_quat(truth[:, 4:8]))(t)
    est_rotation = Rotation.from_quat(estimate[:, 4:8])
    yaw_true = true_rotation[0].as_euler("xyz")[2]
    yaw_est = est_rotation[0].as_euler("xyz")[2]
    alignment = Rotation.from_euler("z", yaw_true - yaw_est)
    aligned = alignment.apply(estimate[:, 1:4])
    aligned += positions[0] - aligned[0]
    errors = np.linalg.norm(aligned - positions, axis=1)
    angles = (true_rotation.inv() * (alignment * est_rotation)).magnitude() * 180 / np.pi
    result = dict(
        samples=len(t),
        first_output_sim_s=float(t[0]),
        position_rmse_m=float(np.sqrt(np.mean(errors**2))),
        position_p95_m=float(np.percentile(errors, 95)),
        endpoint_error_m=float(errors[-1]),
        orientation_p95_deg=float(np.percentile(angles, 95)),
        scale_fitted=False,
        yaw_alignment_rad=float(yaw_true - yaw_est),
        reference_travel_m=float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()),
    )
    return result, np.column_stack([t, positions, aligned, errors, angles])


def estimator_diagnostics(out):
    """从算法日志读取初始化、零速判断、视觉更新和偏置。"""
    log = Path(out) / "openvins.log"
    if not log.exists():
        return {"available": False}
    text = re.sub(r"\x1b\[[0-9;]*m", "", log.read_text(errors="replace"))
    counts = [int(n) for n in re.findall(r"MSCKF update \((\d+) feats\)", text)]
    biases = re.findall(r"bg = ([^|\n]+) \| ba = ([^\n]+)", text)

    def vector(value):
        return [float(v) for v in value.strip().split(",")]

    return dict(
        available=True,
        initialization_success="successful initialization" in text,
        zupt_accepted=len(re.findall(r"\[ZUPT\]: accepted", text)),
        zupt_rejected=len(re.findall(r"\[ZUPT\]: rejected", text)),
        msckf_update_frames=len(counts),
        msckf_nonzero_update_frames=sum(n > 0 for n in counts),
        msckf_updated_features_total=sum(counts),
        msckf_last_100_nonzero_frames=sum(n > 0 for n in counts[-100:]),
        final_gyro_bias_rad_s=None if not biases else vector(biases[-1][0]),
        final_accel_bias_m_s2=None if not biases else vector(biases[-1][1]),
        state_file_exists=(Path(out) / "state-estimate.txt").exists(),
    )


def evaluate(source, out):
    source, out = Path(source), Path(out)
    bridge = CvBridge()
    truth, estimate = [], []
    odometry = []
    imu = []
    depth_truth = {}
    first_image = None
    for topic, msg in read_messages(
        source / "inputs", [TRUTH, DEPTH_TRUTH, LEFT, "/boom_birds/imu"]
    ):
        if topic == TRUTH:
            truth.append(pose_row(msg))
        elif topic == "/boom_birds/imu":
            a, w = msg.linear_acceleration, msg.angular_velocity
            imu.append([ns(msg.header.stamp) * 1e-9, a.x, a.y, a.z, w.x, w.y, w.z])
        elif topic == DEPTH_TRUTH:
            array = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            depth_truth[ns(msg.header.stamp)] = cv2.resize(
                array, (320, 240), interpolation=cv2.INTER_NEAREST
            )
        elif first_image is None:
            first_image = bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            cv2.imwrite(str(out / "left.png"), first_image)
    # 去掉真值中重复时间，不用重新标定算法时间。
    truth = np.asarray(truth)
    if len(truth):
        _, indices = np.unique(truth[:, 0], return_index=True)
        truth = truth[np.sort(indices)]
    residuals, ratios = [], []
    range_bins = {
        (a, b): [] for a, b in ((0.2, 2.0), (2.0, 3.0), (3.0, 4.0), (4.0, 6.0), (6.0, 15.0))
    }
    signed_residuals = []
    paired_depth = 0
    stamp_misses = 0
    contracts = True
    all_depth_count = 0
    for topic, msg in read_messages(out / "outputs", [POSE, DEPTH, ODOM]):
        if topic == POSE:
            estimate.append(pose_row(msg))
            continue
        if topic == ODOM:
            odometry.append(pose_row(msg))
            continue
        all_depth_count += 1
        contracts &= msg.encoding == "32FC1" and (msg.width, msg.height) == (320, 240)
        array = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        contracts &= not bool(np.isinf(array).any()) and not bool((array == 0).any())
        gt = depth_truth.get(ns(msg.header.stamp))
        if gt is None:
            stamp_misses += 1
            continue
        # 固定中心区域，排除近边界和左侧视差搜索边界。
        observed = array[40:200, 60:280]
        expected = gt[40:200, 60:280]
        target = np.isfinite(expected) & (expected >= 0.2) & (expected <= 15.0)
        valid = target & np.isfinite(observed)
        ratios.append(float(valid.sum() / max(target.sum(), 1)))
        signed = observed[valid] - expected[valid]
        signed_residuals.append(signed)
        residuals.append(np.abs(signed))
        for (lo, hi), entries in range_bins.items():
            mask = valid & (expected >= lo) & (expected < hi)
            entries.append(np.abs(observed[mask] - expected[mask]))
        paired_depth += 1
        if paired_depth == 5:
            preview = np.nan_to_num(array, nan=0.0)
            cv2.imwrite(
                str(out / "depth.png"),
                cv2.applyColorMap(np.uint8(np.clip(preview / 8, 0, 1) * 255), cv2.COLORMAP_TURBO),
            )
    gates_path = out / "gates.json"
    gates = json.loads(gates_path.read_text()) if gates_path.exists() else dict(GATES)
    capture = json.loads((source / "capture.json").read_text())
    report = {
        "scope": "TEST_ONLY_Gazebo_sensor_rig_and_recorded_playback",
        "gates": gates,
        "capture_pass": capture["pass"],
        "errors": [],
        "sensor_physics": check_imu_truth(truth, imu),
        "estimator_diagnostics": estimator_diagnostics(out),
    }
    if len(estimate) > 1 and len(truth) > 1:
        try:
            vio, trajectory = pose_errors(truth, estimate)
            initial = capture["first_sim_s"]
            vio["first_pose_elapsed_sim_s"] = vio["first_output_sim_s"] - initial
            image_stamps = json.loads((source / "raw-image-stamps.json").read_text())[0]
            remaining = sum(t * 1e-9 >= vio["first_output_sim_s"] for t in image_stamps)
            vio["pose_coverage"] = min(1.0, vio["samples"] / max(remaining, 1))
            vio["pass"] = bool(
                vio["samples"] >= gates["pose_min_count"]
                and vio["first_pose_elapsed_sim_s"] <= gates["init_max_s"]
                and vio["pose_coverage"] >= gates["pose_coverage_min"]
                and vio["position_rmse_m"] <= gates["position_rmse_max_m"]
                and vio["endpoint_error_m"] <= gates["endpoint_error_max_m"]
                and vio["orientation_p95_deg"] <= gates["orientation_p95_max_deg"]
            )
            vio["visual_updates_observed"] = (
                report["estimator_diagnostics"].get("msckf_nonzero_update_frames", 0) > 0
            )
            if gates.get("require_visual_updates", False):
                vio["pass"] &= vio["visual_updates_observed"]
            vio["max_pose_gap_sim_s"] = float(np.max(np.diff(trajectory[:, 0])))
            vio["error_by_time_window"] = []
            for lo in range(0, 45, 5):
                selected = trajectory[(trajectory[:, 0] >= lo) & (trajectory[:, 0] < lo + 5)]
                if len(selected):
                    vio["error_by_time_window"].append(
                        dict(
                            start_sim_s=lo,
                            end_sim_s=lo + 5,
                            samples=len(selected),
                            rmse_m=float(np.sqrt(np.mean(selected[:, 7] ** 2))),
                            max_error_m=float(np.max(selected[:, 7])),
                        )
                    )
            report["vio"] = vio
            np.savetxt(
                out / "trajectory.csv",
                trajectory,
                delimiter=",",
                header="t,gt_x,gt_y,gt_z,vio_x,vio_y,vio_z,error_m,angle_deg",
            )
        except ValueError as error:
            report["errors"].append(str(error))
    else:
        report["vio"] = {
            "pass": False,
            "samples": len(estimate),
            "reason": "no initialized image-driven VIO",
        }
    values = np.concatenate(residuals) if residuals else np.array([])
    depth = dict(
        matched_frames=paired_depth,
        output_frames=all_depth_count,
        unmatched_output_stamps=stamp_misses,
        contracts_pass=bool(contracts),
        roi=[40, 200, 60, 280],
    )
    if len(values):
        depth.update(
            median_absolute_error_m=float(np.median(values)),
            p95_absolute_error_m=float(np.percentile(values, 95)),
            mean_valid_ratio=float(np.mean(ratios)),
        )
    depth["error_by_reference_range"] = []
    for (lo, hi), entries in range_bins.items():
        errors = np.concatenate(entries) if entries else np.array([])
        if len(errors):
            depth["error_by_reference_range"].append(
                dict(
                    min_m=lo,
                    max_m=hi,
                    pixels=len(errors),
                    median_abs_m=float(np.median(errors)),
                    p95_abs_m=float(np.percentile(errors, 95)),
                )
            )
    depth["median_signed_error_m"] = (
        None if not signed_residuals else float(np.median(np.concatenate(signed_residuals)))
    )
    depth["pass"] = bool(
        len(values)
        and contracts
        and paired_depth >= gates["depth_pair_min_count"]
        and depth["median_absolute_error_m"] <= gates["depth_median_abs_max_m"]
        and depth["p95_absolute_error_m"] <= gates["depth_p95_abs_max_m"]
        and depth["mean_valid_ratio"] >= gates["depth_valid_min_ratio"]
    )
    report["depth"] = depth
    report["imu_odometry"] = dict(
        samples=len(odometry),
        first_output_elapsed_sim_s=(
            None if not odometry else odometry[0][0] - capture["first_sim_s"]
        ),
        finite=bool(odometry and np.isfinite(odometry).all()),
    )
    report["pass"] = bool(
        capture["pass"]
        and report.get("vio", {}).get("pass")
        and depth["pass"]
        and report["sensor_physics"]["pass_check"]
    )
    (out / "evaluation.json").write_text(json.dumps(report, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    report = evaluate(args.input, args.out)
    print(json.dumps(report, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
