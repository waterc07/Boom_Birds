#!/usr/bin/env python3
"""脱机核对单目订阅及合成特征滤波；不验收真实 VIO、前端或 Pi 预算。"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from boom_birds_bringup.compute_profile import prepare_mono_vio, openvins_parameters, opencv_yaml
from boom_birds_bringup.hardware_profile import read_yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True, help="新的证据目录")
    args = parser.parse_args()
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    src = Path(__file__).resolve().parents[1] / "src/open_vins"
    config = prepare_mono_vio(src / "config/boombirds_synthetic/estimator_config.yaml", out / "mono")
    params = openvins_parameters(config)
    # 合成测试使用已发布的 EuRoC 噪声模型，不把零噪声占位当实机数据。
    imu = read_yaml(src / "config/euroc_mav/kalibr_imu_chain.yaml")
    imu["imu0"]["rostopic"] = "/boom_birds/imu"
    (config.parent / "kalibr_imu_chain.yaml").write_text("%YAML:1.0\n# TEST-ONLY EuRoC noise\n" + opencv_yaml(imu))
    executable = Path(get_package_prefix("ov_msckf")) / "lib/ov_msckf"
    param_file = out / "subscriber_params.yaml"
    param_file.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
    env = dict(os.environ, ROS_LOCALHOST_ONLY="1")
    commands = []
    command = [str(executable / "run_subscribe_msckf"), "--ros-args", "--params-file", str(param_file)]
    commands.append(command)
    rclpy.init()
    node = rclpy.create_node("mono_subscription_check")
    counts = {}
    try:
        with (out / "subscriber.log").open("w") as log:
            proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=env)
            try:
                deadline = time.monotonic() + 20
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.1)
                    counts = {t: node.count_subscribers(t) for t in
                              ("/boom_birds/stereo/left_raw", "/boom_birds/stereo/right_raw", "/boom_birds/imu")}
                    if proc.poll() is not None:
                        raise RuntimeError("OpenVINS 启动失败，见 subscriber.log")
                    if counts["/boom_birds/stereo/left_raw"] == 1 and counts["/boom_birds/imu"] == 1:
                        break
                if list(counts.values()) != [1, 0, 1]:
                    raise RuntimeError("订阅不符合单目配置: " + str(counts))
            finally:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    # 截取前 20 秒轨迹；run_simulation 使用真值初始化并直接输入特征，绕过 KLT。
    lines = (src / "ov_data/sim/udel_arl_short.txt").read_text().splitlines()
    samples = [line for line in lines if line and not line.startswith("#")]
    start = float(samples[0].split()[0])
    samples = [line for line in samples if float(line.split()[0]) <= start + 20]
    trajectory = out / "trajectory.txt"
    trajectory.write_text("\n".join(samples) + "\n")
    params.update(sim_seed_state_init=0, sim_seed_preturb=0, sim_seed_measurements=0,
                  sim_distance_threshold=1.1, sim_min_feature_gen_dist=5., sim_max_feature_gen_dist=7.,
                  sim_traj_path=str(trajectory), sim_freq_cam=20., sim_freq_imu=200., sim_do_perturbation=False,
                  save_total_state=True, filepath_est=str(out / "estimate.txt"), filepath_std=str(out / "std.txt"),
                  filepath_gt=str(out / "truth.txt"))
    param_file = out / "simulation_params.yaml"
    param_file.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
    command = [str(executable / "run_simulation"), "--ros-args", "--params-file", str(param_file)]
    commands.append(command)
    with (out / "simulation.log").open("w") as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, env=env, timeout=90)
    if result.returncode != 0:
        raise RuntimeError("合成滤波失败，见 simulation.log")
    states = np.loadtxt(out / "estimate.txt")
    if states.ndim != 2 or len(states) < 100 or not np.isfinite(states).all():
        raise RuntimeError("合成状态输出不足或非有限")
    report = {"subscription": "PASS", "subscriber_counts": counts,
              "synthetic_filter": "PASS", "finite_states": len(states),
              "synthetic_initialization": "GROUND_TRUTH", "image_frontend": "NOT RUN",
              "pi_full_load_budget": "NOT RUN", "real_vio_quality": "NOT RUN", "commands": commands}
    (out / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
