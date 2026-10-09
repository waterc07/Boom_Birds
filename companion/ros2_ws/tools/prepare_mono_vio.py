#!/usr/bin/env python3
"""从已有 OpenVINS 标定生成单目算力配置；不访问设备。"""
import argparse
from boom_birds_bringup.compute_profile import prepare_mono_vio

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, help="已有 estimator_config.yaml")
    parser.add_argument("--out", required=True, help="新目录，不覆盖已有目录")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()
    print(prepare_mono_vio(args.source, args.out, args.width, args.height))
