#!/usr/bin/env bash
# EGO 单机默认 random_forest 场景；保留合成双目 -> 深度链。
set -euo pipefail
BB_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$BB_PROJECT_ROOT"
# synth_map_resolution_m 是仿真点云体素分辨率（不是高度/接管阈值），与 random_forest
# 的 map/resolution 对齐；合成图像发布高度下限同样由 launch 从 RuntimeConfig 取值。
exec bash companion/ros2_ws/tools/run_px4_sitl_motion_visible.sh \
  use_random_forest:=true bootstrap_only:=true \
  synth_map_topic:=/boom_birds/sitl/map synth_map_resolution_m:=0.1 "$@"
