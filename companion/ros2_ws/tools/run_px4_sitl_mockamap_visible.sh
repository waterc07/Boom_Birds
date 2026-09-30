#!/usr/bin/env bash
set -euo pipefail
BB_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$BB_PROJECT_ROOT"
echo "EGO mockamap 复杂场景；保留合成左右图 -> depth_node -> EGO -> PX4 SIH 链。"
# 合成图像发布高度下限不在这里写数值：px4_sitl_motion.launch.py 从 RuntimeConfig 的
# image_publish_min_altitude_agl_m 取值（需要临时改值时显式传 synth_min_altitude_m:=）。
exec bash companion/ros2_ws/tools/run_px4_sitl_motion_visible.sh \
  use_mockamap:=true synth_map_topic:=/boom_birds/sitl/map "$@"
