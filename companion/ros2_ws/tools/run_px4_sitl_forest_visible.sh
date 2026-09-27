#!/usr/bin/env bash
# EGO 单机默认 random_forest 场景；保留合成双目 -> 深度链。
set -euo pipefail
cd /home/waterc/workspace/Boom_Birds
exec bash companion/ros2_ws/tools/run_px4_sitl_motion_visible.sh \
  use_random_forest:=true bootstrap_only:=true \
  synth_map_topic:=/boom_birds/sitl/map synth_map_resolution_m:=0.1 \
  synth_min_altitude_m:=1.3 "$@"
