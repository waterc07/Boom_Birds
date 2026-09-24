#!/usr/bin/env bash
set -euo pipefail
cd /home/waterc/workspace/Boom_Birds
echo "EGO mockamap 复杂场景；保留合成左右图 -> depth_node -> EGO -> PX4 SIH 链。"
exec bash companion/ros2_ws/tools/run_px4_sitl_motion_visible.sh \
  use_mockamap:=true synth_map_topic:=/boom_birds/sitl/mockamap \
  synth_min_altitude_m:=2.0 "$@"
