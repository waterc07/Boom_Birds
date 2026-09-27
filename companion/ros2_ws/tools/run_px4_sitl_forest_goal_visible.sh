#!/usr/bin/env bash
# 森林场景的目标规划在 SIH 进入 OFFBOARD 后启动。
set -euo pipefail
cd /home/waterc/workspace/Boom_Birds
if [ "$#" -eq 0 ]; then
  set -- goal_x:=4.0 goal_y:=-3.0 goal_z:=1.5
fi
exec bash companion/ros2_ws/tools/run_px4_sitl_mockamap_goal_visible.sh "$@"
