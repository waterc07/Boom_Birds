#!/usr/bin/env bash
# 仅在 SIH 已进入 OFFBOARD 并稳定悬停后启动目标规划。
set -eo pipefail
cd /home/waterc/workspace/Boom_Birds
source companion/ros2_ws/tools/activate_python_env.sh
source /home/waterc/bb_build/main/install/setup.bash
PX4_ROOT=/home/waterc/PX4-Autopilot/build/px4_sitl_default/rootfs/0
PX4_COMMAND=/home/waterc/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander
if ! (cd "$PX4_ROOT" && "$PX4_COMMAND" status) | grep -q 'navigation mode: Offboard'; then
  echo "拒绝启动目标规划：本机 PX4 SIH 尚未进入 Offboard。" >&2
  exit 1
fi
echo "SIH 已进入 Offboard，启动 EGO 目标规划。"
exec ros2 launch ego_planner boom_birds_offline.launch.py \
  position_cmd_topic:=/boom_birds/ego/position_cmd_raw \
  fx:=129.4485112145058 fy:=129.4485112145058 \
  cx:=117.6467628479004 cy:=87.11334800720215 "$@"
