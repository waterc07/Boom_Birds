#!/usr/bin/env bash
# PX4 SIH 已在 Hold 且悬停时，等待规划指令后切 OFFBOARD。
set -eo pipefail
cd /home/waterc/workspace/Boom_Birds
source companion/ros2_ws/tools/activate_python_env.sh
source /home/waterc/bb_build/main/install/setup.bash
PX4_ROOT=/home/waterc/PX4-Autopilot/build/px4_sitl_default/rootfs/0
PX4_COMMAND=/home/waterc/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander
PX4_LISTENER=/home/waterc/PX4-Autopilot/build/px4_sitl_default/bin/px4-listener
cd "$PX4_ROOT"
"$PX4_COMMAND" status
alt_ned="$("$PX4_LISTENER" vehicle_local_position -n 1 | awk '$1 == "z:" {print $2; exit}')"
if ! awk -v z="$alt_ned" 'BEGIN { exit !(z <= -2.0) }'; then
  echo "拒绝切 OFFBOARD：当前 NED z=$alt_ned m；请先在 SIH 起飞并稳定高于 2 m。" >&2
  exit 1
fi
echo "已核对 SIH 高度：NED z=$alt_ned m"
echo "等待 EGO 规划输出，再切本机 PX4 SIH OFFBOARD..."
timeout 45s ros2 topic echo /boom_birds/ego/position_cmd \
  quadrotor_msgs/msg/PositionCommand --once >/dev/null
"$PX4_COMMAND" mode offboard
"$PX4_COMMAND" status
exec bash
