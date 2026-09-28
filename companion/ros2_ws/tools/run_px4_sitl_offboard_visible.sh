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
origin_x="${BB_SITL_WORLD_ORIGIN_X:-0}"
origin_y="${BB_SITL_WORLD_ORIGIN_Y:-0}"
origin_z="${BB_SITL_WORLD_ORIGIN_Z:-0}"
"$PX4_COMMAND" status
alt_ned="$("$PX4_LISTENER" vehicle_local_position -n 1 | awk '$1 == "z:" {print $2; exit}')"
if ! awk -v z="$alt_ned" 'BEGIN { exit !(z <= -1.3) }'; then
  echo "拒绝切 OFFBOARD：当前 NED z=$alt_ned m；请先在 SIH 起飞并稳定高于 1.3 m。" >&2
  exit 1
fi
echo "已核对 SIH 高度：NED z=$alt_ned m"
echo "等待 EGO 首个规划指令，再切本机 PX4 SIH OFFBOARD；请先开本终端，再启动规划链。"
first_cmd="$(timeout 90s ros2 topic echo /boom_birds/ego/position_cmd \
  quadrotor_msgs/msg/PositionCommand --once --field position)"
local_pose="$("$PX4_LISTENER" vehicle_local_position -n 1)"
cmd_x="$(printf '%s\n' "$first_cmd" | awk '$1 == "x:" {print $2; exit}')"
cmd_y="$(printf '%s\n' "$first_cmd" | awk '$1 == "y:" {print $2; exit}')"
cmd_z="$(printf '%s\n' "$first_cmd" | awk '$1 == "z:" {print $2; exit}')"
local_x="$(printf '%s\n' "$local_pose" | awk '$1 == "x:" {print $2; exit}')"
local_y="$(printf '%s\n' "$local_pose" | awk '$1 == "y:" {print $2; exit}')"
local_z="$(printf '%s\n' "$local_pose" | awk '$1 == "z:" {print $2; exit}')"
if ! awk -v cx="$cmd_x" -v cy="$cmd_y" -v cz="$cmd_z" \
    -v x="$local_x" -v y="$local_y" -v z="$local_z" \
    -v ox="$origin_x" -v oy="$origin_y" -v oz="$origin_z" \
    'BEGIN { if (cx == "" || cy == "" || cz == "" || x == "" || y == "" || z == "") exit 1;
             dx=cx-(x+ox); dy=cy-(-y+oy); dz=cz-(-z+oz);
             exit !((dx*dx + dy*dy + dz*dz) <= 0.25) }'; then
  echo "拒绝切 OFFBOARD：首个规划设定点与当前位置相距超过 0.5 m，可能已错过轨迹前段。" >&2
  echo "规划位置 ROS=($cmd_x,$cmd_y,$cmd_z)；PX4 NED=($local_x,$local_y,$local_z)" >&2
  exit 1
fi
"$PX4_COMMAND" mode offboard
"$PX4_COMMAND" status
if [[ "${BB_SITL_CLOSE_ON_DONE:-0}" == "1" ]]; then exit 0; fi
exec bash
