#!/usr/bin/env bash
# 仅对本机 PX4 SIH -i 0；起飞并显示实际高度，供 OFFBOARD 前核验。
set -eo pipefail
PX4_ROOT=/home/waterc/PX4-Autopilot/build/px4_sitl_default/rootfs/0
PX4_BIN=/home/waterc/PX4-Autopilot/build/px4_sitl_default/bin
cd "$PX4_ROOT"
"$PX4_BIN/px4-commander" status
"$PX4_BIN/px4-commander" takeoff
for i in $(seq 1 30); do
  z="$("$PX4_BIN/px4-listener" vehicle_local_position -n 1 | awk '$1 == "z:" {print $2; exit}')"
  printf 'SIH NED z=%s m（高于原点约 %.2f m）\n' "$z" "$(awk -v z="$z" 'BEGIN { print -z }')"
  if awk -v z="$z" 'BEGIN { exit !(z <= -2.0) }'; then
    "$PX4_BIN/px4-commander" status
    echo "已达到 2 m，高度闸门允许后续 OFFBOARD 切换。"
    exec bash
  fi
  sleep 1
done
echo "30 s 内未达到 2 m；请检查 PX4 状态，未切 OFFBOARD。" >&2
exec bash
