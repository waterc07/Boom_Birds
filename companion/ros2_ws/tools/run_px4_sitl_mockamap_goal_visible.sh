#!/usr/bin/env bash
# 仅在编排器已接管（EXECUTING）后启动目标规划。
# 模式判定不在 shell：等编排器状态（lifecycle_cli ← /boom_birds/mission/status），
# 本脚本不 grep PX4 模式字符串。
set -eo pipefail
BB_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$BB_PROJECT_ROOT"
source companion/ros2_ws/tools/activate_python_env.sh
source "${INSTALL_BASE:-${HOME}/bb_build/main/install}/setup.bash"
wait_s="${BB_SITL_WAIT_S:-$(python3 -m boom_birds_bringup.sih_params --get takeoff_timeout_s)}"
if ! python3 -m boom_birds_bringup.lifecycle_cli status --wait-state EXECUTING --timeout "$wait_s"; then
  echo "拒绝启动目标规划：编排器未在 ${wait_s}s 内进入 EXECUTING（OFFBOARD 未确认或已闭锁）。" >&2
  exit 1
fi
echo "编排器已进入 EXECUTING，启动 EGO 目标规划。"
# 几何由正在运行的深度节点提供，禁止命令行覆盖成另一套内参。
for arg in "$@"; do
  case "$arg" in fx:=*|fy:=*|cx:=*|cy:=*) echo "内参由深度 CameraInfo 提供，不能覆盖。" >&2; exit 2;; esac
done
python3 -m boom_birds_nav.camera_geometry >/dev/null
# 等待 CameraInfo 期间编排器可能已闭锁/退出 EXECUTING，再核对一次（判据仍在编排器）。
if ! python3 -m boom_birds_bringup.lifecycle_cli status --wait-state EXECUTING --timeout "$wait_s"; then
  echo "拒绝启动目标规划：等待内参期间编排器已不在 EXECUTING。" >&2
  exit 1
fi
exec ros2 launch ego_planner boom_birds_offline.launch.py \
  use_camera_info:=true position_cmd_topic:=/boom_birds/ego/position_cmd_raw "$@"
