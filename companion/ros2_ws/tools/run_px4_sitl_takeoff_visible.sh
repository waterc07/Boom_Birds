#!/usr/bin/env bash
# 仅对本机 PX4 SIH -i 0：下发 PX4 起飞参数 + 提供手工核验入口。
#
# 边界（A2 单一配置来源）：
#   * PX4 起飞参数只由 boom_birds_bringup.sih_params 从 RuntimeConfig 生成：数值取
#     takeoff_altitude_agl_m，参数名取 px4_takeoff_param_name；本脚本不出现任何
#     高度字面量，也不写 PX4 参数名。
#   * 本脚本不做高度/模式判定，也不调用 px4-commander 做模式控制：解锁、起飞、
#     切 OFFBOARD 由编排器经 PX4 接口服务执行（lifecycle_node → VehicleAction）。
set -eo pipefail
BB_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$BB_PROJECT_ROOT"
source companion/ros2_ws/tools/activate_python_env.sh
source "${INSTALL_BASE:-${HOME}/bb_build/main/install}/setup.bash"
PX4_BUILD="${PX4_BUILD:-${PX4_SOURCE:-${HOME}/PX4-Autopilot}/build/px4_sitl_default}"
PX4_ROOT="${PX4_BUILD}/rootfs/0"
PX4_BIN="${PX4_BUILD}/bin"
cd "$PX4_ROOT"

echo "--- PX4 SIH 当前状态 ---"
"$PX4_BIN/px4-commander" status

echo "--- 即将下发的 PX4 参数（来源：config/runtime.yaml）---"
python3 -m boom_birds_bringup.sih_params --format text
python3 -m boom_birds_bringup.sih_params --apply --param-bin "$PX4_BIN/px4-param"

echo "--- 手工核验入口：以下只显示回读值，不参与任何判定 ---"
"$PX4_BIN/px4-listener" vehicle_local_position -n 1
cat <<'BB_HINT'
解锁/起飞/切 OFFBOARD 由编排器执行，不要在本终端手工切模式：
  ros2 launch boom_birds_nav px4_sih_mission.launch.py scene:=local
  python3 -m boom_birds_bringup.lifecycle_cli start --goal X Y Z
  python3 -m boom_birds_bringup.lifecycle_cli status --wait-state HOLD_READY
BB_HINT

if [[ "${BB_SIH_INTERACTIVE:-1}" == "1" ]]; then
  exec bash
fi
