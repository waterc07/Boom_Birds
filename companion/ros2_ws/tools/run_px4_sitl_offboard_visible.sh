#!/usr/bin/env bash
# 本机 PX4 SIH：等待编排器完成 OFFBOARD 接管 + 提供手工核验入口。
#
# 边界（A2 单一配置来源）：
#   * 高度判定、接管距离判定、模式判定都在编排器（lifecycle_node，经共享判定函数）
#     里；本脚本不基于 NED z 判定高度，也不自己计算设定点距离。
#   * 模式控制迁到 PX4 接口服务通路：lifecycle_node → /boom_birds/control/action
#     → px4_interface_node → PX4；本脚本不调用 px4-commander 切模式。
set -eo pipefail
BB_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$BB_PROJECT_ROOT"
for originVar in BB_SITL_WORLD_ORIGIN_X BB_SITL_WORLD_ORIGIN_Y BB_SITL_WORLD_ORIGIN_Z; do
  if [[ -n "${!originVar+x}" ]]; then
    echo "旧原点变量不再受支持；30 m 场景请在 launch 里用 scene:=forest_30m。" >&2
    exit 2
  fi
done
source companion/ros2_ws/tools/activate_python_env.sh
source "${INSTALL_BASE:-${HOME}/bb_build/main/install}/setup.bash"
PX4_BUILD="${PX4_BUILD:-${PX4_SOURCE:-${HOME}/PX4-Autopilot}/build/px4_sitl_default}"
px4Root="${PX4_BUILD}/rootfs/0"
px4Command="${PX4_BUILD}/bin/px4-commander"
px4Listener="${PX4_BUILD}/bin/px4-listener"
cd "$px4Root"

echo "--- PX4 SIH 当前状态 ---"
"$px4Command" status
echo "--- 当前局部位置回读（仅显示，不参与判定）---"
"$px4Listener" vehicle_local_position -n 1

waitS="${BB_SITL_WAIT_S:-$(python3 -m boom_birds_sim.sih_params --get takeoff_timeout_s)}"
echo "等待编排器把任务推进到 EXECUTING（窗口 ${waitS}s）；判定与切模式都不在本脚本。"
if ! python3 -m boom_birds_bringup.lifecycle_cli status --wait-state EXECUTING --timeout "$waitS"; then
  echo "编排器未在 ${waitS}s 内进入 EXECUTING：OFFBOARD 未确认或已闭锁；请查 /boom_birds/mission/status。" >&2
  exit 1
fi
"$px4Command" status
if [[ "${BB_SITL_CLOSE_ON_DONE:-0}" == "1" ]]; then
  exit 0
fi
exec bash
