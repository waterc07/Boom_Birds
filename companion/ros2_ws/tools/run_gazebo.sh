#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "${root}/companion/ros2_ws/tools/activate_python_env.sh"
source "${BB_NAV_INSTALL:-${HOME}/bb_build/ego-single/install}/setup.bash"
source "${BB_OV_INSTALL:-${HOME}/bb_build/ov/install}/setup.bash"
source "${BB_GAZEBO_BUILD_ROOT:-${HOME}/bb_build/gazebo}/install/setup.bash"
# 仿真与回放读取当前源码；编译插件仍来自 Gazebo 安装前缀。
if [[ "${BB_GAZEBO_SOURCE_OVERLAY:-1}" == 1 ]]; then
export PYTHONPATH="${root}/companion/ros2_ws/src/boom_birds_sim:${root}/companion/ros2_ws/src/boom_birds_control:${root}/companion/ros2_ws/src/boom_birds_sensing:${root}/companion/ros2_ws/src/stereo_depth:${PYTHONPATH:-}"
fi
if [[ -n "${BB_GAZEBO_MIDDLEWARE_ENV:-}" ]]; then
  source "${BB_GAZEBO_MIDDLEWARE_ENV}"
fi
export OPENBLAS_NUM_THREADS=1
case "${1:-}" in
  session)
    shift
    exec python3 -m boom_birds_sim.gazebo_session "$@";;
  baseline|capture)
    exec python3 -m boom_birds_sim.gazebo_run "$@";;
  replay)
    shift
    exec python3 -m boom_birds_sim.gazebo_replay "$@";;
  matrix)
    shift
    exec python3 -m boom_birds_sim.gazebo_matrix "$@";;
  evaluate)
    shift
    exec python3 -m boom_birds_sim.gazebo_evaluate "$@";;
  *)
    echo "用法：$0 session|baseline|capture|replay|evaluate|matrix [参数]" >&2
    exit 2;;
esac
