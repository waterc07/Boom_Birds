#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "${ROOT}/companion/ros2_ws/tools/activate_python_env.sh"
source "${BB_NAV_INSTALL:-${HOME}/bb_build/ego-single/install}/setup.bash"
source "${BB_OV_INSTALL:-${HOME}/bb_build/ov/install}/setup.bash"
source "${BB_GAZEBO_BUILD_ROOT:-${HOME}/bb_build/gazebo}/install/setup.bash"
# 回放读取当前感知源码；深度使用默认 SGBM 参数。
export PYTHONPATH="${ROOT}/companion/ros2_ws/src/boom_birds_sensing:${ROOT}/companion/ros2_ws/src/stereo_depth:${PYTHONPATH:-}"
export OPENBLAS_NUM_THREADS=1
case "${1:-}" in
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
    echo "用法：$0 baseline|capture|replay|evaluate|matrix [参数]" >&2
    exit 2;;
esac
