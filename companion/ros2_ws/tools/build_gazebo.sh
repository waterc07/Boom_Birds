#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "${root}/companion/ros2_ws/tools/activate_python_env.sh"
source "${BB_NAV_INSTALL:-${HOME}/bb_build/ego-single/install}/setup.bash"
source "${BB_OV_INSTALL:-${HOME}/bb_build/ov/install}/setup.bash"
BB_GAZEBO_BUILD_ROOT="${BB_GAZEBO_BUILD_ROOT:-${HOME}/bb_build/gazebo}"
BUILD_BASE="${HOME}/bb_build/ov/build" INSTALL_BASE="${BB_OV_INSTALL:-${HOME}/bb_build/ov/install}" \
  LOG_BASE="${BB_GAZEBO_BUILD_ROOT}/ov-log" \
  bash "${root}/companion/ros2_ws/tools/build_all.sh" \
  --base-paths "${root}/companion/ros2_ws/src/open_vins" --packages-select ov_msckf
BUILD_BASE="${BB_GAZEBO_BUILD_ROOT}/build" INSTALL_BASE="${BB_GAZEBO_BUILD_ROOT}/install" \
  LOG_BASE="${BB_GAZEBO_BUILD_ROOT}/log" \
  bash "${root}/companion/ros2_ws/tools/build_all.sh" \
  --base-paths "${root}/companion/ros2_ws/src/stereo_depth" \
  "${root}/companion/ros2_ws/src/boom_birds_sensing" \
  "${root}/companion/ros2_ws/src/boom_birds_sim" \
  --packages-select stereo_depth boom_birds_sensing boom_birds_sim \
  --cmake-args -DBB_BUILD_GAZEBO_MOTION=ON
