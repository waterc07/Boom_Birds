#!/usr/bin/env bash
BB_PLATFORM_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${BB_PLATFORM_WS}/tools/activate_python_env.sh"
BB_PLATFORM_PREFIX="${BB_PLATFORM_PREFIX:-${HOME}/bb_build/ego-single/install}"
if [[ -f "${BB_PLATFORM_PREFIX}/setup.bash" ]]; then
  source "${BB_PLATFORM_PREFIX}/setup.bash"
fi
BB_PLATFORM_OV_PREFIX="${OV_INSTALL:-${HOME}/bb_build/ov/install}"
if [[ -f "${BB_PLATFORM_OV_PREFIX}/local_setup.bash" ]]; then
  source "${BB_PLATFORM_OV_PREFIX}/local_setup.bash"
fi
BB_PLATFORM_SOURCE="${BB_PLATFORM_WS}/src"
export PYTHONPATH="${BB_PLATFORM_SOURCE}/boom_birds_control:${BB_PLATFORM_SOURCE}/boom_birds_sensing:${BB_PLATFORM_SOURCE}/boom_birds_bringup:${BB_PLATFORM_SOURCE}/boom_birds_nav:${BB_PLATFORM_SOURCE}/boom_birds_sim:${PYTHONPATH:-}"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-197}"
