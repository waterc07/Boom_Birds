#!/usr/bin/env bash
set -eo pipefail
BB_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$BB_PROJECT_ROOT"
source companion/ros2_ws/tools/activate_python_env.sh
source "${INSTALL_BASE:-${HOME}/bb_build/main/install}/setup.bash"
exec rviz2 -d "$(ros2 pkg prefix --share boom_birds_sim)/config/px4_sitl_motion.rviz"
