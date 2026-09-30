#!/usr/bin/env bash
set -eo pipefail
BB_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$BB_PROJECT_ROOT"
source companion/ros2_ws/tools/activate_python_env.sh
source "${INSTALL_BASE:-${HOME}/bb_build/main/install}/setup.bash"
python3 -m boom_birds_nav.synthetic \
  --write-calibration /tmp/boom_birds_synth/synthetic_candidate.npz
echo "Boom_Birds TEST-ONLY 动态双目/深度/EGO/PX4 高层 setpoint 链"
exec ros2 launch boom_birds_nav px4_sitl_motion.launch.py "$@"
