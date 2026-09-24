#!/usr/bin/env bash
set -eo pipefail
cd /home/waterc/workspace/Boom_Birds
source companion/ros2_ws/tools/activate_python_env.sh
source /home/waterc/bb_build/main/install/setup.bash
exec rviz2 -d /home/waterc/bb_build/main/install/boom_birds_nav/share/boom_birds_nav/config/px4_sitl_motion.rviz
