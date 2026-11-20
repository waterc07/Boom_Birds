#!/usr/bin/env bash
# ******************************************************************************
# @file           : run_px4_sitl_rviz_visible.sh
# @brief          : 加载 PX4 运动仿真的 RViz 配置以显示导航和感知话题。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

set -eo pipefail
BB_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$BB_PROJECT_ROOT"
source companion/ros2_ws/tools/activate_python_env.sh
source "${INSTALL_BASE:-${HOME}/bb_build/main/install}/setup.bash"
exec rviz2 -d "$(ros2 pkg prefix --share boom_birds_sim)/config/px4_sitl_motion.rviz"
