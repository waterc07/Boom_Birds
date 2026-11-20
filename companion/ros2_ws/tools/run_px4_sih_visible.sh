#!/usr/bin/env bash
# ******************************************************************************
# @file           : run_px4_sih_visible.sh
# @brief          : 在当前终端启动本机 PX4 SIH 实例。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

set -euo pipefail
cd "${PX4_SOURCE:-${HOME}/PX4-Autopilot}"
echo "PX4 SIH 仿真 -i 0；此窗口只连接本机回环端口。"
exec env px4SimModel=sihsim_quadx px4Simulator=sihsim px4SysAutostart=10040 \
  "${PX4_BUILD:-build/px4_sitl_default}/bin/px4" -d -i 0
