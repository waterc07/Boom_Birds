#!/usr/bin/env bash
set -euo pipefail
cd /home/waterc/PX4-Autopilot
echo "PX4 SIH 仿真 -i 0；此窗口只连接本机回环端口。"
exec env PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim PX4_SYS_AUTOSTART=10040 \
  build/px4_sitl_default/bin/px4 -d -i 0
