#!/usr/bin/env bash
# 三项独立 Tag 任务；不构建 EGO、地图或仿真工具。
set -euo pipefail
wsRoot="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export BUILD_BASE="${BUILD_BASE:-${HOME}/bb_build/tag-basic/build}"
export INSTALL_BASE="${INSTALL_BASE:-${HOME}/bb_build/tag-basic/install}"
export LOG_BASE="${LOG_BASE:-${HOME}/bb_build/tag-basic/log}"
if [[ $# -ne 0 ]]; then
  echo "用法：BUILD_BASE=... INSTALL_BASE=... LOG_BASE=... bash tools/build_tag_tasks.sh" >&2
  exit 2
fi
for package in boom_birds_sim boom_birds_nav ego_planner ov_eval; do
  if [[ -d "${INSTALL_BASE}/${package}" ||
        -e "${INSTALL_BASE}/share/ament_index/resource_index/packages/${package}" ]]; then
    echo "安装目录已有非基础任务包 ${package}，请指定独立 INSTALL_BASE" >&2
    exit 2
  fi
done
exec bash "${wsRoot}/tools/build_all.sh" \
  --packages-up-to boom_birds_bringup ov_msckf \
  --cmake-args -DBUILD_TESTING=OFF -DBB_BUILD_OPENVINS_TOOLS=OFF
