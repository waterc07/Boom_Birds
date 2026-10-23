#!/usr/bin/env bash
# 独立运行安装树：不构建 sim/nav 或测试目标，不复用开发安装前缀。
set -euo pipefail
wsRoot="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export BUILD_BASE="${BUILD_BASE:-${HOME}/bb_build/runtime/build}"
export INSTALL_BASE="${INSTALL_BASE:-${HOME}/bb_build/runtime/install}"
export LOG_BASE="${LOG_BASE:-${HOME}/bb_build/runtime/log}"
if [[ $# -ne 0 ]]; then
  echo "用法：BUILD_BASE=... INSTALL_BASE=... LOG_BASE=... bash tools/build_runtime.sh" >&2
  exit 2
fi
# 已有开发包的安装前缀不当作运行前缀，避免残留仿真节点进入部署包。
for package in boom_birds_sim boom_birds_nav ov_eval map_generator mockamap; do
  if [[ -e "${INSTALL_BASE}/share/ament_index/resource_index/packages/${package}" ||
        -d "${INSTALL_BASE}/${package}" ]]; then
    echo "运行安装前缀包含开发包 ${package}，请指定独立 INSTALL_BASE：${INSTALL_BASE}" >&2
    exit 2
  fi
done
exec bash "${wsRoot}/tools/build_all.sh" \
  --packages-up-to boom_birds_bringup ego_planner ov_msckf \
  --cmake-args -DBUILD_TESTING=OFF -DBB_BUILD_GRIDMAP_TESTS=OFF -DBB_BUILD_GAZEBO_MOTION=OFF -DBB_BUILD_OPENVINS_TOOLS=OFF -DBB_BUILD_EGO_TOOLS=OFF
