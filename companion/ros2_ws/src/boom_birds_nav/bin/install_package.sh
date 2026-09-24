#!/usr/bin/env bash
# 由 CMakeLists.txt 在安装阶段调用：安装 boom_birds_nav Python 包并布置 libexec。
#
# ROS 2 的 ros2 run/launch 约定：可执行文件必须位于 <prefix>/lib/<pkg>/。
# 本仓库用 setup.py install 安装（不走 ament_python 的 build_py），因此安装后
# 需要把 console_scripts 从 <prefix>/bin 移到 <prefix>/lib/boom_birds_nav。
# 参数：$1=python 解释器 $2=源码目录 $3=安装前缀
set -euo pipefail
PY="${1:?python}"; SRC="${2:?source}"; PREFIX="${3:?prefix}"
PKG=boom_birds_nav

cd "${SRC}"
"${PY}" setup.py install \
  --prefix="${PREFIX}" --single-version-externally-managed --record=install_manifest.txt

LIBEXEC="${PREFIX}/lib/${PKG}"
mkdir -p "${LIBEXEC}"
for name in stereo_source depth_node pose_adapter vio_source mavlink_imu_node camera_timestamp_probe px4_interface_node sitl_truth_source; do
  if [[ -f "${PREFIX}/bin/${name}" ]]; then
    mv -f "${PREFIX}/bin/${name}" "${LIBEXEC}/${name}"
    chmod +x "${LIBEXEC}/${name}"
  fi
done
echo "[boom_birds_nav] libexec: $(ls "${LIBEXEC}" | tr '\n' ' ')"
