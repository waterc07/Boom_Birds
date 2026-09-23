#!/usr/bin/env bash
# 由 CMakeLists.txt 在安装阶段调用：把 depth_preview 模块装到 site-packages。
#
# stereo_depth 是算法库包，没有节点可执行文件；但 ROS 2 的包资源查询会检查
# <prefix>/lib/stereo_depth，因此显式创建空目录，避免 ros2 工具报路径不存在。
# 参数：$1=python 解释器 $2=源码目录 $3=安装前缀
set -euo pipefail
PY="${1:?python}"; SRC="${2:?source}"; PREFIX="${3:?prefix}"
cd "${SRC}"
BOOM_BIRDS_COLCON_BUILD=1 "${PY}" setup.py install \
  --prefix="${PREFIX}" --single-version-externally-managed --record=install_manifest.txt
mkdir -p "${PREFIX}/lib/stereo_depth"
