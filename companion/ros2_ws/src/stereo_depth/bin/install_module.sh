#!/usr/bin/env bash
# 由 CMakeLists.txt 安装 stereo_depth 运行库。
#
# stereo_depth 是算法库包，没有节点可执行文件；但 ROS 2 的包资源查询会检查
# <prefix>/lib/stereo_depth，因此显式创建空目录，避免 ros2 工具报路径不存在。
# 参数：$1=python 解释器 $2=源码目录 $3=安装前缀 $4=构建目录
set -euo pipefail
PY="${1:?python}"; SRC="${2:?source}"; PREFIX="${3:?prefix}"
BUILD_DIR="${4:?build directory}"
cd "${SRC}"
BOOM_BIRDS_COLCON_BUILD=1 "${PY}" setup.py egg_info --egg-base="${BUILD_DIR}" build --build-base="${BUILD_DIR}/python" install \
  --prefix="${PREFIX}" --single-version-externally-managed --record="${BUILD_DIR}/install_manifest.txt"
mkdir -p "${PREFIX}/lib/stereo_depth"
