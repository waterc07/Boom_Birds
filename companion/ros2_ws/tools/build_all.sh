#!/usr/bin/env bash
# 统一构建入口：加锁串行 + 严格限制并发与内存。
#
# 背景（2026-09-22 实测）：colcon 在配置 CMake 时会按 CPU 核数写入并行度，
# 实际调用 `cmake --build ... -- -j24 -l24`，于是"--parallel-workers 1"只限制了包数，
# 包内仍是 24 个 C++ 编译进程。本机 24 核 / 7.6 GB 内存，ov_msckf 阶段因此三次
# 把 WSL 发行版拖到无响应（Wsl/Service/0x8007274c、E_UNEXPECTED），只能 wsl --shutdown。
#
# 因此本脚本：
#   1) 用 flock 保证同一时间只有一个构建；
#   2) 用 `--executor sequential` + `--parallel-workers 1` 逐包构建；
#   3) 导出 CMAKE_BUILD_PARALLEL_LEVEL（CMake >= 3.12 会据此设置 `--build --parallel`），
#      即使配置缓存里写着 -j24 也会被环境变量覆盖；
#   4) 用 ulimit -v 限制单进程地址空间，让内存炸弹以明确报错失败而不是拖死整机。
#
# 用法：
#   bash companion/ros2_ws/tools/build_all.sh [colcon build 参数...]
#   例：bash tools/build_all.sh --packages-select stereo_depth boom_birds_nav
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_BASE="${BUILD_BASE:-${HOME}/bb_build/main/build}"
INSTALL_BASE="${INSTALL_BASE:-${HOME}/bb_build/main/install}"
LOG_BASE="${LOG_BASE:-${HOME}/bb_build/main/log}"
LOCK="${BUILD_LOCK:-${HOME}/bb_build/build.lock}"
LOCK_WAIT="${BUILD_LOCK_WAIT:-1800}"

mkdir -p "$(dirname "${LOCK}")"
cd "${WS_ROOT}"
# ROS 的 setup.bash 与 set -u 不兼容（会因 AMENT_TRACE_SETUP_FILES 未定义而退出），
# 因此 source 期间临时关闭 nounset，之后再恢复。
_bb_saved_opts="$(set +o)"
set +u
# shellcheck disable=SC1091
source "${WS_ROOT}/tools/activate_python_env.sh"
eval "${_bb_saved_opts}"
unset _bb_saved_opts
set -euo pipefail

# 关键：限制包内编译并发（覆盖 CMake 配置阶段缓存的 -j24）。
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
export MAKEFLAGS="${MAKEFLAGS:--j${CMAKE_BUILD_PARALLEL_LEVEL}}"
# 限制单个编译/链接进程的地址空间，降低 OOM 拖垮整机（含 WSL VM）的风险。
ulimit -v "${BB_VMEM_KB:-4194304}" 2>/dev/null || true

exec flock -w "${LOCK_WAIT}" "${LOCK}" colcon \
  --log-base "${LOG_BASE}" build \
  --build-base "${BUILD_BASE}" --install-base "${INSTALL_BASE}" \
  --executor sequential --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE="-O2 -g0" \
  -DCMAKE_JOB_POOLS="compile=2" -DCMAKE_JOB_POOL_COMPILE=compile \
  "$@"
