#!/usr/bin/env bash
# 激活 ROS 2 Jazzy 与 Boom_Birds 隔离 Python 环境。
#   source companion/ros2_ws/tools/activate_python_env.sh
#
# 注意：ROS 2 的 setup.bash 与 `set -u` 不兼容，本脚本不会启用 nounset，
# 也不会改动调用方 shell 的选项（除 activate 自身带来的 PATH/环境变量）。
#
# 结果：ROS_DISTRO=jazzy，python3 = 隔离环境解释器，
#       rclpy/cv_bridge 来自系统 ROS，numpy/cv2 来自系统 dist-packages。

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "请用 source 运行：source companion/ros2_ws/tools/activate_python_env.sh" >&2
  exit 2
fi

_BB_WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_BB_VENV="${_BB_WS_ROOT}/.venv"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "错误：未找到 /opt/ros/jazzy/setup.bash" >&2
  return 1
fi
if [[ ! -x "${_BB_VENV}/bin/python" ]]; then
  echo "错误：未找到隔离环境，请先运行 bash ${_BB_WS_ROOT}/tools/setup_python_env.sh" >&2
  return 1
fi

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "${_BB_VENV}/bin/activate"

export BOOM_BIRDS_WS="${_BB_WS_ROOT}"
export BOOM_BIRDS_PYTHON="${_BB_VENV}/bin/python"
echo "[env] ROS_DISTRO=${ROS_DISTRO:-?} python=${BOOM_BIRDS_PYTHON}"
unset _BB_WS_ROOT _BB_VENV
