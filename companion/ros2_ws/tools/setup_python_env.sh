#!/usr/bin/env bash
# Boom_Birds WSL 隔离 Python 环境（受版本管理，可重复执行）。
#
# 背景：~/.local 下的 numpy 2.5.2 会遮蔽系统 numpy 1.26.4，导致系统 OpenCV 4.6（按
# NumPy 1.x ABI 编译）与 rclpy 导入失败。本脚本在仓库内建立 venv，并把系统
# dist-packages 追加到 venv 的 site-packages 之后，使系统 numpy/OpenCV/rclpy 优先生效，
# 同时不触碰用户目录与系统 Python。
#
# 另外追加 ~/.local/lib/python3.12/site-packages（排在系统 dist-packages 之后）：
# 真实 MAVLink IMU 链路需要 pymavlink + pyserial，本机只装在用户目录；该目录自带
# numpy 2.5.2，因此**必须**排在系统 dist-packages 之后，否则会反过来遮蔽系统 numpy
# 并再次破坏 cv2/rclpy。
#
# 用法：
#   bash companion/ros2_ws/tools/setup_python_env.sh          # 创建或修复环境
#   source companion/ros2_ws/tools/activate_python_env.sh     # ROS + venv 一起激活
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${WS_ROOT}/.venv"
SYSTEM_DIST="/usr/lib/python3/dist-packages"
USER_LOCAL="${HOME}/.local/lib/python3.12/site-packages"

if [[ ! -d "${SYSTEM_DIST}" ]]; then
  echo "错误：找不到 ${SYSTEM_DIST}" >&2
  exit 1
fi

if [[ ! -x "${VENV}/bin/python" ]]; then
  echo "[setup] 创建 venv：${VENV}"
  python3 -m venv --without-pip "${VENV}"
fi

SITE="$("${VENV}/bin/python" -c 'import site; print(site.getsitepackages()[0])')"
PTH="${SITE}/zz_system_dist_packages.pth"
printf '%s\n' "${SYSTEM_DIST}" > "${PTH}"
echo "[setup] 已写入 ${PTH}"

PTH_USER="${SITE}/zz_user_local_packages.pth"
if [[ -d "${USER_LOCAL}" ]]; then
  printf '%s\n' "${USER_LOCAL}" > "${PTH_USER}"
  echo "[setup] 已写入 ${PTH_USER}（pymavlink 来源；排在系统 dist-packages 之后）"
else
  rm -f "${PTH_USER}"
  echo "[setup] 未找到 ${USER_LOCAL}：跳过（真实 MAVLink 链路需要 pymavlink）"
fi

echo "[setup] 校验导入："
"${VENV}/bin/python" - <<'PY'
import sys
import numpy, cv2
print("  python  :", sys.executable)
print("  numpy   :", numpy.__version__, numpy.__file__)
print("  cv2     :", cv2.__version__)
# 注意：本脚本不 source ROS，因此这里不校验 rclpy；完整环境用 activate_python_env.sh 校验。
try:
    from pymavlink import mavutil  # noqa: F401
    print("  pymavlink: ok")
except ImportError:
    print("  pymavlink: 缺失（真实 MAVLink 链路不可用；纯函数测试仍可运行）")
PY
echo "[setup] 完成。激活：source ${WS_ROOT}/tools/activate_python_env.sh"
