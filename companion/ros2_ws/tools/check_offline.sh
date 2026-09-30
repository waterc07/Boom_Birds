#!/usr/bin/env bash
# 不启动 SIH、不连接设备；构建后执行，报告留在 Git 忽略的 log 目录。
set -eo pipefail
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${WS_ROOT}/tools/activate_python_env.sh"
# 检查期间不允许有 SIH/PX4 任务在跑：同一套 ROS 话题与域上，SIH 链路的合成
# 深度/位姿发布器会注进被测话题，产出"看起来是失败、其实是被污染"的报告
# （实测：navigation 组收到 30 Hz 深度流，159 条输出 vs 测试自发的 5 张图，
#  两个用例因此假失败）。宁可拒绝运行，也不要留下一份污染的失败证据。
BUSY="$(pgrep -af "px4|run_sih_mission|sih_record|traj_server" || true)"
if [[ -n "${BUSY}" && "${BB_ALLOW_BUSY_CHECK:-0}" != "1" ]]; then
  echo "错误：检测到 SIH/PX4 相关进程，脱机检查会被污染，已拒绝运行：" >&2
  echo "${BUSY}" >&2
  echo "      确认这些进程是历史遗留、或确实要并发时，可设 BB_ALLOW_BUSY_CHECK=1 强制运行。" >&2
  exit 3
fi

INSTALL_PREFIX="${INSTALL_BASE:-${HOME}/bb_build/main/install}"
# 安装前缀必须真的是本次构建的那个：默认值 ~/bb_build/main 与开发机常用的
# 覆盖前缀（architecture 等）不是同一个目录，忘了覆盖就会去陈旧安装树里跑检查，
# 表现成一整片 "Package not found" 假失败（实测）。这里直接停下并说清怎么修，
# 而不是 source 一个不存在的文件后继续跑。
if [[ ! -f "${INSTALL_PREFIX}/setup.bash" ]]; then
  echo "错误：安装前缀里没有 setup.bash：${INSTALL_PREFIX}" >&2
  echo "      请把 INSTALL_BASE 指向本次构建的 install 前缀（与 build_all.sh 用的一致）。" >&2
  exit 2
fi
source "${INSTALL_PREFIX}/setup.bash"
export OV_INSTALL="${OV_INSTALL:-${HOME}/bb_build/ov/install}"
if [[ -f "${OV_INSTALL}/local_setup.bash" ]]; then
  source "${OV_INSTALL}/local_setup.bash"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-194}"
export ROS_LOCALHOST_ONLY=1
exec python3 "${WS_ROOT}/tools/check_offline.py" "$@"
