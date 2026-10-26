#!/usr/bin/env bash
# 脱机 ROS/UDP 回归使用临时网络命名空间；退出后自动释放，不改宿主路由。
set -eo pipefail
if [[ $# -eq 0 || "$1" == "--help" ]]; then
  echo "用法：bash tools/run_network_isolated.sh COMMAND [ARG ...]"
  echo "仅隔离网络（只有 lo），不隔离设备文件；用于脱机测试。"
  if [[ $# -eq 0 ]]; then exit 2; fi
  exit 0
fi
command -v unshare >/dev/null
command -v ip >/dev/null
exec unshare --user --map-root-user --net bash -c '
set -eo pipefail
ip link set lo up
export ROS_LOCALHOST_ONLY=1
exec "$@"
' isolated-network "$@"
