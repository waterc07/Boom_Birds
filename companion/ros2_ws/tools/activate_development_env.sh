#!/usr/bin/env bash
# 加载源码工具；运行包从指定的安装前缀导入。
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "请用 source 加载此脚本" >&2
  exit 2
fi
_BB_DEV_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${_BB_DEV_WS}/tools/activate_python_env.sh" || return 1
_BB_DEV_INSTALL="${INSTALL_BASE:-${HOME}/bb_build/main/install}"
if [[ ! -f "${_BB_DEV_INSTALL}/setup.bash" ]]; then
  echo "安装前缀不存在：${_BB_DEV_INSTALL}；请设置 INSTALL_BASE" >&2
  unset _BB_DEV_WS _BB_DEV_INSTALL
  return 1
fi
source "${_BB_DEV_INSTALL}/setup.bash"
export PYTHONPATH="${_BB_DEV_WS}${PYTHONPATH:+:${PYTHONPATH}}"
unset _BB_DEV_WS _BB_DEV_INSTALL
