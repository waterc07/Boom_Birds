#!/usr/bin/env bash
# 加载源码工具；运行包从指定的安装前缀导入。
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "请用 source 加载此脚本" >&2
  exit 2
fi
_bbDevWs="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${_bbDevWs}/tools/activate_python_env.sh" || return 1
_bbDevInstall="${INSTALL_BASE:-${HOME}/bb_build/main/install}"
if [[ ! -f "${_bbDevInstall}/setup.bash" ]]; then
  echo "安装前缀不存在：${_bbDevInstall}；请设置 INSTALL_BASE" >&2
  unset _bbDevWs _bbDevInstall
  return 1
fi
source "${_bbDevInstall}/setup.bash"
export PYTHONPATH="${_bbDevWs}${PYTHONPATH:+:${PYTHONPATH}}"
unset _bbDevWs _bbDevInstall
