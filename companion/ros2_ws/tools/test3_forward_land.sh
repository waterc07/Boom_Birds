#!/usr/bin/env bash
# ******************************************************************************
# @file           : test3_forward_land.sh
# @brief          : 加载任务测试环境并启动任务 3 的前飞降落测试入口。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

set -euo pipefail
scriptDir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
testEnv="${BB_TAG_TEST_ENV:-$scriptDir/tag_flight_test.env.sh}"
if [[ -f "$testEnv" ]]; then
  set +u
  source "$testEnv"
  set -u
elif [[ -n "${BB_TAG_TEST_ENV:-}" ]]; then
  echo "环境文件不存在：$testEnv" >&2
  exit 2
fi
exec python3 "$scriptDir/tag_flight_test.py" "$@" --task tag_forward_land
