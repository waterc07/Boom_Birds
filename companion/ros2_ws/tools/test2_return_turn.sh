#!/usr/bin/env bash
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
exec python3 "$scriptDir/tag_flight_test.py" "$@" --task tag_return_turn
