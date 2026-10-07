#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/platform_env.sh"
BB_PLATFORM_CONFIG="${BB_PLATFORM_SOURCE}/boom_birds_control/config/platform_landing_test.yaml"
BB_PLATFORM_EVID="${1:?evidence directory required}"
mkdir -p "${BB_PLATFORM_EVID}"
BB_PLATFORM_FAILED=0
for scenario in origin return tag_loss range_jump handoff_failure; do
  python3 "${BB_PLATFORM_WS}/tools/run_platform_sih.py" --config "${BB_PLATFORM_CONFIG}" \
    --out "${BB_PLATFORM_EVID}/${scenario}" --scenario "${scenario}" --allow-simulated-arming \
    > "${BB_PLATFORM_EVID}/${scenario}.log" 2>&1 || BB_PLATFORM_FAILED=1
  cat "${BB_PLATFORM_EVID}/${scenario}.log"
done
exit "${BB_PLATFORM_FAILED}"
