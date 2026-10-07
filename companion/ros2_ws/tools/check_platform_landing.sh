#!/usr/bin/env bash
# No SIH or hardware actions. Build first; isolated ROS domain.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/platform_env.sh"
BB_PLATFORM_EVID="${1:?evidence directory required}"
mkdir -p "${BB_PLATFORM_EVID}"
cd "${BB_PLATFORM_WS}/../.."
git status --short > "${BB_PLATFORM_EVID}/git-status.txt"
git rev-parse HEAD > "${BB_PLATFORM_EVID}/git-head.txt"
cp "${BB_PLATFORM_SOURCE}/boom_birds_control/config/platform_landing_test.yaml" "${BB_PLATFORM_EVID}/config.yaml"
for package in boom_birds_control boom_birds_sensing boom_birds_bringup boom_birds_nav; do
  python3 -m pytest "${BB_PLATFORM_SOURCE}/${package}/test" -q \
    --junitxml="${BB_PLATFORM_EVID}/${package}.xml" > "${BB_PLATFORM_EVID}/${package}.log" 2>&1
  tail -4 "${BB_PLATFORM_EVID}/${package}.log"
done
git diff --check
