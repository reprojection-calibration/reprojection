#!/bin/bash

set -eoux pipefail

# https://github.com/linux-test-project/lcov/issues/129

apt-get update
apt-get install --no-install-recommends --yes \
 bc

coverage_percent=$(lcov --summary /buildroot/coverage.filtered.info | grep "lines" | awk '{ print $2 }' | sed 's/%//')
if (( $(echo "${coverage_percent} < 100" | bc -l) )); then
  echo "Coverage only ${coverage_percent}, please inspect report to get 100% coverage."
  exit 1
else
  echo "Full coverage achieved."
fi
