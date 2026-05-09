#!/bin/bash

set -eoux pipefail

APP_FLAVOR=${APP_FLAVOR:-unset}

case "${APP_FLAVOR:-}" in
  video-file)
    exec /buildroot/reprojection-calibration-application "$@"
    ;;
  *)
    echo "Unknown APP_FLAVOR: ${APP_FLAVOR}" >&2
    exit 1
    ;;
esac