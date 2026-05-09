#!/bin/bash

set -eou pipefail

APP_FLAVOR=${APP_FLAVOR:-unset}

# TODO(Jack): I am really not sure if this is the right thing to do, but if we do not do this then we get a warning
# message in the terminal when trying to display the feature extraction that this variable was not set and is being
# automatically set. If there was just a way to silence the warning I would prefer that, but I do not know where to
# start looking.
export XDG_RUNTIME_DIR="/tmp/runtime-ubuntu"

case "${APP_FLAVOR:-}" in
  video-file)
    exec /buildroot/reprojection-calibration-application "$@"
    ;;
  *)
    echo "Unknown APP_FLAVOR: ${APP_FLAVOR}" >&2
    exit 1
    ;;
esac