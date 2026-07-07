#!/bin/bash

set -eou pipefail

APP_FLAVOR=${APP_FLAVOR:-unset}
# TODO(Jack): I am really not sure if this is the right thing to do, but if we do not do this then we get a warning
# message in the terminal when trying to display the feature extraction that this variable was not set and is being
# automatically set. If there was just a way to silence the warning I would prefer that, but I do not know where to
# start looking.
export XDG_RUNTIME_DIR="/tmp/runtime-ubuntu"

config=""
data=""
workspace=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --config)
          config="${2}"
          shift 2
          ;;
        --data)
          data="${2}"
          shift 2
          ;;
        --workspace)
          workspace="${2}"
          shift 2
          ;;
        *)
          echo "Error: unexpected argument '$1'." >&2
          exit 1
          ;;
    esac
done

[[ -n "${config}" ]] || { echo "Error: missing --config flag." >&2; exit 1; }
[[ -n "${data}" ]] || { echo "Error: missing --data flag." >&2; exit 1; }
[[ -n "${workspace}" ]] || { echo "Error: missing --workspace flag." >&2; exit 1; }

[[ -f "${config}" ]] || { echo "Error: config does not exist: ${config}" >&2; exit 1; }
[[ -e "${data}" ]] || { echo "Error: data does not exist: ${data}" >&2; exit 1; }
[[ -d "${workspace}" ]] || { echo "Error: workspace does not exist: ${workspace}" >&2; exit 1; }

app_args=(--config "${config}" --data "${data}" --workspace "${workspace}")

# -- run the calibration -- #
case "${APP_FLAVOR}" in
  ros1)
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
    set -u
    /buildroot/reprojection-calibration-application "${app_args[@]}"
    ;;
  ros2)
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    set -u
    /buildroot/reprojection-calibration-application "${app_args[@]}"
    ;;
  video-file)
    /buildroot/reprojection-calibration-application "${app_args[@]}"
    ;;
  *)
    echo "Unknown APP_FLAVOR: ${APP_FLAVOR}" >&2
    exit 1
    ;;
esac

# -- run the report generation -- #
# TODO(Jack): Do we need to specify full path for the python interpreter?
# TODO(Jack): Do we need to specify the PYTHONPATH here? Why can we not just activate the venv or something like that,
# is that package not installed there?
PYTHONPATH=/temporary/code/python_tooling \
/buildroot/.reprojection_venv/bin/python \
  /temporary/code/python_tooling/report/run.py \
  --workspace "${workspace}"
