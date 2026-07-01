#!/bin/bash

set -eou pipefail

APP_FLAVOR=${APP_FLAVOR:-unset}
# TODO(Jack): I am really not sure if this is the right thing to do, but if we do not do this then we get a warning
# message in the terminal when trying to display the feature extraction that this variable was not set and is being
# automatically set. If there was just a way to silence the warning I would prefer that, but I do not know where to
# start looking.
export XDG_RUNTIME_DIR="/tmp/runtime-ubuntu"

processed_args=()
data=""
workspace=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --config)
            processed_args+=("--config" "$2")
            shift 2
            ;;
        --workspace)
            workspace="$2"
            shift 2
            ;;
        --data)
            data="$2"
            processed_args+=("--data" "$data")
            shift 2
            ;;
        *)
            echo "(entrypoint.sh) Error: unexpected command line argument '${1}'." >&2
            exit 1
    esac
done

# WARN(Jack): The following logic is what makes the '--workspace' flag optional. If a user provides the workspace
# directly then we use that. If the workspace is not provided then we use the '--data' flag to derive the workspace
# directory. If '--data' is itself a directory then we use it directly and if its a file then we get the directory the
# file is in and use that.
if [[ -n "$workspace" ]]; then
    output_dir="$workspace"
elif [[ -n "$data" ]]; then
    if [[ -d "$data" ]]; then
        output_dir="$data"
    elif [[ -f "$data" ]]; then
        output_dir="$(dirname "$data")"
    else
        echo "(entrypoint.sh) Error: --data path '${data}' is neither an existing file nor an existing directory." >&2
        exit 1
    fi
else
    echo "(entrypoint.sh) Error: either --workspace or --data must be provided." >&2
    exit 1
fi


processed_args+=("--workspace" "$output_dir")

# -- run the calibration -- #
case "${APP_FLAVOR}" in
  ros1)
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
    set -u
    /buildroot/reprojection-calibration-application "${processed_args[@]}"
    ;;
  ros2)
    set +u
    # shellcheck disable=SC1091z
    source /opt/ros/jazzy/setup.bash
    set -u
    /buildroot/reprojection-calibration-application "${processed_args[@]}"
    ;;
  video-file)
    /buildroot/reprojection-calibration-application "${processed_args[@]}"
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
  --workspace "${output_dir}"
