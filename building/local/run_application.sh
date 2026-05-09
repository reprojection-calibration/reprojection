#!/bin/bash
set -euo pipefail

SCRIPT_FOLDER="$(dirname "$(realpath -s "$0")")"
# TODO HAVE USER PASS WHICH APP THEY WANT TO USE!
TAG=reprojection:ros2-app

DOCKER_ARGS=()
APP_ARGS=()

# TODO(Jack): There is an inconsistency that the library application will default to use the data's parent directory as
# the "workspace", here however we are mounting exactly the file paths provided by the user which means

mount_path_arg() {
  local flag="$1"
  local host_path="$2"

  local abs_path
  abs_path="$(realpath "$host_path")"

  local container_path="/data_mount${host_path}"

  DOCKER_ARGS+=(--volume "${abs_path}:${container_path}")
  APP_ARGS+=("${flag}" "${container_path}")
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --config)
      mount_path_arg "$1" "$2"
      shift 2
      ;;
    --data)
      mount_path_arg "$1" "$2"
      shift 2
      ;;
    --workspace)
      mount_path_arg "$1" "$2"
      shift 2
      ;;
    *)
      echo "Unrecognized command line argument!"
      exit 1
      ;;
  esac
done

xhost +local:docker

docker run \
  --user $(id -u):$(id -g) \
  --env DISPLAY="${DISPLAY}" \
  --name reprojection-calibration-application \
  --rm \
  --volume /dev:/dev \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume "${SCRIPT_FOLDER}/../../:/temporary" \
  "${DOCKER_ARGS[@]}" \
  "${TAG}" \
  "${APP_ARGS[@]}"