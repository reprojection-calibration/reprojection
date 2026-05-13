#!/bin/bash

set -euo pipefail

# TODO(Jack): Add some sort of usage or help dialogue that will print out if the appropriate args are not provided

APP_TYPE="${1}"
shift

case "${APP_TYPE}" in
  ros1)
    TAG="reprojection:ros1-app"
    ;;
  ros2)
    TAG="reprojection:ros2-app"
    ;;
  video-file)
    TAG="reprojection:video-file-app"
    ;;
  *)
    echo "Unknown application type: ${APP_TYPE}"
    exit 1
    ;;
esac

DOCKER_ARGS=()
APP_ARGS=()

# NOTE(Jack): This function serves a very important role. The fundamental thing that we need to accomplish is to mount
# the provided date/config/workspace files/folder to the docker container and at the same time pass those paths into the
# docker for the application to consume. This function facilitates this. We keep it simple and instead of shortening the
# provided host paths or anything like that we just prepend "/data_mount" to them so they are recognizable in the
# container.
mount_path_arg() {
  local flag="${1}"
  local host_path="${2}"

  local abs_path
  abs_path="$(realpath "${host_path}")"

  local container_path="/data_mount${host_path}"

  DOCKER_ARGS+=(--volume "${abs_path}:${container_path}")
  APP_ARGS+=("${flag}" "${container_path}")
}

# TODO(Jack): There is an inconsistency that the library application will default to use the data's parent directory as
# the "workspace", here however we are mounting exactly the file paths provided by the user which means that we MUST
# specify the workspace, it is not optional. This in some sense contradicts my original intent when designing the
# library but at the same time is a use case foreseen in the design.
# TODO(Jack): The workspace arg is basically required (maybe not for ros2 cause the folder is passed there?), therefore
# I think it makes sense that we check that here to prevent the user from dealing with an error from the application
# that a database file cannot be opened.

while [[ $# -gt 0 ]]; do
  case "${1}" in
    --config)
      mount_path_arg "${1}" "${2}"
      shift 2
      ;;
    --data)
      mount_path_arg "${1}" "${2}"
      shift 2
      ;;
    --workspace)
      mount_path_arg "${1}" "${2}"
      shift 2
      ;;
    *)
      echo "Unrecognized command line argument!"
      exit 1
      ;;
  esac
done

script_folder="$(dirname "$(realpath -s "${0}")")"
# TODO(Jack): Is this safe or a bad practice?
xhost +local:docker

docker run \
  --env DISPLAY="${DISPLAY}" \
  --name reprojection-calibration-application \
  --rm \
  --user "$(id -u):$(id -g)" \
  --volume /dev:/dev \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  "${DOCKER_ARGS[@]}" \
  "${TAG}" \
  "${APP_ARGS[@]}"