#!/bin/bash

set -euo pipefail

# TODO(Jack): Add some sort of usage or help dialogue that will print out if the appropriate args are not provided

SPDLOG_LEVEL=${SPDLOG_LEVEL:-info}

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
    echo "(run_application.sh) Error: '${APP_TYPE}' is not a recognized application type" >&2
    exit 1
    ;;
esac

DOCKER_ARGS=()
APP_ARGS=()

# NOTE(Jack): This function serves a very important role. The fundamental thing that we need to accomplish is to mount
# the provided date/config/workspace files/folder to the docker container and at the same time pass those paths into the
# docker for the application to consume. This function facilitates this. We keep it simple and instead of shortening the
# provided host paths or anything like that we just prepend "/data/mount" to them so they are recognizable in the
# container.
mount_path_arg() {
  local flag="${1}"
  local host_path="${2}"

  local abs_path
  abs_path="$(realpath "${host_path}")"

  local container_path="/data/mount${host_path}"

  # NOTE(Jack): We use --mount nad not --volume here because volume will actually create the path on the home disk if it
  # does no already exist. Mount on the other hand will fail if the mounted thing does not exist on host. This protects
  # us against some pretty basic but critical errors.
  DOCKER_ARGS+=(--mount "type=bind,source=${abs_path},target=${container_path}")
  APP_ARGS+=("${flag}" "${container_path}")
}

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

# TODO(Jack): Is this safe or a bad practice?
xhost +local:docker

# TODO(Jack): Should we also use bind mounts for /dev and /tmp/.X11-unix like we do above for the mounted app args?
docker run \
  --env DISPLAY="${DISPLAY}" \
  --env SPDLOG_LEVEL="${SPDLOG_LEVEL}" \
  --name reprojection-calibration-application \
  --rm \
  --user "$(id -u):$(id -g)" \
  --volume /dev:/dev:ro \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
  "${DOCKER_ARGS[@]}" \
  "${TAG}" \
  "${APP_ARGS[@]}"