#!/bin/bash

set -eou pipefail

DOCKER_ARGS=()

mount_path_arg() {
  local host_path="${1}"

  local abs_path
  abs_path="$(realpath "${host_path}")"

  local container_path="/workspace"

  DOCKER_ARGS+=(--volume "${abs_path}:${container_path}")
}

mount_path_arg "${1}"

xhost +local:docker

docker run \
  --env DISPLAY="${DISPLAY}" \
  --network=host \
  --name reprojection-calibration-dashboard \
  --rm \
  --user "$(id -u):$(id -g)" \
  --volume /dev:/dev \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
  "${DOCKER_ARGS[@]}" \
  reprojection:dashboard