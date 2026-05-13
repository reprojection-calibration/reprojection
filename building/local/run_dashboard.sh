#!/bin/bash

set -euo pipefail

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
  --name reprojection-calibration-dashboard \
  --publish 8050:8050 \
  --rm \
  --user "$(id -u):$(id -g)" \
  --volume /dev:/dev \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  "${DOCKER_ARGS[@]}" \
  reprojection:dashboard