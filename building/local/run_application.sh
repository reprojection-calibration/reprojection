#!/bin/bash

set -eou pipefail

SCRIPT_FOLDER="$(dirname "$(realpath -s "$0")")"
TAG=reprojection:video-file-app

# TODO(Jack): What is the best solution here for enabling xhost and running the container as --priveleged?
xhost +local:docker

echo "Running container from image '$TAG'..."
docker run \
  --env DISPLAY=:0.0 \
  --name reprojection-calibration-application \
  --privileged \
  --rm \
  --volume /dev:/dev \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume "${SCRIPT_FOLDER}"/../../:/temporary \
  ${TAG}