#!/bin/bash

set -eou pipefail

SCRIPT_FOLDER="$(dirname "$(realpath -s "$0")")"
TAG=reprojection:feature-extraction-demo

echo "Running container from image '$TAG'..."
xhost +
docker run \
  --env DISPLAY=:0.0 \
  --name feature_extraction_webcam_demo \
  --privileged \
  --rm \
  --volume /dev:/dev \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume "${SCRIPT_FOLDER}"/../../:/temporary \
  ${TAG}