#!/bin/bash

set -eou pipefail

usage() {
    echo "Usage: $0 -t <target-stage>"
    echo "  -t <target-stage>     : Target build stage (e.g., build, development)"
    exit 1
}

TARGET_STAGE=development

# TODO: add an option to pass the --no-cache flag 
while getopts ":t:" opt; do
  case ${opt} in
    t ) TARGET_STAGE=$OPTARG ;;
    * ) usage ;;
  esac
done

IMAGE=optimization-architecture
SCRIPT_FOLDER="$(dirname "$(realpath -s "$0")")"
TAG=${IMAGE}:${TARGET_STAGE}

echo "Building image with tag '$TAG' targeting stage '$TARGET_STAGE'..."
DOCKER_BUILDKIT=1 docker build \
    --file "${SCRIPT_FOLDER}"/../Dockerfile \
    --tag "${TAG}" \
    --target "${TARGET_STAGE}"-stage \
    --progress=plain \
    "${SCRIPT_FOLDER}"/../../

BUILD_SUCCESSFUL=$?

if [ ${BUILD_SUCCESSFUL} -eq 0 ]; then
    echo "Build successful: ${TAG}"
else
    echo "Build failed"
    exit 1
fi