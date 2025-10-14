#!/bin/bash

set -eou pipefail

no_cache=()
target_stage=development

for i in "$@"; do
  case $i in
    --no-cache)
      no_cache=("--no-cache")
      shift
      ;;
    -ts=*|--target-stage=*)
      target_stage="${i#*=}"
      shift
      ;;
    -*)
      echo "Unknown option $i"
      exit 1;
      ;;
    *)
      ;;
  esac
done

image=reprojection
script_folder="$(dirname "$(realpath -s "$0")")"
tag=${image}:${target_stage}

echo "Building image with tag '$tag' targeting stage '$target_stage'..."
DOCKER_BUILDKIT=1 docker build \
    --file "${script_folder}"/../Dockerfile \
    "${no_cache[@]}" \
    --tag "${tag}" \
    --target "${target_stage}"-stage \
    --progress=plain \
    "${script_folder}"/../../

BUILD_SUCCESSFUL=$?

if [ ${BUILD_SUCCESSFUL} -eq 0 ]; then
    echo "Build successful: ${tag}"
else
    echo "Build failed"
    exit 1
fi