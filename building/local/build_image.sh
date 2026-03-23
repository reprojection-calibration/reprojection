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

# TODO(Jack): This is not a long term solution - but at least it prevents us from having to edit the docker file
# everytime we want to build the ROS1 stuff.
base_image=ubuntu:24.04
if [[ "$target_stage" == ros1-* ]]; then
    base_image=ubuntu:20.04
fi

echo "Building image with tag '$tag' targeting stage '$target_stage'..."
DOCKER_BUILDKIT=1 docker build \
    --build-arg BASE_IMAGE="${base_image}" \
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