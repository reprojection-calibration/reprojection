#!/bin/bash

set -eou pipefail

no_cache=()
stage=development

for i in "$@"; do
  case "${i}" in
    --no-cache)
      no_cache=("--no-cache")
      shift
      ;;
    --stage)
      stage="${2}"
      shift 2
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
tag=${image}:${stage}

# TODO(Jack): This is not a long term solution - but at least it prevents us from having to edit the docker file
# everytime we want to build the ROS1 stuff.
# TODO(Jack): We have the ubunutu 24.04 hash copy and pasted here and in the docker file AND the github actions file :(
# That is not maintainable!
base_image=ubuntu:24.04@sha256:186072bba1b2f436cbb91ef2567abca677337cfc786c86e107d25b7072feef0c
if [[ "$stage" == ros1-* ]]; then
    base_image=ubuntu:20.04@sha256:8feb4d8ca5354def3d8fce243717141ce31e2c428701f6682bd2fafe15388214
fi

echo "Building image with tag '$tag' targeting stage '$stage'..."
DOCKER_BUILDKIT=1 docker build \
    "${no_cache[@]}" \
    --build-arg BASE_IMAGE="${base_image}" \
    --file "${script_folder}"/../Dockerfile \
    --tag "${tag}" \
    --target "${stage}"-stage \
    --progress=plain \
    "${script_folder}"/../../

BUILD_SUCCESSFUL=$?

if [ ${BUILD_SUCCESSFUL} -eq 0 ]; then
    echo "Build successful: ${tag}"
else
    echo "Build failed"
    exit 1
fi