#!/bin/bash

set -eou pipefail

local_mount=()
target_stage=development

for i in "$@"; do
  case $i in
    --mount-local)
      script_folder="$(dirname "$(realpath -s "$0")")"
      local_mount=("${script_folder}/../../:/temporary")
      shift
      ;;
    -ts=*|--target-stage=*)
      target_stage="${i#*=}"
      shift
      ;;
    -*)
      echo "Unknown option $i"
      exit 1
      ;;
    *)
      ;;
  esac
done

image=reprojection
tag=${image}:${target_stage}

echo "Running container from image with tag '$tag'..."
docker run \
  --entrypoint="" \
  --interactive \
  --volume "${local_mount[@]}" \
  --rm \
  --tty \
  "${tag}" /bin/bash