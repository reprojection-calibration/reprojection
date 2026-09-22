#!/bin/bash

set -eou pipefail

echo "Running docs-report container. Open http://localhost:8080 to view the docs."
echo "Make sure to rebuild the container each time you want to view the latest documentation."
docker run \
  --detach \
  --interactive \
  --name docs-report \
  --publish 8080:80 \
  --rm \
  --tty \
  reprojection:docs-report