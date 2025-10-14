#!/bin/bash

set -eou pipefail

echo "Running code-coverage-report container. Open http://localhost:8080 to view the code coverage."
echo "Make sure to rebuild the container each time you want to view the latest code coverage results."
docker run \
  --detach \
  --interactive \
  --name code-coverage-report \
  --publish 8080:80 \
  --rm \
  --tty \
  reprojection:code-coverage-report