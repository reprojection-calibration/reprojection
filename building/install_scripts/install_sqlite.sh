#!/bin/bash

set -eoux pipefail

# TODO MOVE TO MAIN INSTALL FILE
apt-get update
apt-get install --no-install-recommends --yes \
     libsqlite3-dev

rm --force --recursive /var/lib/apt/lists/*