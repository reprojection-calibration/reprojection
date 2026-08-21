#!/bin/bash

set -eoux pipefail

# System deps required for pyenv to build python - I am not 100% sure we need all of these.
apt-get update
apt-get install --no-install-recommends --yes \
    libbz2-dev \
    libffi-dev \
    liblzma-dev \
    libncursesw5-dev \
    libreadline-dev \
    libxmlsec1-dev \
    libxml2-dev \
    tk-dev

# TODO(Jack): We do not run the "rm --force --recursive /var/lib/apt/lists/*" part here because this script is
# intended to also locally run on the users system if they want to install the python venv.
