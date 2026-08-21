#!/bin/bash

set -eoux pipefail

# NOTE(Jack): Because a developer might also run this script locally where they may or may not have already have
# either of the following things installed we put these in if else blocks.

if command -v pyenv >/dev/null 2>&1; then
  echo "pyenv is already installed."
else
  curl -fsSL https://pyenv.run | bash
fi

if pyenv versions --bare | grep -q '^3\.12'; then
  echo "Python 3.12 is already installed."
else
  LDFLAGS="-L/usr/local/lib -Wl,-rpath,/usr/local/lib" \
    pyenv install 3.12
fi