#!/bin/bash

set -eoux pipefail

# NOTE(Jack): Because a developer might also run this script locally where they may or may not have already have
# either of the following things installed we put these in if else blocks.

if ! command -v pyenv >/dev/null 2>&1; then
    wget -qO- https://pyenv.run | bash
fi

if ! pyenv versions --bare | grep -q '^3\.12'; then
    # NOTE(Jack): This LDFLAGS env var handles some sqlite linking error we get on ubuntu 20.04. Must have something
    # to do with us source installing sqlite.
    LDFLAGS="-L/usr/local/lib -Wl,-rpath,/usr/local/lib" \
        pyenv install 3.12
fi