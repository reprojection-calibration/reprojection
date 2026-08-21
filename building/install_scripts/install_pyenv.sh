#!/bin/bash

set -eoux pipefail

curl -fsSL https://pyenv.run | bash

# TODO(Jack): For some reason if we do not do this then sqlite (which we kinda source install) can be found/linked
# against. I thought I had solved this with ldconfig but I guess not - this gets the job done on 20.04 and 24.04
LDFLAGS="-L/usr/local/lib -Wl,-rpath,/usr/local/lib" \
  pyenv install 3.12