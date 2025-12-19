#!/bin/bash

set -eoux pipefail

wget -qO- https://pyenv.run | bash

touch ~/.bash_profile
cat <<'EOF' | tee -a ~/.bash_profile > /dev/null
export PYENV_ROOT="$HOME/.pyenv"
[[ -d $PYENV_ROOT/bin ]] && export PATH="$PYENV_ROOT/bin:$PATH"
eval "$(pyenv init - bash)"
EOF