#!/bin/bash

set -eoux pipefail

# NOTE(Jack): This needs to be a separate script because we need to write the script to the .bash_profile and then
# source that. Doing that in one single script

wget -qO- https://pyenv.run | bash

# NOTE(Jack): We create the .bashprofile because when we tried to use the .bashrc the other variables/script in the
# .bashrc caused problems.
touch ~/.bash_profile

cat <<'EOF' | tee -a ~/.bash_profile > /dev/null
export PYENV_ROOT="$HOME/.pyenv"
[[ -d $PYENV_ROOT/bin ]] && export PATH="$PYENV_ROOT/bin:$PATH"
eval "$(pyenv init - bash)"
EOF