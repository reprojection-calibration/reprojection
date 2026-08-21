#!/bin/bash

set -eoux pipefail

# NOTE(Jack): This needs to be a separate script because we need to write the .bash_profile and then source that.

# WARN(Jack): This logic here essentially is hardcoding the assumption that the user has installed pyenv on their system
# using this script. If a user has already installed pyenv and added it to something else than the .bash_profile then
# they will need to manually source that. But for our workflow it is internally consistent and works.

# shellcheck disable=SC1090
[[ -f ~/.bash_profile ]] && source ~/.bash_profile
if command -v pyenv >/dev/null 2>&1; then
    echo "Early exit - pyenv is already installed."
    exit 0
fi

PYENV_ROOT=${PYENV_ROOT:-"${HOME}/.pyenv"}

wget -qO- https://pyenv.run | bash

# TODO(Jack): We create the .bash_profile because when we tried to use the .bashrc the other variables/script in the
# .bashrc caused problems. This might cause problems for users that already have pyenv locally installed and want to run
# this script.
touch ~/.bash_profile
cat <<'EOF' | tee -a ~/.bash_profile > /dev/null

[[ -d ${PYENV_ROOT}/bin ]] && export PATH="${PYENV_ROOT}/bin:${PATH}"
eval "$(pyenv init - bash)"
EOF