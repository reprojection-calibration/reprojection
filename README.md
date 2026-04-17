## Get the Repo

Get the code:
    
    git clone git@github.com:reprojection-calibration/reprojection.git

Make sure to also pull the git lfs objects:

    cd reprojection
    git lfs pull

TODO(Jack): Add instruction for any symbolic links that need to be made (ex. the sql folder to python tooling).

### Python Environment

WARN: The following commands will install packages to your system, create a virtual environment in your home directory, 
and symlink a directory. If you are not comfortable doing these things locally then use docker!

NOTE: If the following install setup instructions do not work please see the python-tooling-stage in the Dockerfile. 
That is run in CI and should be up to date.

Install pyenv:

    ./building/install_scripts/install_pyenv.sh

Install the venv system dependencies (run with sudo as needed):

    ./building/install_scripts/apt_install_python_venv_deps.sh

For the next step you will need a c-compiler, I recommend installing the `build-essential` package if you do not already 
satisfy the requirement (run with sudo as needed):

    apt update
    apt install build-essential

Build the actual virtual environment to work in:

    ./building/scripts/build_python_venv.sh

Test and install the python tooling package 

    # We need to symlink the sql definition files into the package - there has to be a better solution for this...
    ln -s ${PWD}/code/sql/ code/python_tooling/database/sql

    # Activate the venv
    source ~/.reprojection_venv/bin/activate

    ./building/scripts/build_python_tooling.sh

    