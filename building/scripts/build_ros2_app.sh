#!/bin/bash

set -eox pipefail

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

set -eoux pipefail

cd /temporary/code/ros2_ws

colcon build
colcon test
colcon test-result --verbose --all