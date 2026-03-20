#!/bin/bash

set -eox pipefail

source /opt/ros/jazzy/setup.bash

set -eoux pipefail

cd /temporary/code/ros2_ws
colcon build

colcon test
colcon test-result --verbose --all