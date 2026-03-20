#!/bin/bash

set -eoux pipefail

if [[ "$(lsb_release -is)" != "Ubuntu" || "$(lsb_release -rs)" != "24.04" ]]; then
    echo "Error: Requires Ubuntu 24.04"
    exit 1
fi

apt-get update
apt-get install --no-install-recommends --yes gnupg2

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2.list

apt-get update
apt-get install --yes ros-jazzy-ros-base
