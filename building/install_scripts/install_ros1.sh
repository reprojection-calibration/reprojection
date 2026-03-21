#!/bin/bash

set -eoux pipefail

if [[ "$(lsb_release -is)" != "Ubuntu" || "$(lsb_release -rs)" != "20.04" ]]; then
    echo "Error: Requires Ubuntu 20.04"
    exit 1
fi

apt-get update
apt-get install --no-install-recommends --yes gnupg2

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list'
curl https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

apt-get update
apt-get install --yes ros-noetic-cv-bridge ros-noetic-ros-base
