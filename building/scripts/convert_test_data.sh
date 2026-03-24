#!/bin/bash

set -eoux pipefail

# NOTE(Jack): Technically when working with python we should use a venv to prevent us from interfering/breaking the
# system packages. But our use case here is so small and discrete that it is not worth the overhead. Therefore we simply
# install python and pip via apt-get. Please note that because we just do a system install we have to pass the
# '--break-system-packages' to the pip3 install command. If this causes trouble one day, or we want to be able to
# execute this locally one day, then we can refactor, but for now it gets the job done!

apt-get update
apt-get install --no-install-recommends --yes \
    python3 \
    python3-pip
rm --force --recursive /var/lib/apt/lists/*

pip3 install --break-system-packages rosbags

# TODO(Jack): Something that we need to be careful about is that we now have these test data paths and names copied all
#  over the place! It would be nice if we designed the system such that they are only specified in one place.
rosbags-convert \
    --src /data/dataset-calib-imu4_512_16.bag \
    --dst /data/dataset-calib-imu4_512_16/