#!/bin/bash

set -eoux pipefail

pip3 install rosbags

# TODO(Jack): Something that we need to be careful about is that we now have these test data paths and names copied all
#  over the place! It would be nice if we designed the system such that they are only specified in one place.
rosbags-convert \
    --src /data/dataset-calib-imu4_512_16.bag \
    --dst /data/dataset-calib-imu4_512_16/