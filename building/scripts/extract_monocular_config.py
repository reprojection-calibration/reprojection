import argparse
import tomllib

import tomli_w

# NOTE(Jack): We need this script for now because our video-file-app only supports monocular camera calibration
# (01.09.2026). Therefore, we process the default config to remove the second camera and imu. This lets us keep the
# of the CI pipeline consistent across all the applications.

parser = argparse.ArgumentParser()
parser.add_argument("input")
parser.add_argument("output")
args = parser.parse_args()

with open(args.input, "rb") as file:
    config = tomllib.load(file)

mono_config = {
    "application": config["application"],
    "cam0": config["cam0"],
    "target": config["target"],
}

with open(args.output, "wb") as file:
    tomli_w.dump(mono_config, file)
