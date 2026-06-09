#pragma once

#include <ceres/solver.h>

#include <toml++/toml.hpp>

#include "types/calibration_types.hpp"
#include "types/enums.hpp"

namespace reprojection::config {

std::pair<std::string, CameraModel> ParseCameraConfig(toml::table camera_cfg);

// NOTE(Jack): The imu config is not required which is why this returns an optional.
std::optional<std::string> ParseImuConfig(toml::table imu_cfg);

TargetInfo ParseTargetConfig(toml::table target_cfg);

// NOTE(Jack): This is not required just like the imu config is also not required to be in the config file, but this is
// not optional because there are sensible known defaults for all values. But honestly as of time of writing
// (09.06.2026) this function is not even used...
ceres::Solver::Options ParseSolverConfig(toml::table solver_cfg);

}  // namespace reprojection::config