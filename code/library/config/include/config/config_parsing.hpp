#pragma once

#include <ceres/solver.h>

#include <toml++/toml.hpp>

#include "types/calibration_types.hpp"
#include "types/enums.hpp"

namespace reprojection::config {

std::pair<std::string, CameraModel> ParseSensorConfig(toml::table sensor_ccfg);

TargetInfo ParseTargetConfig(toml::table target_cfg);

ceres::Solver::Options ParseSolverConfig(toml::table solver_cfg);

}  // namespace reprojection::config