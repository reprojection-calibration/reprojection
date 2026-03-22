#pragma once

#include <ceres/solver.h>

#include <toml++/toml.hpp>

#include "types/enums.hpp"

namespace reprojection::config {

std::pair<std::string, CameraModel> ParseSensorConfig(toml::table solver_cfg);

ceres::Solver::Options ParseSolverConfig(toml::table solver_cfg);

}  // namespace reprojection::config