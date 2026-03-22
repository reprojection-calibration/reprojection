#pragma once

#include <ceres/solver.h>

#include <toml++/toml.hpp>

#include "types/enums.hpp"

namespace reprojection::config {

std::string ParseDataConfig(toml::table data_cfg);

// TODO(Jack): Returning just a pair here is not so nive, because there is no semantic type system enforcement of what
//  these represent. We could return a CameraInfo type from here (which we will probably do one day), but the fact that
//  we do not have the image bounds here too means I do not want to partially construct it and have it be invalid.
std::pair<std::string, CameraModel> ParseSensorConfig(toml::table solver_cfg);

ceres::Solver::Options ParseSolverConfig(toml::table solver_cfg);

}  // namespace reprojection::config