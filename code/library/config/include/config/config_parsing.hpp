#pragma once

#include <ceres/solver.h>

#include <toml++/toml.hpp>

#include "types/enums.hpp"

namespace reprojection::config {

// TODO(Jack): There is a problem... That is, that we basically specify the same information twice, once in the config
//  validation and once in the config parsing like what the types, key names, and whats required and optional. This will
//  make our life very hard if the config gets more complicated! We need a long term strategy here where the information
//  just needs to be specified in one place.

// TODO(Jack): Returning just a pair here is not so nice, because there is no semantic type system enforcement of what
//  these represent. We could return a CameraInfo type from here (which we will probably do one day), but the fact that
//  we do not have the image bounds here too means I do not want to partially construct it and have it be invalid.
std::pair<std::string, CameraModel> ParseSensorConfig(toml::table solver_cfg);

ceres::Solver::Options ParseSolverConfig(toml::table solver_cfg);

}  // namespace reprojection::config