#pragma once

#include <ceres/solver.h>

#include <string>

namespace reprojection::config {

// NOTE(Jack): As we add more configs we will probably return a tuple here or a struct with all the possible configs.
// TODO(Jack): Should we use filesystem paths instead of strings? I think so...
ceres::Solver::Options LoadConfiguration(std::string const& file);

}  // namespace reprojection::config