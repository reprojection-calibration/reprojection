#pragma once

#include <toml++/toml.hpp>
#include <ceres/solver.h>

namespace reprojection::config {

ceres::Solver::Options ParseCeresSolverOptions(toml::table solver_cfg);

}  // namespace reprojection::config