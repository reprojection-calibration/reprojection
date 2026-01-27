#pragma once

#include <toml++/toml.hpp>

namespace reprojection::config {

ceres::Solver::Options ParseFeatureExtractionOptions(toml::table target_cfg);

}  // namespace reprojection::config