#pragma once

#include <toml++/toml.hpp>

namespace reprojection::config {

bool ValidateTargetConfig(toml::table const& target_cfg);

}  // namespace reprojection::config
