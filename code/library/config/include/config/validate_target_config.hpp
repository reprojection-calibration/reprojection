#pragma once

#include <optional>
#include <toml++/toml.hpp>

#include "config/types.hpp"

namespace reprojection::config {

std::optional<ParserErrorMsg> ValidateTargetConfig(toml::table const& target_cfg);

}  // namespace reprojection::config
