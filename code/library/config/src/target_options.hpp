#pragma once

#include <optional>
#include <toml++/toml.hpp>

#include "toml_helpers.hpp"  // TODO REPLACE with types .hpp file after move

namespace reprojection::config {

std::optional<ParseError> ValidateTargetConfig(toml::table const& target_cfg);

}  // namespace reprojection::config
