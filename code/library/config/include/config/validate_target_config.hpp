#pragma once

#include <optional>

#include "config/types.hpp"

#include <toml++/toml.hpp>

namespace reprojection::config {

// NOTE(Jack): This is in the public header section because feature_extraction::CreateTargetExtractor() consumes the
// toml table directly
std::optional<ParserErrorMsg> ValidateTargetConfig(toml::table const& target_cfg);

}  // namespace reprojection::config
