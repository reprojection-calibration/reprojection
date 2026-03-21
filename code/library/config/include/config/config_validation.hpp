#pragma once

#include <optional>

#include <toml++/toml.hpp>

#include "config/types.hpp"

namespace reprojection::config {

std::optional<ParserErrorMsg> ValidateCalibrationConfig(toml::table const& cfg);

// NOTE(Jack): This is in the public header section because feature_extraction::CreateTargetExtractor() consumes the
// toml table directly
std::optional<ParserErrorMsg> ValidateTargetConfig(toml::table const& target_cfg);

}  // namespace reprojection::config