#pragma once

#include <optional>

#include <toml++/toml.hpp>

#include "types/config.hpp"

namespace reprojection::config {

std::optional<TomlErrorMsg> ValidateCalibrationConfig(toml::table const& cfg);

std::optional<TomlErrorMsg> ValidateDataConfig(toml::table const& data_cfg);

std::optional<TomlErrorMsg> ValidateSensorConfig(toml::table const& sensor_cfg);

// NOTE(Jack): This is in the public header section because feature_extraction::CreateTargetExtractor() consumes the
// toml table directly
std::optional<TomlErrorMsg> ValidateTargetConfig(toml::table const& target_cfg);

}  // namespace reprojection::config