#pragma once

#include <optional>

#include <toml++/toml.hpp>

#include "types/config.hpp"

namespace reprojection::config {

std::optional<TomlErrorMsg> ValidateCalibrationConfig(toml::table const& cfg);

}  // namespace reprojection::config