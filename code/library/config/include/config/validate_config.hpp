#pragma once

#include <optional>

#include <toml++/toml.hpp>

#include "config/types.hpp"

namespace reprojection::config {

std::optional<ParserErrorMsg> ValidateCalibrationConfig(toml::table const& cfg);

}  // namespace reprojection::config