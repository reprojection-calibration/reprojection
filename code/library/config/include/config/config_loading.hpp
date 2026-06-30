#pragma once

#include <filesystem>
#include <string>
#include <variant>

#include <toml++/toml.hpp>

#include "types/config.hpp"

namespace reprojection::config {

toml::table LoadConfigFile(std::string const& file);

}  // namespace reprojection::config