#pragma once

#include <string>
#include <variant>

#include <toml++/toml.hpp>

#include "types/config.hpp"

namespace reprojection::config {

std::variant<toml::table, TomlErrorMsg> LoadConfigFile(std::string const& file);

}  // namespace reprojection::config