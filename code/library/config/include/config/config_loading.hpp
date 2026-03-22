#pragma once

#include <string>
#include <variant>

#include <toml++/toml.hpp>

namespace reprojection::config {

std::variant<toml::table, std::string> LoadConfigFile(std::string const& file);

}  // namespace reprojection::config