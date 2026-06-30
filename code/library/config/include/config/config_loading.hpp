#pragma once

#include <filesystem>
#include <string>
#include <variant>

#include <toml++/toml.hpp>

#include "types/config.hpp"

namespace reprojection::config {

// TODO(Jack): Just refactor this to throw instead of returning the variant. I do not think we need this wrapper
// function at all, just use the toml parse function directly.
std::variant<toml::table, TomlErrorMsg> LoadConfigFile(std::string const& file);

}  // namespace reprojection::config