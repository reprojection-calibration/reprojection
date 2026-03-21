#pragma once

#include <optional>
#include <string>

#include <toml++/toml.hpp>

#include "config/enums.hpp"
#include "config/types.hpp"

namespace reprojection::config {

using TomlKeys = std::map<std::string, TomlType>;

std::optional<ParserErrorMsg> ValidateConfigKeys(toml::table const& config, TomlKeys const& required_keys,
                                                 TomlKeys const& optional_keys = {}, bool const allow_unknown = false);

std::optional<ParserErrorMsg> ValidateRequiredKeys(toml::table const& table, TomlKeys const& required_keys);

std::optional<ParserErrorMsg> ValidatePossibleKeys(toml::table const& table, TomlKeys const& possible_keys,
                                                   bool const allow_unknown);

// TODO(Jack): Do this without recursion instead of editing toml_paths in place! Is that even possible?
void GetTomlPaths(toml::table const& table, std::vector<std::string>& toml_paths, std::string_view prefix = "");

}  // namespace reprojection::config
