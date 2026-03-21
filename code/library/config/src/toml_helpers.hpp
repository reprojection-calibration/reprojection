#pragma once

#include <optional>
#include <string>

#include <toml++/toml.hpp>

#include "config/enums.hpp"
#include "config/types.hpp"

namespace reprojection::config {

std::optional<ParserErrorMsg> ValidateConfigKeys(toml::table const& config,
                                                 std::map<std::string, TomlType> const& required_keys,
                                                 std::map<std::string, TomlType> const& optional_keys = {},
                                                 bool const allow_unknown = false);

std::optional<ParserErrorMsg> ValidateRequiredKeys(toml::table const& table,
                                                   std::map<std::string, TomlType> const& required_keys);

std::optional<ParserErrorMsg> ValidatePossibleKeys(toml::table const& table,
                                                   std::map<std::string, TomlType> const& possible_keys,
                                                   bool const allow_unknown);

// TODO(Jack): Do this without recursion instead of editing toml_paths in place! Is that even possible?
// NOTE(Jack): We need to iterate over the entire table to construct the full toml paths. Unfortunately there is no
// reverse of the .at_path() function which we can just use.
void GetTomlPaths(toml::table const& table, std::vector<std::string>& toml_paths, std::string_view prefix = "");

}  // namespace reprojection::config
