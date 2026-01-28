#pragma once

#include <optional>
#include <string>
#include <toml++/toml.hpp>

// TODO MOVE TO TYPES
namespace reprojection {

enum class TomlType {
    Array,
    Boolean,
    FloatingPoint,
    Integer,
    String,
    Table,
};

std::string ToString(TomlType const value);

enum class TomlParseError {
    IncorrectType,
    MissingKey,
    UnknownKey,
};

struct ParserErrorMsg {
    TomlParseError error;
    std::string msg;
};

}  // namespace reprojection

namespace reprojection::config {

std::optional<ParserErrorMsg> ValidateRequiredKeys(toml::table const& table,
                                                   std::map<std::string, TomlType> const& required_keys);

// TEST!!!!
// TEST!!!!
// TEST!!!!
// TEST!!!!
// TODO(Jack): Do this without recursion instead of editing toml_paths in place!
// NOTE(Jack): We need to iterate over the entire table to construct the full toml paths. Unfortunately there is no
// reverse of the .at_path() function which we can just use.
void GetTomlPaths(toml::table const& table, std::vector<std::string>& toml_paths, std::string_view prefix = "");

// TODO(Jack): Would it be useful or more informative to the user to have a version of this that only validates table
//  headers?
std::optional<ParserErrorMsg> ValidatePossibleKeys(toml::table const& table,
                                                   std::map<std::string, TomlType> const& possible_keys);

}  // namespace reprojection::config
