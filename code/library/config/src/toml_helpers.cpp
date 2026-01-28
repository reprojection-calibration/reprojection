#include "toml_helpers.hpp"

#include "enum_string_converters.hpp"

namespace reprojection::config {

template <typename T>
concept IsBoolean = std::is_same_v<T, bool>;

// NOTE(Jack): We do not use the "requires" concept syntax here because the parameter pack expansion prevents us.
template <IsBoolean... T>
bool MutuallyExclusive(T... condition) {
    return (static_cast<int>(condition) + ...) == 1;
}

bool TypeNodeMatch(TomlType const type, toml::node_view<const toml::node> const& node) {
    return MutuallyExclusive(
        type == TomlType::Array and node.is_array(), type == TomlType::Boolean and node.is_boolean(),
        type == TomlType::FloatingPoint and node.is_floating_point(), type == TomlType::Integer and node.is_integer(),
        type == TomlType::String and node.is_string(), type == TomlType::Table and node.is_table());
}

// TODO(Jack): I thought about using a variant here to return a bool or the ParserErrorMsg, but std::optional code that
// idea
//  of positive return or value in a simpler package. Is there a reason that we do need something more complicted than
//  optional?
// WARN(Jack): If for example you forgot the quotes around "TRUST_REGION" this code will
// actually throw an uncontrolled error about misinterpreting a boolean value (because it begins with T). This is not
// conform with our approach below and we should redesign this code to prevent that. Maybe we need to check the string
// values first?
// NOTE(Jack): It is valid to have more keys, this function only checks that certain required keys are present. If there
// are more that is no problem.
std::optional<ParserErrorMsg> ValidateRequiredKeys(toml::table const& table,
                                                   std::map<std::string, TomlType> const& required_keys) {
    for (auto const& [key, type] : required_keys) {
        if (auto const node{table.at_path(key)}) {
            if (not TypeNodeMatch(type, node)) {
                return ParserErrorMsg{TomlParseError::IncorrectType,
                                      "Configuration key: " + key + " is not of expected type: " + ToString(type)};
            }
        } else {
            return ParserErrorMsg{TomlParseError::MissingKey, "Configuration does not contain required key: " + key +
                                                                  " of type: " + ToString(type)};
        }
    }

    return std::nullopt;
}

std::optional<ParserErrorMsg> ValidatePossibleKeys(toml::table const& table,
                                                   std::map<std::string, TomlType> const& possible_keys) {
    std::vector<std::string> full_path_keys;
    GetTomlPaths(table, full_path_keys);

    for (auto const& key : full_path_keys) {
        if (not possible_keys.contains(key)) {
            return ParserErrorMsg{TomlParseError::UnknownKey, "Configuration contains an unexpected key: " + key};
        }

        TomlType const type{possible_keys.at(key)};
        auto const node{table.at_path(key)};
        if (not TypeNodeMatch(type, node)) {
            return ParserErrorMsg{TomlParseError::IncorrectType,
                                  "Configuration key: " + key + " is not of expected type: " + ToString(type)};
        }
    }

    return std::nullopt;
}

void GetTomlPaths(toml::table const& table, std::vector<std::string>& toml_paths, std::string_view prefix) {
    for (auto const& [key, node] : table) {
        std::string const full_path{prefix.empty() ? std::string(key) : std::string(prefix) + "." + std::string(key)};
        if (auto const sub{node.as_table()}) {
            GetTomlPaths(*sub, toml_paths, full_path);
        }

        toml_paths.push_back(full_path);
    }
}

}  // namespace reprojection::config
