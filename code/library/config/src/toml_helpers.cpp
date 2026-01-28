#include "toml_helpers.hpp"

namespace reprojection::config {

template <typename T>
concept IsBoolean = std::is_same_v<T, bool>;

// NOTE(Jack): We do not use the "requires" concept syntax here because the parameter pack expansion prevents us.
template <IsBoolean... T>
bool MutuallyExclusive(T... condition) {
    return (static_cast<int>(condition) + ...) == 1;
}

bool TypeNodeMatch(DataType const type, toml::node_view<const toml::node> const& node) {
    return MutuallyExclusive(
        type == DataType::Array and node.is_array(), type == DataType::Boolean and node.is_boolean(),
        type == DataType::FloatingPoint and node.is_floating_point(), type == DataType::Integer and node.is_integer(),
        type == DataType::String and node.is_string(), type == DataType::Table and node.is_table());
}

// TODO(Jack): I thought about using a variant here to return a bool or the ParseError, but std::optional code that idea
//  of positive return or value in a simpler package. Is there a reason that we do need something more complicted than
//  optional?
// WARN(Jack): If for example you forgot the quotes around "TRUST_REGION" this code will
// actually throw an uncontrolled error about misinterpreting a boolean value (because it begins with T). This is not
// conform with our approach below and we should redesign this code to prevent that. Maybe we need to check the string
// values first?
// NOTE(Jack): It is valid to have more keys, this function only checks that certain required keys are present. If there
// are more that is no problem.
std::optional<ParseError> ValidateRequiredKeys(toml::table const& table,
                                               std::map<std::string, DataType> const& required_keys) {
    for (auto const& [key, type] : required_keys) {
        if (auto const node{table.at_path(key)}) {
            if (not TypeNodeMatch(type, node)) {
                return ParseError{ParseErrorType::IncorrectType,
                                  "Configuration key: " + key + " is not of expected type: " + ToString(type)};
            }
        } else {
            return ParseError{ParseErrorType::MissingKey,
                              "Configuration does not contain required key: " + key + " of type: " + ToString(type)};
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

std::optional<ParseError> ValidatePossibleKeys(toml::table const& table,
                                               std::map<std::string, DataType> const& possible_keys) {
    std::vector<std::string> full_path_keys;
    GetTomlPaths(table, full_path_keys);

    for (auto const& key : full_path_keys) {
        if (not possible_keys.contains(key)) {
            return ParseError{ParseErrorType::UnknownKey, "Configuration contains an unexpected key: " + key};
        }

        DataType const type{possible_keys.at(key)};
        auto const node{table.at_path(key)};
        if (not TypeNodeMatch(type, node)) {
            return ParseError{ParseErrorType::IncorrectType,
                              "Configuration key: " + key + " is not of expected type: " + ToString(type)};
        }
    }

    return std::nullopt;
}

}  // namespace reprojection::config
