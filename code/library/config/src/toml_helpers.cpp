#include "toml_helpers.hpp"

namespace reprojection::config {

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
            if ((type == DataType::Array and not node.is_array()) or
                (type == DataType::FloatingPoint and not node.is_floating_point()) or
                (type == DataType::Integer and not node.is_integer()) or
                (type == DataType::String and not node.is_string()) or
                (type == DataType::Table and not node.is_table())) {
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

}  // namespace reprojection::config
