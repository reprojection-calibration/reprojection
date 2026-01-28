#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <toml++/toml.hpp>

namespace reprojection {

enum class DataType { Array, FloatingPoint, Integer, String, Table };

std::string ToString(DataType const value) {
    if (value == DataType::Array) {
        return "array";
    } else if (value == DataType::FloatingPoint) {
        return "floating_point";
    } else if (value == DataType::Integer) {
        return "integer";
    } else if (value == DataType::String) {
        return "string";
    } else if (value == DataType::Table) {
        return "table";
    } else {
        throw std::runtime_error("DataType enum ToString function has not implemented this type yet!");
    }
}

enum class ParseErrorType { UnknownKey, IncorrectType };

struct ParseError {
    ParseErrorType error;
    std::string msg;
};

}  // namespace reprojection

namespace reprojection::config {

// TODO(Jack): I thought about using a variant here to return a bool or the ParseError, but std::optional code that idea
//  of positive return or value in a simpler package. Is there a reason that we do need something more complicted than
//  optional?
// WARN(Jack): If for example you forgot the quotes around "TRUST_REGION" this code will
// actually throw an uncontrolled error about misinterpreting a boolean value (because it begins with T). This is not
// conform with our approach below and we should redesign this code to prevent that. Maybe we need to check the string
// values first?
std::optional<ParseError> ValidateToml(toml::table const& table, std::map<std::string, DataType> const& required_keys) {
    auto type_error = [](std::string const& key, DataType const type) {
        return ParseError{ParseErrorType::IncorrectType,
                          "Table key: " + key + "is not of expected type: " + ToString(type)};
    };

    for (auto const& [key, type] : required_keys) {
        if (auto const node{table.at_path(key)}) {
            if (type == DataType::Array and not node.is_array()) {
                return type_error(key, type);
            } else if (type == DataType::FloatingPoint and not node.is_floating_point()) {
                return type_error(key, type);
            } else if (type == DataType::Integer and not node.is_integer()) {
                return type_error(key, type);
            } else if (type == DataType::String and not node.is_string()) {
                return type_error(key, type);
            }
        } else {
            return ParseError{ParseErrorType::UnknownKey,
                              "Table does not contain required key: " + key + " of type: " + ToString(type)};
        }
    }

    return std::nullopt;
}

}  // namespace reprojection::config

using namespace reprojection;
using namespace std::string_view_literals;

TEST(ConfigTomlHelpers, TestValidateTomlHappyPath) {
    static constexpr std::string_view config_file{R"(
        [target]
        pattern_size = [3,4]
        unit_dimension = 0.5

        [solver]
        num_threads = 1
        minimizer_type = "TRUST_REGION"

    )"sv};
    toml::table const config{toml::parse(config_file)};

    // Example use case #1 - check that a config has specific table sections.
    std::map<std::string, DataType> const required_keys{{"target", DataType::Table}, {"solver", DataType::Table}};
    auto const error_msg{config::ValidateToml(config, required_keys)};
    EXPECT_FALSE(error_msg.has_value());

    // Example use case #2 - check that a config has specific keys.
    std::map<std::string, DataType> const required_keys_1{{"target.pattern_size", DataType::Array},
                                                          {"target.unit_dimension", DataType::FloatingPoint},
                                                          {"solver.num_threads", DataType::Integer},
                                                          {"solver.minimizer_type", DataType::String}};
    auto const error_msg_1{config::ValidateToml(config, required_keys_1)};
    EXPECT_FALSE(error_msg_1.has_value());
}