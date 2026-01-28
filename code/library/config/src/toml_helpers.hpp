#pragma once

#include <optional>
#include <string>
#include <toml++/toml.hpp>

// TODO MOVE TO TYPES
namespace reprojection {

enum class DataType {
    Array,
    FloatingPoint,
    Integer,
    String,
    Table,
};

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

enum class ParseErrorType {
    IncorrectType,
    UnknownKey,
};

struct ParseError {
    ParseErrorType error;
    std::string msg;
};

}  // namespace reprojection

namespace reprojection::config {

std::optional<ParseError> ValidateRequiredKeys(toml::table const& table,
                                               std::map<std::string, DataType> const& required_keys);

}  // namespace reprojection::config
