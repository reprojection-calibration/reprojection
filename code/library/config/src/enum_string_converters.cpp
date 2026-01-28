#include "enum_string_converters.hpp"

#include <stdexcept>

#include "config/enums.hpp"

namespace reprojection::config {

std::string ToString(TomlType const value) {
    if (value == TomlType::Array) {
        return "array";
    } else if (value == TomlType::Boolean) {
        return "boolean";
    } else if (value == TomlType::FloatingPoint) {
        return "floating_point";
    } else if (value == TomlType::Integer) {
        return "integer";
    } else if (value == TomlType::String) {
        return "string";
    } else if (value == TomlType::Table) {
        return "table";
    } else {
        throw std::runtime_error("Unrecognized argument passed to ToString(TomlType)");  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection::config