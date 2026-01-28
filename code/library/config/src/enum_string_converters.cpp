#include "enum_string_converters.hpp"

#include <stdexcept>

#include "enums.hpp"

namespace reprojection::config {

TargetType ToTargetType(std::string const& enum_string) {
    if (enum_string == "checkerboard") {
        return TargetType::Checkerboard;
    } else if (enum_string == "circle_grid") {
        return TargetType::CircleGrid;
    } else if (enum_string == "april_grid3") {
        return TargetType::AprilGrid3;
    } else {
        throw std::runtime_error("The requested target type string - " + enum_string +
                                 " - is not valid (see the TargetType enum).");
    }
}

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
        throw std::runtime_error(
            "TomlType enum ToString function has not implemented this type yet!");  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection::config