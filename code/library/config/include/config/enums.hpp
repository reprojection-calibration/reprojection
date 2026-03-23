#pragma once

namespace reprojection::config {

enum class TomlError {
    FailedLoad,
    IncorrectType,
    MissingKey,
    UnknownKey,
};

enum class TomlType {
    Array,
    Boolean,
    FloatingPoint,
    Integer,
    String,
    Table,
};

}  // namespace reprojection::config