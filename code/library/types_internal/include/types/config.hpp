#pragma once

#include <string>

namespace reprojection {

enum class TomlError {
    FailedLoad,
    IncorrectType,
    MissingKey,
    UnknownKey,
};

struct TomlErrorMsg {
    TomlError error;
    std::string msg;
};

inline std::string ToString(TomlError const data) {
    if (data == TomlError::FailedLoad) {
        return "failed_load";
    } else if (data == TomlError::IncorrectType) {
        return "incorrect_type";
    } else if (data == TomlError::MissingKey) {
        return "missing_key";
    } else if (data == TomlError::UnknownKey) {
        return "unknown_key";
    } else {
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR - Unrecognized argument passed to ToString(TomlError)");
    }
}

}  // namespace reprojection
