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

}  // namespace reprojection
