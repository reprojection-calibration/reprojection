#pragma once

#include <string>

#include "config/enums.hpp"

namespace reprojection::config {

struct TomlErrorMsg {
    TomlError error;
    std::string msg;
};

}  // namespace reprojection::config
