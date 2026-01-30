#pragma once

#include <string>

#include "config/enums.hpp"

namespace reprojection::config {

struct ParserErrorMsg {
    TomlParseError error;
    std::string msg;
};

}  // namespace reprojection::config
