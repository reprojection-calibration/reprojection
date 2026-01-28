#pragma once

#include <string>

#include "enums.hpp"
#include "types/enums.hpp"

namespace reprojection::config {

TargetType StringToTargetTypeEnum(std::string const& enum_string);

std::string ToString(TomlType const value);

}  // namespace reprojection::config