#pragma once

#include <string>

#include "types/enums.hpp"

namespace reprojection::config {

// TODO(Jack): Do we really need an entire set of files just for this one method? Maybe this can be grouped with other
//  enum conversion code or even moved to the types package.
TargetType StringToTargetTypeEnum(std::string const& enum_string);

}  // namespace reprojection::config