#pragma once

#include "types/eigen_types.hpp"
#include "types/enums.hpp"

namespace reprojection::database {

std::string ToToml(CameraModel const type, ArrayXd const& intrinsics);

ArrayXd FromToml(CameraModel const type, std::string const& json_str);

}