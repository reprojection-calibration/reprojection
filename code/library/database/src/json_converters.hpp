#pragma once

#include "types/eigen_types.hpp"
#include "types/enums.hpp"

namespace reprojection::database {

std::string ToJson(CameraModel const type, ArrayXd const& intrinsics);

}