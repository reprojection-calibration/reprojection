#pragma once

#include "types/eigen_types.hpp"

namespace reprojection::eigen_utilities {

// [fx, fy, cx, cy]
Array4d FromK(Matrix3d const& matrix);

}  // namespace reprojection::eigen_utilities