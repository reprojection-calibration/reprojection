#pragma once

#include "types/eigen_types.hpp"

namespace reprojection::eigen_utilities {

// [f, cx, cy]
Array3d FromK(Matrix3d const& matrix);

}  // namespace reprojection::eigen_utilities