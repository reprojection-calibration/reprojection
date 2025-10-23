#pragma once

#include "types/eigen_types.hpp"

namespace reprojection::eigen_utilities {

// array: [fx, fy, cx, cy]
Matrix3d ToK(Eigen::Array<double, 4, 1> const& array);

// See ToK to understand array order
Eigen::Array<double, 4, 1> FromK(Matrix3d const& matrix);

}  // namespace reprojection::eigen_utilities