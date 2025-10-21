#pragma once

#include <Eigen/Dense>

namespace reprojection::eigen_utilities {

// array: [fx, fy, cx, cy]
Eigen::Matrix3d ToK(Eigen::Array<double, 4, 1> const& array);

// See ToK to get array order
Eigen::Array<double, 4, 1> FromK(Eigen::Matrix3d const& matrix);

}  // namespace reprojection::eigen_utilities