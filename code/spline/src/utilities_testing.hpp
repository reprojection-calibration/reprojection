#pragma once

#include <Eigen/Dense>

namespace reprojection::spline {

bool IsRotation(Eigen::Matrix3d const& R) {
    Eigen::Matrix3d const RRT{R * R.transpose()};  // For rotations R^T = R^-1
    bool const is_orthogonal{(RRT - Eigen::Matrix3d::Identity()).norm() < 1e-10};

    double const D{R.determinant()};
    bool const is_proper{(D - 1) < 1e-15};  // Determinant is positive one

    return is_orthogonal and is_proper;
}

}  // namespace reprojection::spline