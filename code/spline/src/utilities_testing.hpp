#pragma once

#include "types/eigen_types.hpp"

namespace reprojection::spline {

bool IsRotation(Matrix3d const& R) {
    Matrix3d const RRT{R * R.transpose()};  // For rotations R^T = R^-1
    bool const is_orthogonal{(RRT - Matrix3d::Identity()).norm() < 1e-10};

    double const D{R.determinant()};
    bool const is_proper{(D - 1) < 1e-15};  // Determinant is positive one

    return is_orthogonal and is_proper;
}

}  // namespace reprojection::spline