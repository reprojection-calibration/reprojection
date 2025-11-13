#pragma once

#include <vector>

#include "ceres/rotation.h"
#include "types/eigen_types.hpp"

namespace reprojection::geometry {

Isometry3d Exp(Vector6d const& se3);

Vector6d Log(Isometry3d const& SE3);

// NOTE(Jack): We use ceres helper methods because they are autodiff compliant.
template <typename T>
Eigen::Matrix3<T> Exp(Eigen::Vector3<T> const& so3) {
    // ERROR(Jack): What error in our rotation convention requires that we multiply this by negative one? The library
    // was first developer using Eigen axisangle functions here, but when we switched to ceres we had to add this -1 to
    // preserve the reversability of the exp and log.
    Eigen::Vector3<T> const negative_so3{-1.0 * so3};

    T R[9];
    ceres::AngleAxisToRotationMatrix(negative_so3.data(), R);

    Eigen::Map<const Eigen::Matrix<T, 3, 3, Eigen::RowMajor>> SO3(R);

    return SO3;
}

template <typename T>
Eigen::Vector3<T> Log(Eigen::Matrix3<T> const& SO3) {
    Eigen::Vector3<T> so3;
    ceres::RotationMatrixToAngleAxis(SO3.data(), so3.data());

    return so3;
}

std::vector<Isometry3d> ToSE3(std::vector<Array6d> const& se3);

}  // namespace reprojection::geometry