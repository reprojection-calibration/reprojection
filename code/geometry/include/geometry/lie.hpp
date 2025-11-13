#pragma once

#include <vector>

#include "ceres/rotation.h"
#include "types/eigen_types.hpp"

namespace reprojection::geometry {

Isometry3d Exp(Vector6d const& se3);

Vector6d Log(Isometry3d const& SE3);

// NOTE(Jack): We use ceres helper methods here because they are autodiff compliant by default.
template <typename T>
Matrix3<T> Exp(Eigen::Vector3<T> const& so3) {
    T R[9];
    ceres::AngleAxisToRotationMatrix(so3.data(), R);

    Eigen::Map<const Matrix3<T>> SO3(R);

    return SO3;
}

template <typename T>
Eigen::Vector3<T> Log(Matrix3<T> const& SO3) {
    Eigen::Vector3<T> so3;
    ceres::RotationMatrixToAngleAxis(SO3.data(), so3.data());

    return so3;
}

std::vector<Isometry3d> ToSE3(std::vector<Array6d> const& se3);

}  // namespace reprojection::geometry