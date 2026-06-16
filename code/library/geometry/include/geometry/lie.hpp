#pragma once

#include <ceres/rotation.h>

#include "types/eigen_types.hpp"

namespace reprojection::geometry {

Isometry3d Exp(Vector6d const& se3);

Vector6d Log(Isometry3d const& SE3);

// NOTE(Jack): We use ceres here because the methods are autodiff compatible by default.
template <typename T>
Matrix3<T> Exp(Vector3<T> const& so3) {
    // TODO(Jack): Why do we need to code coverage suppress this line here? Makes no sense... should I be scared?
    T R[9];  // LCOV_EXCL_LINE
    ceres::AngleAxisToRotationMatrix(so3.data(), R);

    Eigen::Map<const Matrix3<T>> SO3(R);

    return SO3;
}

template <typename T>
Vector3<T> Log(Matrix3<T> const& SO3) {
    Vector3<T> so3;
    ceres::RotationMatrixToAngleAxis(SO3.data(), so3.data());

    return so3;
}

// TODO(Jack): Unit test!
template <typename T>
Array6<T> InverseTransform(Array6<T> const& tf_a_b) {
    Array3<T> const aa_a_b{tf_a_b.template head<3>()};
    Vector3<T> const p_a_b{tf_a_b.template tail<3>()};

    Matrix3<T> const R_a_b{Exp<T>(aa_a_b)};
    Matrix3<T> const R_b_a{R_a_b.transpose()};

    Array6<T> tf_b_a;
    tf_b_a.template head<3>() = Log<T>(R_b_a);
    tf_b_a.template tail<3>() = -R_b_a * p_a_b;

    return tf_b_a;
}

}  // namespace reprojection::geometry