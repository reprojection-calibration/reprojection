#pragma once

#include <ceres/rotation.h>

#include <Eigen/Core>

#include "types/eigen_types.hpp"

namespace reprojection::optimization {

template <typename T>
Vector3<T> RotatePoint(Eigen::Ref<Eigen::Vector<T, 3> const> const& aa_i_j,
                       Eigen::Ref<Vector3<T> const> const& point_j) {
    Vector3<T> point_i;
    ceres::AngleAxisRotatePoint(aa_i_j.data(), point_j.data(), point_i.data());

    return point_i;
}

// NOTE(Jack): We use Eigen::Ref here so we can pass both maps (in the PinholeCostFunction.operator()) and the direct
// types (in the testing for example).
// TODO(Jack): Does the point here really need to be templated? Or as a constant can we avoid that?
template <typename T>
Vector3<T> TransformPoint(Eigen::Ref<Eigen::Vector<T, 6> const> const& tf_i_j,
                          Eigen::Ref<Vector3<T> const> const& point_j) {
    return RotatePoint<T>(tf_i_j.template topRows<3>(), point_j) + tf_i_j.template bottomRows<3>();
}

template <typename T>
Vector3<T> TransformRigidBodyAcceleration(Eigen::Ref<Eigen::Vector<T, 6> const> const& tf_i_j,
                                          Eigen::Ref<Vector3<T> const> const& omega_j,
                                          Eigen::Ref<Vector3<T> const> const& alpha_j,
                                          Eigen::Ref<Vector3<T> const> const& acc_j) {
    Vector3<T> const r_i_j{tf_i_j.template bottomRows<3>()};
    Vector3<T> const acc_i_j{acc_j + alpha_j.cross(r_i_j) + omega_j.cross(omega_j.cross(r_i_j))};

    Vector3<T> const aa_i_j{tf_i_j.template topRows<3>()};
    Vector3<T> const acc_i{RotatePoint<T>(aa_i_j, acc_i_j)};

    return acc_i;
}

}  // namespace  reprojection::optimization
