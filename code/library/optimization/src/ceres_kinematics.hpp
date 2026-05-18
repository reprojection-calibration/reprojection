#pragma once

#include <Eigen/Core>

#include "types/eigen_types.hpp"

#include "ceres_geometry.hpp"

namespace reprojection::optimization {

// a = C_i_b * (C_b_w * (a_w - g_w) + w_dot_b.cross(r_b) + w_b.cross(w_b.cross(r_b)))
// TODO(Jack): This is almost exactly copy and pasted from ceres_geometry.hpp we should fix that!
template <typename T>
Vector3<T> TransformRigidBodyAcceleration(Eigen::Ref<Eigen::Vector<T, 6> const> const& tf_i_j,
                                          Eigen::Ref<Vector3<T> const> const& omega_j,
                                          Eigen::Ref<Vector3<T> const> const& alpha_j,
                                          Eigen::Ref<Vector3<T> const> const& a_j) {
    Vector3<T> const r3{tf_i_j.template bottomRows<3>()};
    Vector3<T> const XXX{a_j + alpha_j.cross(r3) + omega_j.cross(omega_j.cross(r3))};  // NAMING!!!!

    Vector3<T> const aa_i_j{tf_i_j.template topRows<3>()};
    Vector3<T> const ai{RotatePoint<T>(aa_i_j, XXX)};

    return ai;
}

}  // namespace  reprojection::optimization
