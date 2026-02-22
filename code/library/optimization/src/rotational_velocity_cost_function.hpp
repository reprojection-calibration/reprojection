#pragma once

#include <ceres/autodiff_cost_function.h>
#include <ceres/rotation.h>

#include "types/eigen_types.hpp"

namespace reprojection::optimization {

// NOTE(Jack): The rotational velocities must come from two frames on a rigid body. If they are not on the same body
// this optimization is not meaningful.
class RotationalVelocityCostFunction {
   public:
    RotationalVelocityCostFunction(Vector3d const& omega_a, Vector3d const& omega_b)
        : omega_a_{omega_a}, omega_b_{omega_b} {}

    // TODO CHECK COORDINATE CONVENTIONS ARE CORRECT!
    template <typename T>
    bool operator()(T const* const orientation_ptr, T* const residual) const {
        Eigen::Map<Eigen::Vector<T, 3> const> aa_a_b(orientation_ptr);
        Vector3<T> const T_omega_b{omega_b_.cast<T>()};

        Vector3<T> omega_a;
        ceres::AngleAxisRotatePoint(aa_a_b.data(), T_omega_b.data(), omega_a.data());

        residual[0] = T(omega_a_[0]) - omega_a[0];
        residual[1] = T(omega_a_[1]) - omega_a[1];
        residual[2] = T(omega_a_[2]) - omega_a[2];

        return true;
    }

    static ceres::CostFunction* Create(Vector3d const& omega_a, Vector3d const& omega_b) {
        return new ceres::AutoDiffCostFunction<RotationalVelocityCostFunction, 3, 3>(
            new RotationalVelocityCostFunction(omega_a, omega_b));
    }

    Vector3d omega_a_;
    Vector3d omega_b_;
};

}  // namespace  reprojection::optimization
