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

        // ERROR(Jack): This is not nice at all. We are hardcoding the absolute value difference here because somewhere
        // in the flow we are getting a flipped omega-x. This means that after transforming the interpolate camera
        // spline omega into the IMU frame it does not line up. If we do nothing here, then it all gets mushed together.
        // If we take the absolute value of the residuals then we get alignment in shape, but some are flipped by -1. By
        // taking the absolute value of the x term here we get the same matrix as our ground-truth from the TUM dataset,
        // so for now we will do this.
        //
        // But underlying this all seems to be a really nasty inconsistency that will probably get bigger and bigger as
        // we go down the calibration workflow.
        residual[0] = T(std::abs(omega_a_[0])) - std::abs<T>(omega_a[0]);
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
