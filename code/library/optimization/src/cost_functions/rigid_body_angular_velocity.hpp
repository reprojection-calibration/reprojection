#pragma once

#include <ceres/autodiff_cost_function.h>

#include "cost_functions/utils.hpp"
#include "spline/so3_spline.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

#include "ceres_geometry.hpp"

namespace reprojection::optimization::cost_functions {

// NOTE(Jack): We actually only need the rotation part of the spline to calculate the cost. However, ceres does not
// allow us to partially reference/alias parameter blocks therefore when we use this function we usually pass in a
// pointer to the entire control point. Here we ignore that fact and only map the first three values which are the
// rotation component.
class RigidBodyAngularVelocity {
   public:
    template <typename T>
    bool operator()(T const* const tf_imu_co_ptr, T const* const cp_0_ptr, T const* const cp_1_ptr,
                    T const* const cp_2_ptr, T const* const cp_3_ptr, T* const residual_ptr) const {
        auto const P{BuildP<T, 3>(cp_0_ptr, cp_1_ptr, cp_2_ptr, cp_3_ptr)};

        Array3<T> const omega_co{spline::So3Spline::Evaluate<T, spline::DerivativeOrder::First>(P, u_i_, delta_t_ns_)};

        Eigen::Map<Eigen::Vector<T, 3> const> aa_imu_co(tf_imu_co_ptr);
        Vector3<T> const omega_imu{RotatePoint<T>(aa_imu_co, omega_co)};

        Eigen::Map<Array3<T>> residual(residual_ptr);
        residual = omega_imu_.template cast<T>() - omega_imu;

        return true;
    }

    // NOTE(Jack): The rigid body angular velocity optimization actually only requires the rotation components of the
    // extrinsic and the rotation components of the spline. But we use this cost function also in the full optimization
    // (not just for the extrinsic initialization) and there we have full size control point parameter blocks. Ceres
    // does not allow use to have two cost functions attached to a problem that reference the same pointer to a
    // parameter block but with different sizes. It throws this error if we try:
    //
    //      1 problem_impl.cc:132] Check failed: size == existing_size Tried adding a parameter block with the same
    //      double pointer, 0x7ffe17a85940, twice, but with different block sizes. Original size was 3 but new size is 6
    //
    // Therefore we are forced to make the RigidBodyAngularVelocity cost function accept the full length 6 control
    // points parameter blocks, but in the cost function itself we will only use the rotation parts.
    static ceres::CostFunction* Create(Vector3d const& omega_imu, double const u_i, uint64_t const delta_t_ns) {
        return new ceres::AutoDiffCostFunction<RigidBodyAngularVelocity, 3, 6, 6, 6, 6, 6>(
            new RigidBodyAngularVelocity(omega_imu, u_i, delta_t_ns));
    }

    Vector3d omega_imu_;

    double u_i_;
    uint64_t delta_t_ns_;
};

}  // namespace reprojection::optimization::cost_functions
