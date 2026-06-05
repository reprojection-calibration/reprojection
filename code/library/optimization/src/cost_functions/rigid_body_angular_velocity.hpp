#pragma once

#include <ceres/autodiff_cost_function.h>

#include "spline/constants.hpp"
#include "spline/so3_spline.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

#include "ceres_geometry.hpp"

namespace reprojection::optimization::cost_functions {

class RigidBodyAngularVelocity {
   public:
    // TODO(Jack): Are we sure the coordinate frame naming and conventions/usage are actually correct?
    template <typename T>
    bool operator()(T const* const tf_imu_co_ptr, T const* const control_point_0_ptr,
                    T const* const control_point_1_ptr, T const* const control_point_2_ptr,
                    T const* const control_point_3_ptr, T* const residual) const {
        // Only map the first three values of the tf means we only get the rotation component.
        Eigen::Map<Eigen::Vector<T, 3> const> aa_imu_co(tf_imu_co_ptr);

        // WARN(Jack): Here we expect only length three control points but in the spline reprojection error cost
        // function we expect full length six control points. We need to formalize this!
        //
        // Map control point pointers into a usable control point matrix block and calculate omega.
        std::array<T const* const, spline::constants::order> const ptrs{control_point_0_ptr, control_point_1_ptr,
                                                                        control_point_2_ptr, control_point_3_ptr};

        // Only map the first three values of the spline control points - means we only get the rotation components.
        spline::MatrixNK<T> so3_control_points;
        for (int i{0}; i < spline::constants::order; ++i) {
            so3_control_points.col(i) = Eigen::Map<Eigen::Vector<T, 3> const>(ptrs[i], 3, 1);
        }
        Array3<T> const omega_co{
            spline::So3Spline::Evaluate<T, spline::DerivativeOrder::First>(so3_control_points, u_i_, delta_t_ns_)};

        // TODO(Jack): Is it really appropriate to rotate points and vectors interchangeably here? At least the naming?
        Vector3<T> const omega_imu{RotatePoint<T>(aa_imu_co, omega_co)};

        residual[0] = T(omega_imu_[0]) - omega_imu[0];
        residual[1] = T(omega_imu_[1]) - omega_imu[1];
        residual[2] = T(omega_imu_[2]) - omega_imu[2];

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
