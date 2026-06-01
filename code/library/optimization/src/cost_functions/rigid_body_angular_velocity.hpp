#pragma once

#include <ceres/autodiff_cost_function.h>
#include <ceres/rotation.h>

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
    bool operator()(T const* const aa_a_b_ptr, T const* const control_point_0_ptr, T const* const control_point_1_ptr,
                    T const* const control_point_2_ptr, T const* const control_point_3_ptr, T* const residual) const {
        Eigen::Map<Eigen::Vector<T, 3> const> aa_a_b(aa_a_b_ptr);

        // WARN(Jack): Here we expect only length three control points but in the spline reprojection error cost
        // function we expect full length six control points. We need to formalize this!
        //
        // Map control point pointers into a usable control point matrix block and calculate omega.
        std::array<T const* const, spline::constants::order> const ptrs{control_point_0_ptr, control_point_1_ptr,
                                                                        control_point_2_ptr, control_point_3_ptr};
        spline::MatrixNK<T> so3_control_points;
        for (int i{0}; i < spline::constants::order; ++i) {
            so3_control_points.col(i) = Eigen::Map<Eigen::Vector<T, 3> const>(ptrs[i], 3, 1);
        }
        Array3<T> const omega_b{
            spline::So3Spline::Evaluate<T, spline::DerivativeOrder::First>(so3_control_points, u_i_, delta_t_ns_)};

        // TODO(Jack): Is it really appropriate to rotate points and vectors interchangeably here? At least the naming?
        Vector3<T> const omega_a{RotatePoint<T>(aa_a_b, omega_b)};

        residual[0] = T(omega_a_[0]) - omega_a[0];
        residual[1] = T(omega_a_[1]) - omega_a[1];
        residual[2] = T(omega_a_[2]) - omega_a[2];

        return true;
    }

    static ceres::CostFunction* Create(Vector3d const& omega_a, double const u_i, uint64_t const delta_t_ns) {
        return new ceres::AutoDiffCostFunction<RigidBodyAngularVelocity, 3, 3, 3, 3, 3, 3>(
            new RigidBodyAngularVelocity(omega_a, u_i, delta_t_ns));
    }

    Vector3d omega_a_;

    double u_i_;
    uint64_t delta_t_ns_;
};

}  // namespace reprojection::optimization::cost_functions
