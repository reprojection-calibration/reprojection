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
    bool operator()(T const* const aa_imu_co_ptr, T const* const control_point_0_ptr,
                    T const* const control_point_1_ptr, T const* const control_point_2_ptr,
                    T const* const control_point_3_ptr, T* const residual) const {
        std::array<T const* const, spline::constants::order> const ptrs{control_point_0_ptr, control_point_1_ptr,
                                                                        control_point_2_ptr, control_point_3_ptr};
        spline::Matrix2NK<T> control_points;
        for (int i{0}; i < spline::constants::order; ++i) {
            control_points.col(i) = Eigen::Map<Eigen::Vector<T, 6> const>(ptrs[i], 6, 1);
        }
        Array3<T> const omega_co{spline::So3Spline::Evaluate<T, spline::DerivativeOrder::First>(
            control_points.template topRows<3>(), u_i_, delta_t_ns_)};

        Eigen::Map<Eigen::Vector<T, 6> const> tf_imu_co(aa_imu_co_ptr);
        // TODO(Jack): Is it really appropriate to rotate points and vectors interchangeably here? At least the naming?
        Vector3<T> const omega_imu{RotatePoint<T>(tf_imu_co.template topRows<3>(), omega_co)};

        residual[0] = T(omega_imu_[0]) - omega_imu[0];
        residual[1] = T(omega_imu_[1]) - omega_imu[1];
        residual[2] = T(omega_imu_[2]) - omega_imu[2];

        return true;
    }

    static ceres::CostFunction* Create(Vector3d const& omega_imu, double const u_i, uint64_t const delta_t_ns) {
        return new ceres::AutoDiffCostFunction<RigidBodyAngularVelocity, 3, 6, 6, 6, 6, 6>(
            new RigidBodyAngularVelocity(omega_imu, u_i, delta_t_ns));
    }

    Vector3d omega_imu_;

    double u_i_;
    uint64_t delta_t_ns_;
};

}  // namespace reprojection::optimization::cost_functions
