#pragma once

#include <ceres/autodiff_cost_function.h>

#include "types/eigen_types.hpp"

#include "ceres_geometry.hpp"

namespace reprojection::optimization::cost_functions {

class RigidBodyLinearAcceleration {
   public:
    // TODO(Jack): Are we sure the coordinate frame naming and conventions/usage are actually correct?
    template <typename T>
    bool operator()(T const* const tf_imu_co_ptr, T const* const control_point_0_ptr,
                    T const* const control_point_1_ptr, T const* const control_point_2_ptr,
                    T const* const control_point_3_ptr, T* const residual) const {
        Eigen::Map<Eigen::Vector<T, 6> const> tf_imu_co(tf_imu_co_ptr);

        // REMOVE
        (void)control_point_0_ptr;
        (void)control_point_1_ptr;
        (void)control_point_2_ptr;
        (void)control_point_3_ptr;
        Vector3<T> const acc_imu{T(1.0), T(2.0), T(3.0)};

        residual[0] = T(acc_imu_[0]) - acc_imu[0];
        residual[1] = T(acc_imu_[1]) - acc_imu[1];
        residual[2] = T(acc_imu_[2]) - acc_imu[2];

        return true;
    }

    static ceres::CostFunction* Create(Vector3d const& acc_imu, double const u_i, uint64_t const delta_t_ns) {
        return new ceres::AutoDiffCostFunction<RigidBodyLinearAcceleration, 3, 6, 6, 6, 6, 6>(
            new RigidBodyLinearAcceleration(acc_imu, u_i, delta_t_ns));
    }

    Vector3d acc_imu_;

    double u_i_;
    uint64_t delta_t_ns_;
};

}  // namespace reprojection::optimization::cost_functions
