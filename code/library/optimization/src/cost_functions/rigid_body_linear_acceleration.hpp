#pragma once

#include <ceres/autodiff_cost_function.h>

#include "spline/constants.hpp"
#include "spline/r3_spline.hpp"
#include "spline/so3_spline.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

#include "ceres_geometry.hpp"

namespace reprojection::optimization::cost_functions {

using So3Spline = spline::So3Spline;
using R3Spline = spline::R3Spline;
using Order = spline::DerivativeOrder;

class RigidBodyLinearAcceleration {
   public:
    template <typename T>
    bool operator()(T const* const tf_imu_co_ptr, T const* const gravity_w_ptr, T const* const control_point_0_ptr,
                    T const* const control_point_1_ptr, T const* const control_point_2_ptr,
                    T const* const control_point_3_ptr, T* const residual) const {
        // TODO(Jack): Do not copy and paste this - three places right now!
        std::array<T const* const, spline::constants::order> const ptrs{control_point_0_ptr, control_point_1_ptr,
                                                                        control_point_2_ptr, control_point_3_ptr};
        spline::Matrix2NK<T> control_points;
        for (int i{0}; i < spline::constants::order; ++i) {
            control_points.col(i) = Eigen::Map<Eigen::Vector<T, 6> const>(ptrs[i], 6, 1);
        }

        auto const so3{control_points.template topRows<3>()};
        auto const r3{control_points.template bottomRows<3>()};

        // TODO(Jack): What is the actual form of a_w?
        // TODO(Jack): Edit naming to reflect the fact that the imu is actually measuring specific force and not the
        // motion acceleration.
        Vector3<T> const aa_w_co{So3Spline::Evaluate<T, Order::Null>(so3, u_i_, delta_t_ns_)};
        Vector3<T> const acc_co{R3Spline::Evaluate<T, Order::Second>(r3, u_i_, delta_t_ns_)};
        Eigen::Map<Eigen::Vector<T, 3> const> gravity_w(gravity_w_ptr);
        Vector3<T> const acc_co_XXX{(geometry::Exp<T>(aa_w_co).inverse() * gravity_w) + acc_co};

        Eigen::Map<Eigen::Vector<T, 6> const> tf_imu_co(tf_imu_co_ptr);
        Vector3<T> const omega_co{So3Spline::Evaluate<T, Order::First>(so3, u_i_, delta_t_ns_)};
        Vector3<T> const alpha_co{So3Spline::Evaluate<T, Order::Second>(so3, u_i_, delta_t_ns_)};
        Vector3<T> const acc_imu{TransformRigidBodyAcceleration<T>(tf_imu_co, omega_co, alpha_co, acc_co_XXX)};

        residual[0] = T(acc_imu_[0]) - acc_imu[0];
        residual[1] = T(acc_imu_[1]) - acc_imu[1];
        residual[2] = T(acc_imu_[2]) - acc_imu[2];

        return true;
    }

    static ceres::CostFunction* Create(Vector3d const& acc_imu, double const u_i, uint64_t const delta_t_ns) {
        return new ceres::AutoDiffCostFunction<RigidBodyLinearAcceleration, 3, 6, 3, 6, 6, 6, 6>(
            new RigidBodyLinearAcceleration(acc_imu, u_i, delta_t_ns));
    }

    Vector3d acc_imu_;

    double u_i_;
    uint64_t delta_t_ns_;
};

}  // namespace reprojection::optimization::cost_functions
