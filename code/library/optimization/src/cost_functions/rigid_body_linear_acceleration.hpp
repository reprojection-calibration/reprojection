#pragma once

#include <ceres/autodiff_cost_function.h>

#include "cost_functions/utils.hpp"
#include "spline/constants.hpp"
#include "spline/r3_spline.hpp"
#include "spline/so3_spline.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"
#include "types/physics_constants.hpp"

#include "ceres_geometry.hpp"

namespace reprojection::optimization::cost_functions {

using So3Spline = spline::So3Spline;
using R3Spline = spline::R3Spline;
using Order = spline::DerivativeOrder;

class RigidBodyLinearAcceleration {
   public:
    template <typename T>
    bool operator()(T const* const tf_imu_co_ptr, T const* const gravity_w_ptr, T const* const cp_0_ptr,
                    T const* const cp_1_ptr, T const* const cp_2_ptr, T const* const cp_3_ptr,
                    T* const residual) const {
        auto const P{BuildP<T, 6>(cp_0_ptr, cp_1_ptr, cp_2_ptr, cp_3_ptr)};
        auto const so3{P.template topRows<3>()};

        Eigen::Map<Eigen::Vector<T, 6> const> tf_imu_co(tf_imu_co_ptr);
        Vector3<T> const omega_co{So3Spline::Evaluate<T, Order::First>(so3, u_i_, delta_t_ns_)};
        Vector3<T> const alpha_co{So3Spline::Evaluate<T, Order::Second>(so3, u_i_, delta_t_ns_)};

        Vector3<T> const aa_w_co{So3Spline::Evaluate<T, Order::Null>(so3, u_i_, delta_t_ns_)};
        Matrix3<T> const R_co_w{geometry::Exp<T>(aa_w_co).transpose()};

        Vector3<T> const acc_w{R3Spline::Evaluate<T, Order::Second>(P.template bottomRows<3>(), u_i_, delta_t_ns_)};
        Vector3<T> const acc_co{R_co_w * acc_w};

        Vector3<T> const acc_imu{TransformRigidBodyAcceleration<T>(tf_imu_co, omega_co, alpha_co, acc_co)};

        Eigen::Map<Eigen::Vector<T, 3> const> gravity_w(gravity_w_ptr);
        Vector3<T> const gravity_imu{geometry::Exp<T>(tf_imu_co.template topRows<3>()) * R_co_w * gravity_w};

        // NOTE(Jack): Imus really measure specific force (i.e. acceleration plus gravity), but our naming throughout
        // the code base does not reflect this/is not consistent.
        Vector3<T> const specific_force_imu{acc_imu + gravity_imu};

        residual[0] = T(acc_imu_[0]) - specific_force_imu[0];
        residual[1] = T(acc_imu_[1]) - specific_force_imu[1];
        residual[2] = T(acc_imu_[2]) - specific_force_imu[2];
        // TODO(Jack): Fixing this here might be/likely is adding a fixed bias to then estimate proportional to the
        // deviation between real gravity as measured and the value of kGravity.
        residual[3] = T(kGravity * kGravity) -
                      (gravity_w[0] * gravity_w[0] + gravity_w[1] * gravity_w[1] + gravity_w[2] * gravity_w[2]);

        return true;
    }

    static ceres::CostFunction* Create(Vector3d const& acc_imu, double const u_i, uint64_t const delta_t_ns) {
        return new ceres::AutoDiffCostFunction<RigidBodyLinearAcceleration, 4, 6, 3, 6, 6, 6, 6>(
            new RigidBodyLinearAcceleration(acc_imu, u_i, delta_t_ns));
    }

    Vector3d acc_imu_;
    double u_i_;
    uint64_t delta_t_ns_;
};

}  // namespace reprojection::optimization::cost_functions
