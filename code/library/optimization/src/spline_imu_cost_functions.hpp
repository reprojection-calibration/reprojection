#pragma once

#include <ceres/autodiff_cost_function.h>

#include "spline/se3_spline.hpp"
#include "spline/types.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

#include "ceres_geometry.hpp"

namespace reprojection::optimization {

ceres::CostFunction* Create(Vector3d const& angular_velocity, double const u_i, uint64_t const delta_t_ns);

class SplineRotationalVelocityCostFunction {
   public:
    SplineRotationalVelocityCostFunction(Vector3d const& omega, double const u_i, uint64_t const delta_t_ns)
        : omega_{omega}, u_i_{u_i}, delta_t_ns_{delta_t_ns} {}

    // TODO MAKE FULL 6D TF!!!
    template <typename T>
    bool operator()(T const* const aa_imu_co, T const* const control_point_0_ptr, T const* const control_point_1_ptr,
                    T const* const control_point_2_ptr, T const* const control_point_3_ptr, T* const residual) const {
        // Map control point pointers into a usable control point matrix block.
        std::array<T const* const, spline::constants::order> ptrs{control_point_0_ptr, control_point_1_ptr,
                                                                  control_point_2_ptr, control_point_3_ptr};
        spline::Matrix2NK<T> control_points;
        for (int i{0}; i < spline::constants::order; ++i) {
            control_points.col(i) = Eigen::Map<Eigen::Vector<T, 6> const>(ptrs[i], 6, 1);
        }

        Vector3<T> const omega_co{spline::So3Spline::Evaluate<T, spline::DerivativeOrder::First>(
            control_points.template topRows<3>(), u_i_, delta_t_ns_)};

        Eigen::Map<Eigen::Vector<T, 3> const> aa_imu_co_XXX(aa_imu_co);
        Array3<T> const omega_imu{geometry::Exp<T>(aa_imu_co_XXX)*omega_co};

        residual[0] = T(omega_[0]) - omega_imu[0];
        residual[1] = T(omega_[1]) - omega_imu[1];
        residual[2] = T(omega_[2]) - omega_imu[2];

        return true;
    }

    static ceres::CostFunction* Create(Vector3d const& angular_velocity, double const u_i, uint64_t const delta_t_ns) {
        return new ceres::AutoDiffCostFunction<SplineRotationalVelocityCostFunction, 3, 3, 6, 6, 6, 6>(
            new SplineRotationalVelocityCostFunction(angular_velocity, u_i, delta_t_ns));
    }

    Vector3d omega_;

    double u_i_;
    uint64_t delta_t_ns_;
};

}  // namespace  reprojection::optimization