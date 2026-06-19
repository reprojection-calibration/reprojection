#pragma once

#include <ceres/autodiff_cost_function.h>

#include "spline/constants.hpp"
#include "spline/r3_spline.hpp"
#include "spline/so3_spline.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

#include "ceres_geometry.hpp"

namespace reprojection::optimization::cost_functions {

// cp = "control point"

class SplineEnergy {
   public:
    template <typename T>
    bool operator()(T const* const cp_0_ptr, T const* const cp_1_ptr, T const* const cp_2_ptr, T const* const cp_3_ptr,
                    T* const residual) const {
        // TODO(Jack): Do not copy and paste this - multiple places right now use this same code!
        std::array<T const* const, spline::constants::order> const ptrs{cp_0_ptr, cp_1_ptr, cp_2_ptr, cp_3_ptr};
        spline::Matrix2NK<T> control_points;
        for (int i{0}; i < spline::constants::order; ++i) {
            control_points.col(i) = Eigen::Map<Eigen::Vector<T, 6> const>(ptrs[i], 6, 1);
        }

        return true;
    }

    static ceres::CostFunction* Create(Vector3d const& acc_imu, double const u_i, uint64_t const delta_t_ns) {
        return new ceres::AutoDiffCostFunction<SplineEnergy, 24, 6, 6, 6, 6>(
            new SplineEnergy(acc_imu, u_i, delta_t_ns));
    }

    Vector3d acc_imu_;

    double u_i_;
    uint64_t delta_t_ns_;
};

}  // namespace reprojection::optimization::cost_functions
