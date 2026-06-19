#pragma once

#include <ceres/autodiff_cost_function.h>

#include "spline/constants.hpp"
#include "spline/spline_initialization.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization::cost_functions {

// cp = "control point"

class SplineEnergy {
   public:
    template <typename T>
    bool operator()(T const* const cp_0_ptr, T const* const cp_1_ptr, T const* const cp_2_ptr, T const* const cp_3_ptr,
                    T* const residual_ptr) const {
        // TODO(Jack): Do not copy and paste this - multiple places right now use this same code!
        std::array<T const* const, spline::constants::order> const ptrs{cp_0_ptr, cp_1_ptr, cp_2_ptr, cp_3_ptr};
        spline::Matrix2NK<T> control_points;
        for (int i{0}; i < spline::constants::order; ++i) {
            control_points.col(i) = Eigen::Map<Eigen::Vector<T, 6> const>(ptrs[i], 6, 1);
        }

        Eigen::Vector<T, 12> const rotations{control_points.template topRows<3>().reshaped()};
        Eigen::Vector<T, 12> const translations{control_points.template bottomRows<3>().reshaped()};

        Eigen::Vector<T, 24> residuals{Eigen::Map<Eigen::Vector<T, 24>>(residual_ptr, 24, 1)};

        residuals.template topRows<12>() = omega_ * rotations;
        residuals.template bottomRows<12>() = omega_ * translations;

        return true;
    }

    static ceres::CostFunction* Create(uint64_t const delta_t_ns) {
        // TODO(Jack): Do not hardcode lambda!?
        spline::CoefficientBlock const omega{spline::BuildOmega(delta_t_ns, 1e12)};

        return new ceres::AutoDiffCostFunction<SplineEnergy, 24, 6, 6, 6, 6>(new SplineEnergy(omega));
    }

    // TODO(Jack): Do we want omega here or square root omega?
    spline::CoefficientBlock omega_;
};

}  // namespace reprojection::optimization::cost_functions
