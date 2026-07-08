#pragma once

#include <ceres/autodiff_cost_function.h>

#include "spline/constants.hpp"
#include "spline/spline_initialization.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"
#include "utils.hpp"

namespace reprojection::optimization::cost_functions {

// cp = "control point"

class SplineEnergy {
   public:
    template <typename T>
    bool operator()(T const* const cp_0_ptr, T const* const cp_1_ptr, T const* const cp_2_ptr, T const* const cp_3_ptr,
                    T* const residual_ptr) const {
        auto const P{BuildP<T, 6>(cp_0_ptr, cp_1_ptr, cp_2_ptr, cp_3_ptr)};

        // TODO(Jack): When we first designed the spline interpolation code we designed it such that it only
        // interpolates one part of the full se3 spline at a time (i.e. rotation or translation). But this I think was
        // an arbitrary decision and now we know that we use the same code to initialize both the rotation and
        // orientation parts of the spline. That is why we see this logic repeated here just like we see it repeated in
        // the spline initialization function. Maybe one day we can unite them and just make it a single 6d spline that
        // is initialized instead of two 3d ones that are then merged.
        Eigen::Vector<T, 12> const rotations{P.template topRows<3>().reshaped()};
        Eigen::Vector<T, 12> const translations{P.template bottomRows<3>().reshaped()};

        Eigen::Ref<Eigen::Vector<T, 24>> residuals{Eigen::Map<Eigen::Vector<T, 24>>(residual_ptr, 24, 1)};

        residuals.template topRows<12>() = omega_ * rotations;
        residuals.template bottomRows<12>() = omega_ * translations;

        return true;
    }

    static ceres::CostFunction* Create(uint64_t const delta_t_ns) {
        // WARN(Jack): Do not hardcode lambda!?
        spline::CoefficientBlock const omega{spline::BuildOmega(delta_t_ns, 1e3)};

        return new ceres::AutoDiffCostFunction<SplineEnergy, 24, 6, 6, 6, 6>(new SplineEnergy(omega));
    }

    // TODO(Jack): Is it proper to have omega here directly? I am not 100% sure that is the proper way to carry over the
    // constraint from the linear initialization solver to here, but it seems to work at least for the simple case.
    // Someone with a strong understanding of optimization should take a look here.
    spline::CoefficientBlock omega_;
};

}  // namespace reprojection::optimization::cost_functions
