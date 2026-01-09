#pragma once

#include <ceres/ceres.h>

#include <tuple>
#include <vector>

#include "calibration_data_views/optimization_view.hpp"
#include "optimization/spline_cost_function.hpp"
#include "spline/spline_evaluation_concept.hpp"
#include "spline/spline_state.hpp"
#include "spline/types.hpp"

namespace reprojection::optimization {

// NOTE(Jack): We are hardcoding that fact that the intrinsics are the same for all cameras! I.e. not that every image
// could have another camera. Each data view is a view into one single camera so this makes sense!
void CameraNonlinearRefinement(OptimizationDataView data_view);

// TODO(Jack): This does not need to be part of the public interface as it is used internally only as part of
//  CameraNonlinearRefinement, maybe we should also test this, but it is so simple.
ArrayX2d EvaluateReprojectionResiduals(ceres::Problem const& problem,
                                       std::vector<ceres::ResidualBlockId> const& residual_ids);

// NOTE(Jack): At this time it is still not entirely clear if we need to solve this type of problem at all, however this
// code serves as a learning base and step on the way to full pose spline optimization.
// TODO(Jack): Do we really need a class here or can we make it a pass through function? There is no very strong reason
// for state expect that maybe we can better handle invalid constraint input. But that is not clear yet.
class CubicBSplineC3Refinement {
   public:
    explicit CubicBSplineC3Refinement(spline::CubicBSplineC3 const& spline);

    // NOTE(Jack): We will keep this as no discard because I want to force the user to responsibly handle invalid
    // conditions when adding constraints. This is one distinction from the camera bundle adjustment case, that here we
    // have an explicit condition that determines what is valid and what is not, which requires more fine grained
    // control from the user side.
    template <typename T_Model>
        requires spline::CanEvaluateCubicBSplineC3<T_Model>
    [[nodiscard]] bool AddConstraint(spline::C3Measurement const& constraint) {
        auto const normalized_position{
            spline_.time_handler.SplinePosition(constraint.t_ns, std::size(spline_.control_points))};
        if (not normalized_position.has_value()) {
            return false;
        }
        auto const [u_i, i]{normalized_position.value()};

        ceres::CostFunction* const cost_function{optimization::CreateSplineCostFunction_T<T_Model>(
            constraint.type, constraint.r3, u_i, spline_.time_handler.delta_t_ns_)};

        problem_.AddResidualBlock(cost_function, nullptr, spline_.control_points[i].data(),
                                  spline_.control_points[i + 1].data(), spline_.control_points[i + 2].data(),
                                  spline_.control_points[i + 3].data());

        return true;
    }

    // TODO(Jack): There is no protection which would prevent the user from calling this on an invalid problem (ex. they
    // forgot to add any data). We need to codify the real long term usage strategy here, this is not the final answer!
    ceres::Solver::Summary Solve();

    spline::CubicBSplineC3 GetSpline() const;

   private:
    spline::CubicBSplineC3 spline_;  // Stores the state we are optimizing
    ceres::Problem problem_;
};

}  // namespace  reprojection::optimization
