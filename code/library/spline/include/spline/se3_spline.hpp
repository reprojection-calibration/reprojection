#pragma once

#include <optional>

#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

class Se3Spline {
   public:
    Se3Spline(TimeHandler const& time_handler, Eigen::Ref<Matrix2NXd const> const& control_points);

    Se3Spline(TimeHandler const& time_handler, std::vector<Vector6d> const& control_points);

    std::optional<Vector6d> Evaluate(std::uint64_t const t_ns,
                                     DerivativeOrder const derivative = DerivativeOrder::Null) const;

   private:
    // TODO(Jack): We have the time hanlder duplicated inside of these two splines. It would be nicer if somehow we
    //  could just store the control points in one big matrix (Matrix2NXd) and then one common time handler that is then
    //  passed off to the evaluation functions for each respective spline.
    CubicBSplineC3 so3_spline_;
    CubicBSplineC3 r3_spline_;
};

}  // namespace reprojection::spline
