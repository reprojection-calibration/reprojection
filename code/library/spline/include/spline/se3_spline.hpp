#pragma once

#include <cstdint>
#include <optional>

#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

class Se3Spline {
   public:
    Se3Spline(TimeHandler const& time_handler, Eigen::Ref<Matrix6Xd const> const& control_points);

    Se3Spline(TimeHandler const& time_handler, std::vector<Vector6d> const& control_points);

    // WARN(Jack): This assumes/has as a prerequisite that the control points come from evenly spaced points in
    // time! This is the delta_t_ns passed to the constructor.
    void AddControlPoint(Vector6d const& control_point);

    std::optional<Vector6d> Evaluate(std::uint64_t const t_ns,
                                     DerivativeOrder const derivative = DerivativeOrder::Null) const;

   private:
    CubicBSplineC3 so3_spline_;
    CubicBSplineC3 r3_spline_;
};

}  // namespace reprojection::spline
