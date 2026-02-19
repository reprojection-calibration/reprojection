#include "spline/se3_spline.hpp"

#include "geometry/lie.hpp"
#include "spline/r3_spline.hpp"
#include "spline/so3_spline.hpp"
#include "spline/spline_evaluation.hpp"

namespace reprojection::spline {

Se3Spline::Se3Spline(std::uint64_t const t0_ns, std::uint64_t const delta_t_ns)
    : so3_spline_{t0_ns, delta_t_ns}, r3_spline_{t0_ns, delta_t_ns} {}

void Se3Spline::AddControlPoint(Vector6d const& control_point) {
    so3_spline_.control_points.push_back(control_point.topRows(3));
    r3_spline_.control_points.push_back(control_point.bottomRows(3));
}

std::optional<Vector6d> Se3Spline::Evaluate(std::uint64_t const t_ns, DerivativeOrder const derivative) const {
    // TODO(Jack): This is in essence repeating logic that we already have implemented elsewhere, is there anything
    //  we can do to streamline this?
    // TODO(Jack): Naming! so3_term and r3_term might not communicated clearly enough what these are here!
    auto const r3_term{EvaluateSpline<R3Spline>(t_ns, r3_spline_, derivative)};
    auto const so3_term{EvaluateSpline<So3Spline>(t_ns, so3_spline_, derivative)};
    if (not(r3_term.has_value() and so3_term.has_value())) {
        return std::nullopt;
    }

    Vector6d result;
    result << so3_term.value(), r3_term.value();

    return result;
}

}  // namespace reprojection::spline
