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
    auto const so3_term{EvaluateSpline<So3Spline>(so3_spline_, t_ns, derivative)};
    auto const r3_term{EvaluateSpline<R3Spline>(r3_spline_, t_ns, derivative)};
    if (not(so3_term.has_value() and r3_term.has_value())) {
        return std::nullopt;
    }

    Vector6d result;
    result << so3_term.value(), r3_term.value();

    return result;
}

}  // namespace reprojection::spline
