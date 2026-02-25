#include "spline/se3_spline.hpp"

#include "geometry/lie.hpp"
#include "spline/r3_spline.hpp"
#include "spline/so3_spline.hpp"
#include "spline/spline_evaluation.hpp"

namespace reprojection::spline {

Se3Spline::Se3Spline(TimeHandler const& time_handler, Eigen::Ref<Matrix2NXd const> const& control_points)
    : so3_spline_{control_points.topRows(3), time_handler}, r3_spline_{control_points.bottomRows(3), time_handler} {}

Se3Spline::Se3Spline(TimeHandler const& time_handler, std::vector<Vector6d> const& control_points)
    : Se3Spline(time_handler, Eigen::Map<Matrix2NXd const>(control_points[0].data(), 6, std::size(control_points))) {}

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
