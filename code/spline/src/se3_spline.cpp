#include "spline/se3_spline.hpp"

#include "geometry/lie.hpp"
#include "spline/r3_spline.hpp"
#include "spline/so3_spline.hpp"
#include "spline/spline_evaluation.hpp"

namespace reprojection::spline {

Se3Spline::Se3Spline(std::uint64_t const t0_ns, std::uint64_t const delta_t_ns)
    : so3_spline_{t0_ns, delta_t_ns}, r3_spline_{t0_ns, delta_t_ns} {}

// TODO(Jack): Refactor this to take se3 vector directly?
void Se3Spline::AddControlPoint(Isometry3d const control_point) {
    r3_spline_.control_points.push_back(control_point.translation());
    so3_spline_.control_points.push_back(geometry::Log<double>(control_point.linear()));
}

std::optional<Vector6d> Se3Spline::Evaluate(std::uint64_t const t_ns) const {
    // TODO(Jack): This is in essence repeating logic that we already have implemented elsewhere, is there anything
    // we can do to streamline this?
    auto const position{EvaluateSpline<R3Spline>(t_ns, r3_spline_)};
    auto const rotation{EvaluateSpline<So3Spline>(t_ns, so3_spline_)};
    if (not(position.has_value() and rotation.has_value())) {
        return std::nullopt;
    }

    Vector6d result;
    result << rotation.value(), position.value();

    return result;
}

}  // namespace reprojection::spline
