#include "spline/se3_spline.hpp"

#include "geometry/lie.hpp"
#include "spline/r3_spline.hpp"
#include "spline/so3_spline.hpp"
#include "spline/spline_evaluation.hpp"

namespace reprojection::spline {

Se3Spline::Se3Spline(std::uint64_t const t0_ns, std::uint64_t const delta_t_ns)
    : so3_spline_{t0_ns, delta_t_ns}, r3_spline_{t0_ns, delta_t_ns} {}

Se3Spline::Se3Spline(CubicBSplineC3 const& so3_spline, CubicBSplineC3 const& r3_spline)
    : so3_spline_{so3_spline}, r3_spline_{r3_spline} {
    assert(std::size(so3_spline_.control_points) == std::size(r3_spline_.control_points));
    // TODO(Jack): The fact that the so3 and r3 splines have different time handlers that need to be exactly the
    // same, shows that maybe we are missing the point somewhere. We should keep our eyes peeled here for a real
    // solution.
    assert(so3_spline_.time_handler.t0_ns_ == r3_spline_.time_handler.t0_ns_);
    assert(so3_spline_.time_handler.delta_t_ns_ == r3_spline_.time_handler.delta_t_ns_);
}

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
