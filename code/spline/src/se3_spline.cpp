#include "spline/se3_spline.hpp"

namespace reprojection::spline {

Se3Spline::Se3Spline(std::uint64_t const t0_ns, std::uint64_t const delta_t_ns)
    : r3_spline_{t0_ns, delta_t_ns}, so3_spline_{t0_ns, delta_t_ns} {}

void Se3Spline::AddControlPoint(Isometry3d const control_point) {
    r3_spline_.control_points.push_back(control_point.translation());
    so3_spline_.control_points.push_back(control_point.linear());
}

std::optional<Isometry3d> Se3Spline::Evaluate(std::uint64_t const t_ns) const {
    // TODO(Jack): This is in essence repeating logic that we already have implemented elsewhere, is there anything
    // we can do to streamline this?
    auto const position{EvaluateR3(t_ns, r3_spline_)};
    auto const rotation{EvaluateSO3(t_ns, so3_spline_)};
    if (not(position.has_value() and rotation.has_value())) {
        return std::nullopt;
    }

    Isometry3d result{Isometry3d::Identity()};
    result.rotate(rotation.value());
    result.translation() = position.value();

    return result;
}

}  // namespace reprojection::spline
