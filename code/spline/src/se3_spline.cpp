#include "spline/se3_spline.hpp"

namespace reprojection::spline {

Se3Spline::Se3Spline(uint64_t const t0_ns, uint64_t const delta_t_ns)
    : r3_spline_{t0_ns, delta_t_ns}, so3_spline_{t0_ns, delta_t_ns} {}

void Se3Spline::AddKnot(Isometry3d const knot) {
    r3_spline_.knots_.push_back(knot.translation());
    so3_spline_.knots_.push_back(knot.linear());
}

std::optional<Isometry3d> Se3Spline::Evaluate(uint64_t const t_ns) const {
    // TODO(Jack): This is in essence repeating logic that we already have implemented elsewhere, is there anything
    // we can do to streamline this?
    auto const position{r3_spline_.Evaluate(t_ns)};
    auto const rotation{so3_spline_.Evaluate(t_ns)};
    if (not(position.has_value() and rotation.has_value())) {
        return std::nullopt;
    }

    Isometry3d result{Isometry3d::Identity()};
    result.rotate(rotation.value());
    result.translation() = position.value();

    return result;
}

}  // namespace reprojection::spline
