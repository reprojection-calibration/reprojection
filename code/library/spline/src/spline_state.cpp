

#include "spline/spline_state.hpp"

namespace reprojection::spline {

CubicBSplineC3::CubicBSplineC3(Eigen::Ref<MatrixNXd const> const& control_points, TimeHandler const& time_handler)
    : control_points_{control_points}, time_handler_{time_handler} {}

CubicBSplineC3::CubicBSplineC3(std::vector<Vector3d> const& control_points, TimeHandler const& time_handler)
    : CubicBSplineC3(
          Eigen::Map<MatrixNXd const>(control_points[0].data(), constants::states, std::size(control_points)),
          time_handler) {}

Eigen::Ref<MatrixNXd const> CubicBSplineC3::ControlPoints() const { return control_points_; }

Eigen::Ref<MatrixNXd> CubicBSplineC3::MutableControlPoints() { return control_points_; }

std::optional<std::pair<double, int>> CubicBSplineC3::Position(std::uint64_t const t_ns) const {
    return time_handler_.SplinePosition(t_ns, this->Size());
}

std::uint64_t CubicBSplineC3::DeltaTNs() const { return time_handler_.delta_t_ns_; }

size_t CubicBSplineC3::Size() const {
    return control_points_.cols();
}

;

}  // namespace reprojection::spline