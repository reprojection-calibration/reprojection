#include "spline/r3_spline.hpp"

#include "spline/constants.hpp"
#include "spline/types.hpp"
#include "spline/utilities.hpp"

namespace reprojection::spline {

r3Spline::r3Spline(std::uint64_t const t0_ns, std::uint64_t const delta_t_ns)
    : time_handler_{t0_ns, delta_t_ns, constants::order} {}

// TODO(Jack): Make this a free function that accepts the spline state, this state can then also be used directly in the
// optimization problem!
std::optional<Vector3d> r3Spline::Evaluate(std::uint64_t const t_ns, DerivativeOrder const derivative) const {
    auto const normalized_position{time_handler_.SplinePosition(t_ns, std::size(control_points_))};
    if (not normalized_position.has_value()) {
        return std::nullopt;
    }
    auto const [u_i, i]{normalized_position.value()};

    Matrix3Kd const P{Eigen::Map<const Matrix3Kd>(control_points_[i].data(), 3, constants::order)};

    if (derivative == DerivativeOrder::Null) {
        return R3SplineEvaluation::Evaluate<double, DerivativeOrder::Null>(P, u_i, time_handler_.delta_t_ns_);
    } else if (derivative == DerivativeOrder::First) {
        return R3SplineEvaluation::Evaluate<double, DerivativeOrder::First>(P, u_i, time_handler_.delta_t_ns_);
    } else if (derivative == DerivativeOrder::Second) {
        return R3SplineEvaluation::Evaluate<double, DerivativeOrder::Second>(P, u_i, time_handler_.delta_t_ns_);
    } else {
        throw std::runtime_error("Requested unknown derivative order from r3Spline::Evaluate()");  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection::spline
